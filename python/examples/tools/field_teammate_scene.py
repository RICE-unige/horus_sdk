#!/usr/bin/env python3
"""Present native Field geometry and camera poses in one metric, grounded view.

The datum is the lowest observed startup geometry, not a measured physical floor.
Only TF/manifest frame aliases change; original ROS maps and sensor data do not.
"""

from collections import deque
from dataclasses import dataclass
import argparse
import json
import math
import time


def scene_frames(name):
    prefix = f"{name}_scene"
    return f"{prefix}/floor", f"{prefix}/source", f"{prefix}/camera"


class SceneDatum:
    def __init__(self):
        self.epoch = None
        self.source_frame = ""
        self.floor_z = None
        self.samples = deque(maxlen=3)

    def observe(self, manifest):
        epoch = manifest.get("epoch")
        bounds = manifest.get("height_bounds_m")
        source_frame = manifest.get("source_frame_id", manifest.get("frame_id", ""))
        if epoch is None or not source_frame:
            return False
        if (epoch, source_frame) != (self.epoch, self.source_frame):
            self.epoch, self.source_frame = epoch, source_frame
            self.floor_z = None
            self.samples.clear()
        if (not isinstance(bounds, list) or len(bounds) != 2 or
                not all(isinstance(value, (float, int)) and not isinstance(value, bool)
                        and math.isfinite(value) for value in bounds) or
                bounds[1] - bounds[0] < 0.5 or manifest.get("triangles", 0) < 100):
            return False
        if self.floor_z is not None:
            return True
        self.samples.append(bounds[0])
        if len(self.samples) == 3 and max(self.samples) - min(self.samples) <= 0.03:
            self.floor_z = min(self.samples)
        return self.floor_z is not None


def camera_pose_is_consistent(camera, depth, source_frame):
    if camera.header.frame_id != source_frame or depth.header.frame_id != source_frame:
        return False
    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec / 1e9
    dt = abs(stamp(camera) - stamp(depth))
    a, b = camera.pose.position, depth.pose.position
    values = (a.x, a.y, a.z, b.x, b.y, b.z)
    q = camera.pose.orientation
    rotation = (q.x, q.y, q.z, q.w)
    return (all(math.isfinite(v) for v in (*values, *rotation)) and
            abs(sum(v * v for v in rotation) - 1) < .02 and dt <= .35 and
            math.dist(values[:3], values[3:]) <= .30 + dt * 1.5)


def quaternion_product(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotate(q, position):
    inverse = (-q[0], -q[1], -q[2], q[3])
    return quaternion_product(quaternion_product(q, (*position, 0)), inverse)[:3]


def rotation_distance(a, b):
    return 2 * math.acos(min(1.0, abs(sum(x * y for x, y in zip(a, b)))))


def relative_pose(head, camera):
    inverse = (-head.orientation[0], -head.orientation[1], -head.orientation[2], head.orientation[3])
    return (rotate(inverse, tuple(b - a for a, b in zip(head.position, camera.position))),
            quaternion_product(inverse, camera.orientation))


def compose_pose(head, extrinsic):
    offset = rotate(head.orientation, extrinsic[0])
    return (tuple(a + b for a, b in zip(head.position, offset)),
            quaternion_product(head.orientation, extrinsic[1]))


@dataclass(frozen=True)
class TrackingSample:
    stamp_ns: int
    source_time: float
    origin: tuple
    valid: bool
    position: tuple | None
    orientation: tuple | None


def tracking_sample(packet, frame, child, source):
    """Decode the relay's local atomic tracking contract, never a legacy fallback."""
    if (packet["schema"] != "horus.field_tracking.v1" or
            packet["header"]["frame_id"] != frame or packet["child_frame_id"] != child or
            packet["pose_source"] != source or not isinstance(packet["session_id"], str) or
            not packet["session_id"] or type(packet["tracking_valid"]) is not bool):
        raise ValueError("Invalid tracking identity")
    stamp = packet["header"]["stamp"]
    if (type(stamp["sec"]) is not int or type(stamp["nanosec"]) is not int or
            stamp["sec"] < 0 or not 0 <= stamp["nanosec"] < 1_000_000_000):
        raise ValueError("Invalid tracking stamp")
    revision = packet["tracking_origin_revision"]
    valid = packet["tracking_valid"]
    source_time = packet["source_timestamp_sec"]
    position = orientation = None
    if valid:
        if (type(revision) is not int or revision < 0 or
                not isinstance(source_time, (float, int)) or isinstance(source_time, bool) or
                not math.isfinite(source_time) or source_time <= 0):
            raise ValueError("Missing tracking epoch or capture clock")
        position, orientation = tuple(packet["position"]), tuple(packet["orientation_xyzw"])
        if (len(position) != 3 or len(orientation) != 4 or
                not all(type(v) in (float, int) and math.isfinite(v) for v in (*position, *orientation)) or
                abs(sum(v * v for v in orientation) - 1) >= .02):
            raise ValueError("Invalid measured tracking pose")
        norm = math.sqrt(sum(v * v for v in orientation))
        orientation = tuple(v / norm for v in orientation)
    return TrackingSample(stamp["sec"] * 1_000_000_000 + stamp["nanosec"], source_time,
                          (packet["session_id"], revision), valid, position, orientation)


class SceneTracking:
    """Latch origin alignment, not depth availability; never replay a head sample.

    The scene camera remains the PV body frame used by registration. Its rigid
    head->PV offset is measured from time-matched native samples, not assumed
    identity. A discontinuity cannot be repaired by fitting a new world origin.
    """
    def __init__(self, name):
        self.name = name
        self.frame = ""
        self.geometry = None
        self.origin = None
        self.retired_sessions = set()
        self.last_head = None
        self.last_source_time = -math.inf
        self.last_arrival = -math.inf
        self.last_camera = None
        self.last_camera_arrival = -math.inf
        self.last_output_ns = -1
        self.blocked = False
        self.native_motion_discontinuities = 0
        self.native_depth_residual_m = None
        self.invalidate("waiting_for_observed_geometry")

    def _invalidate_head(self, reason):
        self.aligned = False
        self.extrinsic = None
        self.candidates = deque(maxlen=3)
        self.heads = deque(maxlen=120)
        self.paired_times = set()
        self.state = reason

    def invalidate(self, reason):
        self._invalidate_head(reason)
        self.native_aligned = False
        self.head_tracking_valid = False
        self.pose_source = None
        self.cameras = deque(maxlen=40)
        self.depths = deque(maxlen=10)
        self.boundary_ns = None

    def observe_geometry(self, epoch, frame):
        if (epoch, frame) != self.geometry:
            self.geometry = (epoch, frame)
            self.frame = frame
            self.blocked = False
            self.last_head = None
            self.last_camera = None
            self.invalidate("waiting_for_origin_alignment")

    def _sample(self, packet, kind):
        return tracking_sample(packet, self.frame, f"{self.name}/{kind}",
                               "unity_head" if kind == "head" else "hololens_locatable_pv_camera")

    @staticmethod
    def _fresh(sample, ros_now):
        return -.1 <= ros_now - sample.stamp_ns / 1e9 <= .25

    def observe_head(self, packet, now, ros_now):
        try:
            sample = self._sample(packet, "head")
        except (KeyError, TypeError, ValueError, OverflowError):
            self.invalidate("invalid_head_packet")
            return None
        session, revision = sample.origin
        if session in self.retired_sessions:
            return None
        if (self.origin is not None and session == self.origin[0] and type(revision) is int and
                revision < self.origin[1] and (sample.valid or revision > 0)):
            return None
        if not self._fresh(sample, ros_now):
            self._invalidate_head("waiting_for_fresh_head_tracking")
            return None
        if not sample.valid:
            self.invalidate("waiting_for_revisioned_head_tracking" if revision is None else "waiting_for_head_tracking")
            return None
        if sample.origin != self.origin:
            if self.origin is not None and session != self.origin[0]:
                self.retired_sessions.add(self.origin[0])
            self.origin = sample.origin
            self.last_source_time = -math.inf
            self.invalidate("waiting_for_origin_alignment")
        if sample.source_time <= self.last_source_time:
            return None
        self.last_source_time = sample.source_time
        self.head_tracking_valid = True
        if self.boundary_ns is None:
            self.boundary_ns = sample.stamp_ns
        if self.blocked:
            self.state = "origin_jump_requires_new_geometry"
            return None
        previous = self.last_head
        if previous is not None:
            elapsed = ((sample.source_time - previous.source_time) if sample.origin == previous.origin
                       else (sample.stamp_ns - previous.stamp_ns) / 1e9)
            if elapsed <= 0:
                return None
            gap = elapsed > .5 or now - self.last_arrival > .5
            # Motion during missing samples is unknown. Revalidate a same-epoch
            # return, rather than declaring a legitimate walk an origin jump.
            if gap:
                if sample.origin != previous.origin:
                    self.blocked = True
                    self._invalidate_head("origin_jump_requires_new_geometry")
                    return None
                self._invalidate_head("waiting_for_origin_alignment")
            elif (math.dist(sample.position, previous.position) > .15 + 3 * elapsed or
                    rotation_distance(sample.orientation, previous.orientation) > .25 + 10 * elapsed):
                self.blocked = True
                self._invalidate_head("origin_jump_requires_new_geometry")
                return None
        self.last_head = sample
        self.last_arrival = now
        self.heads.append(sample)
        self._try_align(now)
        if not self.aligned or sample.stamp_ns <= self.last_output_ns:
            return None
        self.state = "live"
        self.pose_source = "measured_head_with_observed_rigid_pv_extrinsic"
        self.last_output_ns = sample.stamp_ns
        return sample, compose_pose(sample, self.extrinsic)

    def observe_camera(self, packet, now, ros_now):
        try:
            sample = self._sample(packet, "pv_camera")
        except (KeyError, TypeError, ValueError, OverflowError):
            return
        if sample.origin != self.origin or not self._fresh(sample, ros_now):
            return
        if not sample.valid:
            self.invalidate("waiting_for_native_camera_tracking")
            return
        if (not self.head_tracking_valid or self.boundary_ns is None or
                sample.stamp_ns < self.boundary_ns):
            return
        previous = self.last_camera
        if previous is not None:
            if sample.origin == previous.origin and sample.source_time <= previous.source_time:
                return
            if sample.stamp_ns <= previous.stamp_ns:
                return
            elapsed = sample.source_time - previous.source_time
            if (sample.origin == previous.origin and elapsed > 0 and
                    (math.dist(sample.position, previous.position) > .15 + 3 * elapsed or
                     rotation_distance(sample.orientation, previous.orientation) > .25 + 10 * elapsed)):
                self.native_motion_discontinuities += 1
            # Arrival bursts and movement are not origin-change evidence. The
            # native tracking validity/revision is authoritative for this pose.
        self.last_camera = sample
        self.last_camera_arrival = now
        self.cameras.append((sample, now))
        self.paired_times.intersection_update(camera.source_time for camera, _ in self.cameras)
        self._try_align(now)
        if (self.native_aligned and not (self.aligned and now - self.last_arrival <= .5) and
                sample.stamp_ns > self.last_output_ns):
            self.pose_source = "native_pv"
            self.last_output_ns = sample.stamp_ns
            return sample

    def observe_depth(self, message, now):
        if message.header.frame_id != self.frame or self.boundary_ns is None:
            return
        position, q = message.pose.position, message.pose.orientation
        position, rotation = (position.x, position.y, position.z), (q.x, q.y, q.z, q.w)
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if (stamp_ns < self.boundary_ns or
                not all(math.isfinite(v) for v in (*position, *rotation)) or
                abs(sum(v * v for v in rotation) - 1) >= .02):
            return
        self.depths.append((stamp_ns, position, now))
        self._try_align(now)

    def _try_align(self, now):
        origin_checked = False
        # Native PV is already in the measured map origin. Validate it against
        # depth independently of the Unity head's potentially cosmetic origin.
        camera = self.cameras[-1][0] if self.cameras and now - self.cameras[-1][1] <= .5 else None
        pairs = [(abs(stamp_ns - camera.stamp_ns) / 1e9, camera, position)
                 for stamp_ns, position, depth_arrival in self.depths
                 if camera is not None and now - depth_arrival <= 1.5 and
                 abs(stamp_ns - camera.stamp_ns) <= 350_000_000]
        if pairs:
            dt, camera, position = min(pairs, key=lambda pair: pair[0])
            self.native_depth_residual_m = math.dist(camera.position, position)
            origin_checked = self.native_depth_residual_m <= .30 + 1.5 * dt
            if not origin_checked and not self.native_aligned:
                self.state = "camera_frames_not_aligned"
        # Depth headers use receive time, not the PV capture clock. Once this
        # revision is validated, asynchronous residuals are diagnostic only;
        # they must not permanently freeze a valid native camera trajectory.
        if origin_checked:
            self.native_aligned = True
        if self.blocked or not self.heads:
            return
        for camera, arrival in self.cameras:
            if now - arrival > .5:
                continue
            head = min(self.heads, key=lambda h: abs(h.source_time - camera.source_time))
            # Both clocks here are Unity sample time, not ROS receive timestamps.
            delta = abs(head.source_time - camera.source_time)
            if delta > .025:
                if not self.aligned:
                    self.state = "waiting_for_shared_capture_clock" if delta > 10 else "waiting_for_time_matched_head_pv"
                continue
            extrinsic = relative_pose(head, camera)
            if self.aligned:
                if (math.dist(extrinsic[0], self.extrinsic[0]) > .30 or
                        rotation_distance(extrinsic[1], self.extrinsic[1]) > math.radians(20)):
                    self.blocked = True
                    self._invalidate_head("origin_jump_requires_new_geometry")
                    return
            elif camera.source_time not in self.paired_times:
                self.paired_times.add(camera.source_time)
                if math.dist(extrinsic[0], (0, 0, 0)) > .30:
                    self.candidates.clear()
                    self.state = "head_camera_frames_not_aligned"
                    continue
                if self.candidates and (math.dist(extrinsic[0], self.candidates[0][1][0]) > .02 or
                        rotation_distance(extrinsic[1], self.candidates[0][1][1]) > math.radians(3)):
                    self.candidates.clear()
                # Check every matched pose above, but retain evidence spanning
                # time instead of the three newest frames at the camera rate.
                if not self.candidates or camera.source_time - self.candidates[-1][0] >= .1 - 1e-9:
                    self.candidates.append((camera.source_time, extrinsic))
        if (not self.aligned and self.native_aligned and len(self.candidates) == 3 and
                self.candidates[-1][0] - self.candidates[0][0] >= .2 - 1e-9):
            self.extrinsic = self.candidates[0][1]
            self.aligned = True
            self.state = "live"

    def status(self, now):
        if self.aligned and now - self.last_arrival > .5:
            self._invalidate_head("waiting_for_fresh_head_tracking")
        if self.aligned or (self.native_aligned and now - self.last_camera_arrival <= .5):
            return "live"
        if self.native_aligned:
            return "waiting_for_fresh_native_camera_pose"
        return self.state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="field_teammate_1")
    args = parser.parse_args()
    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    class ScenePresentation(Node):
        def __init__(self):
            super().__init__("horus_field_scene")
            self.datum = SceneDatum()
            self.floor_frame, self.source_frame, self.camera_frame = scene_frames(args.name)
            self.tracking = SceneTracking(args.name)
            self.last_static = None
            self.state = "waiting_for_observed_geometry"
            self.dynamic = TransformBroadcaster(self)
            self.static = StaticTransformBroadcaster(self)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            tracking_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(String, f"/{args.name}/tracking/head_pose", self.on_head, qos)
            self.create_subscription(String, f"/{args.name}/tracking/pv_pose", self.on_camera, tracking_qos)
            self.create_subscription(PoseStamped, f"/{args.name}/research/long_throw/pose", self.on_depth, qos)
            self.create_subscription(String, "/horus/field/mesh/indexed_manifest", self.on_manifest,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.status = self.create_publisher(String, f"/{args.name}/scene/status", 1)
            self.timer = self.create_timer(1.0, self.publish_status)

        def on_manifest(self, message):
            try:
                manifest = json.loads(message.data)
                if not isinstance(manifest, dict):
                    return
                ready = self.datum.observe(manifest)
                if self.datum.epoch is not None:
                    self.tracking.observe_geometry(self.datum.epoch, self.datum.source_frame)
                if not ready:
                    return
            except (ValueError, TypeError):
                return
            signature = (self.datum.epoch, self.datum.source_frame, self.datum.floor_z)
            if signature == self.last_static:
                return
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self.floor_frame
            transform.child_frame_id = self.source_frame
            transform.transform.translation.z = -self.datum.floor_z
            transform.transform.rotation.w = 1.0
            # Give fixed-frame consumers an explicit identity sample, not just an implicit TF root.
            anchor = TransformStamped()
            anchor.header.stamp = transform.header.stamp
            anchor.header.frame_id = f"{args.name}_scene/world"
            anchor.child_frame_id = self.floor_frame
            anchor.transform.rotation.w = 1.0
            self.static.sendTransform([anchor, transform])
            self.last_static = signature
            self.state = "waiting_for_native_camera_pose"
            self.get_logger().info(f"Visual geometry datum locked at source Z={self.datum.floor_z:.3f}m; map vertices unchanged")

        def on_depth(self, message):
            self.tracking.observe_depth(message, time.monotonic())

        def on_camera(self, message):
            try:
                packet = json.loads(message.data)
            except (ValueError, TypeError):
                return
            sample = self.tracking.observe_camera(packet, time.monotonic(), self.get_clock().now().nanoseconds / 1e9)
            if sample is not None and self.datum.floor_z is not None:
                self.dynamic.sendTransform(self.make_transform(
                    sample, self.camera_frame, (sample.position, sample.orientation)))

        def on_head(self, message):
            try:
                packet = json.loads(message.data)
            except (ValueError, TypeError):
                self.tracking.invalidate("invalid_head_packet")
                return
            result = self.tracking.observe_head(packet, time.monotonic(), self.get_clock().now().nanoseconds / 1e9)
            if result is None or self.datum.floor_z is None:
                return
            sample, camera_pose = result
            head = self.make_transform(sample, f"{args.name}_scene/head", (sample.position, sample.orientation))
            camera = self.make_transform(sample, self.camera_frame, camera_pose)
            self.dynamic.sendTransform([head, camera])

        def make_transform(self, sample, child, pose):
            transform = TransformStamped()
            transform.header.stamp.sec = sample.stamp_ns // 1_000_000_000
            transform.header.stamp.nanosec = sample.stamp_ns % 1_000_000_000
            transform.header.frame_id = self.source_frame
            transform.child_frame_id = child
            t, q = transform.transform.translation, transform.transform.rotation
            t.x, t.y, t.z = pose[0]
            q.x, q.y, q.z, q.w = pose[1]
            return transform

        def publish_status(self):
            self.state = self.tracking.status(time.monotonic()) if self.datum.floor_z is not None else "waiting_for_observed_geometry"
            self.status.publish(String(data=json.dumps({"state": self.state,
                "datum": "lowest_observed_startup_geometry", "source_floor_z_m": self.datum.floor_z,
                "floor_frame": self.floor_frame, "source_frame": self.source_frame,
                "camera_frame": self.camera_frame, "head_frame": f"{args.name}_scene/head",
                "tracking_origin": self.tracking.origin,
                "head_pv_extrinsic": self.tracking.extrinsic,
                "camera_pose_source": self.tracking.pose_source,
                "highrate_head_state": self.tracking.state,
                "native_motion_discontinuities": self.tracking.native_motion_discontinuities,
                "native_depth_residual_m": self.tracking.native_depth_residual_m})))

    rclpy.init()
    node = None
    try:
        node = ScenePresentation()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
