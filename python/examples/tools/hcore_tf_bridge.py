#!/usr/bin/env python3
"""Normalize H-CoRE simulation TF frames for HORUS.

The H-CoRE simulation publishes a flat graph with awkward frame names. The drone
pose is the unprefixed ``base_link`` under ``drone/map -> odom -> base_link``.
The rover pose root is ``rover/base_footprint`` and every rover body/sensor frame
belongs below that root. This bridge keeps the source graph untouched and
republishes a HORUS-facing TF graph under ``/hcore/tf`` and ``/hcore/tf_static``.
"""

from __future__ import annotations

import argparse
import copy
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _normalize_frame(frame_id: str) -> str:
    return str(frame_id or "").strip().lstrip("/")


def _quaternion_multiply(
    lhs: tuple[float, float, float, float],
    rhs: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = lhs
    bx, by, bz, bw = rhs
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quaternion_inverse(
    quat: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x, y, z, w = quat
    norm_sq = x * x + y * y + z * z + w * w
    if norm_sq <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (-x / norm_sq, -y / norm_sq, -z / norm_sq, w / norm_sq)


def _quaternion_rotate(
    quat: tuple[float, float, float, float],
    vec: tuple[float, float, float],
) -> tuple[float, float, float]:
    rotated = _quaternion_multiply(
        _quaternion_multiply(quat, (vec[0], vec[1], vec[2], 0.0)),
        _quaternion_inverse(quat),
    )
    return (rotated[0], rotated[1], rotated[2])


def _compose_pose(
    parent_to_mid: tuple[tuple[float, float, float], tuple[float, float, float, float]],
    mid_to_child: tuple[tuple[float, float, float], tuple[float, float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    a_xyz, a_q = parent_to_mid
    b_xyz, b_q = mid_to_child
    b_rotated = _quaternion_rotate(a_q, b_xyz)
    xyz = (
        a_xyz[0] + b_rotated[0],
        a_xyz[1] + b_rotated[1],
        a_xyz[2] + b_rotated[2],
    )
    return xyz, _quaternion_multiply(a_q, b_q)


def _invert_pose(
    pose: tuple[tuple[float, float, float], tuple[float, float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    xyz, quat = pose
    inv_q = _quaternion_inverse(quat)
    inv_xyz = _quaternion_rotate(inv_q, (-xyz[0], -xyz[1], -xyz[2]))
    return inv_xyz, inv_q


def _transform(
    parent: str,
    child: str,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> TransformStamped:
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(xyz[0])
    msg.transform.translation.y = float(xyz[1])
    msg.transform.translation.z = float(xyz[2])
    msg.transform.rotation.x = float(xyzw[0])
    msg.transform.rotation.y = float(xyzw[1])
    msg.transform.rotation.z = float(xyzw[2])
    msg.transform.rotation.w = float(xyzw[3])
    return msg


class HCoreTFBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("hcore_tf_bridge")
        self.args = args
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.tf_pub = self.create_publisher(TFMessage, args.output_tf_topic, tf_qos)
        self.static_pub = self.create_publisher(TFMessage, args.output_tf_static_topic, static_qos)
        self.create_subscription(TFMessage, args.input_tf_topic, self._on_tf, tf_qos)
        self.create_subscription(TFMessage, args.input_tf_static_topic, self._on_tf_static, static_qos)
        if args.rover_pose_topic:
            self.create_subscription(PoseWithCovarianceStamped, args.rover_pose_topic, self._on_rover_pose, 20)
        self._last_static_publish = 0.0
        self._static_seen: dict[tuple[str, str], TransformStamped] = {}
        self._source_tf: dict[tuple[str, str], tuple[tuple[float, float, float], tuple[float, float, float, float]]] = {}
        self._latest_rover_pose: TransformStamped | None = None
        self._rover_pose_source = ""
        self._warned_missing_rover_base_tf = False
        self._static_timer = self.create_timer(1.0, self._publish_static)
        self.get_logger().info(
            f"Republishing normalized H-CoRE TF: {args.input_tf_topic}->{args.output_tf_topic}, "
            f"{args.input_tf_static_topic}->{args.output_tf_static_topic}"
        )
        self.get_logger().info(
            f"Rover pose sources: pose={args.rover_pose_topic or 'disabled'}, TF tree; "
            f"output base={self.rover_base_frame}"
        )

    @property
    def drone_odom_frame(self) -> str:
        return f"{self.args.drone_prefix}/odom"

    @property
    def drone_map_frame(self) -> str:
        return self.args.drone_map_frame

    @property
    def drone_base_frame(self) -> str:
        return f"{self.args.drone_prefix}/base_link"

    @property
    def rover_base_frame(self) -> str:
        return f"{self.args.rover_prefix}/base_footprint"

    @property
    def rover_body_frame(self) -> str:
        return f"{self.args.rover_prefix}/base_link"

    def _remap_frame(self, frame_id: str) -> str:
        frame = _normalize_frame(frame_id)
        if not frame:
            return frame

        if frame == self.args.world_frame:
            return self.args.world_frame
        if frame == self.args.drone_map_frame:
            return self.drone_map_frame
        if frame == self.args.drone_odom_frame:
            return self.drone_odom_frame
        if frame == self.args.drone_base_frame:
            return self.drone_base_frame
        if frame.startswith(self.args.drone_model_prefix + "/"):
            return f"{self.args.drone_prefix}/{frame}"
        if frame == "base_link_FRD":
            return f"{self.args.drone_prefix}/{frame}"

        if frame == self.args.rover_base_frame:
            return self.rover_base_frame
        rover_prefix = self.args.rover_source_prefix.rstrip("/") + "/"
        if frame.startswith(rover_prefix):
            return f"{self.args.rover_prefix}/{frame[len(rover_prefix):]}"

        return frame

    def _store_source_transform(self, transform: TransformStamped) -> None:
        parent = _normalize_frame(transform.header.frame_id)
        child = _normalize_frame(transform.child_frame_id)
        if not parent or not child or parent == child:
            return
        self._source_tf[(parent, child)] = (
            (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                float(transform.transform.translation.z),
            ),
            (
                float(transform.transform.rotation.x),
                float(transform.transform.rotation.y),
                float(transform.transform.rotation.z),
                float(transform.transform.rotation.w),
            ),
        )

    def _lookup_source_transform(
        self,
        parent: str,
        child: str,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]] | None:
        parent = _normalize_frame(parent)
        child = _normalize_frame(child)
        if not parent or not child:
            return None
        if parent == child:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)

        queue: list[tuple[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]]] = [
            (parent, ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
        ]
        visited = {parent}
        adjacency: dict[str, list[tuple[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]]]] = {}
        for (edge_parent, edge_child), pose in self._source_tf.items():
            adjacency.setdefault(edge_parent, []).append((edge_child, pose))
            adjacency.setdefault(edge_child, []).append((edge_parent, _invert_pose(pose)))

        while queue:
            frame, accumulated = queue.pop(0)
            for next_frame, edge_pose in adjacency.get(frame, []):
                if next_frame in visited:
                    continue
                next_pose = _compose_pose(accumulated, edge_pose)
                if next_frame == child:
                    return next_pose
                visited.add(next_frame)
                queue.append((next_frame, next_pose))
        return None

    def _refresh_rover_pose_from_tf(self, stamp) -> None:
        source = f"{self.args.input_tf_topic}/{self.args.input_tf_static_topic}:{self.args.world_frame}->{self.args.rover_base_frame}"
        pose = self._lookup_source_transform(self.args.world_frame, self.args.rover_base_frame)
        if pose is None:
            pose = self._lookup_source_transform(self.args.world_frame, self.args.rover_odom_frame)
            source = (
                f"{self.args.input_tf_topic}/{self.args.input_tf_static_topic}:"
                f"{self.args.world_frame}->{self.args.rover_odom_frame} as {self.rover_base_frame}"
            )
            if pose is not None and not self._warned_missing_rover_base_tf:
                self._warned_missing_rover_base_tf = True
                self.get_logger().warning(
                    f"No source TF path {self.args.world_frame}->{self.args.rover_base_frame} exists. "
                    f"Using {self.args.world_frame}->{self.args.rover_odom_frame} as the rover base pose."
                )
        if pose is None:
            return

        xyz, quat = pose
        transform = _transform(self.args.world_frame, self.rover_base_frame, xyz, quat)
        transform.header.stamp = stamp
        if self._rover_pose_source != source:
            self._rover_pose_source = source
            self.get_logger().info(
                f"Using {source}: xyz=({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) "
                f"for map->{self.rover_base_frame}."
            )
        self._latest_rover_pose = transform

    def _is_output_frame(self, frame_id: str) -> bool:
        frame = str(frame_id or "").strip().lstrip("/")
        return (
            frame == self.args.world_frame
            or frame == self.drone_map_frame
            or frame.startswith(f"{self.args.drone_prefix}/")
            or frame.startswith(f"{self.args.rover_prefix}/")
        )

    def _remap_transform(self, transform: TransformStamped) -> TransformStamped | None:
        mapped = copy.deepcopy(transform)
        parent = self._remap_frame(mapped.header.frame_id)
        child = self._remap_frame(mapped.child_frame_id)
        if not parent or not child or parent == child:
            return None
        if not self._is_output_frame(parent) or not self._is_output_frame(child):
            return None
        mapped.header.frame_id = parent
        mapped.child_frame_id = child
        return mapped

    def _on_tf(self, msg: TFMessage) -> None:
        transforms = []
        anchor_stamp = self.get_clock().now().to_msg()
        for transform in msg.transforms:
            self._store_source_transform(transform)
            anchor_stamp = transform.header.stamp
            mapped = self._remap_transform(transform)
            if mapped is not None:
                transforms.append(mapped)
        self._refresh_rover_pose_from_tf(anchor_stamp)
        transforms.extend(self._dynamic_anchor_transforms(anchor_stamp))
        if transforms:
            self.tf_pub.publish(TFMessage(transforms=transforms))

    def _on_tf_static(self, msg: TFMessage) -> None:
        changed = False
        for transform in msg.transforms:
            self._store_source_transform(transform)
            mapped = self._remap_transform(transform)
            if mapped is None:
                continue
            key = (mapped.header.frame_id, mapped.child_frame_id)
            self._static_seen[key] = mapped
            changed = True
        self._refresh_rover_pose_from_tf(self.get_clock().now().to_msg())
        if changed:
            self._publish_static()

    def _pose_to_rover_transform(self, pose, stamp, source: str) -> TransformStamped:
        transform = _transform(self.args.world_frame, self.rover_base_frame)
        transform.header.stamp = stamp
        transform.transform.translation.x = float(pose.position.x)
        transform.transform.translation.y = float(pose.position.y)
        transform.transform.translation.z = float(pose.position.z)
        transform.transform.rotation = copy.deepcopy(pose.orientation)
        if self._rover_pose_source != source:
            self._rover_pose_source = source
            self.get_logger().info(f"Using {source} for map->{self.rover_base_frame}.")
        return transform

    def _on_rover_pose(self, msg: PoseWithCovarianceStamped) -> None:
        frame = str(msg.header.frame_id or "").strip().strip("/")
        if frame and frame != self.args.world_frame:
            self.get_logger().warning(
                f"Ignoring rover pose from {self.args.rover_pose_topic}: "
                f"frame '{frame}' is not the world frame '{self.args.world_frame}'."
            )
            return
        self._latest_rover_pose = self._pose_to_rover_transform(
            msg.pose.pose,
            msg.header.stamp,
            self.args.rover_pose_topic,
        )

    def _builtin_static_transforms(self, stamp=None) -> list[TransformStamped]:
        now = stamp if stamp is not None else self.get_clock().now().to_msg()
        transforms = [
            _transform(self.args.world_frame, self.drone_map_frame, self.args.drone_map_offset),
            _transform(self.rover_base_frame, self.rover_body_frame, (0.0, 0.0, 0.06175)),
            _transform(self.rover_body_frame, f"{self.args.rover_prefix}/laser"),
            _transform(
                self.drone_base_frame,
                f"{self.args.drone_prefix}/base_link_FRD",
                xyzw=_quaternion_from_rpy(math.pi, 0.0, 0.0),
            ),
            _transform(
                self.drone_base_frame,
                f"{self.args.drone_prefix}/{self.args.drone_rgb_frame}",
                (0.2, 0.0, -0.03),
                _quaternion_from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0),
            ),
            _transform(
                self.drone_base_frame,
                f"{self.args.drone_prefix}/{self.args.drone_depth_frame}",
                (0.2, 0.0, -0.03),
                _quaternion_from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0),
            ),
        ]
        for transform in transforms:
            transform.header.stamp = now
        return transforms

    def _dynamic_anchor_transforms(self, stamp=None) -> list[TransformStamped]:
        # HORUS robot placement follows the dynamic TF stream. Repeat the
        # static sensor/body frames on /hcore/tf, but never overwrite the rover
        # base with a fixed transform once live odometry/pose is available.
        transforms = self._builtin_static_transforms(stamp)
        if self._latest_rover_pose is not None:
            transforms.append(copy.deepcopy(self._latest_rover_pose))
        elif self.args.publish_rover_initial_pose:
            initial = _transform(self.args.world_frame, self.rover_base_frame, self.args.rover_initial_pose)
            initial.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
            transforms.append(initial)
        return transforms

    def _publish_static(self) -> None:
        now_monotonic = time.monotonic()
        if now_monotonic - self._last_static_publish < 0.25:
            return
        self._last_static_publish = now_monotonic
        now = self.get_clock().now().to_msg()
        transforms = []
        for transform in self._builtin_static_transforms():
            transforms.append(transform)
        for transform in self._static_seen.values():
            mapped = copy.deepcopy(transform)
            mapped.header.stamp = now
            transforms.append(mapped)
        self.static_pub.publish(TFMessage(transforms=transforms))


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Normalize H-CoRE TF for HORUS.")
    parser.add_argument("--input-tf-topic", default="/tf")
    parser.add_argument("--input-tf-static-topic", default="/tf_static")
    parser.add_argument("--output-tf-topic", default="/hcore/tf")
    parser.add_argument("--output-tf-static-topic", default="/hcore/tf_static")
    parser.add_argument("--world-frame", default="map")
    parser.add_argument("--drone-prefix", default="hcore_drone")
    parser.add_argument("--drone-map-frame", default="drone/map")
    parser.add_argument("--drone-map-offset", nargs=3, type=float, default=(0.0, 1.0, 0.0))
    parser.add_argument("--drone-odom-frame", default="odom")
    parser.add_argument("--drone-base-frame", default="base_link")
    parser.add_argument("--drone-model-prefix", default="baby_k_0")
    parser.add_argument("--drone-rgb-frame", default="baby_k_0/OakD-Lite/base_link/IMX214")
    parser.add_argument("--drone-depth-frame", default="baby_k_0/OakD-Lite/base_link/StereoOV7251")
    parser.add_argument("--rover-prefix", default="hcore_rover")
    parser.add_argument("--rover-source-prefix", default="rover")
    parser.add_argument(
        "--rover-pose-topic",
        default="",
        help="Optional map-frame PoseWithCovarianceStamped source for rover pose. Disabled by default.",
    )
    parser.add_argument("--rover-odom-topic", default="/odom/wheels", help=argparse.SUPPRESS)
    parser.add_argument("--publish-rover-initial-pose", action="store_true")
    parser.add_argument("--rover-initial-pose", nargs=3, type=float, default=(1.2, 6.78, 0.0))
    parser.add_argument("--rover-odom-frame", default="rover/odom")
    parser.add_argument("--rover-base-frame", default="rover/base_footprint")
    return parser


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()
    rclpy.init()
    node = HCoreTFBridge(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
