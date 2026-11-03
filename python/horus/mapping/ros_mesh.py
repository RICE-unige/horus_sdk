"""ROS Marker/MarkerArray to HORUS live mesh transport.

Run with ``python -m horus.mapping.ros_mesh --input-topic /map_mesh``.
The input is an incremental map: omitted IDs are unchanged, not deleted.
"""

import argparse
from array import array
from collections import deque
import json
import math
import time

from .marker_mesh import MarkerMeshState, vector, quaternion
from .mesh_page_scheduler import MeshPageScheduler


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", required=True)
    parser.add_argument("--input-type", choices=("Marker", "MarkerArray"), default="Marker")
    parser.add_argument("--output-topic", default="/horus/maps/live")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--triangles-per-page", type=int, default=4096)
    parser.add_argument("--max-pages", type=int, default=1024)
    parser.add_argument("--max-cache-mib", type=float, default=256)
    parser.add_argument("--max-publish-mib-per-second", type=float, default=4)
    args, ros_args = parser.parse_known_args(argv)
    if ros_args and ros_args[0] != "--ros-args":
        parser.error(f"unrecognized arguments: {' '.join(ros_args)}")
    if (not args.input_topic.startswith("/") or not args.output_topic.startswith("/")
            or args.input_topic == args.output_topic or args.output_topic.endswith("/")):
        parser.error("use distinct absolute input/output topics, without a trailing slash")
    if not 1 <= args.triangles_per_page <= 20000 or not 1 <= args.max_pages <= 1024:
        parser.error("page limits: 1..20000 triangles, 1..1024 pages")
    if not math.isfinite(args.max_cache_mib) or args.max_cache_mib < 2:
        parser.error("cache size must be finite and at least 2 MiB")
    if not math.isfinite(args.max_publish_mib_per_second) or args.max_publish_mib_per_second <= 0:
        parser.error("publish byte rate must be finite and positive")
    if not args.frame_id.strip().lstrip("/"):
        parser.error("a fixed frame is required")
    return args, ros_args


def create_node(args):
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.time import Time
    from std_msgs.msg import String, UInt8MultiArray
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import Buffer, TransformListener, TransformException

    class MeshAdapter(Node):
        def __init__(self):
            super().__init__("horus_live_mesh_adapter")
            self.map = MarkerMeshState(
                frame_id=args.frame_id.strip().lstrip("/"), triangles_per_page=args.triangles_per_page,
                max_pages=args.max_pages, max_payload_bytes=2 * 1024 * 1024,
                max_cache_bytes=int(args.max_cache_mib * 1024 * 1024))
            self.scheduler = MeshPageScheduler(pages_per_second=60,
                bytes_per_second=args.max_publish_mib_per_second * 1024 * 1024,
                max_pages=args.max_pages, max_payload_bytes=self.map.state.max_payload_bytes,
                now=time.monotonic())
            self.publisher = self.create_publisher(UInt8MultiArray, args.output_topic + "/indexed", 64)
            self.manifest = self.create_publisher(String, args.output_topic + "/indexed_manifest",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.status = self.create_publisher(String, args.output_topic + "/status", 1)
            self.request = self.create_subscription(String, args.output_topic + "/indexed_request", self.repair, 16)
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.source = None
            self.received = self.rejected = self.sent = self.sent_bytes = self.repairs = 0
            self.last_warning = -math.inf
            self.last_repair = {}
            self.last_sent = {}
            self.deferred = deque()
            self.deferred_points = 0
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.discovery = self.create_timer(0.5, self.connect_source, clock=steady)
            self.pump = self.create_timer(0.01, self.publish_pending, clock=steady)
            self.inventory = self.create_timer(2, self.publish_manifest, clock=steady)

        def connect_source(self):
            if self.source is not None:
                return
            endpoints = self.get_publishers_info_by_topic(args.input_topic)
            if not endpoints:
                return
            expected = "visualization_msgs/msg/" + args.input_type
            if any(p.topic_type != expected for p in endpoints):
                self.warn(f"input must have type {expected}")
                return
            reliable = all(p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in endpoints)
            transient = all(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in endpoints)
            qos = QoSProfile(depth=64,
                reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
            self.source = self.create_subscription(Marker if args.input_type == "Marker" else MarkerArray,
                                                    args.input_topic, self.receive, qos)
            self.get_logger().info(f"Map source {args.input_topic}: reliable={reliable}, transient={transient}")
            if not reliable:
                self.warn("Best-effort source can lose changes before the cache; use reliable input or a source snapshot.")

        def ros_now(self):
            return self.get_clock().now().nanoseconds / 1e9

        def resolve(self, frame, stamp):
            transform = self.tf.lookup_transform(self.map.state.frame_id, frame, Time.from_msg(stamp)).transform
            return vector(transform.translation), quaternion(transform.rotation)

        def receive(self, message):
            self.received += 1
            markers = message.markers if args.input_type == "MarkerArray" else [message]
            points = sum(len(marker.points) for marker in markers)
            if len(markers) > 1024 or points > self.map.max_input_points:
                self.rejected += 1
                self.warn("Map input exceeds the bounded marker/point capacity")
                return
            received_at = self.ros_now()
            if not self.deferred and self.apply(markers, received_at):
                return
            if len(self.deferred) >= 64 or self.deferred_points + points > self.map.max_input_points:
                self.rejected += 1
                self.warn("Map TF-wait queue is full; request a source snapshot after TF recovers")
                return
            self.deferred.append((markers, received_at, time.monotonic(), points))
            self.deferred_points += points

        def apply(self, markers, received_at):
            try:
                transforms = {}
                def resolve(frame, stamp):
                    key = (frame, stamp.sec, stamp.nanosec)
                    if key not in transforms:
                        transforms[key] = self.resolve(frame, stamp)
                    return transforms[key]
                # Preflight transforms before decoding a potentially large array.
                # A missing late marker must not rebuild earlier pages every tick.
                for marker in markers:
                    frame = marker.header.frame_id.strip().lstrip("/")
                    if marker.action == 0 and frame and frame != self.map.state.frame_id:
                        resolve(frame, marker.header.stamp)
                updates = self.map.apply(markers, now=received_at, resolve_transform=resolve)
                for update in updates:
                    self.scheduler.enqueue(update)
            except TransformException:
                return False
            except (ValueError, RuntimeError, OverflowError) as exc:
                self.rejected += 1
                self.warn(f"Map update rejected, last valid map retained: {exc}")
            return True

        def repair(self, message):
            if len(message.data) > 256 * 1024:
                return
            try:
                request = json.loads(message.data)
            except (ValueError, TypeError):
                return
            now = time.monotonic()
            for update in self.map.state.repair_updates(request):
                # Coalesce simultaneous headsets without permanently suppressing repair.
                previous = self.last_repair.get(update.chunk_id)
                token = (update.epoch, update.revision)
                sent = self.last_sent.get(update.chunk_id)
                if sent and sent[:2] == token and now - sent[2] < 2:
                    continue
                if previous and previous[:2] == token and now - previous[2] < 1:
                    continue
                self.last_repair[update.chunk_id] = (*token, now)
                if self.scheduler.enqueue(update):
                    self.repairs += 1

        def publish_pending(self):
            # Preserve source order while waiting for TF; never apply an older
            # deferred section over a newer replacement or DELETEALL.
            for _ in range(4):
                if not self.deferred:
                    break
                markers, received_at, queued_at, points = self.deferred[0]
                if time.monotonic() - queued_at > 5:
                    self.rejected += 1
                    self.warn("Map transform unavailable for 5s; update rejected, source replay required")
                elif not self.apply(markers, received_at):
                    break
                self.deferred.popleft()
                self.deferred_points -= points
            for update in self.map.expire(self.ros_now()):
                self.scheduler.enqueue(update)
            if not self.publisher.get_subscription_count():
                return
            # Bounded turn: discovery, repair and TF callbacks cannot be starved.
            for _ in range(4):
                update = self.scheduler.pop_ready(time.monotonic())
                if update is None:
                    break
                self.publisher.publish(UInt8MultiArray(data=array("B", update.payload)))
                self.last_sent[update.chunk_id] = (update.epoch, update.revision, time.monotonic())
                self.sent += 1
                self.sent_bytes += len(update.payload)

        def publish_manifest(self):
            self.manifest.publish(String(data=json.dumps(self.map.state.manifest(), separators=(",", ":"))))
            self.status.publish(String(data=json.dumps({
                "received": self.received, "rejected": self.rejected,
                "tf_wait_messages": len(self.deferred), "tf_wait_points": self.deferred_points,
                "unchanged_pages": self.map.state.unchanged_pages,
                "suppressed_bytes": self.map.state.suppressed_bytes,
                "cached_bytes": self.map.state.cached_bytes,
                "pending_pages": self.scheduler.pending_count,
                "pending_bytes": self.scheduler.pending_bytes,
                "sent_pages": self.sent, "sent_bytes": self.sent_bytes,
                "repair_pages": self.repairs}, separators=(",", ":"))))

        def warn(self, text):
            now = time.monotonic()
            if now - self.last_warning >= 2:
                self.get_logger().warning(text)
                self.last_warning = now

    return MeshAdapter()


def main(argv=None):
    import rclpy
    args, ros_args = arguments(argv)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = create_node(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
