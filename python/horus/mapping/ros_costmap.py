"""Normalize full/raw Nav2 costmaps and incremental updates for HORUS."""

import argparse
from array import array
import math
import time

from .costmap import CostmapState, GridGeometry


def stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def apply_full(state, msg, raw):
    meta = msg.metadata if raw else msg.info
    p, q = meta.origin.position, meta.origin.orientation
    geometry = GridGeometry(
        msg.header.frame_id, meta.size_x if raw else meta.width,
        meta.size_y if raw else meta.height, meta.resolution,
        (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    return state.full(geometry, msg.data, stamp_ns(msg.header), raw=raw)


def apply_patch(state, msg, raw):
    return state.patch(msg.header.frame_id, msg.x, msg.y,
        msg.size_x if raw else msg.width, msg.size_y if raw else msg.height,
        msg.data, stamp_ns(msg.header), raw=raw)


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", required=True)
    parser.add_argument("--input-format", choices=("occupancy", "raw"), default="occupancy")
    parser.add_argument("--updates-topic")
    parser.add_argument("--snapshot-service", help="Nav2 GetCostmap service; required for incremental input")
    parser.add_argument("--output-topic", default="/horus/costmap")
    parser.add_argument("--publish-hz", type=float, default=5)
    parser.add_argument("--resync-seconds", type=float, default=2)
    parser.add_argument("--volatile-full", action="store_true",
                        help="Use only with a full-map publisher that is not transient-local")
    args, ros_args = parser.parse_known_args(argv)
    if ros_args and ros_args[0] != "--ros-args":
        parser.error("unrecognized arguments: " + " ".join(ros_args))
    topics = [args.input_topic, args.output_topic] + ([args.updates_topic] if args.updates_topic else [])
    if len(set(topics)) != len(topics) or any(not t.startswith("/") or t.endswith("/") for t in topics):
        parser.error("use distinct absolute topics without trailing slashes")
    if args.updates_topic and not args.snapshot_service:
        parser.error("incremental input needs --snapshot-service to recover missing/late-join patches")
    if args.snapshot_service and not args.snapshot_service.startswith("/"):
        parser.error("snapshot service must be absolute")
    if not math.isfinite(args.publish_hz) or not 0 < args.publish_hz <= 30:
        parser.error("publish-hz must be in (0, 30]")
    if not math.isfinite(args.resync_seconds) or args.resync_seconds < 0.2:
        parser.error("resync-seconds must be finite and at least 0.2")
    return args, ros_args


def create_node(args, *, context=None):
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.event_handler import SubscriptionEventCallbacks
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from map_msgs.msg import OccupancyGridUpdate
    from nav2_msgs.msg import Costmap, CostmapUpdate
    from nav2_msgs.srv import GetCostmap

    class CostmapAdapter(Node):
        def __init__(self):
            super().__init__("horus_costmap_adapter", context=context)
            self.state = CostmapState()
            self.ready = not bool(args.snapshot_service)
            self.dirty = False
            self.future = None
            self.requested_at = self.next_resync = 0.0
            self.last_publish = self.last_warning = -math.inf
            self.journal = []
            self.journal_bytes = 0
            self.journal_lost = False
            self.raw = args.input_format == "raw"
            self.publisher = self.create_publisher(OccupancyGrid, args.output_topic,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))
            events = SubscriptionEventCallbacks(message_lost=lambda _: self.invalidate("DDS lost costmap data"))
            full_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE if args.volatile_full else DurabilityPolicy.TRANSIENT_LOCAL)
            self.full_sub = self.create_subscription(Costmap if self.raw else OccupancyGrid,
                args.input_topic, lambda msg: self.receive(msg, True), full_qos, event_callbacks=events)
            if args.updates_topic:
                self.patch_sub = self.create_subscription(CostmapUpdate if self.raw else OccupancyGridUpdate,
                    args.updates_topic, lambda msg: self.receive(msg, False),
                    QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE),
                    event_callbacks=events)
            self.client = self.create_client(GetCostmap, args.snapshot_service) if args.snapshot_service else None
            self.timer = self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def invalidate(self, reason):
            self.ready = False
            self.dirty = False
            self.next_resync = 0
            if self.future is not None:
                self.journal_lost = True
            if time.monotonic() - self.last_warning > 2:
                self.get_logger().warning(reason + "; waiting for a complete snapshot")
                self.last_warning = time.monotonic()

        def receive(self, msg, full):
            if self.future is not None:
                self.journal_bytes += len(msg.data)
                if len(self.journal) >= 64 or self.journal_bytes > 8 * 1024 * 1024:
                    self.journal_lost = True
                    self.journal.clear()
                    self.invalidate("Costmap recovery journal exceeded its bounded capacity")
                elif not self.journal_lost:
                    self.journal.append((msg, full))
            if not self.ready:
                # A cached full map plus only the latest patch is not a valid late-join snapshot.
                if self.client is not None or not full:
                    return
            try:
                changed = apply_full(self.state, msg, self.raw) if full else apply_patch(self.state, msg, self.raw)
                self.dirty |= changed
                if full:
                    self.ready = True
            except (ValueError, TypeError, OverflowError) as exc:
                self.invalidate(str(exc))

        def tick(self):
            now = time.monotonic()
            if self.future is not None:
                if self.future.done():
                    try:
                        if self.journal_lost:
                            raise ValueError("Costmap recovery lost an update")
                        fresh = CostmapState()
                        apply_full(fresh, self.future.result().map, True)
                        for msg, full in self.journal:
                            if stamp_ns(msg.header) < fresh.stamp_ns:
                                continue
                            if full:
                                apply_full(fresh, msg, self.raw)
                            else:
                                apply_patch(fresh, msg, self.raw)
                        self.state, self.ready, self.dirty = fresh, True, True
                        self.next_resync = now + args.resync_seconds
                    except (ValueError, TypeError, OverflowError, AttributeError, RuntimeError) as exc:
                        self.invalidate(str(exc))
                        self.next_resync = now + 0.5
                    finally:
                        self.future = None
                        self.journal.clear()
                        self.journal_bytes = 0
                elif now - self.requested_at > 2:
                    self.client.remove_pending_request(self.future)
                    self.future.cancel()
                    self.future = None
                    self.journal.clear()
                    self.invalidate("Costmap snapshot service timed out")
                    self.next_resync = now + 0.5
            if self.client is not None and self.future is None and now >= self.next_resync:
                if self.client.service_is_ready():
                    self.journal, self.journal_bytes, self.journal_lost = [], 0, False
                    self.future = self.client.call_async(GetCostmap.Request())
                    self.requested_at = now
                elif not self.ready:
                    self.invalidate("Costmap snapshot service is unavailable: " + args.snapshot_service)
            if not self.ready or not self.dirty or now - self.last_publish < 1 / args.publish_hz:
                return
            state, msg = self.state, OccupancyGrid()
            geometry = state.geometry
            if geometry is None:
                return
            msg.header.frame_id = geometry.frame
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(state.stamp_ns, 1_000_000_000)
            msg.info.width, msg.info.height = geometry.width, geometry.height
            msg.info.resolution = geometry.resolution
            p, q = msg.info.origin.position, msg.info.origin.orientation
            p.x, p.y, p.z, q.x, q.y, q.z, q.w = geometry.origin
            msg.data = array("b", state.data.reshape(-1).tobytes())
            self.publisher.publish(msg)
            self.last_publish, self.dirty = now, False

    return CostmapAdapter()


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
