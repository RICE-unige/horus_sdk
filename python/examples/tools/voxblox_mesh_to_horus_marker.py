#!/usr/bin/env python3
"""Relay Voxblox mesh output to the HORUS mesh-map Marker topic.

Primary use with the ETHZ ASL Cow & Lady dataset:

1. Run Voxblox on the ROS 1 bag so it publishes /voxblox_node/mesh.
2. Run this relay in the same ROS 1 environment:

       python3 voxblox_mesh_to_horus_marker.py --ros-api ros1

3. Bridge /map_3d_mesh to ROS 2 with ros1_bridge, then run:

       PYTHONPATH=python:$PYTHONPATH python3 python/examples/mesh_map_registration.py

The relay also works in ROS 2 when voxblox_msgs is available there.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Sequence


UINT16_SCALE = 1.0 / 65535.0


@dataclass
class MeshBlockData:
    points: list[tuple[float, float, float]]
    colors: list[tuple[float, float, float, float]]


class MeshAccumulator:
    def __init__(
        self,
        *,
        fallback_color: tuple[float, float, float, float],
        max_triangles: int,
        accumulate_blocks: bool,
    ) -> None:
        self.fallback_color = fallback_color
        self.max_triangles = max(0, int(max_triangles))
        self.accumulate_blocks = accumulate_blocks
        self.blocks: dict[tuple[int, int, int], MeshBlockData] = {}
        self.last_frame_id = "map"

    def update_from_voxblox_mesh(self, msg: object) -> tuple[int, int]:
        block_edge = max(1e-9, float(getattr(msg, "block_edge_length", 0.0) or 0.0))
        header = getattr(msg, "header", None)
        frame_id = str(getattr(header, "frame_id", "") or "").strip()
        if frame_id:
            self.last_frame_id = frame_id
        if not self.accumulate_blocks:
            self.blocks.clear()

        block_count = 0
        triangle_count = 0
        for block in getattr(msg, "mesh_blocks", []) or []:
            key = tuple(int(v) for v in getattr(block, "index", (0, 0, 0))[:3])
            data = self._decode_voxblox_block(block, block_edge)
            if not data.points:
                self.blocks.pop(key, None)
                continue
            self.blocks[key] = data
            block_count += 1
            triangle_count += len(data.points) // 3
        return block_count, triangle_count

    def replace_from_marker(self, marker: object) -> tuple[int, int]:
        self.blocks.clear()
        header = getattr(marker, "header", None)
        frame_id = str(getattr(header, "frame_id", "") or "").strip()
        if frame_id:
            self.last_frame_id = frame_id
        points = [
            (float(p.x), float(p.y), float(p.z))
            for p in list(getattr(marker, "points", []) or [])
        ]
        points = points[: len(points) - (len(points) % 3)]
        colors = self._marker_colors(marker, len(points))
        if points:
            self.blocks[(0, 0, 0)] = MeshBlockData(points=points, colors=colors)
        return 1 if points else 0, len(points) // 3

    def replace_from_marker_array(self, marker_array: object) -> tuple[int, int]:
        self.blocks.clear()
        marker_count = 0
        triangle_count = 0
        for idx, marker in enumerate(getattr(marker_array, "markers", []) or []):
            points = [
                (float(p.x), float(p.y), float(p.z))
                for p in list(getattr(marker, "points", []) or [])
            ]
            points = points[: len(points) - (len(points) % 3)]
            if not points:
                continue
            header = getattr(marker, "header", None)
            frame_id = str(getattr(header, "frame_id", "") or "").strip()
            if frame_id:
                self.last_frame_id = frame_id
            colors = self._marker_colors(marker, len(points))
            self.blocks[(idx, 0, 0)] = MeshBlockData(points=points, colors=colors)
            marker_count += 1
            triangle_count += len(points) // 3
        return marker_count, triangle_count

    def snapshot(self) -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float, float]]]:
        points: list[tuple[float, float, float]] = []
        colors: list[tuple[float, float, float, float]] = []
        max_points = self.max_triangles * 3 if self.max_triangles > 0 else 0
        for key in sorted(self.blocks):
            block = self.blocks[key]
            remaining = max_points - len(points) if max_points > 0 else len(block.points)
            if max_points > 0 and remaining <= 0:
                break
            count = min(len(block.points), remaining) if max_points > 0 else len(block.points)
            count -= count % 3
            if count <= 0:
                continue
            points.extend(block.points[:count])
            colors.extend(block.colors[:count] if len(block.colors) >= count else [self.fallback_color] * count)
        return points, colors

    def _decode_voxblox_block(self, block: object, block_edge: float) -> MeshBlockData:
        xs = list(getattr(block, "x", []) or [])
        ys = list(getattr(block, "y", []) or [])
        zs = list(getattr(block, "z", []) or [])
        count = min(len(xs), len(ys), len(zs))
        count -= count % 3
        if count <= 0:
            return MeshBlockData(points=[], colors=[])

        index = tuple(int(v) for v in getattr(block, "index", (0, 0, 0))[:3])
        origin = (index[0] * block_edge, index[1] * block_edge, index[2] * block_edge)
        scale = block_edge * UINT16_SCALE
        points = [
            (
                origin[0] + float(xs[i]) * scale,
                origin[1] + float(ys[i]) * scale,
                origin[2] + float(zs[i]) * scale,
            )
            for i in range(count)
        ]

        rs = list(getattr(block, "r", []) or [])
        gs = list(getattr(block, "g", []) or [])
        bs = list(getattr(block, "b", []) or [])
        if min(len(rs), len(gs), len(bs)) >= count:
            colors = [
                (
                    max(0.0, min(1.0, float(rs[i]) / 255.0)),
                    max(0.0, min(1.0, float(gs[i]) / 255.0)),
                    max(0.0, min(1.0, float(bs[i]) / 255.0)),
                    self.fallback_color[3],
                )
                for i in range(count)
            ]
        else:
            colors = [self.fallback_color] * count
        return MeshBlockData(points=points, colors=colors)

    def _marker_colors(self, marker: object, count: int) -> list[tuple[float, float, float, float]]:
        raw_colors = list(getattr(marker, "colors", []) or [])
        if len(raw_colors) >= count:
            return [
                (
                    max(0.0, min(1.0, float(c.r))),
                    max(0.0, min(1.0, float(c.g))),
                    max(0.0, min(1.0, float(c.b))),
                    max(0.0, min(1.0, float(getattr(c, "a", self.fallback_color[3])))),
                )
                for c in raw_colors[:count]
            ]
        marker_color = getattr(marker, "color", None)
        if marker_color is None:
            return [self.fallback_color] * count
        color = (
            max(0.0, min(1.0, float(marker_color.r))),
            max(0.0, min(1.0, float(marker_color.g))),
            max(0.0, min(1.0, float(marker_color.b))),
            max(0.0, min(1.0, float(getattr(marker_color, "a", self.fallback_color[3])))),
        )
        return [color] * count


def parse_color(value: str) -> tuple[float, float, float, float]:
    parts = [p.strip() for p in str(value).split(",")]
    if len(parts) not in (3, 4):
        raise argparse.ArgumentTypeError("color must be r,g,b or r,g,b,a")
    try:
        floats = [float(p) for p in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("color components must be numbers") from exc
    if any(v > 1.0 for v in floats):
        floats = [v / 255.0 for v in floats]
    if len(floats) == 3:
        floats.append(1.0)
    return tuple(max(0.0, min(1.0, v)) for v in floats)  # type: ignore[return-value]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros-api", choices=("auto", "ros1", "ros2"), default="auto")
    parser.add_argument(
        "--input-type",
        choices=("voxblox_mesh", "marker", "marker_array"),
        default="voxblox_mesh",
        help="Input message kind. Default: voxblox_mesh.",
    )
    parser.add_argument("--input-topic", default="/voxblox_node/mesh")
    parser.add_argument("--output-topic", default="/map_3d_mesh")
    parser.add_argument("--output-frame", default="", help="Override output frame. Defaults to input frame.")
    parser.add_argument("--chunk-max-triangles", type=int, default=5000)
    parser.add_argument("--max-triangles", type=int, default=220000, help="0 means no cap.")
    parser.add_argument("--republish-interval", type=float, default=2.0)
    parser.add_argument("--fallback-color", type=parse_color, default=parse_color("178,184,198,255"))
    parser.add_argument("--no-accumulate-blocks", action="store_true")
    parser.add_argument("--no-clear-on-start", action="store_true")
    parser.add_argument("--no-latch", action="store_true", help="ROS 1 only: disable latched output.")
    parser.add_argument("--transient-local", action="store_true", help="ROS 2 only: use transient-local output QoS.")
    return parser.parse_args()


def chunk_ranges(point_count: int, chunk_max_triangles: int) -> Iterable[tuple[int, int, int]]:
    points_per_chunk = max(1, int(chunk_max_triangles)) * 3
    chunk_id = 0
    for start in range(0, point_count, points_per_chunk):
        end = min(point_count, start + points_per_chunk)
        end -= (end - start) % 3
        if end > start:
            yield chunk_id, start, end
            chunk_id += 1


def should_log(now: float, last_log: float) -> bool:
    return now - last_log >= 2.0


def select_ros_api(requested: str) -> str:
    if requested != "auto":
        return requested
    try:
        import rclpy  # noqa: F401

        return "ros2"
    except Exception:
        return "ros1"


def run_ros2(args: argparse.Namespace) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray

    if args.input_type == "voxblox_mesh":
        try:
            from voxblox_msgs.msg import Mesh as VoxbloxMesh
        except Exception as exc:
            raise SystemExit(
                "voxblox_msgs is not available in this ROS 2 environment. "
                "Run the relay in ROS 1 and bridge the standard /map_3d_mesh Marker topic, "
                "or install matching voxblox_msgs for ROS 2."
            ) from exc
        input_msg_type = VoxbloxMesh
    elif args.input_type == "marker_array":
        input_msg_type = MarkerArray
    else:
        input_msg_type = Marker

    class Relay(Node):
        def __init__(self) -> None:
            super().__init__("voxblox_mesh_to_horus_marker")
            self.accumulator = MeshAccumulator(
                fallback_color=args.fallback_color,
                max_triangles=args.max_triangles,
                accumulate_blocks=not args.no_accumulate_blocks,
            )
            self.last_marker_count = 0
            self.last_publish_time = 0.0
            self.last_log_time = 0.0
            qos = QoSProfile(depth=64)
            qos.reliability = ReliabilityPolicy.RELIABLE
            if args.transient_local:
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.pub = self.create_publisher(Marker, args.output_topic, qos)
            self.sub = self.create_subscription(input_msg_type, args.input_topic, self.on_input, 10)
            if args.republish_interval > 0:
                self.timer = self.create_timer(max(0.2, args.republish_interval), self.republish)
            else:
                self.timer = None
            if not args.no_clear_on_start:
                self.publish_delete_all()
            self.get_logger().info(
                f"Relaying {args.input_topic} ({args.input_type}) -> {args.output_topic}"
            )

        def on_input(self, msg: object) -> None:
            if args.input_type == "voxblox_mesh":
                source_count, source_triangles = self.accumulator.update_from_voxblox_mesh(msg)
            elif args.input_type == "marker_array":
                source_count, source_triangles = self.accumulator.replace_from_marker_array(msg)
            else:
                source_count, source_triangles = self.accumulator.replace_from_marker(msg)
            self.publish_snapshot(getattr(getattr(msg, "header", None), "stamp", self.get_clock().now().to_msg()))
            now = time.monotonic()
            if should_log(now, self.last_log_time):
                points, _ = self.accumulator.snapshot()
                self.get_logger().info(
                    f"source={source_count} units/{source_triangles} tris, "
                    f"published={len(points) // 3} tris in {self.last_marker_count} chunks"
                )
                self.last_log_time = now

        def republish(self) -> None:
            if self.last_marker_count > 0:
                self.publish_snapshot(self.get_clock().now().to_msg())

        def make_marker(self, marker_id: int, points: Sequence[tuple[float, float, float]], colors: Sequence[tuple[float, float, float, float]], stamp: object) -> Marker:
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = args.output_frame.strip() or self.accumulator.last_frame_id or "map"
            marker.ns = "voxblox_chunks"
            marker.id = int(marker_id)
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.color = ColorRGBA(
                r=args.fallback_color[0],
                g=args.fallback_color[1],
                b=args.fallback_color[2],
                a=args.fallback_color[3],
            )
            marker.points = [Point(x=x, y=y, z=z) for x, y, z in points]
            marker.colors = [ColorRGBA(r=r, g=g, b=b, a=a) for r, g, b, a in colors]
            return marker

        def publish_delete_all(self) -> None:
            marker = Marker()
            marker.header.frame_id = args.output_frame.strip() or self.accumulator.last_frame_id or "map"
            marker.ns = "voxblox_chunks"
            marker.id = 0
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.DELETEALL
            self.pub.publish(marker)
            self.last_marker_count = 0

        def publish_snapshot(self, stamp: object) -> None:
            points, colors = self.accumulator.snapshot()
            marker_count = 0
            for marker_id, start, end in chunk_ranges(len(points), args.chunk_max_triangles):
                self.pub.publish(self.make_marker(marker_id, points[start:end], colors[start:end], stamp))
                marker_count += 1
            for marker_id in range(marker_count, self.last_marker_count):
                marker = Marker()
                marker.header.stamp = stamp
                marker.header.frame_id = args.output_frame.strip() or self.accumulator.last_frame_id or "map"
                marker.ns = "voxblox_chunks"
                marker.id = int(marker_id)
                marker.type = Marker.TRIANGLE_LIST
                marker.action = Marker.DELETE
                self.pub.publish(marker)
            self.last_marker_count = marker_count
            self.last_publish_time = time.monotonic()

    rclpy.init()
    node = Relay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_ros1(args: argparse.Namespace) -> None:
    import rospy
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray

    if args.input_type == "voxblox_mesh":
        try:
            from voxblox_msgs.msg import Mesh as VoxbloxMesh
        except Exception as exc:
            raise SystemExit("voxblox_msgs is not available in this ROS 1 environment.") from exc
        input_msg_type = VoxbloxMesh
    elif args.input_type == "marker_array":
        input_msg_type = MarkerArray
    else:
        input_msg_type = Marker

    rospy.init_node("voxblox_mesh_to_horus_marker", anonymous=False)
    accumulator = MeshAccumulator(
        fallback_color=args.fallback_color,
        max_triangles=args.max_triangles,
        accumulate_blocks=not args.no_accumulate_blocks,
    )
    pub = rospy.Publisher(args.output_topic, Marker, queue_size=64, latch=not args.no_latch)
    state = {"last_marker_count": 0, "last_log_time": 0.0}

    def make_marker(marker_id: int, points: Sequence[tuple[float, float, float]], colors: Sequence[tuple[float, float, float, float]], stamp: object) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = args.output_frame.strip() or accumulator.last_frame_id or "map"
        marker.ns = "voxblox_chunks"
        marker.id = int(marker_id)
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color = ColorRGBA(
            r=args.fallback_color[0],
            g=args.fallback_color[1],
            b=args.fallback_color[2],
            a=args.fallback_color[3],
        )
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in points]
        marker.colors = [ColorRGBA(r=r, g=g, b=b, a=a) for r, g, b, a in colors]
        return marker

    def publish_delete_all() -> None:
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = args.output_frame.strip() or accumulator.last_frame_id or "map"
        marker.ns = "voxblox_chunks"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.DELETEALL
        pub.publish(marker)
        state["last_marker_count"] = 0

    def publish_snapshot(stamp: object | None = None) -> None:
        stamp = stamp or rospy.Time.now()
        points, colors = accumulator.snapshot()
        marker_count = 0
        for marker_id, start, end in chunk_ranges(len(points), args.chunk_max_triangles):
            pub.publish(make_marker(marker_id, points[start:end], colors[start:end], stamp))
            marker_count += 1
        for marker_id in range(marker_count, int(state["last_marker_count"])):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = args.output_frame.strip() or accumulator.last_frame_id or "map"
            marker.ns = "voxblox_chunks"
            marker.id = int(marker_id)
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.DELETE
            pub.publish(marker)
        state["last_marker_count"] = marker_count

    def on_input(msg: object) -> None:
        if args.input_type == "voxblox_mesh":
            source_count, source_triangles = accumulator.update_from_voxblox_mesh(msg)
        elif args.input_type == "marker_array":
            source_count, source_triangles = accumulator.replace_from_marker_array(msg)
        else:
            source_count, source_triangles = accumulator.replace_from_marker(msg)
        stamp = getattr(getattr(msg, "header", None), "stamp", rospy.Time.now())
        publish_snapshot(stamp)
        now = time.monotonic()
        if should_log(now, float(state["last_log_time"])):
            points, _ = accumulator.snapshot()
            rospy.loginfo(
                "source=%s units/%s tris, published=%s tris in %s chunks",
                source_count,
                source_triangles,
                len(points) // 3,
                state["last_marker_count"],
            )
            state["last_log_time"] = now

    if not args.no_clear_on_start:
        publish_delete_all()
    sub = rospy.Subscriber(args.input_topic, input_msg_type, on_input, queue_size=4)
    rospy.loginfo("Relaying %s (%s) -> %s", args.input_topic, args.input_type, args.output_topic)
    rate = rospy.Rate(1.0 / max(0.2, args.republish_interval)) if args.republish_interval > 0 else None
    while not rospy.is_shutdown():
        if rate is None:
            rospy.spin()
            break
        if int(state["last_marker_count"]) > 0:
            publish_snapshot(rospy.Time.now())
        rate.sleep()
    sub.unregister()


def main() -> None:
    args = parse_args()
    ros_api = select_ros_api(args.ros_api)
    if not math.isfinite(args.republish_interval) or args.republish_interval < 0:
        raise SystemExit("--republish-interval must be >= 0")
    if ros_api == "ros2":
        run_ros2(args)
    else:
        run_ros1(args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
