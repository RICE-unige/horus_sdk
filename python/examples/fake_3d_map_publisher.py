#!/usr/bin/env python3
"""Publish a synthetic 3D map as sensor_msgs/PointCloud2.

Usage examples:
  python3 python/examples/fake_3d_map_publisher.py
  python3 python/examples/fake_3d_map_publisher.py --topic /map_3d --frame map
  python3 python/examples/fake_3d_map_publisher.py --extent-x 12 --extent-y 10 --resolution 0.12
"""

import argparse
import random
import struct
import sys
from typing import List, Tuple

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    sys.exit(1)


class Fake3DMapPublisher(Node):
    def __init__(
        self,
        topic: str,
        frame_id: str,
        rate_hz: float,
        extent_x: float,
        extent_y: float,
        max_height: float,
        resolution: float,
        obstacle_count: int,
        seed: int,
    ) -> None:
        super().__init__("horus_fake_3d_map_publisher")
        self.topic = topic
        self.frame_id = frame_id
        self.rate_hz = max(0.1, float(rate_hz))
        self.extent_x = max(1.0, float(extent_x))
        self.extent_y = max(1.0, float(extent_y))
        self.max_height = max(0.5, float(max_height))
        self.resolution = max(0.05, float(resolution))
        self.obstacle_count = max(0, int(obstacle_count))
        self.random = random.Random(seed)

        self._points = self._build_points()
        self._point_struct = struct.Struct("<ffff")
        self._fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(PointCloud2, self.topic, qos)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._publish)
        self._publish()

        self.get_logger().info(
            f"Publishing fake 3D map on {self.topic} ({len(self._points)} points, "
            f"extent={self.extent_x:.1f}m x {self.extent_y:.1f}m, max_height={self.max_height:.1f}m, "
            f"resolution={self.resolution:.2f}m, rate={self.rate_hz:.2f}Hz)"
        )

    def _build_points(self) -> List[Tuple[float, float, float, int]]:
        points: List[Tuple[float, float, float, int]] = []

        half_x = self.extent_x * 0.5
        half_y = self.extent_y * 0.5
        step = self.resolution

        # Floor layer.
        x = -half_x
        while x <= half_x:
            y = -half_y
            while y <= half_y:
                points.append((x, y, 0.0, self._rgb_u32(90, 130, 90)))
                y += step
            x += step

        # Boundary walls.
        z = 0.0
        while z <= self.max_height:
            # North/South
            x = -half_x
            while x <= half_x:
                points.append((x, -half_y, z, self._rgb_u32(150, 170, 220)))
                points.append((x, half_y, z, self._rgb_u32(150, 170, 220)))
                x += step
            # East/West
            y = -half_y
            while y <= half_y:
                points.append((-half_x, y, z, self._rgb_u32(150, 170, 220)))
                points.append((half_x, y, z, self._rgb_u32(150, 170, 220)))
                y += step
            z += step

        # Random box obstacles.
        for _ in range(self.obstacle_count):
            cx = self.random.uniform(-half_x * 0.7, half_x * 0.7)
            cy = self.random.uniform(-half_y * 0.7, half_y * 0.7)
            sx = self.random.uniform(0.4, 1.2)
            sy = self.random.uniform(0.4, 1.2)
            sz = self.random.uniform(0.3, min(self.max_height * 0.9, 2.0))
            self._append_box(points, cx, cy, sx, sy, sz, step)

        return points

    def _append_box(
        self,
        points: List[Tuple[float, float, float, int]],
        cx: float,
        cy: float,
        sx: float,
        sy: float,
        sz: float,
        step: float,
    ) -> None:
        min_x = cx - sx * 0.5
        max_x = cx + sx * 0.5
        min_y = cy - sy * 0.5
        max_y = cy + sy * 0.5

        # Top face.
        x = min_x
        while x <= max_x:
            y = min_y
            while y <= max_y:
                points.append((x, y, sz, self._rgb_u32(210, 120, 90)))
                y += step
            x += step

        # Side edges.
        z = 0.0
        while z <= sz:
            points.append((min_x, min_y, z, self._rgb_u32(200, 110, 80)))
            points.append((min_x, max_y, z, self._rgb_u32(200, 110, 80)))
            points.append((max_x, min_y, z, self._rgb_u32(200, 110, 80)))
            points.append((max_x, max_y, z, self._rgb_u32(200, 110, 80)))
            z += step

    @staticmethod
    def _rgb_u32(r: int, g: int, b: int) -> int:
        return ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF)

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        msg = PointCloud2()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(self._points)
        msg.fields = self._fields
        msg.is_bigendian = False
        msg.point_step = self._point_struct.size
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        data = bytearray(msg.row_step)
        pack_into = self._point_struct.pack_into
        for i, (x, y, z, rgb_u32) in enumerate(self._points):
            rgb_float = struct.unpack("<f", struct.pack("<I", rgb_u32))[0]
            pack_into(data, i * msg.point_step, float(x), float(y), float(z), rgb_float)

        msg.data = data
        self.publisher.publish(msg)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish synthetic 3D map PointCloud2 data for Horus testing.")
    parser.add_argument("--topic", default="/map_3d", help="PointCloud2 topic name (default: /map_3d).")
    parser.add_argument("--frame", default="map", help="Frame id for cloud points (default: map).")
    parser.add_argument("--rate", type=float, default=1.0, help="Publish rate in Hz (default: 1.0).")
    parser.add_argument("--extent-x", type=float, default=10.0, help="Map width in meters (default: 10.0).")
    parser.add_argument("--extent-y", type=float, default=8.0, help="Map depth in meters (default: 8.0).")
    parser.add_argument("--max-height", type=float, default=2.0, help="Wall/obstacle max height in meters (default: 2.0).")
    parser.add_argument("--resolution", type=float, default=0.15, help="Sampling resolution in meters (default: 0.15).")
    parser.add_argument("--obstacle-count", type=int, default=12, help="Number of random box obstacles (default: 12).")
    parser.add_argument("--seed", type=int, default=42, help="Random seed (default: 42).")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    rclpy.init()
    node = Fake3DMapPublisher(
        topic=args.topic,
        frame_id=args.frame,
        rate_hz=args.rate,
        extent_x=args.extent_x,
        extent_y=args.extent_y,
        max_height=args.max_height,
        resolution=args.resolution,
        obstacle_count=args.obstacle_count,
        seed=args.seed,
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
