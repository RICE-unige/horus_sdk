#!/usr/bin/env python3
"""Publish a compact, colorful synthetic house map as PointCloud2.

This profile is intended for Quest 3 minimap tests where you want:
- a smaller indoor scene,
- dense local structure,
- vivid colors,
- a hard point cap (default: 100000).

Usage:
  python3 python/examples/fake_3d_map_publisher_compact_house.py
  python3 python/examples/fake_3d_map_publisher_compact_house.py --max-points 100000 --publish-mode on_change
"""

import argparse
import math
import random
import struct
import sys
from typing import List, Optional, Tuple

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    sys.exit(1)


PointRecord = Tuple[float, float, float, int, str]


class CompactHouse3DMapPublisher(Node):
    def __init__(
        self,
        topic: str,
        frame_id: str,
        rate_hz: float,
        extent_x: float,
        extent_y: float,
        max_height: float,
        resolution: float,
        density_multiplier: float,
        seed: int,
        max_points: int,
        include_ceiling: bool,
        publish_mode: str,
        on_change_republish_interval: float,
    ) -> None:
        super().__init__("horus_fake_3d_map_publisher_compact_house")
        self.topic = topic
        self.frame_id = frame_id
        self.rate_hz = max(0.1, float(rate_hz))
        self.extent_x = max(4.0, float(extent_x))
        self.extent_y = max(4.0, float(extent_y))
        self.max_height = max(1.8, float(max_height))
        self.base_resolution = max(0.05, float(resolution))
        self.density_multiplier = max(1.0, float(density_multiplier))
        self.step = max(0.03, self.base_resolution / math.sqrt(self.density_multiplier))
        self.max_points = max(5000, int(max_points))
        self.include_ceiling = bool(include_ceiling)
        self.publish_mode = publish_mode.strip().lower()
        self.on_change_republish_interval = max(0.0, float(on_change_republish_interval))
        self.random = random.Random(seed)
        self.seed = int(seed)

        self._point_struct = struct.Struct("<ffff")
        self._fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        raw_points = self._build_scene_points()
        self._raw_point_count = len(raw_points)
        self._records = self._limit_points(raw_points)
        self._points = [(x, y, z, rgb) for x, y, z, rgb, _ in self._records]

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(PointCloud2, self.topic, qos)

        self.timer = None
        self.change_timer = None
        self._last_subscriber_count = -1
        self._last_publish_time = 0.0

        self._publish()
        if self.publish_mode == "continuous":
            self.timer = self.create_timer(1.0 / self.rate_hz, self._publish)
        else:
            self.change_timer = self.create_timer(0.5, self._on_change_tick)

        self.get_logger().info(
            "Publishing compact colorful house map on "
            f"{self.topic} ({len(self._points)} points published, raw={self._raw_point_count}, "
            f"extent={self.extent_x:.1f}x{self.extent_y:.1f}m, "
            f"height={self.max_height:.1f}m, step={self.step:.3f}m, "
            f"max_points={self.max_points}, mode={self.publish_mode}, rate={self.rate_hz:.2f}Hz)"
        )

    @staticmethod
    def _rgb_u32(r: int, g: int, b: int) -> int:
        return ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF)

    def _shade(self, r: int, g: int, b: int, variance: int = 10) -> int:
        return self._rgb_u32(
            max(0, min(255, r + self.random.randint(-variance, variance))),
            max(0, min(255, g + self.random.randint(-variance, variance))),
            max(0, min(255, b + self.random.randint(-variance, variance))),
        )

    def _sample_range(self, a: float, b: float, step: Optional[float] = None) -> List[float]:
        step_size = self.step if step is None else max(0.01, float(step))
        values: List[float] = []
        v = min(a, b)
        end = max(a, b)
        while v <= end + 1e-6:
            values.append(v)
            v += step_size
        return values

    def _add_floor(self, points: List[PointRecord]) -> None:
        half_x = self.extent_x * 0.5
        half_y = self.extent_y * 0.5
        floor_step = self.step * 0.65
        tile = max(0.4, self.step * 7.0)
        palette = [
            (72, 102, 132),
            (76, 120, 98),
            (122, 95, 76),
            (98, 88, 124),
        ]
        for x in self._sample_range(-half_x, half_x, floor_step):
            for y in self._sample_range(-half_y, half_y, floor_step):
                tx = int((x + half_x) / tile)
                ty = int((y + half_y) / tile)
                r, g, b = palette[(tx + ty) % len(palette)]
                points.append((x, y, 0.0, self._shade(r, g, b, variance=7), "floor"))

    def _add_wall_segment(
        self,
        points: List[PointRecord],
        x0: float,
        y0: float,
        x1: float,
        y1: float,
        color: Tuple[int, int, int],
        accent_primary: Tuple[int, int, int],
        accent_secondary: Tuple[int, int, int],
        door_gaps: Optional[List[Tuple[float, float]]] = None,
    ) -> None:
        dx = x1 - x0
        dy = y1 - y0
        length = max(1e-6, math.hypot(dx, dy))
        ux = dx / length
        uy = dy / length

        d = 0.0
        while d <= length + 1e-6:
            in_gap = False
            if door_gaps:
                for g0, g1 in door_gaps:
                    if g0 <= d <= g1:
                        in_gap = True
                        break
            if in_gap:
                d += self.step
                continue

            px = x0 + ux * d
            py = y0 + uy * d
            z = 0.0
            while z <= self.max_height + 1e-6:
                # Wall paint pattern: subtle trim + central mural band.
                wall_color = color
                trim_height = self.max_height * 0.42
                if abs(z - trim_height) <= self.step * 0.8:
                    wall_color = accent_primary
                elif self.max_height * 0.36 <= z <= self.max_height * 0.72:
                    center_span = length * 0.24
                    if abs(d - length * 0.5) <= center_span:
                        pattern = int((d / max(self.step, 0.01)) * 0.5 + (z / max(self.step, 0.01)) * 0.8)
                        wall_color = accent_primary if (pattern % 4) < 2 else accent_secondary
                points.append((px, py, z, self._shade(*wall_color, variance=7), "wall"))
                z += self.step
            d += self.step

    def _add_wall_art(
        self,
        points: List[PointRecord],
        x0: float,
        y0: float,
        x1: float,
        y1: float,
        z0: float,
        z1: float,
        palette: List[Tuple[int, int, int]],
    ) -> None:
        dx = x1 - x0
        dy = y1 - y0
        length = max(1e-6, math.hypot(dx, dy))
        ux = dx / length
        uy = dy / length
        spacing = max(self.step * 0.65, 0.03)
        d = 0.0
        while d <= length + 1e-6:
            px = x0 + ux * d
            py = y0 + uy * d
            z = z0
            while z <= z1 + 1e-6:
                band = int((d / spacing) * 0.7 + (z / spacing) * 1.2)
                c = palette[band % len(palette)]
                points.append((px, py, z, self._shade(*c, variance=5), "feature"))
                z += spacing
            d += spacing

    def _add_layout(self, points: List[PointRecord]) -> None:
        half_x = self.extent_x * 0.5
        half_y = self.extent_y * 0.5

        # Perimeter walls, each with a different color family.
        self._add_wall_segment(
            points, -half_x, -half_y, half_x, -half_y,
            (214, 208, 196), (198, 118, 95), (226, 176, 132))
        self._add_wall_segment(
            points, -half_x, half_y, half_x, half_y,
            (206, 214, 224), (96, 150, 198), (170, 204, 230))
        self._add_wall_segment(
            points, -half_x, -half_y, -half_x, half_y,
            (222, 210, 188), (152, 132, 102), (196, 164, 124))
        self._add_wall_segment(
            points, half_x, -half_y, half_x, half_y,
            (205, 220, 206), (104, 162, 124), (158, 208, 172))

        # Two interior walls with doorway gaps.
        x_len = self.extent_x * 0.72
        gap = max(0.7, self.extent_x * 0.10)
        self._add_wall_segment(
            points,
            -x_len * 0.5,
            -self.extent_y * 0.08,
            x_len * 0.5,
            -self.extent_y * 0.08,
            (214, 208, 222),
            (152, 132, 188),
            (188, 166, 216),
            door_gaps=[(x_len * 0.45 - gap * 0.5, x_len * 0.45 + gap * 0.5)],
        )

        y_len = self.extent_y * 0.66
        self._add_wall_segment(
            points,
            self.extent_x * 0.11,
            -y_len * 0.5,
            self.extent_x * 0.11,
            y_len * 0.5,
            (206, 220, 214),
            (120, 172, 160),
            (166, 202, 188),
            door_gaps=[(y_len * 0.28 - gap * 0.5, y_len * 0.28 + gap * 0.5)],
        )

        # Add a den room (top-right area) with a dedicated doorway.
        room_x0 = self.extent_x * 0.12
        room_x1 = half_x - self.step * 1.2
        room_y0 = self.extent_y * 0.12
        room_y1 = half_y - self.step * 1.2
        door = max(0.75, self.extent_x * 0.11)
        self._add_wall_segment(
            points,
            room_x0,
            room_y0,
            room_x1,
            room_y0,
            (218, 212, 228),
            (168, 132, 194),
            (206, 176, 224),
            door_gaps=[((room_x1 - room_x0) * 0.5 - door * 0.5, (room_x1 - room_x0) * 0.5 + door * 0.5)],
        )
        self._add_wall_segment(
            points,
            room_x0,
            room_y0,
            room_x0,
            room_y1,
            (210, 225, 214),
            (124, 182, 146),
            (176, 214, 184),
            door_gaps=[((room_y1 - room_y0) * 0.72 - door * 0.45, (room_y1 - room_y0) * 0.72 + door * 0.45)],
        )
        self._add_wall_segment(
            points,
            room_x0,
            room_y1,
            room_x1,
            room_y1,
            (226, 216, 198),
            (214, 146, 96),
            (236, 188, 136),
        )

        # Colorful mural in the den room.
        self._add_wall_art(
            points,
            room_x0 + self.step * 2.0,
            room_y1,
            room_x1 - self.step * 2.0,
            room_y1,
            self.max_height * 0.35,
            self.max_height * 0.72,
            [
                (224, 92, 92),
                (236, 168, 72),
                (76, 168, 232),
                (112, 196, 128),
                (192, 126, 222),
            ],
        )

    def _add_box(
        self,
        points: List[PointRecord],
        cx: float,
        cy: float,
        sx: float,
        sy: float,
        sz: float,
        top_color: Tuple[int, int, int],
        side_color: Tuple[int, int, int],
    ) -> None:
        min_x = cx - sx * 0.5
        max_x = cx + sx * 0.5
        min_y = cy - sy * 0.5
        max_y = cy + sy * 0.5

        for x in self._sample_range(min_x, max_x):
            for y in self._sample_range(min_y, max_y):
                points.append((x, y, sz, self._shade(*top_color, variance=8), "feature"))

        for z in self._sample_range(0.0, sz):
            for x in self._sample_range(min_x, max_x):
                points.append((x, min_y, z, self._shade(*side_color, variance=8), "feature"))
                points.append((x, max_y, z, self._shade(*side_color, variance=8), "feature"))
            for y in self._sample_range(min_y, max_y):
                points.append((min_x, y, z, self._shade(*side_color, variance=8), "feature"))
                points.append((max_x, y, z, self._shade(*side_color, variance=8), "feature"))

    def _add_furniture(self, points: List[PointRecord]) -> None:
        half_x = self.extent_x * 0.5
        half_y = self.extent_y * 0.5

        wood_pairs = [
            ((166, 118, 84), (136, 94, 66)),
            ((152, 108, 76), (124, 86, 60)),
            ((174, 132, 96), (142, 104, 74)),
            ((160, 116, 82), (132, 92, 64)),
        ]
        accent_pairs = [
            ((128, 168, 224), (96, 136, 198)),
            ((122, 194, 158), (93, 167, 132)),
            ((214, 181, 115), (188, 156, 87)),
        ]

        # Tables / shelves
        for idx in range(14):
            cx = self.random.uniform(-half_x * 0.72, half_x * 0.72)
            cy = self.random.uniform(-half_y * 0.72, half_y * 0.72)
            sx = self.random.uniform(0.5, 1.0)
            sy = self.random.uniform(0.4, 0.9)
            sz = self.random.uniform(0.6, 1.6)
            # Mostly wood furniture, with a few colorful accent pieces.
            if idx < 10:
                top_color, side_color = wood_pairs[idx % len(wood_pairs)]
            else:
                top_color, side_color = accent_pairs[(idx - 10) % len(accent_pairs)]
            self._add_box(points, cx, cy, sx, sy, sz, top_color, side_color)

        # Pillar-style decorations
        for _ in range(6):
            cx = self.random.uniform(-half_x * 0.65, half_x * 0.65)
            cy = self.random.uniform(-half_y * 0.65, half_y * 0.65)
            radius = self.random.uniform(0.12, 0.2)
            z = 0.0
            while z <= self.max_height * 0.95:
                theta = 0.0
                while theta < math.tau:
                    x = cx + math.cos(theta) * radius
                    y = cy + math.sin(theta) * radius
                    points.append((x, y, z, self._shade(148, 156, 162, variance=9), "feature"))
                    theta += self.step / max(radius, self.step)
                z += self.step

        # Dedicated den-room content: couch, coffee table, bookshelves, and cabinets.
        room_center_x = self.extent_x * 0.28
        room_center_y = self.extent_y * 0.28
        couch_top = (176, 132, 104)
        couch_side = (142, 102, 76)
        self._add_box(points, room_center_x, room_center_y - 0.35, 1.55, 0.72, 0.56, couch_top, couch_side)
        self._add_box(points, room_center_x - 0.62, room_center_y - 0.35, 0.34, 0.72, 0.86, couch_top, couch_side)
        self._add_box(points, room_center_x + 0.62, room_center_y - 0.35, 0.34, 0.72, 0.86, couch_top, couch_side)

        table_top = (168, 124, 86)
        table_side = (132, 94, 62)
        self._add_box(points, room_center_x, room_center_y + 0.22, 0.86, 0.52, 0.46, table_top, table_side)

        shelf_top = (154, 112, 78)
        shelf_side = (120, 84, 56)
        shelf_x = self.extent_x * 0.44
        for idx in range(5):
            level_h = 0.35 + idx * 0.32
            self._add_box(points, shelf_x, room_center_y + 0.56, 0.88, 0.18, level_h, shelf_top, shelf_side)

        # Small colorful decor cubes (books / decorations).
        decor_colors = [
            ((94, 162, 224), (74, 136, 188)),
            ((212, 126, 92), (178, 100, 70)),
            ((132, 198, 132), (106, 166, 106)),
            ((198, 146, 216), (162, 114, 182)),
        ]
        for i in range(12):
            dx = self.random.uniform(-0.42, 0.42)
            dy = self.random.uniform(-0.14, 0.14)
            sz = self.random.uniform(0.18, 0.36)
            c_top, c_side = decor_colors[i % len(decor_colors)]
            self._add_box(
                points,
                shelf_x + dx,
                room_center_y + 0.56 + dy,
                0.12,
                0.12,
                sz,
                c_top,
                c_side,
            )

    def _add_ceiling(self, points: List[PointRecord]) -> None:
        if not self.include_ceiling:
            return
        half_x = self.extent_x * 0.5
        half_y = self.extent_y * 0.5
        z = self.max_height
        for x in self._sample_range(-half_x, half_x, self.step * 1.05):
            for y in self._sample_range(-half_y, half_y, self.step * 1.05):
                points.append((x, y, z, self._shade(154, 168, 188, variance=6), "ceiling"))

    def _build_scene_points(self) -> List[PointRecord]:
        points: List[PointRecord] = []
        self._add_floor(points)
        self._add_layout(points)
        self._add_furniture(points)
        self._add_ceiling(points)
        return points

    def _limit_points(self, points: List[PointRecord]) -> List[PointRecord]:
        if len(points) <= self.max_points:
            return points

        floor_points = [p for p in points if p[4] == "floor"]
        structural_points = [p for p in points if p[4] != "floor"]

        rng = random.Random(self.seed + 1337)

        # Keep most structural detail, trim floor aggressively first.
        structural_budget = min(len(structural_points), int(self.max_points * 0.86))
        selected_structural = (
            structural_points
            if len(structural_points) <= structural_budget
            else rng.sample(structural_points, structural_budget)
        )

        remaining_budget = self.max_points - len(selected_structural)
        selected_floor = (
            floor_points
            if len(floor_points) <= remaining_budget
            else rng.sample(floor_points, remaining_budget)
        )

        merged = selected_structural + selected_floor
        rng.shuffle(merged)
        return merged[: self.max_points]

    def _on_change_tick(self) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9

        try:
            sub_count = int(self.publisher.get_subscription_count())
        except Exception:
            sub_count = 0

        if sub_count != self._last_subscriber_count:
            self._last_subscriber_count = sub_count
            if sub_count > 0:
                self._publish()
                self.get_logger().info(
                    f"Subscriber count changed to {sub_count}; republished compact house snapshot."
                )

        if (
            self.on_change_republish_interval > 0.0
            and now_s - self._last_publish_time >= self.on_change_republish_interval
        ):
            self._publish()

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        self._last_publish_time = now.sec + (now.nanosec / 1e9)

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
    parser = argparse.ArgumentParser(description="Publish a compact colorful house PointCloud2 map for Horus tests.")
    parser.add_argument("--topic", default="/map_3d", help="PointCloud2 topic name (default: /map_3d).")
    parser.add_argument("--frame", default="map", help="Frame id (default: map).")
    parser.add_argument("--rate", type=float, default=1.0, help="Publish rate in Hz (default: 1.0).")
    parser.add_argument("--extent-x", type=float, default=7.0, help="House width in meters (default: 7.0).")
    parser.add_argument("--extent-y", type=float, default=6.0, help="House depth in meters (default: 6.0).")
    parser.add_argument("--max-height", type=float, default=2.4, help="Wall height in meters (default: 2.4).")
    parser.add_argument("--resolution", type=float, default=0.10, help="Base resolution before density multiplier (default: 0.10).")
    parser.add_argument("--density-multiplier", type=float, default=3.6, help="Density multiplier (default: 3.6).")
    parser.add_argument("--seed", type=int, default=42, help="Random seed (default: 42).")
    parser.add_argument("--max-points", type=int, default=100000, help="Hard max points after capping (default: 100000).")
    parser.add_argument("--include-ceiling", action="store_true", default=False, help="Include ceiling points (default: off).")
    parser.add_argument(
        "--publish-mode",
        choices=("on_change", "continuous"),
        default="on_change",
        help="on_change publishes once + republish events, continuous republishes at --rate.",
    )
    parser.add_argument(
        "--on-change-republish-interval",
        type=float,
        default=2.0,
        help="Safety republish interval in seconds for on_change mode (default: 2.0).",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    rclpy.init()
    node = CompactHouse3DMapPublisher(
        topic=args.topic,
        frame_id=args.frame,
        rate_hz=args.rate,
        extent_x=args.extent_x,
        extent_y=args.extent_y,
        max_height=args.max_height,
        resolution=args.resolution,
        density_multiplier=args.density_multiplier,
        seed=args.seed,
        max_points=args.max_points,
        include_ceiling=args.include_ceiling,
        publish_mode=args.publish_mode,
        on_change_republish_interval=args.on_change_republish_interval,
    )

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
