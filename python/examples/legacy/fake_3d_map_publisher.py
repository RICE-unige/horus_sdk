#!/usr/bin/env python3
"""Publish synthetic 3D map PointCloud2 data for HORUS testing.

The legacy 3D map variants are consolidated into one launcher:
- basic: lightweight room + simple obstacles.
- compact_house: Quest-friendly colorful indoor map with a hard point cap.
- realistic: dense indoor stress-test map with replay bursts for multi-operator sessions.

Usage examples:
  python3 python/examples/legacy/fake_3d_map_publisher.py
  python3 python/examples/legacy/fake_3d_map_publisher.py --profile compact_house --max-points 100000
  python3 python/examples/legacy/fake_3d_map_publisher.py --profile realistic --density-multiplier 3.0
  python3 python/examples/legacy/fake_3d_map_publisher.py --profile realistic --publish-mode continuous
"""

import argparse
import json
import math
import random
import struct
import sys
import time
from typing import Dict, List, Optional, Tuple

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import String
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


PointKey = Tuple[int, int, int]
PointValue = Tuple[float, float, float, int]
SDK_REPLAY_REQUEST_TOPIC = "/horus/multi_operator/sdk_registration_replay_request"
SDK_REPLAY_END_TOPIC = "/horus/multi_operator/sdk_registry_replay_end"
REPLAY_BURST_COUNT_DEFAULT = 8
REPLAY_BURST_INTERVAL_DEFAULT = 0.5


class PointCloudBuilder:
    """Deduplicates points in a quantized grid while keeping latest color."""

    def __init__(self, quantization_step: float) -> None:
        self._q = max(0.001, float(quantization_step))
        self._points: Dict[PointKey, PointValue] = {}

    def add(self, x: float, y: float, z: float, rgb_u32: int) -> None:
        key = (
            int(round(x / self._q)),
            int(round(y / self._q)),
            int(round(z / self._q)),
        )
        self._points[key] = (x, y, z, rgb_u32)

    def to_list(self) -> List[PointValue]:
        return list(self._points.values())


class Fake3DMapPublisherRealistic(Node):
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
        include_ceiling: bool,
        dropout_rate: float,
        noise_std: float,
        table_count: int,
        shelf_count: int,
        pillar_count: int,
        publish_mode: str,
        map_change_interval: float,
        on_change_republish_interval: float,
        max_points: int = 0,
    ) -> None:
        super().__init__("horus_fake_3d_map_publisher_realistic")
        self.topic = topic
        self.frame_id = frame_id
        self.rate_hz = max(0.1, float(rate_hz))
        self.extent_x = max(4.0, float(extent_x))
        self.extent_y = max(4.0, float(extent_y))
        self.max_height = max(1.5, float(max_height))
        self.base_resolution = max(0.05, float(resolution))
        self.density_multiplier = max(1.0, float(density_multiplier))
        self.step = max(0.03, self.base_resolution / math.sqrt(self.density_multiplier))
        # Keep the no-ceiling profile lighter for Quest stress tests:
        # - floor becomes much sparser
        # - structures are modestly thinned
        self.floor_step = self.step * (8.0 if not include_ceiling else 1.0)
        self.structure_step = self.step * (1.245 if not include_ceiling else 1.0)
        self.random = random.Random(seed)
        self.include_ceiling = bool(include_ceiling)
        self.dropout_rate = min(0.6, max(0.0, float(dropout_rate)))
        self.noise_std = max(0.0, float(noise_std))
        self.table_count = max(0, int(table_count))
        self.shelf_count = max(0, int(shelf_count))
        self.pillar_count = max(0, int(pillar_count))
        self.publish_mode = publish_mode.strip().lower()
        self.map_change_interval = max(0.0, float(map_change_interval))
        self.on_change_republish_interval = max(0.0, float(on_change_republish_interval))
        self.max_points = max(0, int(max_points))
        self.seed_base = int(seed)
        self.scene_revision = 0

        self._point_struct = struct.Struct("<ffff")
        self._fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self._points = self._build_scene_points()
        if self.max_points > 0 and len(self._points) > self.max_points:
            self.random.shuffle(self._points)
            self._points = self._points[: self.max_points]

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(PointCloud2, self.topic, qos)
        self.sdk_replay_request_sub = self.create_subscription(
            String,
            SDK_REPLAY_REQUEST_TOPIC,
            self._on_sdk_replay_request,
            10,
        )
        self.sdk_replay_end_sub = self.create_subscription(
            String,
            SDK_REPLAY_END_TOPIC,
            self._on_sdk_replay_end,
            10,
        )
        self.timer = None
        self.change_timer = None
        self.replay_timer = self.create_timer(0.1, self._on_replay_burst_tick)
        self._last_change_pub_time = self.get_clock().now().nanoseconds / 1e9
        self._last_subscriber_count = -1
        self._last_publish_time = 0.0
        self._replay_burst_count = REPLAY_BURST_COUNT_DEFAULT
        self._replay_burst_interval = REPLAY_BURST_INTERVAL_DEFAULT
        self._replay_burst_remaining = 0
        self._replay_burst_next_time = 0.0
        self._replay_burst_total = 0
        self._replay_burst_published = 0
        self._replay_burst_source = ""
        self._replay_burst_request_id = ""

        # Publish initial map once (latched via TRANSIENT_LOCAL).
        self._publish()

        if self.publish_mode == "continuous":
            self.timer = self.create_timer(1.0 / self.rate_hz, self._publish)
        else:
            # on_change mode: lightweight periodic check for late subscribers and optional map changes.
            check_period = 0.5
            if self.map_change_interval > 0.0:
                check_period = min(check_period, max(0.1, self.map_change_interval / 10.0))
            self.change_timer = self.create_timer(check_period, self._on_change_tick)

        self.get_logger().info(
            "Publishing realistic fake 3D map on "
            f"{self.topic} ({len(self._points)} points, "
            f"extent={self.extent_x:.1f}x{self.extent_y:.1f}m, "
            f"height={self.max_height:.1f}m, base_res={self.base_resolution:.3f}m, "
            f"density_multiplier={self.density_multiplier:.2f}, step={self.step:.3f}m, "
            f"mode={self.publish_mode}, rate={self.rate_hz:.2f}Hz, "
            f"map_change_interval={self.map_change_interval:.2f}s, "
            f"on_change_republish_interval={self.on_change_republish_interval:.2f}s, "
            f"replay_burst={self._replay_burst_count}x{self._replay_burst_interval:.2f}s)"
        )

    @staticmethod
    def _rgb_u32(r: int, g: int, b: int) -> int:
        return ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF)

    def _shade(self, r: int, g: int, b: int, variance: int = 12) -> int:
        dr = self.random.randint(-variance, variance)
        dg = self.random.randint(-variance, variance)
        db = self.random.randint(-variance, variance)
        return self._rgb_u32(
            max(0, min(255, r + dr)),
            max(0, min(255, g + dg)),
            max(0, min(255, b + db)),
        )

    def _maybe_add(
        self,
        cloud: PointCloudBuilder,
        x: float,
        y: float,
        z: float,
        color: int,
    ) -> None:
        if self.dropout_rate > 0.0 and self.random.random() < self.dropout_rate:
            return

        if self.noise_std > 0.0:
            x += self.random.gauss(0.0, self.noise_std)
            y += self.random.gauss(0.0, self.noise_std)
            z += self.random.gauss(0.0, self.noise_std * 0.5)

        cloud.add(x, y, z, color)

    def _sample_range(self, a: float, b: float, step: Optional[float] = None) -> List[float]:
        step_size = self.step if step is None else max(0.01, float(step))
        values: List[float] = []
        v = min(a, b)
        end = max(a, b)
        while v <= end + 1e-6:
            values.append(v)
            v += step_size
        return values

    def _add_floor(self, cloud: PointCloudBuilder) -> None:
        half_x = self.extent_x * 0.5
        half_y = self.extent_y * 0.5
        for x in self._sample_range(-half_x, half_x, self.floor_step):
            for y in self._sample_range(-half_y, half_y, self.floor_step):
                color = self._shade(95, 100, 95, variance=6)
                self._maybe_add(cloud, x, y, 0.0, color)

    def _add_ceiling(self, cloud: PointCloudBuilder) -> None:
        if not self.include_ceiling:
            return
        z = self.max_height
        half_x = self.extent_x * 0.5
        half_y = self.extent_y * 0.5
        for x in self._sample_range(-half_x, half_x, self.structure_step):
            for y in self._sample_range(-half_y, half_y, self.structure_step):
                color = self._shade(120, 125, 130, variance=6)
                self._maybe_add(cloud, x, y, z, color)

    def _add_wall_segment(
        self,
        cloud: PointCloudBuilder,
        x0: float,
        y0: float,
        x1: float,
        y1: float,
        z_max: Optional[float] = None,
        thickness: Optional[float] = None,
        door_gaps: Optional[List[Tuple[float, float]]] = None,
        color_rgb: Tuple[int, int, int] = (150, 160, 185),
    ) -> None:
        z_top = self.max_height if z_max is None else max(0.3, z_max)
        step = self.structure_step
        t = max(step, thickness if thickness is not None else step * 0.9)
        dx = x1 - x0
        dy = y1 - y0
        length = max(1e-6, math.hypot(dx, dy))
        ux = dx / length
        uy = dy / length
        nx = -uy
        ny = ux

        d = 0.0
        while d <= length + 1e-6:
            in_gap = False
            if door_gaps:
                for g0, g1 in door_gaps:
                    if g0 <= d <= g1:
                        in_gap = True
                        break
            if in_gap:
                d += step
                continue

            cx = x0 + ux * d
            cy = y0 + uy * d
            zz = 0.0
            while zz <= z_top + 1e-6:
                color = self._shade(*color_rgb, variance=8)
                self._maybe_add(cloud, cx + nx * t * 0.5, cy + ny * t * 0.5, zz, color)
                self._maybe_add(cloud, cx - nx * t * 0.5, cy - ny * t * 0.5, zz, color)
                zz += step
            d += step

    def _add_box(
        self,
        cloud: PointCloudBuilder,
        cx: float,
        cy: float,
        sx: float,
        sy: float,
        sz: float,
        color_top: Tuple[int, int, int],
        color_side: Tuple[int, int, int],
    ) -> None:
        min_x = cx - sx * 0.5
        max_x = cx + sx * 0.5
        min_y = cy - sy * 0.5
        max_y = cy + sy * 0.5

        step = self.structure_step
        for x in self._sample_range(min_x, max_x, step):
            for y in self._sample_range(min_y, max_y, step):
                self._maybe_add(cloud, x, y, sz, self._shade(*color_top, variance=8))

        for z in self._sample_range(0.0, sz, step):
            for x in self._sample_range(min_x, max_x, step):
                self._maybe_add(cloud, x, min_y, z, self._shade(*color_side, variance=8))
                self._maybe_add(cloud, x, max_y, z, self._shade(*color_side, variance=8))
            for y in self._sample_range(min_y, max_y, step):
                self._maybe_add(cloud, min_x, y, z, self._shade(*color_side, variance=8))
                self._maybe_add(cloud, max_x, y, z, self._shade(*color_side, variance=8))

    def _add_pillar(
        self,
        cloud: PointCloudBuilder,
        cx: float,
        cy: float,
        radius: float,
        height: float,
    ) -> None:
        step = self.structure_step
        radius = max(step * 1.5, radius)
        for z in self._sample_range(0.0, height, step):
            theta = 0.0
            while theta < math.tau:
                x = cx + math.cos(theta) * radius
                y = cy + math.sin(theta) * radius
                self._maybe_add(cloud, x, y, z, self._shade(165, 165, 172, variance=7))
                theta += step / max(radius, step)

    def _add_layout(self, cloud: PointCloudBuilder) -> None:
        half_x = self.extent_x * 0.5
        half_y = self.extent_y * 0.5

        # Perimeter walls.
        self._add_wall_segment(cloud, -half_x, -half_y, half_x, -half_y)
        self._add_wall_segment(cloud, -half_x, half_y, half_x, half_y)
        self._add_wall_segment(cloud, -half_x, -half_y, -half_x, half_y)
        self._add_wall_segment(cloud, half_x, -half_y, half_x, half_y)

        # Internal room separators with door gaps.
        wall_len_x = self.extent_x * 0.8
        wall_start_x = -wall_len_x * 0.5
        wall_end_x = wall_len_x * 0.5
        door_width = max(0.8, self.extent_x * 0.08)
        center_gap = wall_len_x * 0.5
        self._add_wall_segment(
            cloud,
            wall_start_x,
            -self.extent_y * 0.12,
            wall_end_x,
            -self.extent_y * 0.12,
            door_gaps=[(center_gap - door_width * 0.5, center_gap + door_width * 0.5)],
            color_rgb=(150, 155, 175),
        )

        wall_len_y = self.extent_y * 0.7
        wall_start_y = -wall_len_y * 0.5
        wall_end_y = wall_len_y * 0.5
        door_pos = wall_len_y * 0.28
        self._add_wall_segment(
            cloud,
            self.extent_x * 0.10,
            wall_start_y,
            self.extent_x * 0.10,
            wall_end_y,
            door_gaps=[(door_pos - door_width * 0.5, door_pos + door_width * 0.5)],
            color_rgb=(150, 155, 175),
        )

    def _add_furniture(self, cloud: PointCloudBuilder) -> None:
        half_x = self.extent_x * 0.5
        half_y = self.extent_y * 0.5

        for _ in range(self.table_count):
            cx = self.random.uniform(-half_x * 0.7, half_x * 0.7)
            cy = self.random.uniform(-half_y * 0.7, half_y * 0.7)
            sx = self.random.uniform(0.7, 1.4)
            sy = self.random.uniform(0.6, 1.2)
            h = self.random.uniform(0.65, 0.9)
            self._add_box(
                cloud,
                cx=cx,
                cy=cy,
                sx=sx,
                sy=sy,
                sz=h,
                color_top=(165, 122, 95),
                color_side=(145, 105, 82),
            )

        for _ in range(self.shelf_count):
            cx = self.random.uniform(-half_x * 0.8, half_x * 0.8)
            cy = self.random.uniform(-half_y * 0.8, half_y * 0.8)
            sx = self.random.uniform(0.4, 0.8)
            sy = self.random.uniform(1.0, 1.8)
            h = self.random.uniform(1.2, min(self.max_height * 0.95, 2.4))
            self._add_box(
                cloud,
                cx=cx,
                cy=cy,
                sx=sx,
                sy=sy,
                sz=h,
                color_top=(150, 130, 105),
                color_side=(130, 112, 92),
            )

        for _ in range(self.pillar_count):
            cx = self.random.uniform(-half_x * 0.75, half_x * 0.75)
            cy = self.random.uniform(-half_y * 0.75, half_y * 0.75)
            radius = self.random.uniform(0.15, 0.3)
            self._add_pillar(cloud, cx, cy, radius, self.max_height)

    def _build_scene_points(self) -> List[PointValue]:
        cloud = PointCloudBuilder(quantization_step=self.step * 0.5)
        self._add_floor(cloud)
        self._add_ceiling(cloud)
        self._add_layout(cloud)
        self._add_furniture(cloud)
        return cloud.to_list()

    def _on_change_tick(self) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9

        try:
            sub_count = int(self.publisher.get_subscription_count())
        except Exception:
            sub_count = 0

        previous_sub_count = self._last_subscriber_count
        if sub_count != previous_sub_count:
            self._last_subscriber_count = sub_count
            if sub_count > 0 and previous_sub_count <= 0:
                self._publish()
                self.get_logger().info(
                    f"Subscriber count changed to {sub_count}; republished map snapshot."
                )

        # Safety republish for volatile subscribers that may attach after initial snapshot.
        if (
            self.on_change_republish_interval > 0.0
            and now_s - self._last_publish_time >= self.on_change_republish_interval
        ):
            self._publish()

        self._maybe_publish_changed_map()

    def _maybe_publish_changed_map(self) -> None:
        if self.map_change_interval <= 0.0:
            return

        now_s = self.get_clock().now().nanoseconds / 1e9
        if now_s - self._last_change_pub_time < self.map_change_interval:
            return

        self._last_change_pub_time = now_s
        self.scene_revision += 1
        self.random = random.Random(self.seed_base + self.scene_revision)
        self._points = self._build_scene_points()
        if self.max_points > 0 and len(self._points) > self.max_points:
            self.random.shuffle(self._points)
            self._points = self._points[: self.max_points]
        self._publish()
        self.get_logger().info(
            f"Map changed -> published revision {self.scene_revision} ({len(self._points)} points)."
        )

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

    def _on_replay_burst_tick(self) -> None:
        self._service_replay_burst()

    def _service_replay_burst(self) -> None:
        if self._replay_burst_remaining <= 0:
            return

        now = time.monotonic()
        if now < self._replay_burst_next_time:
            return

        attempt = (self._replay_burst_total - self._replay_burst_remaining) + 1
        snapshot_available = len(self._points) > 0
        if snapshot_available:
            self._publish()
            self._replay_burst_published += 1
            self.get_logger().info(
                f"[3D-MAP][replay_burst] source={self._replay_burst_source}, "
                f"request_id={self._replay_burst_request_id or '-'}, publish={attempt}/{self._replay_burst_total}, "
                f"snapshot_available=True, points={len(self._points)}."
            )
        else:
            self.get_logger().warning(
                f"[3D-MAP][replay_burst] source={self._replay_burst_source}, "
                f"request_id={self._replay_burst_request_id or '-'}, publish={attempt}/{self._replay_burst_total}, "
                "snapshot_available=False (waiting for first pointcloud snapshot)."
            )

        self._replay_burst_remaining -= 1
        self._replay_burst_next_time = now + self._replay_burst_interval
        if self._replay_burst_remaining <= 0:
            self.get_logger().info(
                f"[3D-MAP][replay_burst] completed source={self._replay_burst_source}, "
                f"request_id={self._replay_burst_request_id or '-'}, "
                f"published={self._replay_burst_published}/{self._replay_burst_total}."
            )
            self._replay_burst_source = ""
            self._replay_burst_request_id = ""

    def _arm_replay_burst(self, source: str, request_id: str) -> None:
        self._replay_burst_source = source
        self._replay_burst_request_id = request_id
        self._replay_burst_remaining = self._replay_burst_count
        self._replay_burst_total = self._replay_burst_count
        self._replay_burst_published = 0
        self._replay_burst_next_time = time.monotonic()
        self.get_logger().info(
            f"[3D-MAP][replay_burst] armed source={source}, request_id={request_id or '-'}, "
            f"count={self._replay_burst_count}, interval={self._replay_burst_interval:.2f}s."
        )

    @staticmethod
    def _extract_request_id(msg: String) -> str:
        payload = str(getattr(msg, "data", "") or "").strip()
        if not payload:
            return ""
        try:
            parsed = json.loads(payload)
            if isinstance(parsed, dict):
                return str(parsed.get("request_id") or "").strip()
        except Exception:
            return ""
        return ""

    def _on_sdk_replay_request(self, msg: String) -> None:
        request_id = self._extract_request_id(msg)
        self._arm_replay_burst("request", request_id)

    def _on_sdk_replay_end(self, msg: String) -> None:
        request_id = self._extract_request_id(msg)
        self._arm_replay_burst("replay_end", request_id)

PROFILE_DEFAULTS = {
    "basic": {
        "rate": 1.0,
        "extent_x": 10.0,
        "extent_y": 8.0,
        "max_height": 2.0,
        "resolution": 0.15,
        "obstacle_count": 12,
        "seed": 42,
    },
    "compact_house": {
        "rate": 1.0,
        "extent_x": 7.0,
        "extent_y": 6.0,
        "max_height": 2.4,
        "resolution": 0.10,
        "density_multiplier": 3.6,
        "seed": 42,
        "max_points": 100000,
        "include_ceiling": False,
        "publish_mode": "on_change",
        "on_change_republish_interval": 2.0,
    },
    "realistic": {
        "rate": 1.0,
        "extent_x": 12.0,
        "extent_y": 10.0,
        "max_height": 2.4,
        "resolution": 0.15,
        "density_multiplier": 3.0,
        "seed": 42,
        "include_ceiling": False,
        "dropout_rate": 0.02,
        "noise_std": 0.004,
        "table_count": 18,
        "shelf_count": 10,
        "pillar_count": 8,
        "publish_mode": "on_change",
        "map_change_interval": 0.0,
        "on_change_republish_interval": 0.0,
        "max_points": 0,
    },
}


def _value(args: argparse.Namespace, defaults: Dict[str, object], name: str):
    value = getattr(args, name)
    if value is None:
        return defaults[name]
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publish synthetic 3D map PointCloud2 data for HORUS testing."
    )
    parser.add_argument(
        "--profile",
        choices=tuple(PROFILE_DEFAULTS.keys()),
        default="basic",
        help="Map content profile: basic|compact_house|realistic (default: basic).",
    )
    parser.add_argument("--topic", default="/map_3d", help="PointCloud2 topic name (default: /map_3d).")
    parser.add_argument("--frame", default="map", help="Frame id for cloud points (default: map).")
    parser.add_argument("--rate", type=float, default=None, help="Publish rate in Hz. Profile default is used when omitted.")
    parser.add_argument("--extent-x", type=float, default=None, help="Map width in meters. Profile default is used when omitted.")
    parser.add_argument("--extent-y", type=float, default=None, help="Map depth in meters. Profile default is used when omitted.")
    parser.add_argument("--max-height", type=float, default=None, help="Wall/obstacle max height in meters. Profile default is used when omitted.")
    parser.add_argument("--resolution", type=float, default=None, help="Sampling resolution in meters. Profile default is used when omitted.")
    parser.add_argument("--seed", type=int, default=None, help="Random seed. Profile default is used when omitted.")

    parser.add_argument("--obstacle-count", type=int, default=None, help="Basic profile random box count (default: 12).")
    parser.add_argument("--density-multiplier", type=float, default=None, help="Compact/realistic density multiplier. Profile default is used when omitted.")
    parser.add_argument("--max-points", type=int, default=None, help="Hard max points after generation for compact/realistic profiles.")
    parser.add_argument("--include-ceiling", dest="include_ceiling", action="store_true", default=None, help="Include ceiling points for compact/realistic profiles.")
    parser.add_argument("--no-ceiling", dest="include_ceiling", action="store_false", help="Disable ceiling points for compact/realistic profiles.")
    parser.add_argument(
        "--publish-mode",
        choices=("on_change", "continuous"),
        default=None,
        help="Compact/realistic publish policy. Profile default is used when omitted.",
    )
    parser.add_argument(
        "--on-change-republish-interval",
        type=float,
        default=None,
        help="Safety republish interval in seconds for on_change mode.",
    )
    parser.add_argument("--dropout-rate", type=float, default=None, help="Realistic profile random dropout ratio [0..0.6].")
    parser.add_argument("--noise-std", type=float, default=None, help="Realistic profile Gaussian position noise std-dev in meters.")
    parser.add_argument("--table-count", type=int, default=None, help="Realistic profile table-like obstacle count.")
    parser.add_argument("--shelf-count", type=int, default=None, help="Realistic profile shelf-like obstacle count.")
    parser.add_argument("--pillar-count", type=int, default=None, help="Realistic profile pillar count.")
    parser.add_argument(
        "--map-change-interval",
        type=float,
        default=None,
        help="Realistic profile: regenerate and publish the map every N seconds when >0.",
    )
    return parser


def _build_node(args: argparse.Namespace) -> Node:
    profile = str(args.profile or "basic").strip().lower()
    defaults = PROFILE_DEFAULTS[profile]

    if profile == "basic":
        return Fake3DMapPublisher(
            topic=args.topic,
            frame_id=args.frame,
            rate_hz=float(_value(args, defaults, "rate")),
            extent_x=float(_value(args, defaults, "extent_x")),
            extent_y=float(_value(args, defaults, "extent_y")),
            max_height=float(_value(args, defaults, "max_height")),
            resolution=float(_value(args, defaults, "resolution")),
            obstacle_count=int(_value(args, defaults, "obstacle_count")),
            seed=int(_value(args, defaults, "seed")),
        )

    if profile == "compact_house":
        return CompactHouse3DMapPublisher(
            topic=args.topic,
            frame_id=args.frame,
            rate_hz=float(_value(args, defaults, "rate")),
            extent_x=float(_value(args, defaults, "extent_x")),
            extent_y=float(_value(args, defaults, "extent_y")),
            max_height=float(_value(args, defaults, "max_height")),
            resolution=float(_value(args, defaults, "resolution")),
            density_multiplier=float(_value(args, defaults, "density_multiplier")),
            seed=int(_value(args, defaults, "seed")),
            max_points=int(_value(args, defaults, "max_points")),
            include_ceiling=bool(_value(args, defaults, "include_ceiling")),
            publish_mode=str(_value(args, defaults, "publish_mode")),
            on_change_republish_interval=float(_value(args, defaults, "on_change_republish_interval")),
        )

    return Fake3DMapPublisherRealistic(
        topic=args.topic,
        frame_id=args.frame,
        rate_hz=float(_value(args, defaults, "rate")),
        extent_x=float(_value(args, defaults, "extent_x")),
        extent_y=float(_value(args, defaults, "extent_y")),
        max_height=float(_value(args, defaults, "max_height")),
        resolution=float(_value(args, defaults, "resolution")),
        density_multiplier=float(_value(args, defaults, "density_multiplier")),
        seed=int(_value(args, defaults, "seed")),
        include_ceiling=bool(_value(args, defaults, "include_ceiling")),
        dropout_rate=float(_value(args, defaults, "dropout_rate")),
        noise_std=float(_value(args, defaults, "noise_std")),
        table_count=int(_value(args, defaults, "table_count")),
        shelf_count=int(_value(args, defaults, "shelf_count")),
        pillar_count=int(_value(args, defaults, "pillar_count")),
        publish_mode=str(_value(args, defaults, "publish_mode")),
        map_change_interval=float(_value(args, defaults, "map_change_interval")),
        on_change_republish_interval=float(_value(args, defaults, "on_change_republish_interval")),
        max_points=int(_value(args, defaults, "max_points")),
    )


def main() -> None:
    args = build_parser().parse_args()
    rclpy.init()
    node = _build_node(args)

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
