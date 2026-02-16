#!/usr/bin/env python3
"""Publish a dense, more realistic synthetic 3D map as sensor_msgs/PointCloud2.

This variant is intended for Quest performance stress tests.
Compared to fake_3d_map_publisher.py it produces a denser cloud
and richer indoor structure (rooms, doors, furniture, pillars).

By default it publishes in on-change mode:
- publish initial map once,
- republish when subscriber count changes (late-join safety),
- low-rate safety republish for volatile subscribers,
- optionally regenerate/publish when map changes.
Use --publish-mode continuous to republish every timer tick, or
--map-change-interval to regenerate and publish only when the map changes.

Usage examples:
  python3 python/examples/fake_3d_map_publisher_realistic.py
  python3 python/examples/fake_3d_map_publisher_realistic.py --topic /map_3d --frame map
  python3 python/examples/fake_3d_map_publisher_realistic.py --density-multiplier 3.0
  python3 python/examples/fake_3d_map_publisher_realistic.py --publish-mode continuous
  python3 python/examples/fake_3d_map_publisher_realistic.py --map-change-interval 5.0
"""

import argparse
import math
import random
import struct
import sys
from typing import Dict, List, Optional, Tuple

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    sys.exit(1)


PointKey = Tuple[int, int, int]
PointValue = Tuple[float, float, float, int]


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
        self.timer = None
        self.change_timer = None
        self._last_change_pub_time = self.get_clock().now().nanoseconds / 1e9
        self._last_subscriber_count = -1
        self._last_publish_time = 0.0

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
            f"on_change_republish_interval={self.on_change_republish_interval:.2f}s)"
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

        if sub_count != self._last_subscriber_count:
            self._last_subscriber_count = sub_count
            if sub_count > 0:
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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publish a dense realistic PointCloud2 3D map for Horus Quest stress tests."
    )
    parser.add_argument("--topic", default="/map_3d", help="PointCloud2 topic name (default: /map_3d).")
    parser.add_argument("--frame", default="map", help="Frame id (default: map).")
    parser.add_argument("--rate", type=float, default=1.0, help="Publish rate in Hz (default: 1.0).")
    parser.add_argument("--extent-x", type=float, default=12.0, help="Map width in meters (default: 12.0).")
    parser.add_argument("--extent-y", type=float, default=10.0, help="Map depth in meters (default: 10.0).")
    parser.add_argument("--max-height", type=float, default=2.4, help="Wall/ceiling height in meters (default: 2.4).")
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.15,
        help="Base sampling resolution before density multiplier (default: 0.15).",
    )
    parser.add_argument(
        "--density-multiplier",
        type=float,
        default=3.0,
        help="Point-density multiplier (default: 3.0 ~ around 3x cloud density).",
    )
    parser.add_argument("--seed", type=int, default=42, help="Random seed (default: 42).")
    parser.add_argument("--include-ceiling", action="store_true", default=False, help="Include ceiling points (default: off).")
    parser.add_argument("--no-ceiling", dest="include_ceiling", action="store_false", help="Disable ceiling points (default).")
    parser.add_argument("--dropout-rate", type=float, default=0.02, help="Random dropout ratio [0..0.6] (default: 0.02).")
    parser.add_argument("--noise-std", type=float, default=0.004, help="Gaussian position noise std-dev in meters (default: 0.004).")
    parser.add_argument("--table-count", type=int, default=18, help="Number of table-like obstacles (default: 18).")
    parser.add_argument("--shelf-count", type=int, default=10, help="Number of shelf-like obstacles (default: 10).")
    parser.add_argument("--pillar-count", type=int, default=8, help="Number of pillars (default: 8).")
    parser.add_argument(
        "--publish-mode",
        choices=("on_change", "continuous"),
        default="on_change",
        help="Publish policy: on_change publishes once (and on map changes), continuous republishes at --rate.",
    )
    parser.add_argument(
        "--map-change-interval",
        type=float,
        default=0.0,
        help="If >0 in on_change mode, regenerate and publish map every N seconds.",
    )
    parser.add_argument(
        "--on-change-republish-interval",
        type=float,
        default=2.0,
        help="Safety republish interval in seconds for on_change mode (default: 2.0, set 0 to disable).",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=0,
        help="Hard max points after generation (default: 0 = unlimited). Randomly samples if exceeded.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    rclpy.init()
    node = Fake3DMapPublisherRealistic(
        topic=args.topic,
        frame_id=args.frame,
        rate_hz=args.rate,
        extent_x=args.extent_x,
        extent_y=args.extent_y,
        max_height=args.max_height,
        resolution=args.resolution,
        density_multiplier=args.density_multiplier,
        seed=args.seed,
        include_ceiling=args.include_ceiling,
        dropout_rate=args.dropout_rate,
        noise_std=args.noise_std,
        table_count=args.table_count,
        shelf_count=args.shelf_count,
        pillar_count=args.pillar_count,
        publish_mode=args.publish_mode,
        map_change_interval=args.map_change_interval,
        on_change_republish_interval=args.on_change_republish_interval,
        max_points=args.max_points,
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
