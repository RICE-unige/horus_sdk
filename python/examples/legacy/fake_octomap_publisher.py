#!/usr/bin/env python3
"""Publish a deterministic fake OctoMap workflow for Horus demos.

Primary output is a mesh marker TRIANGLE_LIST (Quest-friendly rendering path).
If octomap_msgs is available, a best-effort native octomap topic is also published.
"""

from __future__ import annotations

import argparse
import json
import importlib.util
import os
import random
import sys
import time
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.join(SCRIPT_DIR, "..", "..")
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

try:
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import ColorRGBA, String
    from visualization_msgs.msg import Marker
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    sys.exit(1)

try:
    from octomap_msgs.msg import Octomap

    HAS_NATIVE_OCTOMAP = True
except Exception:
    HAS_NATIVE_OCTOMAP = False
    Octomap = None  # type: ignore

VOXEL_MESH_PATH = os.path.join(PACKAGE_ROOT, "horus", "utils", "voxel_mesh.py")
_voxel_spec = importlib.util.spec_from_file_location("horus_voxel_mesh_local", VOXEL_MESH_PATH)
if _voxel_spec is None or _voxel_spec.loader is None:
    raise RuntimeError(f"Unable to load voxel mesh helper from {VOXEL_MESH_PATH}")
_voxel_module = importlib.util.module_from_spec(_voxel_spec)
sys.modules[_voxel_spec.name] = _voxel_module
_voxel_spec.loader.exec_module(_voxel_module)
build_greedy_surface_mesh = _voxel_module.build_greedy_surface_mesh


VoxelKey = Tuple[int, int, int]
ColorU8 = Tuple[int, int, int]
SDK_REPLAY_REQUEST_TOPIC = "/horus/multi_operator/sdk_registration_replay_request"
SDK_REPLAY_END_TOPIC = "/horus/multi_operator/sdk_registry_replay_end"
REPLAY_BURST_COUNT_DEFAULT = 8
REPLAY_BURST_INTERVAL_DEFAULT = 0.5


class FakeOctomapPublisher(Node):
    def __init__(
        self,
        octomap_topic: str,
        mesh_topic: str,
        frame_id: str,
        voxel_size: float,
        max_voxels: int,
        max_triangles: int,
        republish_interval: float,
        detailed: bool,
        seed: int,
    ) -> None:
        super().__init__("horus_fake_octomap_publisher")
        self.octomap_topic = str(octomap_topic)
        self.mesh_topic = str(mesh_topic)
        self.frame_id = str(frame_id)
        self.voxel_size = max(0.02, float(voxel_size))
        self.max_voxels = max(0, int(max_voxels))
        self.max_triangles = max(0, int(max_triangles))
        self.republish_interval = max(0.0, float(republish_interval))
        self.detailed = bool(detailed)
        self.random = random.Random(int(seed))
        self._warned_metadata_native = False

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.mesh_pub = self.create_publisher(Marker, self.mesh_topic, qos)
        self.octomap_pub = (
            self.create_publisher(Octomap, self.octomap_topic, qos)
            if HAS_NATIVE_OCTOMAP
            else None
        )
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

        voxel_coords, voxel_colors = self._build_scene_voxels()
        mesh_result = build_greedy_surface_mesh(
            voxels=voxel_coords,
            voxel_size=self.voxel_size,
            voxel_colors=voxel_colors,
            max_triangles=self.max_triangles,
            color_quant_step=8,
        )

        if not mesh_result.vertices:
            raise RuntimeError("Failed to build fake OctoMap mesh: no triangles generated.")

        self.marker = self._build_marker(mesh_result.vertices, mesh_result.colors)
        self.octomap_msg = self._build_octomap_message()
        self.scene_voxel_count = int(voxel_coords.shape[0])
        self.scene_triangle_count = int(mesh_result.triangle_count)

        self._last_mesh_subs = -1
        self._last_octomap_subs = -1
        self._last_publish_time = 0.0
        self._replay_burst_count = REPLAY_BURST_COUNT_DEFAULT
        self._replay_burst_interval = REPLAY_BURST_INTERVAL_DEFAULT
        self._replay_burst_remaining = 0
        self._replay_burst_next_time = 0.0
        self._replay_burst_total = 0
        self._replay_burst_published = 0
        self._replay_burst_source = ""
        self._replay_burst_request_id = ""
        self.timer = self.create_timer(0.5, self._tick)

        self._publish(reason="initial")
        self.get_logger().info(
            "Fake OctoMap ready: "
            f"mesh_topic={self.mesh_topic}, octomap_topic={self.octomap_topic}, "
            f"voxels={self.scene_voxel_count}, triangles={self.scene_triangle_count}, "
            f"voxel_size={self.voxel_size:.3f}, detailed={self.detailed}, "
            f"native_octomap={'on' if self.octomap_pub is not None else 'off'}, "
            f"replay_burst={self._replay_burst_count}x{self._replay_burst_interval:.2f}s"
        )

    def _build_scene_voxels(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        extent_x = 18.0 if self.detailed else 12.0
        extent_y = 14.0 if self.detailed else 9.0
        wall_height = 2.8 if self.detailed else 2.4
        half_x = int(round((extent_x * 0.5) / self.voxel_size))
        half_y = int(round((extent_y * 0.5) / self.voxel_size))
        wall_h = max(1, int(round(wall_height / self.voxel_size)))

        voxels: Dict[VoxelKey, ColorU8] = {}

        def paint(ix: int, iy: int, iz: int, color: ColorU8) -> None:
            voxels[(ix, iy, iz)] = color

        def paint_box(
            ix0: int,
            ix1: int,
            iy0: int,
            iy1: int,
            iz0: int,
            iz1: int,
            color: ColorU8,
        ) -> None:
            for ix in range(min(ix0, ix1), max(ix0, ix1) + 1):
                for iy in range(min(iy0, iy1), max(iy0, iy1) + 1):
                    for iz in range(min(iz0, iz1), max(iz0, iz1) + 1):
                        paint(ix, iy, iz, color)

        floor_color = (96, 106, 96)
        wall_color = (150, 160, 184)
        pillar_color = (170, 170, 178)
        table_color = (182, 132, 92)
        shelf_color = (132, 114, 92)

        # Floor
        for ix in range(-half_x, half_x + 1):
            for iy in range(-half_y, half_y + 1):
                paint(ix, iy, 0, floor_color)

        # Outer walls
        for iz in range(0, wall_h + 1):
            for ix in range(-half_x, half_x + 1):
                paint(ix, -half_y, iz, wall_color)
                paint(ix, half_y, iz, wall_color)
            for iy in range(-half_y, half_y + 1):
                paint(-half_x, iy, iz, wall_color)
                paint(half_x, iy, iz, wall_color)

        # Two interior walls with door gaps
        door_half = max(1, int(round(0.7 / self.voxel_size)))
        wall_ix = int(round(0.25 * half_x))
        for iz in range(0, wall_h):
            for iy in range(-half_y + 2, half_y - 1):
                if -door_half <= iy <= door_half:
                    continue
                paint(wall_ix, iy, iz, wall_color)
        wall_iy = int(round(-0.20 * half_y))
        for iz in range(0, wall_h):
            for ix in range(-half_x + 2, half_x - 1):
                if -door_half <= ix <= door_half:
                    continue
                paint(ix, wall_iy, iz, wall_color)

        # Pillars
        pillar_radius = max(1, int(round(0.20 / self.voxel_size)))
        pillar_height = max(1, int(round((2.5 if self.detailed else 2.0) / self.voxel_size)))
        for cx, cy in [(-half_x // 3, -half_y // 4), (half_x // 4, half_y // 3), (0, 0)]:
            for iz in range(0, pillar_height):
                for dx in range(-pillar_radius, pillar_radius + 1):
                    for dy in range(-pillar_radius, pillar_radius + 1):
                        if dx * dx + dy * dy > pillar_radius * pillar_radius:
                            continue
                        paint(cx + dx, cy + dy, iz, pillar_color)

        # Tables
        table_count = 14 if self.detailed else 9
        table_half_x = max(1, int(round(0.35 / self.voxel_size)))
        table_half_y = max(1, int(round(0.25 / self.voxel_size)))
        table_h = max(1, int(round(0.8 / self.voxel_size)))
        for _ in range(table_count):
            cx = self.random.randint(-half_x + 4, half_x - 4)
            cy = self.random.randint(-half_y + 4, half_y - 4)
            paint_box(
                cx - table_half_x,
                cx + table_half_x,
                cy - table_half_y,
                cy + table_half_y,
                max(1, table_h - 1),
                table_h,
                table_color,
            )
            leg_h = max(1, table_h - 2)
            for sx in (-table_half_x, table_half_x):
                for sy in (-table_half_y, table_half_y):
                    paint_box(cx + sx, cx + sx, cy + sy, cy + sy, 1, leg_h, table_color)

        # Shelves
        shelf_count = 10 if self.detailed else 6
        shelf_half_x = max(1, int(round(0.4 / self.voxel_size)))
        shelf_half_y = max(1, int(round(0.2 / self.voxel_size)))
        shelf_h = max(2, int(round(1.8 / self.voxel_size)))
        for _ in range(shelf_count):
            cx = self.random.randint(-half_x + 5, half_x - 5)
            cy = self.random.randint(-half_y + 5, half_y - 5)
            paint_box(
                cx - shelf_half_x,
                cx + shelf_half_x,
                cy - shelf_half_y,
                cy + shelf_half_y,
                1,
                shelf_h,
                shelf_color,
            )

        coords = np.asarray(list(voxels.keys()), dtype=np.int32)
        colors = np.asarray(list(voxels.values()), dtype=np.uint8)
        if self.max_voxels > 0 and coords.shape[0] > self.max_voxels:
            stride = max(1, coords.shape[0] // self.max_voxels)
            keep = np.arange(0, coords.shape[0], stride, dtype=np.int64)
            if keep.shape[0] > self.max_voxels:
                keep = keep[: self.max_voxels]
            coords = coords[keep]
            colors = colors[keep]

        return coords, colors

    def _build_marker(
        self,
        vertices: Iterable[Tuple[float, float, float]],
        colors: Optional[Iterable[Tuple[int, int, int]]],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "map_octomap_mesh"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in vertices]
        if colors is not None:
            marker.colors = [
                ColorRGBA(
                    r=float(r) / 255.0,
                    g=float(g) / 255.0,
                    b=float(b) / 255.0,
                    a=1.0,
                )
                for r, g, b in colors
            ]
        return marker

    def _build_octomap_message(self) -> Optional["Octomap"]:
        if self.octomap_pub is None:
            return None
        msg = Octomap()
        msg.header.frame_id = self.frame_id
        msg.binary = True
        msg.id = "OcTree"
        msg.resolution = float(self.voxel_size)
        # Best-effort native publish: octree serialization is intentionally omitted.
        msg.data = []
        return msg

    def _publish(self, reason: str) -> None:
        now = self.get_clock().now().to_msg()
        self.marker.header.stamp = now
        self.mesh_pub.publish(self.marker)
        if self.octomap_msg is not None and self.octomap_pub is not None:
            self.octomap_msg.header.stamp = now
            self.octomap_pub.publish(self.octomap_msg)
            if not self._warned_metadata_native:
                self._warned_metadata_native = True
                self.get_logger().warning(
                    "Native OctoMap topic is available but published message carries metadata-only "
                    "payload (empty binary octree data). Mesh marker topic remains authoritative."
                )
        self._last_publish_time = time.monotonic()
        self.get_logger().info(
            f"Published fake octomap ({reason}): voxels={self.scene_voxel_count}, triangles={self.scene_triangle_count}"
        )

    def _tick(self) -> None:
        self._service_replay_burst()

        mesh_subs = int(self.mesh_pub.get_subscription_count())
        octo_subs = int(self.octomap_pub.get_subscription_count()) if self.octomap_pub is not None else 0
        now = time.monotonic()

        should_publish = False
        reason = ""
        if mesh_subs != self._last_mesh_subs or octo_subs != self._last_octomap_subs:
            self._last_mesh_subs = mesh_subs
            self._last_octomap_subs = octo_subs
            if (mesh_subs + octo_subs) > 0:
                should_publish = True
                reason = "subscriber_change"

        if (
            not should_publish
            and self.republish_interval > 0.0
            and (now - self._last_publish_time) >= self.republish_interval
        ):
            should_publish = True
            reason = "periodic_keepalive"

        if should_publish:
            self._publish(reason=reason)

    def _service_replay_burst(self) -> None:
        if self._replay_burst_remaining <= 0:
            return

        now = time.monotonic()
        if now < self._replay_burst_next_time:
            return

        attempt = (self._replay_burst_total - self._replay_burst_remaining) + 1
        snapshot_available = self.marker is not None
        if snapshot_available:
            self._publish(reason=f"replay_burst_{attempt}")
            self._replay_burst_published += 1
            self.get_logger().info(
                f"[3D-MAP][replay_burst] source={self._replay_burst_source}, "
                f"request_id={self._replay_burst_request_id or '-'}, publish={attempt}/{self._replay_burst_total}, "
                f"snapshot_available=True, voxels={self.scene_voxel_count}, triangles={self.scene_triangle_count}."
            )
        else:
            self.get_logger().warning(
                f"[3D-MAP][replay_burst] source={self._replay_burst_source}, "
                f"request_id={self._replay_burst_request_id or '-'}, publish={attempt}/{self._replay_burst_total}, "
                "snapshot_available=False (waiting for first octomap snapshot)."
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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish fake OctoMap mesh/native topics for Horus testing.")
    parser.add_argument("--octomap-topic", default="/map_3d_octomap", help="Native octomap topic.")
    parser.add_argument("--mesh-topic", default="/map_3d_octomap_mesh", help="Mesh marker topic.")
    parser.add_argument("--frame", default="map", help="Frame id for outputs.")
    parser.add_argument("--voxel-size", type=float, default=0.10, help="Voxel size in meters (default: 0.10).")
    parser.add_argument("--max-voxels", type=int, default=60000, help="Voxel cap (default: 60000, 0=unlimited).")
    parser.add_argument(
        "--max-triangles",
        type=int,
        default=60000,
        help="Triangle cap after greedy meshing (default: 60000, 0=unlimited).",
    )
    parser.add_argument(
        "--republish-interval",
        type=float,
        default=0.0,
        help="Periodic keepalive republish interval seconds (default: 0 = disabled).",
    )
    parser.add_argument("--detailed", action="store_true", default=False, help="Use denser fake map layout.")
    parser.add_argument("--seed", type=int, default=42, help="Scene random seed.")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    rclpy.init()
    node = FakeOctomapPublisher(
        octomap_topic=args.octomap_topic,
        mesh_topic=args.mesh_topic,
        frame_id=args.frame,
        voxel_size=args.voxel_size,
        max_voxels=args.max_voxels,
        max_triangles=args.max_triangles,
        republish_interval=args.republish_interval,
        detailed=bool(args.detailed),
        seed=args.seed,
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
