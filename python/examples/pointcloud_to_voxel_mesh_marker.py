#!/usr/bin/env python3
"""Convert PointCloud2 into a voxel surface mesh and publish it as RViz Marker.

This is a pre-Unity validation tool:
- subscribe to global map PointCloud2 (e.g. /map_3d),
- voxelize and extract outer surfaces,
- publish visualization_msgs/Marker TRIANGLE_LIST for rviz2.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker
    from std_msgs.msg import ColorRGBA
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    sys.exit(1)


@dataclass(frozen=True)
class FaceDef:
    normal: Tuple[int, int, int]
    corners: Tuple[Tuple[float, float, float], ...]


FACE_DEFS: Tuple[FaceDef, ...] = (
    FaceDef((1, 0, 0), ((0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (0.5, 0.5, 0.5), (0.5, -0.5, 0.5))),
    FaceDef((-1, 0, 0), ((-0.5, -0.5, 0.5), (-0.5, 0.5, 0.5), (-0.5, 0.5, -0.5), (-0.5, -0.5, -0.5))),
    FaceDef((0, 1, 0), ((-0.5, 0.5, -0.5), (0.5, 0.5, -0.5), (0.5, 0.5, 0.5), (-0.5, 0.5, 0.5))),
    FaceDef((0, -1, 0), ((-0.5, -0.5, 0.5), (0.5, -0.5, 0.5), (0.5, -0.5, -0.5), (-0.5, -0.5, -0.5))),
    FaceDef((0, 0, 1), ((-0.5, -0.5, 0.5), (-0.5, 0.5, 0.5), (0.5, 0.5, 0.5), (0.5, -0.5, 0.5))),
    FaceDef((0, 0, -1), ((0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (-0.5, 0.5, -0.5), (-0.5, -0.5, -0.5))),
)


class PointCloudToVoxelMeshNode(Node):
    def __init__(
        self,
        cloud_topic: str,
        marker_topic: str,
        voxel_size: float,
        max_voxels: int,
        max_triangles: int,
        update_mode: str,
        color_rgb: Tuple[float, float, float],
        alpha: float,
        output_obj: Optional[str],
        marker_text: str,
        marker_mesh_resource: str,
        on_change_republish_interval: float,
    ) -> None:
        super().__init__("horus_pointcloud_to_voxel_mesh")

        self.cloud_topic = cloud_topic
        self.marker_topic = marker_topic
        self.voxel_size = max(0.02, float(voxel_size))
        self.max_voxels = max(0, int(max_voxels))
        self.max_triangles = max(0, int(max_triangles))
        self.update_mode = update_mode
        self.color_rgb = color_rgb
        self.alpha = max(0.0, min(1.0, alpha))
        self.output_obj = output_obj
        self.marker_text = marker_text if marker_text else "map_mesh"
        self.marker_mesh_resource = (
            marker_mesh_resource if marker_mesh_resource else "mesh://map_mesh"
        )
        self.on_change_republish_interval = max(0.0, float(on_change_republish_interval))

        self._received_once = False
        self._last_signature: Optional[Tuple[int, int, int]] = None
        self._last_marker_point_count = 0
        self._last_marker: Optional[Marker] = None

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub = self.create_subscription(PointCloud2, self.cloud_topic, self._on_cloud, qos)
        self.pub = self.create_publisher(Marker, self.marker_topic, qos)
        self.republish_timer = None
        if self.on_change_republish_interval > 0.0:
            self.republish_timer = self.create_timer(
                self.on_change_republish_interval,
                self._republish_latest_marker,
            )

        self.get_logger().info(
            f"Voxel mesh converter started: cloud={self.cloud_topic}, marker={self.marker_topic}, "
            f"voxel_size={self.voxel_size:.3f}, max_voxels={self.max_voxels}, "
            f"max_triangles={self.max_triangles}, update_mode={self.update_mode}, "
            f"on_change_republish_interval={self.on_change_republish_interval:.2f}s"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self.update_mode == "once" and self._received_once:
            return

        signature = (int(msg.width), int(msg.height), int(msg.row_step))
        if self.update_mode == "on_change" and self._last_signature == signature:
            return

        xyz, rgb = self._extract_xyzrgb(msg)
        if xyz.size == 0:
            self.get_logger().warning("Received empty or invalid point cloud; skipping.")
            return

        voxel_coords, voxel_colors = self._voxelize_points(xyz, rgb)
        raw_voxels = int(voxel_coords.shape[0])

        if self.max_voxels > 0 and raw_voxels > self.max_voxels:
            step = max(1, raw_voxels // self.max_voxels)
            sample_idx = np.arange(0, raw_voxels, step, dtype=np.int64)
            if sample_idx.shape[0] > self.max_voxels:
                sample_idx = sample_idx[: self.max_voxels]
            voxel_coords = voxel_coords[sample_idx]
            if voxel_colors is not None:
                voxel_colors = voxel_colors[sample_idx]

        marker_points, marker_colors, triangle_count = self._build_surface_triangles(voxel_coords, voxel_colors)
        if not marker_points:
            self.get_logger().warning("No surface triangles generated from voxelized cloud.")
            return

        # Unity/bridge stability degrades with very large Marker payloads.
        # geometry_msgs/Point (24B) + ColorRGBA (16B) for each of 3 vertices per triangle ~= 120B/triangle.
        estimated_payload_mb = (triangle_count * 120.0) / (1024.0 * 1024.0)
        if estimated_payload_mb > 10.0:
            self.get_logger().warning(
                f"Large Marker payload (~{estimated_payload_mb:.1f} MB). "
                "If Unity deserialization fails, lower --max-triangles or increase --voxel-size."
            )

        marker = self._make_triangle_marker(msg, marker_points, marker_colors)
        self.pub.publish(marker)
        self._last_marker = marker

        if self.output_obj:
            self._write_obj(self.output_obj, marker_points)

        self._received_once = True
        self._last_signature = signature
        self._last_marker_point_count = len(marker_points)

        self.get_logger().info(
            f"Mesh updated: raw_points={xyz.shape[0]}, raw_voxels={raw_voxels}, "
            f"used_voxels={voxel_coords.shape[0]}, triangles={triangle_count}, marker_points={len(marker_points)}"
        )

    def _republish_latest_marker(self) -> None:
        if self._last_marker is None:
            return

        if self.update_mode not in {"once", "on_change"}:
            return

        self.pub.publish(self._last_marker)
        self.get_logger().debug(
            f"Republished latest mesh marker ({self._last_marker_point_count} points) for late subscribers."
        )

    def _extract_xyzrgb(self, msg: PointCloud2) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        x_off, y_off, z_off = self._field_offsets(msg.fields)
        if x_off < 0 or y_off < 0 or z_off < 0:
            self.get_logger().error("PointCloud2 is missing x/y/z fields.")
            return np.empty((0, 3), dtype=np.float32), None

        fields_by_name = {f.name.lower(): f for f in msg.fields}
        packed_color_field = fields_by_name.get("rgb") or fields_by_name.get("rgba")
        split_r = fields_by_name.get("r")
        split_g = fields_by_name.get("g")
        split_b = fields_by_name.get("b")
        has_split_color = split_r is not None and split_g is not None and split_b is not None
        has_color = packed_color_field is not None or has_split_color

        width = int(msg.width)
        height = int(msg.height)
        row_step = int(msg.row_step)
        point_step = int(msg.point_step)
        if width <= 0 or height <= 0 or row_step <= 0 or point_step <= 0:
            return np.empty((0, 3), dtype=np.float32), None

        raw = np.frombuffer(msg.data, dtype=np.uint8)

        # Build (total, point_step) view of per-point bytes
        if row_step == width * point_step:
            # No row padding — contiguous layout (common case)
            total = width * height
            usable = min(total, raw.size // point_step)
            if usable <= 0:
                return np.empty((0, 3), dtype=np.float32), None
            points = raw[: usable * point_step].reshape(usable, point_step)
        else:
            # Row padding present — gather valid point bytes per row
            row_starts = np.arange(height, dtype=np.int64) * row_step
            col_starts = np.arange(width, dtype=np.int64) * point_step
            offsets = (row_starts[:, None] + col_starts[None, :]).ravel()
            # Bounds check
            offsets = offsets[offsets + point_step <= raw.size]
            usable = offsets.size
            if usable <= 0:
                return np.empty((0, 3), dtype=np.float32), None
            byte_indices = offsets[:, None] + np.arange(point_step, dtype=np.int64)[None, :]
            points = raw[byte_indices]

        # Extract XYZ as float32 (copy to ensure contiguous alignment)
        xyz = np.empty((usable, 3), dtype=np.float32)
        for ax, off in enumerate((x_off, y_off, z_off)):
            col_bytes = points[:, off : off + 4].copy()
            xyz[:, ax] = col_bytes.view(np.float32).ravel()

        # Filter NaN/Inf
        valid = np.all(np.isfinite(xyz), axis=1)
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float32), None
        xyz = xyz[valid]

        if not has_color:
            return xyz, None

        # Extract colors (vectorized)
        rgb = np.empty((usable, 3), dtype=np.uint8)
        if packed_color_field is not None:
            color_off = int(packed_color_field.offset)
            packed_bytes = points[:, color_off : color_off + 4].copy()
            packed_u32 = packed_bytes.view(np.uint32).ravel()
            rgb[:, 0] = ((packed_u32 >> 16) & 0xFF).astype(np.uint8)
            rgb[:, 1] = ((packed_u32 >> 8) & 0xFF).astype(np.uint8)
            rgb[:, 2] = (packed_u32 & 0xFF).astype(np.uint8)
        elif has_split_color:
            for ch, field in enumerate((split_r, split_g, split_b)):
                rgb[:, ch] = self._extract_channel_vectorized(
                    points, int(field.offset), field.datatype,
                )

        rgb = rgb[valid]
        return xyz, rgb

    @staticmethod
    def _extract_channel_vectorized(
        points: np.ndarray, offset: int, datatype: int,
    ) -> np.ndarray:
        """Vectorized extraction of a single color channel from per-point bytes."""
        n = points.shape[0]
        if datatype == PointField.UINT8:
            return points[:, offset].copy()
        if datatype == PointField.INT8:
            raw = points[:, offset : offset + 1].copy().view(np.int8).ravel()
            return np.clip(raw, 0, 127).astype(np.uint8)
        if datatype == PointField.UINT16:
            raw = points[:, offset : offset + 2].copy().view(np.uint16).ravel()
            return np.clip(raw // 257, 0, 255).astype(np.uint8)
        if datatype == PointField.FLOAT32:
            raw = points[:, offset : offset + 4].copy().view(np.float32).ravel()
            # Values in [0,1] are normalized; values > 1 are assumed 0-255
            scaled = np.where(raw <= 1.0, raw * 255.0, raw)
            return np.clip(np.rint(scaled), 0, 255).astype(np.uint8)
        return np.zeros(n, dtype=np.uint8)

    @staticmethod
    def _field_offsets(fields: Sequence[PointField]) -> Tuple[int, int, int]:
        x_off = y_off = z_off = -1
        for f in fields:
            if f.name == "x":
                x_off = int(f.offset)
            elif f.name == "y":
                y_off = int(f.offset)
            elif f.name == "z":
                z_off = int(f.offset)
        return x_off, y_off, z_off

    def _voxelize_points(self, xyz: np.ndarray, rgb: Optional[np.ndarray]) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        voxel_coords = np.floor(xyz / self.voxel_size).astype(np.int32)
        unique_voxels, inverse = np.unique(voxel_coords, axis=0, return_inverse=True)

        if rgb is None:
            return unique_voxels, None

        counts = np.bincount(inverse)
        if counts.size == 0:
            return unique_voxels, None

        colors_f = rgb.astype(np.float64)
        r_sum = np.bincount(inverse, weights=colors_f[:, 0], minlength=counts.size)
        g_sum = np.bincount(inverse, weights=colors_f[:, 1], minlength=counts.size)
        b_sum = np.bincount(inverse, weights=colors_f[:, 2], minlength=counts.size)
        denom = np.maximum(counts, 1)

        voxel_colors = np.stack(
            (
                np.rint(r_sum / denom),
                np.rint(g_sum / denom),
                np.rint(b_sum / denom),
            ),
            axis=1,
        )
        voxel_colors = np.clip(voxel_colors, 0, 255).astype(np.uint8)
        return unique_voxels, voxel_colors

    def _build_surface_triangles(
        self,
        voxels: np.ndarray,
        voxel_colors: Optional[np.ndarray],
    ) -> Tuple[List[Point], Optional[List[Tuple[int, int, int]]], int]:
        if voxels.size == 0:
            return [], None, 0

        if voxel_colors is not None and voxel_colors.shape[0] == voxels.shape[0]:
            occ: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}
            for idx, v in enumerate(voxels):
                occ[(int(v[0]), int(v[1]), int(v[2]))] = (
                    int(voxel_colors[idx, 0]),
                    int(voxel_colors[idx, 1]),
                    int(voxel_colors[idx, 2]),
                )
            occ_keys = set(occ.keys())
        else:
            occ = {}
            occ_keys = {tuple(v.tolist()) for v in voxels}

        marker_points: List[Point] = []
        marker_colors: Optional[List[Tuple[int, int, int]]] = [] if occ else None
        triangle_count = 0

        for vx, vy, vz in occ_keys:
            face_rgb = occ.get((vx, vy, vz))
            center = np.array(
                [(vx + 0.5) * self.voxel_size, (vy + 0.5) * self.voxel_size, (vz + 0.5) * self.voxel_size],
                dtype=np.float32,
            )
            for face in FACE_DEFS:
                nx, ny, nz = face.normal
                if (vx + nx, vy + ny, vz + nz) in occ_keys:
                    continue

                c0, c1, c2, c3 = face.corners
                v0 = center + np.array(c0, dtype=np.float32) * self.voxel_size
                v1 = center + np.array(c1, dtype=np.float32) * self.voxel_size
                v2 = center + np.array(c2, dtype=np.float32) * self.voxel_size
                v3 = center + np.array(c3, dtype=np.float32) * self.voxel_size

                marker_points.extend((self._to_point(v0), self._to_point(v1), self._to_point(v2)))
                marker_points.extend((self._to_point(v0), self._to_point(v2), self._to_point(v3)))
                if marker_colors is not None and face_rgb is not None:
                    marker_colors.extend((face_rgb, face_rgb, face_rgb, face_rgb, face_rgb, face_rgb))
                triangle_count += 2

                if self.max_triangles > 0 and triangle_count >= self.max_triangles:
                    return marker_points, marker_colors, triangle_count

        return marker_points, marker_colors, triangle_count

    def _make_triangle_marker(
        self,
        msg: PointCloud2,
        points: List[Point],
        colors: Optional[List[Tuple[int, int, int]]],
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = msg.header.frame_id
        marker.ns = "map_mesh"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = float(self.color_rgb[0])
        marker.color.g = float(self.color_rgb[1])
        marker.color.b = float(self.color_rgb[2])
        marker.color.a = float(self.alpha)
        marker.text = self.marker_text
        marker.mesh_resource = self.marker_mesh_resource
        marker.frame_locked = False
        marker.mesh_use_embedded_materials = False
        marker.points = points
        if colors is not None and len(colors) == len(points):
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.colors = [self._to_color(c) for c in colors]
        return marker

    @staticmethod
    def _to_point(v: np.ndarray) -> Point:
        p = Point()
        p.x = float(v[0])
        p.y = float(v[1])
        p.z = float(v[2])
        return p

    def _to_color(self, rgb: Tuple[int, int, int]) -> ColorRGBA:
        c = ColorRGBA()
        c.r = float(rgb[0]) / 255.0
        c.g = float(rgb[1]) / 255.0
        c.b = float(rgb[2]) / 255.0
        c.a = float(self.alpha)
        return c

    @staticmethod
    def _write_obj(path: str, marker_points: Sequence[Point]) -> None:
        with open(path, "w", encoding="utf-8") as f:
            f.write("# Generated by pointcloud_to_voxel_mesh_marker.py\n")
            for p in marker_points:
                f.write(f"v {p.x:.6f} {p.y:.6f} {p.z:.6f}\n")
            tri_count = len(marker_points) // 3
            for i in range(tri_count):
                base = i * 3 + 1
                f.write(f"f {base} {base + 1} {base + 2}\n")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert PointCloud2 map to voxel mesh marker for rviz2 validation."
    )
    parser.add_argument("--cloud-topic", default="/map_3d", help="Input PointCloud2 topic.")
    parser.add_argument("--mesh-topic", default="/map_3d_mesh", help="Output Marker topic.")
    parser.add_argument("--voxel-size", type=float, default=0.10, help="Voxel size in meters.")
    parser.add_argument("--max-voxels", type=int, default=60000, help="Hard cap for voxel cells (0=unlimited).")
    parser.add_argument(
        "--max-triangles",
        type=int,
        default=50000,
        help="Hard cap for triangles in marker output (0=unlimited).",
    )
    parser.add_argument(
        "--update-mode",
        choices=("once", "on_change", "continuous"),
        default="once",
        help="once: single conversion, on_change: new signature only, continuous: convert every cloud.",
    )
    parser.add_argument(
        "--on-change-republish-interval",
        type=float,
        default=2.0,
        help="For once/on_change modes, periodically republish latest marker for late subscribers (0 disables).",
    )
    parser.add_argument("--color-r", type=float, default=0.70, help="Marker red [0..1].")
    parser.add_argument("--color-g", type=float, default=0.72, help="Marker green [0..1].")
    parser.add_argument("--color-b", type=float, default=0.78, help="Marker blue [0..1].")
    parser.add_argument("--alpha", type=float, default=1.0, help="Marker alpha [0..1].")
    parser.add_argument(
        "--marker-text",
        default="map_mesh",
        help="Marker text field (kept non-empty for Unity ROS2 deserialization safety).",
    )
    parser.add_argument(
        "--marker-mesh-resource",
        default="mesh://map_mesh",
        help="Marker mesh_resource field (kept non-empty for Unity ROS2 deserialization safety).",
    )
    parser.add_argument(
        "--output-obj",
        default="",
        help="Optional path to write OBJ after each conversion.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    rclpy.init()
    node = PointCloudToVoxelMeshNode(
        cloud_topic=args.cloud_topic,
        marker_topic=args.mesh_topic,
        voxel_size=args.voxel_size,
        max_voxels=args.max_voxels,
        max_triangles=args.max_triangles,
        update_mode=args.update_mode,
        color_rgb=(args.color_r, args.color_g, args.color_b),
        alpha=args.alpha,
        output_obj=args.output_obj.strip() or None,
        marker_text=args.marker_text.strip() or "map_mesh",
        marker_mesh_resource=args.marker_mesh_resource.strip() or "mesh://map_mesh",
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
