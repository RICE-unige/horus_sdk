#!/usr/bin/env python3
"""Convert PointCloud2 into voxel mesh markers for Unity/RViz validation.

Supports two output transports:
- marker: single visualization_msgs/Marker TRIANGLE_LIST on --mesh-topic
- marker_array: chunked visualization_msgs/MarkerArray on --mesh-array-topic
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.join(SCRIPT_DIR, "..", "..")
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from horus.utils.voxel_mesh import build_greedy_surface_mesh

try:
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import ColorRGBA, String
    from visualization_msgs.msg import Marker, MarkerArray
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    sys.exit(1)


MESH_TRANSPORT_MARKER = "marker"
MESH_TRANSPORT_MARKER_ARRAY = "marker_array"
SDK_REPLAY_REQUEST_TOPIC = "/horus/multi_operator/sdk_registration_replay_request"
SDK_REPLAY_END_TOPIC = "/horus/multi_operator/sdk_registry_replay_end"
REPLAY_BURST_COUNT_DEFAULT = 8
REPLAY_BURST_INTERVAL_DEFAULT = 0.5


@dataclass(frozen=True)
class ChunkSignature:
    """Compact deterministic signature for one mesh marker chunk."""

    point_count: int
    color_count: int
    sample_hash: int


class PointCloudToVoxelMeshNode(Node):
    def __init__(
        self,
        cloud_topic: str,
        marker_topic: str,
        marker_array_topic: str,
        mesh_transport: str,
        chunk_max_triangles: int,
        voxel_size: float,
        max_voxels: int,
        max_triangles: int,
        update_mode: str,
        update_policy: str,
        color_quant_step: int,
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
        self.marker_array_topic = marker_array_topic
        # Marker-only rollback mode: keep marker_array args for compatibility, but
        # force runtime transport to marker to preserve the known-stable Unity path.
        self.mesh_transport = MESH_TRANSPORT_MARKER
        self.chunk_max_triangles = max(256, int(chunk_max_triangles))
        self.voxel_size = max(0.02, float(voxel_size))
        self.max_voxels = max(0, int(max_voxels))
        self.max_triangles = max(0, int(max_triangles))
        self.update_mode = update_mode
        self.update_policy = update_policy
        self.color_quant_step = max(1, int(color_quant_step))
        self.color_rgb = color_rgb
        self.alpha = max(0.0, min(1.0, alpha))
        self.output_obj = output_obj
        self.marker_text = marker_text if marker_text else "map_mesh"
        self.marker_mesh_resource = (
            marker_mesh_resource if marker_mesh_resource else "mesh://map_mesh"
        )
        self.on_change_republish_interval = max(0.0, float(on_change_republish_interval))

        self._received_once = False
        self._last_signature: Optional[Tuple[int, int, int, int, int, int, str, int]] = None
        self._last_marker_point_count = 0
        self._last_chunk_count = 0
        self._last_marker: Optional[Marker] = None
        self._last_marker_array_full: Optional[MarkerArray] = None
        self._last_chunk_signatures: Dict[int, ChunkSignature] = {}
        self._last_mesh_subscriber_count = -1
        self._last_periodic_republish_time = 0.0
        self._replay_burst_count = REPLAY_BURST_COUNT_DEFAULT
        self._replay_burst_interval = REPLAY_BURST_INTERVAL_DEFAULT
        self._replay_burst_remaining = 0
        self._replay_burst_next_time = 0.0
        self._replay_burst_total = 0
        self._replay_burst_published = 0
        self._replay_burst_source = ""
        self._replay_burst_request_id = ""

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub = self.create_subscription(PointCloud2, self.cloud_topic, self._on_cloud, qos)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, qos)
        self.marker_array_pub = self.create_publisher(MarkerArray, self.marker_array_topic, qos)
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

        self.republish_timer = self.create_timer(0.5, self._on_timer_tick)

        self.get_logger().info(
            f"Voxel mesh converter started: cloud={self.cloud_topic}, transport={self.mesh_transport}, "
            f"marker={self.marker_topic}, marker_array={self.marker_array_topic}, "
            f"chunk_max_triangles={self.chunk_max_triangles}, voxel_size={self.voxel_size:.3f}, "
            f"max_voxels={self.max_voxels}, max_triangles={self.max_triangles}, "
            f"update_policy={self.update_policy}, update_mode={self.update_mode}, "
            f"quant_step={self.color_quant_step}, "
            f"on_change_republish_interval={self.on_change_republish_interval:.2f}s, meshing=greedy, "
            f"replay_burst={self._replay_burst_count}x{self._replay_burst_interval:.2f}s"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self.update_mode == "once" and self._received_once:
            return

        signature = self._compute_cloud_signature(msg)
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

        mesh_result = build_greedy_surface_mesh(
            voxels=voxel_coords,
            voxel_size=self.voxel_size,
            voxel_colors=voxel_colors,
            max_triangles=self.max_triangles,
            color_quant_step=self.color_quant_step,
        )
        if not mesh_result.vertices:
            self._publish_clear_if_needed(msg)
            self.get_logger().warning("No surface triangles generated from voxelized cloud.")
            return

        # geometry_msgs/Point (24B) + ColorRGBA (16B) for each of 3 vertices per triangle ~= 120B/triangle.
        estimated_payload_mb = (mesh_result.triangle_count * 120.0) / (1024.0 * 1024.0)
        if estimated_payload_mb > 10.0:
            self.get_logger().warning(
                f"Large mesh payload estimate (~{estimated_payload_mb:.1f} MB). "
                "If Unity deserialization fails, lower --max-triangles, lower --chunk-max-triangles, "
                "or increase --voxel-size."
            )

        if self.mesh_transport == MESH_TRANSPORT_MARKER_ARRAY:
            full_array = self._build_chunked_marker_array(
                msg=msg,
                vertices=mesh_result.vertices,
                colors=mesh_result.colors,
            )
            diff_array, chunk_count, marker_points = self._build_marker_array_diff(full_array)
            if diff_array.markers:
                self.marker_array_pub.publish(diff_array)

            # Compatibility fallback path: keep a single-marker payload available for
            # runtimes that are temporarily not ingesting MarkerArray transport.
            compat_triangle_cap = min(mesh_result.triangle_count, 60000)
            compat_point_count = compat_triangle_cap * 3
            compat_points = [self._to_point(v) for v in mesh_result.vertices[:compat_point_count]]
            compat_colors = (
                mesh_result.colors[:compat_point_count]
                if mesh_result.colors is not None and len(mesh_result.colors) >= compat_point_count
                else None
            )
            self._last_marker = self._make_triangle_marker(
                msg=msg,
                marker_id=0,
                points=compat_points,
                colors=compat_colors,
                action=Marker.ADD,
            )
            if self.marker_pub.get_subscription_count() > 0:
                self.marker_pub.publish(self._last_marker)

            self._last_marker_array_full = full_array
            self._last_marker_point_count = marker_points
            self._last_chunk_count = chunk_count
        else:
            points = [self._to_point(v) for v in mesh_result.vertices]
            colors = mesh_result.colors
            marker = self._make_triangle_marker(
                msg=msg,
                marker_id=0,
                points=points,
                colors=colors,
                action=Marker.ADD,
            )
            self.marker_pub.publish(marker)
            self._last_marker = marker
            self._last_marker_array_full = None
            self._last_chunk_signatures = {0: self._compute_marker_chunk_signature(marker)}
            self._last_marker_point_count = len(points)
            self._last_chunk_count = 1

            if self.output_obj:
                self._write_obj(self.output_obj, points)

        self._received_once = True
        self._last_signature = signature
        self._last_periodic_republish_time = time.monotonic()
        self._last_mesh_subscriber_count = self._active_mesh_subscriber_count()

        self.get_logger().info(
            f"Mesh updated: raw_points={xyz.shape[0]}, raw_voxels={raw_voxels}, "
            f"used_voxels={voxel_coords.shape[0]}, triangles={mesh_result.triangle_count}, "
            f"marker_points={self._last_marker_point_count}, chunks={self._last_chunk_count}, "
            f"transport={self.mesh_transport}"
        )

    def _publish_clear_if_needed(self, msg: PointCloud2) -> None:
        if self.mesh_transport == MESH_TRANSPORT_MARKER_ARRAY:
            if not self._last_chunk_signatures:
                return
            clear_marker = Marker()
            clear_marker.header.stamp = msg.header.stamp
            clear_marker.header.frame_id = msg.header.frame_id
            clear_marker.ns = "map_mesh"
            clear_marker.id = 0
            clear_marker.type = Marker.TRIANGLE_LIST
            clear_marker.action = Marker.DELETEALL
            clear = MarkerArray(markers=[clear_marker])
            self.marker_array_pub.publish(clear)
            self.marker_pub.publish(clear_marker)
            self._last_marker_array_full = MarkerArray(markers=[])
            self._last_marker = None
            self._last_chunk_signatures = {}
            self._last_marker_point_count = 0
            self._last_chunk_count = 0
            return

        if self._last_marker is None:
            return

        clear_marker = Marker()
        clear_marker.header.stamp = msg.header.stamp
        clear_marker.header.frame_id = msg.header.frame_id
        clear_marker.ns = "map_mesh"
        clear_marker.id = 0
        clear_marker.type = Marker.TRIANGLE_LIST
        clear_marker.action = Marker.DELETEALL
        clear_marker.pose.orientation.w = 1.0
        self.marker_pub.publish(clear_marker)
        self._last_marker = None
        self._last_chunk_signatures = {}
        self._last_marker_point_count = 0
        self._last_chunk_count = 0

    def _active_mesh_subscriber_count(self) -> int:
        try:
            if self.mesh_transport == MESH_TRANSPORT_MARKER_ARRAY:
                return int(self.marker_array_pub.get_subscription_count()) + int(
                    self.marker_pub.get_subscription_count()
                )
            return int(self.marker_pub.get_subscription_count())
        except Exception:
            return -1

    def _on_timer_tick(self) -> None:
        self._service_replay_burst()
        self._republish_latest_mesh()

    def _republish_latest_mesh(self) -> None:
        if self.update_mode not in {"once", "on_change"}:
            return

        sub_count = self._active_mesh_subscriber_count()
        now = time.monotonic()
        should_publish = False
        reason = ""

        previous_sub_count = self._last_mesh_subscriber_count
        if sub_count != previous_sub_count:
            self._last_mesh_subscriber_count = sub_count
            if sub_count > 0 and previous_sub_count <= 0:
                should_publish = True
                reason = "subscriber_change"

        if (
            not should_publish
            and self.on_change_republish_interval > 0.0
            and now - self._last_periodic_republish_time >= self.on_change_republish_interval
        ):
            should_publish = True
            reason = "periodic_keepalive"

        if not should_publish:
            return

        if not self._publish_latest_mesh_once():
            return

        if reason == "subscriber_change":
            self.get_logger().info(
                f"Subscriber count changed to {sub_count}; republished latest mesh "
                f"(points={self._last_marker_point_count}, chunks={self._last_chunk_count}, "
                f"transport={self.mesh_transport})."
            )
        else:
            self.get_logger().debug(
                f"Republished latest mesh keepalive "
                f"(points={self._last_marker_point_count}, chunks={self._last_chunk_count}, "
                f"transport={self.mesh_transport})."
            )

    def _service_replay_burst(self) -> None:
        if self._replay_burst_remaining <= 0:
            return

        now = time.monotonic()
        if now < self._replay_burst_next_time:
            return

        attempt = (self._replay_burst_total - self._replay_burst_remaining) + 1
        snapshot_available = self._publish_latest_mesh_once()
        if snapshot_available:
            self._replay_burst_published += 1
            self.get_logger().info(
                f"[3D-MAP][replay_burst] source={self._replay_burst_source}, "
                f"request_id={self._replay_burst_request_id or '-'}, publish={attempt}/{self._replay_burst_total}, "
                f"snapshot_available=True, points={self._last_marker_point_count}, chunks={self._last_chunk_count}."
            )
        else:
            self.get_logger().warning(
                f"[3D-MAP][replay_burst] source={self._replay_burst_source}, "
                f"request_id={self._replay_burst_request_id or '-'}, publish={attempt}/{self._replay_burst_total}, "
                "snapshot_available=False (waiting for first mesh snapshot)."
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

    def _publish_latest_mesh_once(self) -> bool:
        if self.mesh_transport == MESH_TRANSPORT_MARKER_ARRAY:
            if self._last_marker_array_full is None:
                return False
            self.marker_array_pub.publish(self._last_marker_array_full)
            if self._last_marker is not None and self.marker_pub.get_subscription_count() > 0:
                self.marker_pub.publish(self._last_marker)
        else:
            if self._last_marker is None:
                return False
            self.marker_pub.publish(self._last_marker)
        self._last_periodic_republish_time = time.monotonic()
        return True

    def _on_sdk_replay_request(self, msg: String) -> None:
        request_id = self._extract_request_id(msg)
        self._arm_replay_burst("request", request_id)

    def _on_sdk_replay_end(self, msg: String) -> None:
        request_id = self._extract_request_id(msg)
        self._arm_replay_burst("replay_end", request_id)

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

    def _build_chunked_marker_array(
        self,
        msg: PointCloud2,
        vertices: Sequence[Tuple[float, float, float]],
        colors: Optional[Sequence[Tuple[int, int, int]]],
    ) -> MarkerArray:
        point_count = len(vertices)
        if point_count < 3:
            return MarkerArray(markers=[])

        triangle_count = point_count // 3
        max_chunk_triangles = max(1, self.chunk_max_triangles)
        markers: List[Marker] = []
        cursor = 0
        marker_id = 0
        while cursor < triangle_count:
            chunk_triangles = min(max_chunk_triangles, triangle_count - cursor)
            start = cursor * 3
            end = (cursor + chunk_triangles) * 3
            chunk_points = [self._to_point(v) for v in vertices[start:end]]
            chunk_colors = colors[start:end] if colors is not None and len(colors) >= end else None
            markers.append(
                self._make_triangle_marker(
                    msg=msg,
                    marker_id=marker_id,
                    points=chunk_points,
                    colors=chunk_colors,
                    action=Marker.ADD,
                )
            )
            marker_id += 1
            cursor += chunk_triangles

        return MarkerArray(markers=markers)

    def _build_marker_array_diff(self, full_array: MarkerArray) -> Tuple[MarkerArray, int, int]:
        new_markers = full_array.markers if full_array is not None else []
        old_signatures = self._last_chunk_signatures
        new_signatures: Dict[int, ChunkSignature] = {}
        changed_markers: List[Marker] = []

        for marker in new_markers:
            marker_id = int(marker.id)
            signature = self._compute_marker_chunk_signature(marker)
            new_signatures[marker_id] = signature
            old_signature = old_signatures.get(marker_id)
            if old_signature is None or old_signature != signature:
                changed_markers.append(marker)

        removed_ids = [marker_id for marker_id in old_signatures.keys() if marker_id not in new_signatures]
        for marker_id in sorted(removed_ids):
            delete_marker = Marker()
            if new_markers:
                delete_marker.header = new_markers[0].header
            delete_marker.ns = "map_mesh"
            delete_marker.id = int(marker_id)
            delete_marker.type = Marker.TRIANGLE_LIST
            delete_marker.action = Marker.DELETE
            delete_marker.pose.orientation.w = 1.0
            delete_marker.scale.x = 1.0
            delete_marker.scale.y = 1.0
            delete_marker.scale.z = 1.0
            changed_markers.append(delete_marker)

        self._last_chunk_signatures = new_signatures
        marker_points = sum(len(marker.points) for marker in new_markers)
        chunk_count = len(new_markers)
        return MarkerArray(markers=changed_markers), chunk_count, marker_points

    @staticmethod
    def _compute_marker_chunk_signature(marker: Marker) -> ChunkSignature:
        points = marker.points if marker is not None and marker.points is not None else []
        colors = marker.colors if marker is not None and marker.colors is not None else []
        point_count = len(points)
        color_count = len(colors)

        if point_count <= 0:
            return ChunkSignature(point_count=0, color_count=color_count, sample_hash=0)

        sample_budget = 48
        step = max(1, point_count // sample_budget)
        sample_hash = 2166136261
        for idx in range(0, point_count, step):
            p = points[idx]
            qx = int(round(float(p.x) * 10000.0))
            qy = int(round(float(p.y) * 10000.0))
            qz = int(round(float(p.z) * 10000.0))
            sample_hash ^= qx & 0xFFFFFFFF
            sample_hash = (sample_hash * 16777619) & 0xFFFFFFFF
            sample_hash ^= qy & 0xFFFFFFFF
            sample_hash = (sample_hash * 16777619) & 0xFFFFFFFF
            sample_hash ^= qz & 0xFFFFFFFF
            sample_hash = (sample_hash * 16777619) & 0xFFFFFFFF
            if color_count > idx:
                c = colors[idx]
                packed = (
                    (int(round(float(c.r) * 255.0)) & 0xFF) << 16
                ) | (
                    (int(round(float(c.g) * 255.0)) & 0xFF) << 8
                ) | (
                    int(round(float(c.b) * 255.0)) & 0xFF
                )
                sample_hash ^= packed & 0xFFFFFFFF
                sample_hash = (sample_hash * 16777619) & 0xFFFFFFFF

        return ChunkSignature(
            point_count=point_count,
            color_count=color_count,
            sample_hash=int(sample_hash),
        )

    @staticmethod
    def _compute_cloud_signature(msg: PointCloud2) -> Tuple[int, int, int, int, int, int, str, int]:
        frame_id = str(getattr(msg.header, "frame_id", "") or "")
        data = msg.data if msg.data is not None else b""
        data_len = len(data)
        sample_hash = PointCloudToVoxelMeshNode._sample_bytes_hash(data)

        return (
            int(msg.width),
            int(msg.height),
            int(msg.row_step),
            int(msg.point_step),
            int(getattr(msg, "is_bigendian", False)),
            data_len,
            frame_id,
            sample_hash,
        )

    @staticmethod
    def _sample_bytes_hash(data: Sequence[int], max_samples: int = 96) -> int:
        data_len = len(data)
        if data_len <= 0:
            return 0

        sample_budget = max(1, int(max_samples))
        step = max(1, data_len // sample_budget)

        hash_value = 2166136261
        for idx in range(0, data_len, step):
            hash_value ^= int(data[idx]) & 0xFF
            hash_value = (hash_value * 16777619) & 0xFFFFFFFF

        hash_value ^= int(data[data_len - 1]) & 0xFF
        hash_value = (hash_value * 16777619) & 0xFFFFFFFF
        return int(hash_value)

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

        if row_step == width * point_step:
            total = width * height
            usable = min(total, raw.size // point_step)
            if usable <= 0:
                return np.empty((0, 3), dtype=np.float32), None
            points = raw[: usable * point_step].reshape(usable, point_step)
        else:
            row_starts = np.arange(height, dtype=np.int64) * row_step
            col_starts = np.arange(width, dtype=np.int64) * point_step
            offsets = (row_starts[:, None] + col_starts[None, :]).ravel()
            offsets = offsets[offsets + point_step <= raw.size]
            usable = offsets.size
            if usable <= 0:
                return np.empty((0, 3), dtype=np.float32), None
            byte_indices = offsets[:, None] + np.arange(point_step, dtype=np.int64)[None, :]
            points = raw[byte_indices]

        xyz = np.empty((usable, 3), dtype=np.float32)
        for ax, off in enumerate((x_off, y_off, z_off)):
            col_bytes = points[:, off : off + 4].copy()
            xyz[:, ax] = col_bytes.view(np.float32).ravel()

        valid = np.all(np.isfinite(xyz), axis=1)
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float32), None
        xyz = xyz[valid]

        if not has_color:
            return xyz, None

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
                    points,
                    int(field.offset),
                    field.datatype,
                )

        rgb = rgb[valid]
        return xyz, rgb

    @staticmethod
    def _extract_channel_vectorized(
        points: np.ndarray,
        offset: int,
        datatype: int,
    ) -> np.ndarray:
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

    def _make_triangle_marker(
        self,
        msg: PointCloud2,
        marker_id: int,
        points: List[Point],
        colors: Optional[Sequence[Tuple[int, int, int]]],
        action: int,
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = msg.header.frame_id
        marker.ns = "map_mesh"
        marker.id = int(marker_id)
        marker.type = Marker.TRIANGLE_LIST
        marker.action = int(action)
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
    def _to_point(v: Tuple[float, float, float]) -> Point:
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
        description="Convert PointCloud2 map to mesh marker payload for Unity/RViz validation."
    )
    parser.add_argument("--cloud-topic", default="/map_3d", help="Input PointCloud2 topic.")
    parser.add_argument("--mesh-topic", default="/map_3d_mesh", help="Output Marker topic.")
    parser.add_argument(
        "--mesh-array-topic",
        default="/map_3d_mesh_array",
        help="Output MarkerArray topic for chunked transport.",
    )
    parser.add_argument(
        "--mesh-transport",
        choices=(MESH_TRANSPORT_MARKER, MESH_TRANSPORT_MARKER_ARRAY),
        default=MESH_TRANSPORT_MARKER,
        help="Mesh transport format: marker|marker_array (default: marker).",
    )
    parser.add_argument(
        "--chunk-max-triangles",
        type=int,
        default=3000,
        help="Chunk triangle cap when --mesh-transport marker_array (default: 3000).",
    )
    parser.add_argument("--voxel-size", type=float, default=0.10, help="Voxel size in meters.")
    parser.add_argument("--max-voxels", type=int, default=60000, help="Hard cap for voxel cells (0=unlimited).")
    parser.add_argument(
        "--max-triangles",
        type=int,
        default=60000,
        help="Hard cap for triangles in output mesh (0=unlimited).",
    )
    parser.add_argument(
        "--update-policy",
        choices=("snapshot", "periodic", "continuous"),
        default="snapshot",
        help=(
            "snapshot: convert once, periodic: convert on change with republish timer, "
            "continuous: convert every cloud."
        ),
    )
    parser.add_argument(
        "--update-mode",
        choices=("once", "on_change", "continuous"),
        default="",
        help="[Deprecated] Legacy alias for converter update mode.",
    )
    parser.add_argument(
        "--on-change-republish-interval",
        type=float,
        default=0.0,
        help="Periodic republish interval for latest mesh (seconds, 0 disables periodic keepalive).",
    )
    parser.add_argument(
        "--color-quant-step",
        type=int,
        default=8,
        help="Per-region color quantization step (default: 8).",
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


def _resolve_update_mode(policy: str, legacy_mode: str) -> Tuple[str, str]:
    normalized_policy = str(policy or "snapshot").strip().lower()
    normalized_legacy = str(legacy_mode or "").strip().lower()

    if normalized_legacy:
        if normalized_legacy == "once":
            return "snapshot", "once"
        if normalized_legacy == "on_change":
            return "periodic", "on_change"
        return "continuous", "continuous"

    if normalized_policy == "periodic":
        return "periodic", "on_change"
    if normalized_policy == "continuous":
        return "continuous", "continuous"
    return "snapshot", "once"


def _resolve_republish_interval(policy: str, configured_interval: float) -> float:
    if policy == "continuous":
        return 0.0
    if configured_interval > 0.0:
        return max(0.5, float(configured_interval))
    if policy == "periodic":
        return 2.0
    return 0.0


def _resolve_mesh_transport(raw: str) -> Tuple[str, bool]:
    normalized = str(raw or "").strip().lower()
    requested_marker_array = normalized == MESH_TRANSPORT_MARKER_ARRAY
    return MESH_TRANSPORT_MARKER, requested_marker_array


def main() -> None:
    args = build_parser().parse_args()

    update_policy, update_mode = _resolve_update_mode(
        policy=args.update_policy,
        legacy_mode=args.update_mode,
    )
    if args.update_mode:
        print(
            "[3D-MAP] WARNING: --update-mode is deprecated. Use --update-policy "
            "{snapshot|periodic|continuous}."
        )

    mesh_transport, requested_marker_array = _resolve_mesh_transport(args.mesh_transport)
    if requested_marker_array:
        print(
            "[3D-MAP] WARNING: Marker-only rollback active. Ignoring marker_array transport and forcing marker transport."
        )
    republish_interval = _resolve_republish_interval(
        policy=update_policy,
        configured_interval=float(args.on_change_republish_interval),
    )

    rclpy.init()
    node = PointCloudToVoxelMeshNode(
        cloud_topic=args.cloud_topic,
        marker_topic=args.mesh_topic,
        marker_array_topic=args.mesh_array_topic,
        mesh_transport=mesh_transport,
        chunk_max_triangles=args.chunk_max_triangles,
        voxel_size=args.voxel_size,
        max_voxels=args.max_voxels,
        max_triangles=args.max_triangles,
        update_mode=update_mode,
        update_policy=update_policy,
        color_quant_step=args.color_quant_step,
        color_rgb=(args.color_r, args.color_g, args.color_b),
        alpha=args.alpha,
        output_obj=args.output_obj.strip() or None,
        marker_text=args.marker_text.strip() or "map_mesh",
        marker_mesh_resource=args.marker_mesh_resource.strip() or "mesh://map_mesh",
        on_change_republish_interval=republish_interval,
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
