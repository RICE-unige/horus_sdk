#!/usr/bin/env python3
"""Publish realistic synthetic ROS 2 load for HORUS experiments."""

from __future__ import annotations

import argparse
import io
import json
import math
import os
from pathlib import Path
import signal
import struct
import sys
import time
from typing import Any

SDK_ROOT = Path(__file__).resolve().parents[3]
PYTHON_ROOT = SDK_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

try:
    from PIL import Image as PillowImage
except Exception:  # pragma: no cover - optional runtime acceleration.
    PillowImage = None

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header, String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker

from horus.experiments.metrics import CsvMetricWriter, NdjsonEventWriter, default_metrics_path
from horus.experiments.workloads import WorkloadConfig, load_workload_config


STOP_REQUESTED = False
CAMERA_ANIMATION_FRAMES = 4


def request_stop(_signum: int, _frame: object) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True


SOURCE_FIELDS = (
    "robot_id",
    "operator_id",
    "stream",
    "topic",
    "seq",
    "source_hz",
    "payload_bytes",
    "transport",
    "pose_x",
    "pose_y",
    "pose_z",
    "yaw",
    "points",
    "point_step",
    "resolution",
    "encoding",
    "representation",
    "vertices",
    "triangles",
    "fallback",
    "notes",
)

OPERATOR_FIELDS = (
    "operator_id",
    "role",
    "state",
    "robot_id",
    "event",
    "seq",
    "payload_bytes",
    "lease_version",
)

FAILURE_FIELDS = (
    "event",
    "target",
    "seq",
    "elapsed_s",
    "duration_ms",
    "notes",
)

POINT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
]


def parse_resolution(value: str) -> tuple[int, int]:
    parts = str(value or "").lower().split("x", 1)
    if len(parts) != 2:
        return 640, 480
    try:
        return max(1, int(parts[0])), max(1, int(parts[1]))
    except ValueError:
        return 640, 480


def clamp_rate(value: float, fallback: float) -> float:
    try:
        rate = float(value)
    except (TypeError, ValueError):
        return fallback
    return rate if rate > 0.0 else fallback


def camera_is_compressed(workload: WorkloadConfig) -> bool:
    encoding = str(workload.camera.encoding or "").strip().lower()
    return encoding in {"compressed", "jpeg", "jpg"}


def camera_uses_webrtc(workload: WorkloadConfig) -> bool:
    transport = str(workload.transport or "").strip().lower()
    encoding = str(workload.camera.encoding or "").strip().lower()
    return "webrtc" in transport or encoding in {"h264", "webrtc"}


def robot_pointcloud_enabled(workload: WorkloadConfig) -> bool:
    extra = dict(getattr(workload, "extra", {}) or {})
    nested = extra.get("extra")
    if isinstance(nested, dict):
        extra.update(nested)
    value = extra.get("robot_pointcloud", True)
    return value is not False and str(value).strip().lower() not in {"0", "false", "no", "off"}


def navigation_enabled(workload: WorkloadConfig) -> bool:
    extra = dict(getattr(workload, "extra", {}) or {})
    nested = extra.get("extra")
    if isinstance(nested, dict):
        extra.update(nested)
    value = extra.get("navigation", False)
    return value is True or str(value).strip().lower() in {"1", "true", "yes", "on"}


def make_header(node: Node, frame_id: str) -> Header:
    header = Header()
    header.stamp = node.get_clock().now().to_msg()
    header.frame_id = frame_id
    return header


class SyntheticWorkloadNode(Node):
    def __init__(self, workload: WorkloadConfig, metrics_csv: str | None) -> None:
        super().__init__("horus_synthetic_workload")
        self.workload = workload
        self.navigation_enabled = navigation_enabled(workload)
        self.control_load_enabled = self._control_load_enabled()
        self.started = time.monotonic()
        self.sequence = 0
        self.map_sequence = 0
        self.operator_sequence = 0
        self.robot_names = [f"exp_robot_{idx}" for idx in range(max(1, workload.robot_count))]
        self.width, self.height = parse_resolution(workload.camera.resolution)
        self.compressed_camera = camera_is_compressed(workload)
        self.webrtc_camera = camera_uses_webrtc(workload)
        self.raw_camera_frames = [
            self._make_camera_payload(self.width, self.height, phase)
            for phase in range(CAMERA_ANIMATION_FRAMES)
        ]
        self.jpeg_camera_frames = self._make_jpeg_frames()
        self.pointcloud_payloads = {
            name: self._make_pointcloud_payload(max(0, workload.pointcloud.points), max(16, workload.pointcloud.point_step), idx)
            for idx, name in enumerate(self.robot_names)
        }
        self.map_pointcloud_payload = self._make_pointcloud_payload(
            max(0, workload.pointcloud.points),
            max(16, workload.pointcloud.point_step),
            99,
            map_cloud=True,
        )
        self.mesh_chunks = self._make_mesh_chunks()

        metrics_path = Path(metrics_csv) if metrics_csv else default_metrics_path("source_metrics.csv")
        self.metrics_writer = (
            CsvMetricWriter(
                metrics_path,
                run_id=os.getenv("HORUS_EXPERIMENT_RUN_ID", "manual"),
                experiment=os.getenv("HORUS_EXPERIMENT", workload.experiment),
                condition=os.getenv("HORUS_EXPERIMENT_CONDITION", workload.condition),
                fieldnames=SOURCE_FIELDS,
            )
            if metrics_path is not None
            else None
        )
        self.operator_writer = self._make_csv_writer("operator_metrics.csv", OPERATOR_FIELDS)
        self.failure_writer = self._make_csv_writer("failure_metrics.csv", FAILURE_FIELDS)
        events_path = default_metrics_path("source_events.ndjson")
        self.event_writer = (
            NdjsonEventWriter(
                events_path,
                run_id=os.getenv("HORUS_EXPERIMENT_RUN_ID", "manual"),
                experiment=os.getenv("HORUS_EXPERIMENT", workload.experiment),
                condition=os.getenv("HORUS_EXPERIMENT_CONDITION", workload.condition),
                source="synthetic_workload",
            )
            if events_path is not None
            else None
        )
        for writer in (self.metrics_writer, self.operator_writer, self.failure_writer, self.event_writer):
            if writer is not None:
                writer.__enter__()

        self.tf_pub = self.create_publisher(TFMessage, "/tf", 20)
        self.presence_pub = self.create_publisher(String, "/horus/multi_operator_presence", 10)
        self.lease_state_pub = self.create_publisher(String, "/horus/multi_operator/control_lease_state", 10)
        self.camera_pubs = self._create_camera_publishers()
        self.pointcloud_pubs = {
            name: self.create_publisher(PointCloud2, f"/{name}/points", 10)
            for name in self.robot_names
            if robot_pointcloud_enabled(workload) and workload.pointcloud.points > 0 and workload.pointcloud.hz > 0
        }
        self.map_points_pub = (
            self.create_publisher(PointCloud2, "/horus/experiment/map_points", 10)
            if "pointcloud" in str(workload.map.representation or "").lower()
            else None
        )
        self.mesh_pub = (
            self.create_publisher(Marker, "/horus/experiment/map_mesh", 10)
            if self.mesh_chunks
            else None
        )
        self.odom_pubs = {name: self.create_publisher(Odometry, f"/{name}/odom", 10) for name in self.robot_names}
        self.path_pubs = (
            {name: self.create_publisher(PathMsg, f"/{name}/global_path", 10) for name in self.robot_names}
            if self.navigation_enabled
            else {}
        )
        self.goal_status_pubs = (
            {name: self.create_publisher(String, f"/{name}/goal_status", 10) for name in self.robot_names}
            if self.control_load_enabled
            else {}
        )
        self.command_pubs = (
            {name: self.create_publisher(Twist, f"/{name}/cmd_vel", 10) for name in self.robot_names}
            if self.control_load_enabled
            else {}
        )
        self.goal_pubs = (
            {name: self.create_publisher(PoseStamped, f"/{name}/goal_pose", 10) for name in self.robot_names}
            if self.control_load_enabled
            else {}
        )
        self.paths = (
            {name: PathMsg(header=make_header(self, "map")) for name in self.robot_names}
            if self.navigation_enabled
            else {}
        )

        self.create_timer(1.0 / 20.0, self.publish_state)
        camera_rate = clamp_rate(workload.camera.fps, 1.0)
        for spec in self.camera_pubs:
            self.publish_camera(spec)
            self.create_timer(1.0 / camera_rate, lambda spec=spec: self.publish_camera(spec))
        if self.pointcloud_pubs:
            self.publish_pointclouds()
            self.create_timer(1.0 / clamp_rate(workload.pointcloud.hz, 1.0), self.publish_pointclouds)
        if self.map_points_pub is not None:
            self.publish_map_points()
            self.create_timer(1.0 / clamp_rate(workload.map.hz, 1.0), self.publish_map_points)
        if self.mesh_pub is not None:
            self.publish_mesh_snapshot()
            self.create_timer(1.0 / clamp_rate(workload.map.hz, 1.0), self.publish_mesh_snapshot)
        if workload.operator_count > 1:
            self.publish_operator_emulation()
            self.create_timer(1.0, self.publish_operator_emulation)
        if self.control_load_enabled:
            self.publish_control_load()
            self.create_timer(0.2, self.publish_control_load)

        self.write_event(
            "workload_started",
            {
                "robots": self.robot_names,
                "camera_streams": len(self.camera_pubs),
                "compressed_camera": self.compressed_camera,
                "jpeg_encoder": "pillow" if self.jpeg_camera_frames else "none",
                "pointcloud_points_per_robot": workload.pointcloud.points,
                "mesh_chunks": len(self.mesh_chunks),
                "mesh_triangles": workload.map.triangles,
                "navigation_enabled": self.navigation_enabled,
                "control_load_enabled": self.control_load_enabled,
            },
        )

    def _make_csv_writer(self, name: str, fields: tuple[str, ...]) -> CsvMetricWriter | None:
        path = default_metrics_path(name)
        if path is None:
            return None
        return CsvMetricWriter(
            path,
            run_id=os.getenv("HORUS_EXPERIMENT_RUN_ID", "manual"),
            experiment=os.getenv("HORUS_EXPERIMENT", self.workload.experiment),
            condition=os.getenv("HORUS_EXPERIMENT_CONDITION", self.workload.condition),
            fieldnames=fields,
        )

    def destroy_node(self) -> bool:
        self.write_event("workload_stopped", {"elapsed_s": time.monotonic() - self.started})
        for writer_name in ("metrics_writer", "operator_writer", "failure_writer", "event_writer"):
            writer = getattr(self, writer_name, None)
            if writer is not None:
                writer.__exit__(None, None, None)
                setattr(self, writer_name, None)
        return super().destroy_node()

    def _create_camera_publishers(self) -> list[dict[str, Any]]:
        specs: list[dict[str, Any]] = []
        for stream_index in range(max(0, self.workload.camera.streams)):
            robot_index = stream_index % len(self.robot_names)
            camera_index = stream_index // len(self.robot_names)
            robot_name = self.robot_names[robot_index]
            base_topic = f"/{robot_name}/camera_{camera_index}/image_raw"
            if self.compressed_camera:
                topic = f"{base_topic}/compressed"
                publisher = self.create_publisher(CompressedImage, topic, 10)
                msg_type = "compressed"
            else:
                topic = base_topic
                publisher = self.create_publisher(Image, topic, 10)
                msg_type = "raw"
            specs.append(
                {
                    "robot": robot_name,
                    "index": camera_index,
                    "stream_index": stream_index,
                    "frame_seq": 0,
                    "topic": topic,
                    "publisher": publisher,
                    "msg_type": msg_type,
                }
            )
        return specs

    def _make_camera_payload(self, width: int, height: int, phase: int) -> bytes:
        data = bytearray(width * height * 3)
        for y in range(height):
            row_offset = y * width * 3
            band = ((y // max(1, height // 12)) + phase) % 2
            for x in range(width):
                offset = row_offset + x * 3
                checker = ((x // 32) + (y // 32) + phase) % 2
                sweep = ((x + phase * max(1, width // CAMERA_ANIMATION_FRAMES)) % max(1, width)) < max(8, width // 18)
                data[offset + 0] = (x * 255 // max(1, width - 1) + phase * 13 + (80 if sweep else 0)) % 256
                data[offset + 1] = (y * 255 // max(1, height - 1) + phase * 7 + (30 if sweep else 0)) % 256
                data[offset + 2] = min(255, 70 + 55 * checker + 30 * band + (95 if sweep else 0))
        return bytes(data)

    def _make_jpeg_frames(self) -> list[bytes]:
        if PillowImage is None:
            return []
        frames: list[bytes] = []
        for payload in self.raw_camera_frames:
            image = PillowImage.frombytes("RGB", (self.width, self.height), payload)
            buffer = io.BytesIO()
            image.save(buffer, format="JPEG", quality=82, optimize=False)
            frames.append(buffer.getvalue())
        return frames

    def _make_pointcloud_payload(self, points: int, point_step: int, seed: int, map_cloud: bool = False) -> bytes:
        if points <= 0:
            return b""
        payload = bytearray(points * point_step)
        grid = max(1, int(math.sqrt(points)))
        for i in range(points):
            gx = i % grid
            gy = i // grid
            angle = (i * 0.61803398875 + seed * 0.17) % (math.pi * 2.0)
            radius = 0.25 + (gx / grid) * (5.0 if map_cloud else 2.2)
            x = math.cos(angle) * radius + (gx / grid - 0.5) * 1.5
            y = math.sin(angle) * radius + (gy / grid - 0.5) * 1.5
            z = 0.04 * math.sin(gx * 0.11 + seed) + (0.02 * (gy % 7))
            if map_cloud:
                z += 0.25 * math.sin((gx + seed) * 0.035) * math.cos(gy * 0.029)
            red = int(90 + 100 * (gx / grid)) & 0xFF
            green = int(120 + 90 * ((gy % grid) / grid)) & 0xFF
            blue = int(170 + 40 * (0.5 + 0.5 * math.sin(angle))) & 0xFF
            rgb = (red << 16) | (green << 8) | blue
            struct.pack_into("<fffI", payload, i * point_step, x, y, z, rgb)
        return bytes(payload)

    def _make_mesh_chunks(self) -> list[Marker]:
        representation = str(self.workload.map.representation or "none").lower()
        triangles = max(0, int(self.workload.map.triangles or 0))
        if representation == "none" or triangles <= 0:
            return []
        chunk_count = max(1, int(self.workload.map.chunks or 1))
        chunks: list[Marker] = []
        side = max(1, int(math.sqrt(triangles)))
        size = 0.075
        for chunk_index in range(chunk_count):
            start = chunk_index * triangles // chunk_count
            end = (chunk_index + 1) * triangles // chunk_count
            marker = Marker()
            marker.ns = "horus_experiment_chunks"
            marker.id = chunk_index
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color = ColorRGBA(r=0.35, g=0.72, b=0.82, a=0.95)
            for i in range(start, end):
                x = (i % side) * size - side * size * 0.5
                z = (i // side) * size - side * size * 0.25
                y = 0.16 * math.sin(x * 0.8) * math.cos(z * 0.5)
                marker.points.append(Point(x=x, y=y, z=z))
                marker.points.append(Point(x=x + size, y=y + 0.03 * math.sin(i), z=z))
                marker.points.append(Point(x=x, y=y + 0.03 * math.cos(i), z=z + size))
            chunks.append(marker)
        return chunks

    def write_metric(self, **row: Any) -> None:
        if self.metrics_writer is not None:
            self.metrics_writer.write(row)

    def write_operator_metric(self, **row: Any) -> None:
        if self.operator_writer is not None:
            self.operator_writer.write(row)

    def write_failure_metric(self, **row: Any) -> None:
        if self.failure_writer is not None:
            self.failure_writer.write(row)

    def write_event(self, name: str, fields: dict[str, Any]) -> None:
        if self.event_writer is not None:
            payload = dict(fields)
            payload["name"] = name
            self.event_writer.write(payload)

    def robot_pose(self, robot_index: int, elapsed: float) -> tuple[float, float, float, float]:
        radius = 1.0 + robot_index * 0.35
        yaw = elapsed * (0.18 + robot_index * 0.015) + robot_index * 0.7
        x = math.cos(yaw) * radius
        y = math.sin(yaw) * radius
        z = 0.55 + 0.08 * math.sin(elapsed * 0.4 + robot_index) if "drone" in self.workload.robot_profile else 0.0
        return x, y, z, yaw

    def publish_state(self) -> None:
        elapsed = time.monotonic() - self.started
        transforms = []
        for robot_index, robot_name in enumerate(self.robot_names):
            x, y, z, yaw = self.robot_pose(robot_index, elapsed)
            transform = TransformStamped()
            transform.header = make_header(self, "map")
            transform.child_frame_id = f"{robot_name}/base_link"
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.z = math.sin(yaw * 0.5)
            transform.transform.rotation.w = math.cos(yaw * 0.5)
            transforms.append(transform)
            self.publish_odom(robot_name, x, y, z, yaw)
        self.tf_pub.publish(TFMessage(transforms=transforms))
        self.write_metric(
            robot_id="fleet",
            stream="tf",
            topic="/tf",
            seq=self.sequence,
            source_hz=20,
            payload_bytes=len(transforms) * 112,
            transport="ros2",
            yaw=elapsed * 0.18,
        )

    def publish_odom(self, robot_name: str, x: float, y: float, z: float, yaw: float) -> None:
        odom = Odometry()
        odom.header = make_header(self, "map")
        odom.child_frame_id = f"{robot_name}/base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.z = math.sin(yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(yaw * 0.5)
        odom.twist.twist.linear.x = 0.25
        odom.twist.twist.angular.z = 0.18
        self.odom_pubs[robot_name].publish(odom)
        if not self.navigation_enabled:
            return
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        path = self.paths[robot_name]
        path.header = odom.header
        path.poses.append(pose)
        if len(path.poses) > 180:
            path.poses = path.poses[-180:]
        self.path_pubs[robot_name].publish(path)

    def publish_camera(self, spec: dict[str, Any]) -> None:
        frame_seq = int(spec.get("frame_seq", 0)) + 1
        spec["frame_seq"] = frame_seq
        self.sequence += 1
        phase = (frame_seq + int(spec.get("stream_index", 0))) % len(self.raw_camera_frames)
        robot_name = spec["robot"]
        camera_index = spec["index"]
        frame_id = f"{robot_name}/camera_{camera_index}_link"
        if spec["msg_type"] == "compressed" and self.jpeg_camera_frames:
            payload = self.jpeg_camera_frames[phase]
            msg = CompressedImage()
            msg.header = make_header(self, frame_id)
            msg.format = "jpeg"
            msg.data = payload
            spec["publisher"].publish(msg)
            encoding = "jpeg"
            fallback = False
        else:
            payload = self.raw_camera_frames[phase]
            msg = Image()
            msg.header = make_header(self, frame_id)
            msg.height = self.height
            msg.width = self.width
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            msg.step = self.width * 3
            msg.data = payload
            spec["publisher"].publish(msg)
            encoding = "rgb8"
            fallback = spec["msg_type"] == "compressed"
        self.write_metric(
            robot_id=robot_name,
            stream="camera",
            topic=spec["topic"],
            seq=frame_seq,
            source_hz=self.workload.camera.fps,
            payload_bytes=len(payload),
            transport="webrtc_source" if self.webrtc_camera else self.workload.transport,
            resolution=f"{self.width}x{self.height}",
            encoding=encoding,
            fallback=str(fallback).lower(),
            notes=f"frame_phase={phase};jpeg_encoder_missing" if fallback else f"frame_phase={phase}",
        )

    def publish_pointclouds(self) -> None:
        for robot_name, publisher in self.pointcloud_pubs.items():
            self.sequence += 1
            payload = self.pointcloud_payloads[robot_name]
            msg = self.make_pointcloud(f"{robot_name}/lidar_link", self.workload.pointcloud.points, payload)
            publisher.publish(msg)
            self.write_metric(
                robot_id=robot_name,
                stream="pointcloud",
                topic=f"/{robot_name}/points",
                seq=self.sequence,
                source_hz=self.workload.pointcloud.hz,
                payload_bytes=len(payload),
                transport="ros2",
                points=msg.width,
                point_step=msg.point_step,
            )

    def publish_map_points(self) -> None:
        if self.map_points_pub is None:
            return
        self.sequence += 1
        msg = self.make_pointcloud("map", self.workload.pointcloud.points, self.map_pointcloud_payload)
        self.map_points_pub.publish(msg)
        self.write_metric(
            robot_id="world",
            stream="map_pointcloud",
            topic="/horus/experiment/map_points",
            seq=self.sequence,
            source_hz=self.workload.map.hz,
            payload_bytes=len(self.map_pointcloud_payload),
            transport="ros2",
            points=msg.width,
            point_step=msg.point_step,
            representation="pointcloud",
        )

    def make_pointcloud(self, frame_id: str, points: int, payload: bytes) -> PointCloud2:
        msg = PointCloud2()
        msg.header = make_header(self, frame_id)
        msg.height = 1
        msg.width = max(0, points)
        msg.is_bigendian = False
        msg.point_step = max(16, self.workload.pointcloud.point_step)
        msg.row_step = msg.width * msg.point_step
        msg.is_dense = True
        msg.fields = POINT_FIELDS
        msg.data = payload
        return msg

    def publish_mesh_snapshot(self) -> None:
        if self.mesh_pub is None:
            return
        incremental = "incremental" in str(self.workload.map_profile or "").lower()
        if self.map_sequence == 0:
            delete_all = Marker()
            delete_all.header = make_header(self, "map")
            delete_all.ns = "horus_experiment_chunks"
            delete_all.id = 0
            delete_all.action = Marker.DELETEALL
            self.mesh_pub.publish(delete_all)
        if incremental and self.map_sequence > 0:
            markers = [self.mesh_chunks[(self.map_sequence - 1) % len(self.mesh_chunks)]]
        else:
            markers = self.mesh_chunks
        for marker in markers:
            self.sequence += 1
            marker.header = make_header(self, "map")
            self.mesh_pub.publish(marker)
            self.write_metric(
                robot_id="world",
                stream="mesh",
                topic="/horus/experiment/map_mesh",
                seq=self.sequence,
                source_hz=self.workload.map.hz,
                payload_bytes=len(marker.points) * 24,
                transport="ros2",
                representation=self.workload.map.representation,
                vertices=len(marker.points),
                triangles=len(marker.points) // 3,
            )
        self.map_sequence += 1

    def publish_control_load(self) -> None:
        elapsed = time.monotonic() - self.started
        for robot_index, robot_name in enumerate(self.robot_names):
            if robot_name not in self.command_pubs:
                continue
            twist = Twist()
            twist.linear.x = 0.25 * math.sin(elapsed + robot_index)
            twist.linear.y = 0.08 * math.cos(elapsed * 0.7 + robot_index)
            twist.angular.z = 0.3 * math.sin(elapsed * 0.5 + robot_index)
            self.command_pubs[robot_name].publish(twist)
            goal = PoseStamped()
            goal.header = make_header(self, "map")
            goal.pose.position.x = 1.2 * math.cos(elapsed * 0.2 + robot_index)
            goal.pose.position.y = 1.2 * math.sin(elapsed * 0.2 + robot_index)
            goal.pose.position.z = 0.7 if "drone" in self.workload.robot_profile else 0.0
            goal.pose.orientation.w = 1.0
            self.goal_pubs[robot_name].publish(goal)
            status = String()
            status.data = json.dumps(
                {
                    "robot_name": robot_name,
                    "state": "tracking",
                    "distance_remaining_m": round(0.35 + 0.2 * abs(math.sin(elapsed)), 3),
                    "ts_unix_ms": int(time.time() * 1000),
                },
                separators=(",", ":"),
            )
            self.goal_status_pubs[robot_name].publish(status)
            self.write_metric(
                robot_id=robot_name,
                stream="control",
                topic=f"/{robot_name}/cmd_vel",
                seq=self.sequence,
                source_hz=5,
                payload_bytes=96,
                transport="ros2",
            )

    def publish_operator_emulation(self) -> None:
        self.operator_sequence += 1
        session_id = f"experiment-{os.getenv('HORUS_EXPERIMENT_RUN_ID', 'manual')}"
        roles = ["host", "joiner", "joiner", "private"]
        states = ["workspace_ready_host", "workspace_synced", "registry_synced", "private_workspace_ready"]
        now_ms = int(time.time() * 1000)
        for index in range(max(1, self.workload.operator_count)):
            operator_id = f"experiment_operator_{index}"
            payload = {
                "app_id": operator_id,
                "ip": f"10.10.0.{20 + index}",
                "role": roles[index % len(roles)],
                "state": states[index % len(states)],
                "session_id": session_id,
                "workspace_active": True,
                "ts_unix_ms": now_ms,
            }
            msg = String()
            msg.data = json.dumps(payload, separators=(",", ":"))
            self.presence_pub.publish(msg)
            self.write_operator_metric(
                operator_id=operator_id,
                role=payload["role"],
                state=payload["state"],
                robot_id="",
                event="presence",
                seq=self.operator_sequence,
                payload_bytes=len(msg.data),
            )

        leases = []
        for robot_index, robot_name in enumerate(self.robot_names):
            holder_index = robot_index % max(1, self.workload.operator_count)
            leases.append(
                {
                    "robot_name": robot_name,
                    "holder_app_id": f"experiment_operator_{holder_index}",
                    "holder_role": roles[holder_index % len(roles)],
                    "session_id": session_id,
                    "acquired_at_ms": now_ms - 1000,
                    "last_heartbeat_ms": now_ms,
                    "panel_open": True,
                    "teleop_active": robot_index % 2 == 0,
                    "task_active": robot_index % 2 == 1,
                    "task_kind": "go_to_point" if robot_index % 2 == 1 else "none",
                    "lease_version": self.operator_sequence,
                }
            )
        envelope = {
            "session_id": session_id,
            "event": "snapshot",
            "request_id": f"synthetic-{self.operator_sequence}",
            "lease_ttl_ms": 3000,
            "lease_version": self.operator_sequence,
            "leases": leases,
        }
        msg = String()
        msg.data = json.dumps(envelope, separators=(",", ":"))
        self.lease_state_pub.publish(msg)
        for lease in leases:
            self.write_operator_metric(
                operator_id=lease["holder_app_id"],
                role=lease["holder_role"],
                state="lease_active",
                robot_id=lease["robot_name"],
                event="lease_snapshot",
                seq=self.operator_sequence,
                payload_bytes=len(msg.data),
                lease_version=lease["lease_version"],
            )

    def _control_load_enabled(self) -> bool:
        extra = dict(getattr(self.workload, "extra", {}) or {})
        nested = extra.get("extra")
        if isinstance(nested, dict):
            extra.update(nested)
        if "control_load" in extra:
            return str(extra.get("control_load")).strip().lower() not in {"0", "false", "no", "off"}
        text = " ".join(
            [
                str(self.workload.experiment).lower(),
                str(self.workload.condition).lower(),
                str(self.workload.robot_profile).lower(),
            ]
        )
        return any(token in text for token in ("goal", "control", "failure"))

    def record_failure_event(self, event: str, target: str, notes: str = "") -> None:
        elapsed = time.monotonic() - self.started
        self.write_failure_metric(
            event=event,
            target=target,
            seq=self.sequence,
            elapsed_s=f"{elapsed:.3f}",
            duration_ms=0.0,
            notes=notes,
        )
        self.write_event("failure_" + event, {"target": target, "elapsed_s": elapsed, "notes": notes})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--metrics-csv", default=None)
    return parser.parse_args()


def main() -> int:
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    args = parse_args()
    workload = load_workload_config(Path(args.config))
    duration = args.duration if args.duration > 0.0 else workload.duration_s
    rclpy.init()
    node = SyntheticWorkloadNode(workload, args.metrics_csv)
    deadline = time.monotonic() + max(0.0, duration)
    try:
        while not STOP_REQUESTED and time.monotonic() < deadline and rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except ExternalShutdownException:
                break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
