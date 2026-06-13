"""Deterministic synthetic metrics for HORUS benchmark smoke runs."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Iterator

from .workloads import WorkloadConfig


@dataclass(frozen=True)
class SyntheticPose:
    x: float
    y: float
    z: float
    yaw: float


def circular_pose(sample_index: int, *, radius: float = 2.0, hz: float = 30.0) -> SyntheticPose:
    t = sample_index / max(hz, 1e-6)
    angle = t * 0.35
    return SyntheticPose(
        x=radius * math.cos(angle),
        y=radius * math.sin(angle),
        z=0.0,
        yaw=angle + math.pi / 2.0,
    )


def estimate_payload_bytes(workload: WorkloadConfig) -> Dict[str, int]:
    camera_bytes = 0
    if workload.camera.streams > 0 and workload.camera.resolution:
        width, height = _parse_resolution(workload.camera.resolution)
        raw_bytes = width * height * 3
        if workload.camera.encoding.lower() in {"h264", "h265", "jpeg", "compressed"}:
            # Conservative benchmark metadata estimate only; actual metrics come
            # from transport logs during a real run.
            camera_bytes = int(max(workload.camera.bitrate_mbps, 0.1) * 1_000_000 / 8 / max(workload.camera.fps, 1))
        else:
            camera_bytes = raw_bytes

    pointcloud_bytes = workload.pointcloud.points * max(workload.pointcloud.point_step, 1)
    mesh_bytes = workload.map.vertices * 24 + workload.map.triangles * 12
    return {
        "camera_payload_bytes": camera_bytes,
        "pointcloud_payload_bytes": pointcloud_bytes,
        "mesh_payload_bytes": mesh_bytes,
    }


def iter_source_smoke_rows(workload: WorkloadConfig, *, samples: int = 10) -> Iterator[Dict[str, object]]:
    payloads = estimate_payload_bytes(workload)
    for index in range(samples):
        pose = circular_pose(index)
        for robot_index in range(workload.robot_count):
            robot_id = f"robot_{robot_index + 1:02d}"
            yield {
                "robot_id": robot_id,
                "stream": "tf",
                "seq": index,
                "source_hz": 30.0,
                "payload_bytes": 256,
                "pose_x": pose.x + robot_index,
                "pose_y": pose.y,
                "pose_z": pose.z,
                "yaw": pose.yaw,
            }
            if workload.pointcloud.points > 0:
                yield {
                    "robot_id": robot_id,
                    "stream": "pointcloud",
                    "seq": index,
                    "source_hz": workload.pointcloud.hz,
                    "points": workload.pointcloud.points,
                    "point_step": workload.pointcloud.point_step,
                    "payload_bytes": payloads["pointcloud_payload_bytes"],
                }
            if workload.map.representation != "none":
                yield {
                    "robot_id": robot_id,
                    "stream": "map",
                    "seq": index,
                    "source_hz": workload.map.hz,
                    "representation": workload.map.representation,
                    "chunks": workload.map.chunks,
                    "vertices": workload.map.vertices,
                    "triangles": workload.map.triangles,
                    "payload_bytes": payloads["mesh_payload_bytes"],
                }
        for camera_index in range(workload.camera.streams):
            yield {
                "robot_id": f"camera_{camera_index + 1:02d}",
                "stream": "camera",
                "seq": index,
                "source_hz": workload.camera.fps,
                "resolution": workload.camera.resolution,
                "encoding": workload.camera.encoding,
                "bitrate_mbps": workload.camera.bitrate_mbps,
                "payload_bytes": payloads["camera_payload_bytes"],
            }


def _parse_resolution(resolution: str) -> tuple[int, int]:
    parts = resolution.lower().replace(" ", "").split("x", 1)
    if len(parts) != 2:
        return 0, 0
    try:
        return int(parts[0]), int(parts[1])
    except ValueError:
        return 0, 0
