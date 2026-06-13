"""Benchmark workload configuration helpers."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import Any, Dict


@dataclass
class CameraWorkload:
    streams: int = 0
    resolution: str = ""
    fps: float = 0.0
    encoding: str = "none"
    bitrate_mbps: float = 0.0


@dataclass
class PointCloudWorkload:
    points: int = 0
    hz: float = 0.0
    point_step: int = 16
    xyzrgb: bool = True


@dataclass
class MapWorkload:
    representation: str = "none"
    chunks: int = 0
    vertices: int = 0
    triangles: int = 0
    hz: float = 0.0


@dataclass
class WorkloadConfig:
    experiment: str = "E0_smoke"
    condition: str = "default"
    duration_s: float = 120.0
    warmup_s: float = 10.0
    repetition: int = 1
    robot_count: int = 1
    operator_count: int = 1
    transport: str = "ros2"
    topology: str = "lan"
    robot_profile: str = "minimal"
    map_profile: str = "none"
    camera: CameraWorkload = field(default_factory=CameraWorkload)
    pointcloud: PointCloudWorkload = field(default_factory=PointCloudWorkload)
    map: MapWorkload = field(default_factory=MapWorkload)
    extra: Dict[str, Any] = field(default_factory=dict)

    @property
    def stream_count(self) -> int:
        count = self.camera.streams
        if self.pointcloud.points > 0 and self.pointcloud.hz > 0:
            count += self.robot_count
        if self.map.representation != "none" and self.map.hz > 0:
            count += self.robot_count
        return count

    @property
    def resolution(self) -> str:
        return self.camera.resolution

    @property
    def target_fps(self) -> float:
        return self.camera.fps

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def load_workload_config(path: Path) -> WorkloadConfig:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return workload_from_dict(payload)


def workload_from_dict(payload: Dict[str, Any]) -> WorkloadConfig:
    camera = CameraWorkload(**payload.pop("camera", {}))
    pointcloud = PointCloudWorkload(**payload.pop("pointcloud", {}))
    map_workload = MapWorkload(**payload.pop("map", {}))
    known = {
        "experiment",
        "condition",
        "duration_s",
        "warmup_s",
        "repetition",
        "robot_count",
        "operator_count",
        "transport",
        "topology",
        "robot_profile",
        "map_profile",
    }
    extra = {key: value for key, value in payload.items() if key not in known}
    values = {key: payload[key] for key in known if key in payload}
    return WorkloadConfig(
        **values,
        camera=camera,
        pointcloud=pointcloud,
        map=map_workload,
        extra=extra,
    )
