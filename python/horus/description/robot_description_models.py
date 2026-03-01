"""
Robot description data models for SDK -> MR transport.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional


@dataclass
class CompiledCollision:
    type: str
    origin_xyz: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    origin_rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    size_xyz: List[float] = field(default_factory=list)
    radius: float = 0.0
    length: float = 0.0
    mesh_uri: str = ""
    proxy_size_xyz: List[float] = field(default_factory=list)

    def to_dict(self) -> Dict:
        payload = asdict(self)
        # Keep payload compact by removing empty shape fields.
        if not payload.get("size_xyz"):
            payload.pop("size_xyz", None)
        if not payload.get("proxy_size_xyz"):
            payload.pop("proxy_size_xyz", None)
        if not payload.get("mesh_uri"):
            payload.pop("mesh_uri", None)
        if self.radius <= 0.0:
            payload.pop("radius", None)
        if self.length <= 0.0:
            payload.pop("length", None)
        return payload


@dataclass
class CompiledLink:
    name: str
    collisions: List[CompiledCollision] = field(default_factory=list)

    def to_dict(self) -> Dict:
        return {
            "name": self.name,
            "collisions": [entry.to_dict() for entry in self.collisions],
        }


@dataclass
class CompiledJoint:
    name: str
    type: str
    parent_link: str
    child_link: str
    origin_xyz: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    origin_rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    axis_xyz: List[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class RobotDescriptionV1:
    version: str
    robot_name: str
    base_frame: str
    links: List[CompiledLink] = field(default_factory=list)
    joints: List[CompiledJoint] = field(default_factory=list)

    def to_dict(self) -> Dict:
        return {
            "version": self.version,
            "robot_name": self.robot_name,
            "base_frame": self.base_frame,
            "links": [entry.to_dict() for entry in self.links],
            "joints": [entry.to_dict() for entry in self.joints],
        }


@dataclass
class RobotDescriptionManifestV1:
    version: str
    description_id: str
    source: str
    base_frame: str
    link_count: int
    joint_count: int
    collision_count: int
    supports_collision: bool
    supports_joints: bool
    is_transparent: bool
    encoding: str
    chunk_size_bytes: int

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class RobotDescriptionArtifact:
    manifest: RobotDescriptionManifestV1
    payload_dict: Dict
    payload_json: str
    encoded_payload: str
    chunks: List[str]
    urdf_path: Optional[str] = None

    def to_manifest_payload(self) -> Dict:
        return self.manifest.to_dict()
