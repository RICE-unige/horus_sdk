"""Shared robot-description showroom fleet configuration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class ShowroomRobotSpec:
    name: str
    robot_type: str
    base_frame: str
    urdf_root_frame: str
    dimensions: Tuple[float, float, float]
    body_mesh_mode: str
    urdf_name: str
    xacro_name: Optional[str]
    x: float
    y: float
    z: float = 0.0


# name == launch namespace == default tf_prefix. z is the static transform from
# the shared showroom floor frame to the URDF root frame.
SHOWROOM_FLEET = [
    ShowroomRobotSpec("g1", "legged", "pelvis", "pelvis", (0.350, 0.450, 1.320), "runtime_high_mesh", "g1.urdf", None, -2.0, 1.2, 0.7923),
    ShowroomRobotSpec("h1", "legged", "pelvis", "pelvis", (0.300, 0.400, 1.800), "runtime_high_mesh", "h1.urdf", None, 0.0, 1.2, 1.0442),
    ShowroomRobotSpec("anymal_c", "legged", "base", "base", (0.930, 0.530, 0.700), "runtime_high_mesh", "anymal_c.urdf", None, 2.0, 1.2, 0.6319),
    ShowroomRobotSpec("spot", "legged", "body", "body", (1.100, 0.500, 0.700), "runtime_high_mesh", "spot.urdf", "spot_simple.urdf.xacro", -1.0, -1.2, 0.6565),
    ShowroomRobotSpec("jackal", "wheeled", "base_link", "base_link", (0.508, 0.430, 0.250), "runtime_high_mesh", "jackal.urdf", "jackal.urdf.xacro", 1.0, -1.2, 0.0653),
]
