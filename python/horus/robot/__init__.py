"""
HORUS SDK Robot Module

Robot management and control functionality.
"""

from .config import (
    DeadmanPolicy,
    EntityCapabilities,
    FieldTeammateConfig,
    TeleopProfile,
    TeleopResponseMode,
    WorkspaceCompassConfig,
    WorkspaceExperimentConfig,
)
from .robot import (
    FieldTeammate,
    Robot,
    RobotDimensions,
    RobotType,
    is_registration_cancelled,
    register_robots,
)

__all__ = [
    "Robot",
    "FieldTeammate",
    "RobotDimensions",
    "RobotType",
    "EntityCapabilities",
    "FieldTeammateConfig",
    "TeleopProfile",
    "TeleopResponseMode",
    "DeadmanPolicy",
    "WorkspaceCompassConfig",
    "WorkspaceExperimentConfig",
    "register_robots",
    "is_registration_cancelled",
]
