"""
HORUS SDK Robot Module

Robot management and control functionality.
"""

from .config import DeadmanPolicy, TeleopProfile, TeleopResponseMode, WorkspaceCompassConfig, WorkspaceExperimentConfig
from .robot import Robot, RobotDimensions, RobotType, is_registration_cancelled, register_robots

__all__ = [
    "Robot",
    "RobotDimensions",
    "RobotType",
    "TeleopProfile",
    "TeleopResponseMode",
    "DeadmanPolicy",
    "WorkspaceCompassConfig",
    "WorkspaceExperimentConfig",
    "register_robots",
    "is_registration_cancelled",
]
