"""Deprecated compatibility path for the ROS-backed registry client."""

from .._compat import warn_deprecated_module
from .robot_registry import RobotRegistryClient, get_robot_registry_client

warn_deprecated_module("horus.bridge.ros2", "horus.bridge.robot_registry")

__all__ = ["RobotRegistryClient", "get_robot_registry_client"]
