"""Deprecated placeholder for the old Unity TCP bridge import path.

The active HORUS SDK registration path is the ROS-backed HORUS registry client.
"""

from .._compat import warn_deprecated_module

warn_deprecated_module("horus.bridge.unity_tcp", "horus.bridge.robot_registry")

__all__: list[str] = []
