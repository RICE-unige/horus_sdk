"""Compatibility guardrails for the public HORUS SDK API."""

from __future__ import annotations

import importlib
import sys
import warnings


def test_top_level_horus_import_shows_project_branding(capsys):
    sys.modules.pop("horus", None)
    importlib.import_module("horus")
    captured = capsys.readouterr()
    assert "Holistic Operational Reality" in captured.out
    assert "HORUS SDK v" in captured.out
    assert captured.err == ""


def test_core_public_imports_remain_available():
    from horus.robot import DeadmanPolicy, Robot, RobotDimensions, RobotType, TeleopProfile, TeleopResponseMode, register_robots
    from horus.dataviz import DataViz, VisualizationType
    from horus.sensors import Camera, CameraImageType, CameraStartupMode, CameraTransport, LaserScan, Lidar3D, StereoLayout
    from horus.bridge.robot_registry import get_robot_registry_client

    assert Robot is not None
    assert RobotDimensions is not None
    assert RobotType.DRONE.value == "drone"
    assert TeleopProfile.WHEELED.value == "wheeled"
    assert TeleopResponseMode.ANALOG.value == "analog"
    assert DeadmanPolicy.EITHER_GRIP_TRIGGER.value == "either_grip_trigger"
    assert callable(register_robots)
    assert DataViz is not None
    assert VisualizationType.SEMANTIC_BOX.value == "semantic_box"
    assert Camera is not None
    assert CameraTransport.ROS.value == "ros"
    assert CameraImageType.COMPRESSED.value == "compressed"
    assert CameraStartupMode.MINIMAP.value == "minimap"
    assert StereoLayout.SIDE_BY_SIDE.value == "side_by_side"
    assert LaserScan is not None
    assert Lidar3D is not None
    assert callable(get_robot_registry_client)


def test_legacy_placeholder_import_paths_still_import():
    modules = [
        "horus.robot.dataviz",
        "horus.robot.status",
        "horus.robot.task",
        "horus.robot.teleop",
        "horus.topics",
        "horus.bridge.ros2",
        "horus.bridge.unity_tcp",
        "horus.utils.logging",
        "horus.plugins.rosbot",
    ]

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        for module in modules:
            assert importlib.import_module(module) is not None


def test_console_entrypoint_target_exists():
    from horus.client import main

    assert callable(main)
