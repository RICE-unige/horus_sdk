"""Tests for bundled local body model payload serialization."""

from horus.bridge.robot_registry import RobotRegistryClient
from horus.robot import Robot, RobotType


def _build_client():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    return client


def test_local_body_model_helper_serializes_robot_model_id():
    client = _build_client()

    robot = Robot(name="carter1", robot_type=RobotType.WHEELED)
    robot.configure_local_body_model("nova_carter")
    dataviz = robot.create_dataviz()

    config = client._build_robot_config_dict(robot, dataviz)

    assert config["robot_model_id"] == "nova_carter"
    assert config["has_visual_mesh_model"] is True


def test_local_body_model_absent_keeps_payload_unchanged():
    client = _build_client()

    robot = Robot(name="legacy_bot", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()

    config = client._build_robot_config_dict(robot, dataviz)

    assert "robot_model_id" not in config
    assert "has_visual_mesh_model" not in config


def test_local_body_model_disabled_skips_visual_mesh_flags():
    client = _build_client()

    robot = Robot(name="carter2", robot_type=RobotType.WHEELED)
    robot.configure_local_body_model("nova_carter", enabled=False)
    dataviz = robot.create_dataviz()

    config = client._build_robot_config_dict(robot, dataviz)

    assert "robot_model_id" not in config
    assert "has_visual_mesh_model" not in config
