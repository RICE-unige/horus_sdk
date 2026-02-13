"""Tests for teleoperation control payload serialization."""

from horus.bridge.robot_registry import RobotRegistryClient
from horus.robot import Robot, RobotType


def _build_client():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    # Prevent __del__ from trying to tear down uninitialized ROS state.
    client.ros_initialized = False
    client.node = None
    return client


def _build_config(robot):
    dataviz = robot.create_dataviz()
    client = _build_client()
    return client._build_robot_config_dict(robot, dataviz)


def test_teleop_control_defaults():
    robot = Robot(name="test_bot", robot_type=RobotType.WHEELED)
    config = _build_config(robot)

    control = config["control"]
    teleop = control["teleop"]

    assert control["drive_topic"] == "/test_bot/cmd_vel"
    assert teleop["enabled"] is True
    assert teleop["command_topic"] == "/test_bot/cmd_vel"
    assert teleop["raw_input_topic"] == "/horus/teleop/test_bot/joy"
    assert teleop["head_pose_topic"] == "/horus/teleop/test_bot/head_pose"
    assert teleop["robot_profile"] == "wheeled"
    assert teleop["response_mode"] == "analog"
    assert teleop["publish_rate_hz"] == 30.0
    assert teleop["custom_passthrough_only"] is False
    assert teleop["deadman"]["policy"] == "either_grip_trigger"
    assert teleop["deadman"]["timeout_ms"] == 200


def test_teleop_control_overrides():
    robot = Robot(name="alpha", robot_type=RobotType.AERIAL)
    robot.add_metadata(
        "teleop_config",
        {
            "command_topic": "/alpha/cmd_vel_custom",
            "raw_input_topic": "/horus/teleop/alpha/joy_raw",
            "head_pose_topic": "/horus/teleop/alpha/head_pose",
            "robot_profile": "aerial",
            "response_mode": "discrete",
            "publish_rate_hz": 45.0,
            "custom_passthrough_only": True,
            "deadman": {
                "policy": "right_index_trigger",
                "timeout_ms": 450,
            },
            "axes": {
                "deadzone": 0.18,
                "expo": 2.0,
                "linear_xy_max_mps": 1.8,
                "linear_z_max_mps": 1.2,
                "angular_z_max_rps": 1.5,
            },
            "discrete": {
                "threshold": 0.55,
                "linear_xy_step_mps": 0.9,
                "linear_z_step_mps": 0.7,
                "angular_z_step_rps": 1.1,
            },
        },
    )

    config = _build_config(robot)
    control = config["control"]
    teleop = control["teleop"]

    assert control["drive_topic"] == "/alpha/cmd_vel_custom"
    assert teleop["command_topic"] == "/alpha/cmd_vel_custom"
    assert teleop["raw_input_topic"] == "/horus/teleop/alpha/joy_raw"
    assert teleop["head_pose_topic"] == "/horus/teleop/alpha/head_pose"
    assert teleop["robot_profile"] == "aerial"
    assert teleop["response_mode"] == "discrete"
    assert teleop["publish_rate_hz"] == 45.0
    assert teleop["custom_passthrough_only"] is True
    assert teleop["deadman"]["policy"] == "right_index_trigger"
    assert teleop["deadman"]["timeout_ms"] == 450
    assert teleop["axes"]["deadzone"] == 0.18
    assert teleop["axes"]["expo"] == 2.0
    assert teleop["discrete"]["threshold"] == 0.55


def test_teleop_control_invalid_values_clamped_to_safe_defaults():
    robot = Robot(name="weird_bot", robot_type=RobotType.WHEELED)
    robot.add_metadata(
        "teleop_config",
        {
            "command_topic": " ",
            "robot_profile": "unknown",
            "response_mode": "bad",
            "publish_rate_hz": -10,
            "deadman": {
                "policy": "bad_policy",
                "timeout_ms": -50,
            },
            "axes": {
                "deadzone": 4.0,
                "expo": 0.3,
                "linear_xy_max_mps": -1.0,
                "linear_z_max_mps": 99.0,
                "angular_z_max_rps": -2.0,
            },
            "discrete": {
                "threshold": 2.0,
                "linear_xy_step_mps": -1.0,
                "linear_z_step_mps": 99.0,
                "angular_z_step_rps": -1.0,
            },
        },
    )

    config = _build_config(robot)
    control = config["control"]
    teleop = control["teleop"]

    # Fallback command topic should remain canonical per-robot cmd_vel.
    assert control["drive_topic"] == "/weird_bot/cmd_vel"
    assert teleop["command_topic"] == "/weird_bot/cmd_vel"
    assert teleop["robot_profile"] == "wheeled"
    assert teleop["response_mode"] == "analog"
    assert teleop["publish_rate_hz"] == 5.0
    assert teleop["deadman"]["policy"] == "either_grip_trigger"
    assert teleop["deadman"]["timeout_ms"] == 50
    assert teleop["axes"]["deadzone"] == 0.5
    assert teleop["axes"]["expo"] == 1.0
    assert teleop["axes"]["linear_xy_max_mps"] == 0.0
    assert teleop["axes"]["linear_z_max_mps"] == 5.0
    assert teleop["axes"]["angular_z_max_rps"] == 0.0
    assert teleop["discrete"]["threshold"] == 1.0
    assert teleop["discrete"]["linear_xy_step_mps"] == 0.0
    assert teleop["discrete"]["linear_z_step_mps"] == 5.0
    assert teleop["discrete"]["angular_z_step_rps"] == 0.0
