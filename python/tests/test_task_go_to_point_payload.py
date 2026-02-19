"""Tests for go-to-point task payload serialization."""

from horus.bridge.robot_registry import RobotRegistryClient
from horus.robot import Robot, RobotType


def _build_client():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    return client


def _build_config(robot):
    dataviz = robot.create_dataviz()
    client = _build_client()
    return client._build_robot_config_dict(robot, dataviz)


def test_go_to_point_task_defaults():
    robot = Robot(name="task_bot", robot_type=RobotType.WHEELED)
    config = _build_config(robot)

    control = config["control"]
    tasks = control["tasks"]
    go_to_point = tasks["go_to_point"]

    assert go_to_point["enabled"] is True
    assert go_to_point["goal_topic"] == "/task_bot/goal_pose"
    assert go_to_point["cancel_topic"] == "/task_bot/goal_cancel"
    assert go_to_point["status_topic"] == "/task_bot/goal_status"
    assert go_to_point["frame_id"] == "map"
    assert go_to_point["position_tolerance_m"] == 0.20
    assert go_to_point["yaw_tolerance_deg"] == 12.0


def test_go_to_point_task_overrides():
    robot = Robot(name="alpha", robot_type=RobotType.WHEELED)
    robot.add_metadata(
        "task_config",
        {
            "go_to_point": {
                "enabled": False,
                "goal_topic": "/alpha/nav_goal",
                "cancel_topic": "/alpha/nav_cancel",
                "status_topic": "/alpha/nav_status",
                "frame_id": "custom_map",
                "position_tolerance_m": 0.35,
                "yaw_tolerance_deg": 25.0,
            }
        },
    )

    config = _build_config(robot)
    go_to_point = config["control"]["tasks"]["go_to_point"]

    assert go_to_point["enabled"] is False
    assert go_to_point["goal_topic"] == "/alpha/nav_goal"
    assert go_to_point["cancel_topic"] == "/alpha/nav_cancel"
    assert go_to_point["status_topic"] == "/alpha/nav_status"
    assert go_to_point["frame_id"] == "custom_map"
    assert go_to_point["position_tolerance_m"] == 0.35
    assert go_to_point["yaw_tolerance_deg"] == 25.0


def test_go_to_point_invalid_values_clamp_to_safe_defaults():
    robot = Robot(name="invalid", robot_type=RobotType.WHEELED)
    robot.add_metadata(
        "task_config",
        {
            "go_to_point": {
                "goal_topic": "   ",
                "cancel_topic": "",
                "status_topic": " ",
                "frame_id": "",
                "position_tolerance_m": -1.0,
                "yaw_tolerance_deg": 999.0,
            }
        },
    )

    config = _build_config(robot)
    go_to_point = config["control"]["tasks"]["go_to_point"]

    assert go_to_point["goal_topic"] == "/invalid/goal_pose"
    assert go_to_point["cancel_topic"] == "/invalid/goal_cancel"
    assert go_to_point["status_topic"] == "/invalid/goal_status"
    assert go_to_point["frame_id"] == "map"
    assert go_to_point["position_tolerance_m"] == 0.01
    assert go_to_point["yaw_tolerance_deg"] == 180.0
