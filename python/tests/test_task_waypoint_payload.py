"""Tests for waypoint task payload serialization."""

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


def test_waypoint_task_defaults():
    robot = Robot(name="path_bot", robot_type=RobotType.WHEELED)
    config = _build_config(robot)

    waypoint = config["control"]["tasks"]["waypoint"]
    assert waypoint["enabled"] is True
    assert waypoint["path_topic"] == "/path_bot/waypoint_path"
    assert waypoint["status_topic"] == "/path_bot/waypoint_status"
    assert waypoint["frame_id"] == "map"
    assert waypoint["position_tolerance_m"] == 0.20
    assert waypoint["yaw_tolerance_deg"] == 12.0


def test_waypoint_task_overrides():
    robot = Robot(name="alpha", robot_type=RobotType.WHEELED)
    robot.add_metadata(
        "task_config",
        {
            "waypoint": {
                "enabled": False,
                "path_topic": "/alpha/route_path",
                "status_topic": "/alpha/route_status",
                "frame_id": "custom_map",
                "position_tolerance_m": 0.35,
                "yaw_tolerance_deg": 25.0,
            }
        },
    )

    config = _build_config(robot)
    waypoint = config["control"]["tasks"]["waypoint"]
    assert waypoint["enabled"] is False
    assert waypoint["path_topic"] == "/alpha/route_path"
    assert waypoint["status_topic"] == "/alpha/route_status"
    assert waypoint["frame_id"] == "custom_map"
    assert waypoint["position_tolerance_m"] == 0.35
    assert waypoint["yaw_tolerance_deg"] == 25.0


def test_waypoint_task_invalid_values_clamp_to_safe_defaults():
    robot = Robot(name="invalid", robot_type=RobotType.WHEELED)
    robot.add_metadata(
        "task_config",
        {
            "waypoint": {
                "path_topic": "   ",
                "status_topic": "",
                "frame_id": "",
                "position_tolerance_m": -1.0,
                "yaw_tolerance_deg": 999.0,
            }
        },
    )

    config = _build_config(robot)
    waypoint = config["control"]["tasks"]["waypoint"]

    assert waypoint["path_topic"] == "/invalid/waypoint_path"
    assert waypoint["status_topic"] == "/invalid/waypoint_status"
    assert waypoint["frame_id"] == "map"
    assert waypoint["position_tolerance_m"] == 0.01
    assert waypoint["yaw_tolerance_deg"] == 180.0
