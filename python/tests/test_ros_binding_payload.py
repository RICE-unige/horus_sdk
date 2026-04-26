"""Tests for flat/prefixed ROS binding resolution in registration payloads."""

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


def test_robot_defaults_to_generic_logical_name_when_empty():
    robot = Robot(name=None, robot_type=RobotType.WHEELED)

    assert robot.name == "robot"
    assert robot.get_ros_binding()["logical_name"] == "robot"


def test_prefixed_ros_binding_remains_default():
    robot = Robot(name="nova", robot_type=RobotType.WHEELED)
    config = _build_config(robot)

    binding = config["ros_binding"]
    assert binding["logical_name"] == "nova"
    assert binding["tf_mode"] == "prefixed"
    assert binding["topic_mode"] == "prefixed"
    assert binding["base_frame"] == "base_link"
    assert binding["tf_prefix"] == "nova"
    assert binding["topic_prefix"] == "/nova"

    assert config["control"]["drive_topic"] == "/nova/cmd_vel"
    assert config["control"]["tasks"]["go_to_point"]["goal_topic"] == "/nova/goal_pose"
    assert config["control"]["tasks"]["waypoint"]["path_topic"] == "/nova/waypoint_path"


def test_flat_ros_binding_uses_root_tf_and_topics():
    robot = Robot(name="solo", robot_type=RobotType.WHEELED)
    robot.configure_ros_binding(
        tf_mode="flat",
        topic_mode="flat",
        base_frame="base_link",
    )

    dataviz = robot.create_dataviz()
    transform_viz = next(item for item in dataviz.visualizations if item.data_source.frame_id == "base_link")
    assert transform_viz.data_source.frame_id == "base_link"
    assert robot.resolve_tf_frame("camera_link") == "camera_link"
    assert robot.resolve_topic("cmd_vel") == "/cmd_vel"

    config = _build_config(robot)
    binding = config["ros_binding"]
    assert binding["logical_name"] == "solo"
    assert binding["tf_mode"] == "flat"
    assert binding["topic_mode"] == "flat"
    assert binding["base_frame"] == "base_link"
    assert binding["tf_prefix"] == ""
    assert binding["topic_prefix"] == ""

    teleop = config["control"]["teleop"]
    go_to_point = config["control"]["tasks"]["go_to_point"]
    waypoint = config["control"]["tasks"]["waypoint"]

    assert config["control"]["drive_topic"] == "/cmd_vel"
    assert teleop["command_topic"] == "/cmd_vel"
    assert teleop["raw_input_topic"] == "/horus/teleop/solo/joy"
    assert teleop["head_pose_topic"] == "/horus/teleop/solo/head_pose"
    assert go_to_point["goal_topic"] == "/goal_pose"
    assert go_to_point["cancel_topic"] == "/goal_cancel"
    assert go_to_point["status_topic"] == "/goal_status"
    assert waypoint["path_topic"] == "/waypoint_path"
    assert waypoint["status_topic"] == "/waypoint_status"


def test_dashboard_topic_group_override_claims_flat_root_topics_for_robot():
    client = _build_client()

    overrides = client._build_topic_group_overrides(
        "solo",
        ["/cmd_vel", "/goal_pose", "/solo/camera/image_raw/compressed"],
    )

    assert overrides["/cmd_vel"] == "solo"
    assert overrides["/goal_pose"] == "solo"
    assert "/solo/camera/image_raw/compressed" not in overrides
