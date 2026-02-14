"""Tests for dashboard control-topic collection and role assignment."""

from horus.bridge.robot_registry import RobotRegistryClient


def _build_client():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    return client


def test_collect_control_topics_includes_drive_raw_and_head_pose():
    client = _build_client()
    config = {
        "control": {
            "drive_topic": "/alpha/cmd_vel",
            "teleop": {
                "command_topic": "/alpha/cmd_vel",
                "raw_input_topic": "/horus/teleop/alpha/joy",
                "head_pose_topic": "/horus/teleop/alpha/head_pose",
            },
        }
    }

    topics = client._collect_control_topics(config)

    assert topics == [
        "/alpha/cmd_vel",
        "/horus/teleop/alpha/joy",
        "/horus/teleop/alpha/head_pose",
    ]


def test_build_topic_roles_marks_control_topics_as_backend_publishers():
    client = _build_client()

    data_topics = ["/alpha/scan", "/alpha/cmd_vel", "/horus/teleop/alpha/joy"]
    control_topics = ["/alpha/cmd_vel", "/horus/teleop/alpha/joy"]

    roles = client._build_topic_roles(data_topics, control_topics=control_topics)

    assert roles["/alpha/scan"] == "backend_sub"
    assert roles["/alpha/cmd_vel"] == "backend_pub"
    assert roles["/horus/teleop/alpha/joy"] == "backend_pub"


def test_has_backend_publisher_falls_back_to_non_local_publishers():
    client = _build_client()

    class _Info:
        def __init__(self, node_name):
            self.node_name = node_name

    class _Node:
        def get_name(self):
            return "robot_registry_client"

        def get_publishers_info_by_topic(self, _topic):
            return [_Info("/unity_runtime_node"), _Info("/robot_registry_client")]

    client.node = _Node()
    assert client._has_backend_publisher("/alpha/cmd_vel") is True
