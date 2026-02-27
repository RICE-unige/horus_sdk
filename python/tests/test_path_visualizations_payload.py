"""Tests for robot path visualization payload serialization."""

from horus.bridge.robot_registry import RobotRegistryClient
from horus.robot import Robot, RobotType


def _build_client():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    return client


def _find_path_entries(config):
    return [entry for entry in config["visualizations"] if entry.get("type") == "path"]


def test_robot_global_and_local_paths_include_explicit_role_metadata():
    robot = Robot(name="atlas", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    robot.add_path_planning_to_dataviz(
        dataviz,
        global_path_topic="/atlas/global_path",
        local_path_topic="/atlas/local_path",
    )

    client = _build_client()
    config = client._build_robot_config_dict(robot, dataviz)
    path_entries = _find_path_entries(config)

    by_topic = {entry["topic"]: entry for entry in path_entries}
    assert "/atlas/global_path" in by_topic
    assert "/atlas/local_path" in by_topic

    global_entry = by_topic["/atlas/global_path"]
    local_entry = by_topic["/atlas/local_path"]

    assert global_entry["scope"] == "robot"
    assert global_entry["frame"] == "map"
    assert global_entry["path"]["role"] == "global"
    assert "line_width" in global_entry["path"]

    assert local_entry["scope"] == "robot"
    assert local_entry["frame"] == "map"
    assert local_entry["path"]["role"] == "local"
    assert local_entry["path"]["line_style"] == "dashed"


def test_path_payload_preserves_line_style_and_width_when_explicitly_set():
    robot = Robot(name="nova", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    dataviz.add_robot_global_path(
        robot_name="nova",
        topic="/nova/global_path",
        frame_id="map",
        render_options={
            "color": "#112233",
            "line_width": 7,
            "line_style": "solid",
        },
    )

    client = _build_client()
    config = client._build_robot_config_dict(robot, dataviz)
    path_entries = _find_path_entries(config)
    assert len(path_entries) >= 1

    entry = next(e for e in path_entries if e["topic"] == "/nova/global_path")
    assert entry["color"] == "#112233"
    assert entry["path"]["role"] == "global"
    assert entry["path"]["line_width"] == 7.0
    assert entry["path"]["line_style"] == "solid"
