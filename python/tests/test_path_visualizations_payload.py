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


def test_default_nav_path_colors_are_robot_unique_and_role_distinct():
    client = _build_client()

    atlas = Robot(name="atlas", robot_type=RobotType.WHEELED)
    atlas_viz = atlas.create_dataviz()
    atlas.add_path_planning_to_dataviz(
        atlas_viz,
        global_path_topic="/atlas/global_path",
        local_path_topic="/atlas/local_path",
    )
    atlas_config = client._build_robot_config_dict(atlas, atlas_viz)
    atlas_by_topic = {entry["topic"]: entry for entry in _find_path_entries(atlas_config)}

    nova = Robot(name="nova", robot_type=RobotType.WHEELED)
    nova_viz = nova.create_dataviz()
    nova.add_path_planning_to_dataviz(
        nova_viz,
        global_path_topic="/nova/global_path",
        local_path_topic="/nova/local_path",
    )
    nova_config = client._build_robot_config_dict(nova, nova_viz)
    nova_by_topic = {entry["topic"]: entry for entry in _find_path_entries(nova_config)}

    atlas_global = atlas_by_topic["/atlas/global_path"]["color"]
    atlas_local = atlas_by_topic["/atlas/local_path"]["color"]
    nova_global = nova_by_topic["/nova/global_path"]["color"]

    assert atlas_global != atlas_local
    assert atlas_global != nova_global
