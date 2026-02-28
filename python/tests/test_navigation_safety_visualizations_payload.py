"""Tests for velocity/trail/collision DataViz payload serialization."""

from horus.bridge.robot_registry import RobotRegistryClient
from horus.robot import Robot, RobotType


def _build_client():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    return client


def _entries_by_type(config):
    by_type = {}
    for entry in config["visualizations"]:
        by_type.setdefault(entry.get("type"), []).append(entry)
    return by_type


def test_navigation_safety_default_payload_shapes():
    robot = Robot(name="atlas", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    robot.add_navigation_safety_to_dataviz(dataviz)

    client = _build_client()
    config = client._build_robot_config_dict(robot, dataviz)
    by_type = _entries_by_type(config)

    velocity_entry = by_type["velocity_data"][0]
    assert velocity_entry["topic"] == "/atlas/odom"
    assert velocity_entry["velocity"]["units"] == "m/s"
    assert velocity_entry["velocity"]["text_back_offset_m"] == 0.36
    assert velocity_entry["velocity"]["floor_offset_m"] == 0.01
    assert velocity_entry["velocity"]["update_hz"] == 10.0

    trail_entry = by_type["odometry_trail"][0]
    assert trail_entry["topic"] == "/atlas/odom"
    assert trail_entry["trail"]["max_points"] == 48
    assert trail_entry["trail"]["history_seconds"] == 3.2
    assert trail_entry["trail"]["min_spacing_m"] == 0.07
    assert trail_entry["trail"]["line_width_m"] == 0.0096
    assert trail_entry["trail"]["trail_back_offset_m"] == 0.44

    collision_entry = by_type["collision_risk"][0]
    assert collision_entry["topic"] == "/atlas/collision_risk"
    assert collision_entry["frame"] == "atlas/base_link"
    assert collision_entry["collision"]["threshold_m"] == 1.2
    assert collision_entry["collision"]["radius_m"] == 1.2
    assert collision_entry["collision"]["source"] == "laser_scan"
    assert collision_entry["collision"]["alpha_min"] == 0.0
    assert collision_entry["collision"]["alpha_max"] == 0.55


def test_navigation_safety_custom_render_options_are_serialized():
    robot = Robot(name="nova", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    dataviz.add_robot_velocity_data(
        robot_name="nova",
        topic="/nova/odom",
        render_options={
            "units": "km/h",
            "text_back_offset_m": 0.4,
            "floor_offset_m": 0.02,
            "update_hz": 5.0,
        },
    )
    dataviz.add_robot_odometry_trail(
        robot_name="nova",
        topic="/nova/odom",
        render_options={
            "max_points": 50,
            "history_seconds": 3.0,
            "min_spacing_m": 0.08,
            "line_width_m": 0.002,
            "trail_back_offset_m": 0.6,
        },
    )
    dataviz.add_robot_collision_risk(
        robot_name="nova",
        topic="/nova/collision_risk",
        frame_id="nova/base_link",
        render_options={
            "threshold_m": 0.9,
            "radius_m": 1.0,
            "source": "point_cloud",
            "alpha_min": 0.05,
            "alpha_max": 0.7,
        },
    )

    client = _build_client()
    config = client._build_robot_config_dict(robot, dataviz)
    by_type = _entries_by_type(config)

    velocity_entry = by_type["velocity_data"][0]
    assert velocity_entry["velocity"]["units"] == "km/h"
    assert velocity_entry["velocity"]["text_back_offset_m"] == 0.4
    assert velocity_entry["velocity"]["floor_offset_m"] == 0.02
    assert velocity_entry["velocity"]["update_hz"] == 5.0

    trail_entry = by_type["odometry_trail"][0]
    assert trail_entry["trail"]["max_points"] == 50
    assert trail_entry["trail"]["history_seconds"] == 3.0
    assert trail_entry["trail"]["min_spacing_m"] == 0.08
    assert trail_entry["trail"]["line_width_m"] == 0.002
    assert trail_entry["trail"]["trail_back_offset_m"] == 0.6

    collision_entry = by_type["collision_risk"][0]
    assert collision_entry["collision"]["threshold_m"] == 0.9
    assert collision_entry["collision"]["radius_m"] == 1.0
    assert collision_entry["collision"]["source"] == "point_cloud"
    assert collision_entry["collision"]["alpha_min"] == 0.05
    assert collision_entry["collision"]["alpha_max"] == 0.7


def test_odometry_trail_color_is_robot_unique_by_default():
    client = _build_client()

    atlas = Robot(name="atlas", robot_type=RobotType.WHEELED)
    atlas_viz = atlas.create_dataviz()
    atlas.add_navigation_safety_to_dataviz(atlas_viz)
    atlas_config = client._build_robot_config_dict(atlas, atlas_viz)

    nova = Robot(name="nova", robot_type=RobotType.WHEELED)
    nova_viz = nova.create_dataviz()
    nova.add_navigation_safety_to_dataviz(nova_viz)
    nova_config = client._build_robot_config_dict(nova, nova_viz)

    atlas_trail = _entries_by_type(atlas_config)["odometry_trail"][0]
    nova_trail = _entries_by_type(nova_config)["odometry_trail"][0]
    assert atlas_trail["color"] != nova_trail["color"]
