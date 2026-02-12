"""Tests for global visualization payload serialization and dedupe."""

from horus.bridge.robot_registry import RobotRegistryClient
from horus.robot import Robot, RobotType


def _build_client():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    # Prevent __del__ from trying to tear down uninitialized ROS state.
    client.ros_initialized = False
    client.node = None
    return client


def test_occupancy_grid_serialized_as_global_visualization():
    robot = Robot(name="test_bot", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    dataviz.add_occupancy_grid(
        topic="/map",
        frame_id="map",
        render_options={
            "show_unknown_space": False,
            "position_scale": 0.15,
            "position_offset": [1.0, 2.0, 3.0],
            "rotation_offset_euler": {"x": 0.0, "y": 90.0, "z": 0.0},
        },
    )

    client = _build_client()
    config = client._build_robot_config_dict(robot, dataviz)

    occupancy_entries = [
        entry
        for entry in config["global_visualizations"]
        if entry.get("type") == "occupancy_grid"
    ]
    assert len(occupancy_entries) == 1

    occupancy_entry = occupancy_entries[0]
    assert occupancy_entry["scope"] == "global"
    assert occupancy_entry["topic"] == "/map"
    assert occupancy_entry["frame"] == "map"
    assert occupancy_entry["occupancy"]["show_unknown_space"] is False
    assert occupancy_entry["occupancy"]["position_scale"] == 0.15
    assert occupancy_entry["occupancy"]["position_offset"] == {"x": 1.0, "y": 2.0, "z": 3.0}
    assert occupancy_entry["occupancy"]["rotation_offset_euler"] == {
        "x": 0.0,
        "y": 90.0,
        "z": 0.0,
    }

    # Backward compatible behavior: robot-scoped list excludes global occupancy entry.
    assert all(entry.get("type") != "occupancy_grid" for entry in config["visualizations"])


def test_global_visualization_dedupes_across_multiple_robots():
    robot_a = Robot(name="bot_a", robot_type=RobotType.WHEELED)
    dataviz_a = robot_a.create_dataviz()
    dataviz_a.add_occupancy_grid(topic="/map", frame_id="map")

    robot_b = Robot(name="bot_b", robot_type=RobotType.WHEELED)
    dataviz_b = robot_b.create_dataviz()
    dataviz_b.add_occupancy_grid(topic="/map", frame_id="map")

    client = _build_client()
    global_payload = client._build_global_visualizations_payload([dataviz_a, dataviz_b])
    occupancy_entries = [
        entry for entry in global_payload if entry.get("type") == "occupancy_grid"
    ]
    assert len(occupancy_entries) == 1
