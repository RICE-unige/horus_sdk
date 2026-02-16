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


def test_point_cloud_serialized_as_global_visualization():
    robot = Robot(name="test_bot", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    dataviz.add_3d_map(
        topic="/map_3d",
        frame_id="map",
        render_options={
            "point_size": 0.025,
            "max_points_per_frame": 20000,
            "base_sample_stride": 3,
            "min_update_interval": 0.2,
            "enable_adaptive_quality": True,
            "target_framerate": 72.0,
            "min_quality_multiplier": 0.65,
            "min_distance": 0.0,
            "max_distance": 30.0,
            "replace_latest": True,
            "render_all_points": False,
            "auto_point_size_by_workspace_scale": False,
            "min_point_size": 0.003,
            "max_point_size": 0.05,
        },
    )

    client = _build_client()
    config = client._build_robot_config_dict(robot, dataviz)
    point_cloud_entries = [
        entry for entry in config["global_visualizations"] if entry.get("type") == "point_cloud"
    ]
    assert len(point_cloud_entries) == 1

    point_cloud_entry = point_cloud_entries[0]
    assert point_cloud_entry["scope"] == "global"
    assert point_cloud_entry["topic"] == "/map_3d"
    assert point_cloud_entry["frame"] == "map"
    assert point_cloud_entry["point_cloud"]["point_size"] == 0.025
    assert point_cloud_entry["point_cloud"]["max_points_per_frame"] == 20000
    assert point_cloud_entry["point_cloud"]["base_sample_stride"] == 3
    assert point_cloud_entry["point_cloud"]["replace_latest"] is True
    assert point_cloud_entry["point_cloud"]["render_all_points"] is False
    assert point_cloud_entry["point_cloud"]["auto_point_size_by_workspace_scale"] is False
    assert point_cloud_entry["point_cloud"]["min_point_size"] == 0.003
    assert point_cloud_entry["point_cloud"]["max_point_size"] == 0.05


def test_point_cloud_defaults_are_emitted_without_render_options():
    robot = Robot(name="test_bot", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    dataviz.add_3d_map(topic="/map_3d", frame_id="map")

    client = _build_client()
    config = client._build_robot_config_dict(robot, dataviz)
    entry = next(
        item for item in config["global_visualizations"] if item.get("type") == "point_cloud"
    )
    point_cloud = entry["point_cloud"]
    assert point_cloud["max_points_per_frame"] == 0
    assert point_cloud["base_sample_stride"] == 1
    assert point_cloud["enable_adaptive_quality"] is False
    assert point_cloud["max_distance"] == 0.0
    assert point_cloud["render_all_points"] is True
    assert point_cloud["auto_point_size_by_workspace_scale"] is True


def test_global_visualization_dedupes_point_cloud_across_multiple_robots():
    robot_a = Robot(name="bot_a", robot_type=RobotType.WHEELED)
    dataviz_a = robot_a.create_dataviz()
    dataviz_a.add_3d_map(topic="/map_3d", frame_id="map")

    robot_b = Robot(name="bot_b", robot_type=RobotType.WHEELED)
    dataviz_b = robot_b.create_dataviz()
    dataviz_b.add_3d_map(topic="/map_3d", frame_id="map")

    client = _build_client()
    global_payload = client._build_global_visualizations_payload([dataviz_a, dataviz_b])
    point_cloud_entries = [
        entry for entry in global_payload if entry.get("type") == "point_cloud"
    ]
    assert len(point_cloud_entries) == 1
