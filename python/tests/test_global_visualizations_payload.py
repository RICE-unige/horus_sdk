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
            "render_mode": "transparent_hq",
            "enable_view_frustum_culling": False,
            "frustum_padding": 0.12,
            "enable_subpixel_culling": False,
            "min_screen_radius_px": 1.0,
            "visible_points_budget": 90000,
            "max_visible_points_budget": 160000,
            "map_static_mode": False,
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
    assert point_cloud_entry["point_cloud"]["render_mode"] == "transparent_hq"
    assert point_cloud_entry["point_cloud"]["enable_view_frustum_culling"] is False
    assert point_cloud_entry["point_cloud"]["frustum_padding"] == 0.12
    assert point_cloud_entry["point_cloud"]["enable_subpixel_culling"] is False
    assert point_cloud_entry["point_cloud"]["min_screen_radius_px"] == 1.0
    assert point_cloud_entry["point_cloud"]["visible_points_budget"] == 90000
    assert point_cloud_entry["point_cloud"]["max_visible_points_budget"] == 160000
    assert point_cloud_entry["point_cloud"]["map_static_mode"] is False


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
    assert point_cloud["point_size"] == 0.05
    assert point_cloud["render_mode"] == "opaque_fast"
    assert point_cloud["enable_view_frustum_culling"] is True
    assert point_cloud["frustum_padding"] == 0.03
    assert point_cloud["enable_subpixel_culling"] is True
    assert point_cloud["min_screen_radius_px"] == 0.8
    assert point_cloud["visible_points_budget"] == 120000
    assert point_cloud["max_visible_points_budget"] == 200000
    assert point_cloud["map_static_mode"] is True


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


def test_mesh_serialized_as_global_visualization():
    robot = Robot(name="test_bot", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    dataviz.add_3d_mesh(
        topic="/map_3d_mesh",
        frame_id="map",
        render_options={
            "use_vertex_colors": False,
            "alpha": 0.85,
            "double_sided": False,
            "max_triangles": 150000,
            "source_coordinate_space": "optical",
        },
    )

    client = _build_client()
    config = client._build_robot_config_dict(robot, dataviz)
    mesh_entries = [
        entry for entry in config["global_visualizations"] if entry.get("type") == "mesh"
    ]
    assert len(mesh_entries) == 1

    mesh_entry = mesh_entries[0]
    assert mesh_entry["scope"] == "global"
    assert mesh_entry["topic"] == "/map_3d_mesh"
    assert mesh_entry["frame"] == "map"
    assert mesh_entry["mesh"]["use_vertex_colors"] is False
    assert mesh_entry["mesh"]["alpha"] == 0.85
    assert mesh_entry["mesh"]["double_sided"] is False
    assert mesh_entry["mesh"]["max_triangles"] == 150000
    assert mesh_entry["mesh"]["source_coordinate_space"] == "optical"


def test_mesh_defaults_are_emitted_without_render_options():
    robot = Robot(name="test_bot", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    dataviz.add_3d_mesh(topic="/map_3d_mesh", frame_id="map")

    client = _build_client()
    config = client._build_robot_config_dict(robot, dataviz)
    entry = next(
        item for item in config["global_visualizations"] if item.get("type") == "mesh"
    )
    mesh = entry["mesh"]
    assert mesh["use_vertex_colors"] is True
    assert mesh["alpha"] == 1.0
    assert mesh["double_sided"] is True
    assert mesh["max_triangles"] == 200000
    assert mesh["source_coordinate_space"] == "enu"


def test_global_visualization_dedupes_mesh_across_multiple_robots():
    robot_a = Robot(name="bot_a", robot_type=RobotType.WHEELED)
    dataviz_a = robot_a.create_dataviz()
    dataviz_a.add_3d_mesh(topic="/map_3d_mesh", frame_id="map")

    robot_b = Robot(name="bot_b", robot_type=RobotType.WHEELED)
    dataviz_b = robot_b.create_dataviz()
    dataviz_b.add_3d_mesh(topic="/map_3d_mesh", frame_id="map")

    client = _build_client()
    global_payload = client._build_global_visualizations_payload([dataviz_a, dataviz_b])
    mesh_entries = [entry for entry in global_payload if entry.get("type") == "mesh"]
    assert len(mesh_entries) == 1


def test_mesh_overrides_are_coerced_and_clamped():
    robot = Robot(name="test_bot", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    dataviz.add_3d_mesh(
        topic="/map_3d_mesh",
        frame_id="map",
        render_options={
            "use_vertex_colors": "0",
            "alpha": 3.0,
            "double_sided": "false",
            "max_triangles": -10,
            "source_coordinate_space": "unknown_space",
        },
    )

    client = _build_client()
    config = client._build_robot_config_dict(robot, dataviz)
    entry = next(
        item for item in config["global_visualizations"] if item.get("type") == "mesh"
    )
    mesh = entry["mesh"]
    assert mesh["use_vertex_colors"] is False
    assert mesh["alpha"] == 1.0
    assert mesh["double_sided"] is False
    assert mesh["max_triangles"] == 1000
    assert mesh["source_coordinate_space"] == "enu"


def test_octomap_serialized_as_global_visualization():
    robot = Robot(name="test_bot", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    dataviz.add_3d_octomap(
        topic="/map_3d_octomap_mesh",
        frame_id="map",
        render_options={
            "render_mode": "surface_mesh",
            "use_vertex_colors": False,
            "alpha": 0.85,
            "double_sided": False,
            "max_triangles": 125000,
            "source_coordinate_space": "optical",
            "native_topic": "/map_3d_octomap",
            "native_frame": "map",
            "native_binary_only": True,
        },
    )

    client = _build_client()
    config = client._build_robot_config_dict(robot, dataviz)
    octomap_entries = [
        entry for entry in config["global_visualizations"] if entry.get("type") == "octomap"
    ]
    assert len(octomap_entries) == 1

    octomap_entry = octomap_entries[0]
    assert octomap_entry["scope"] == "global"
    assert octomap_entry["topic"] == "/map_3d_octomap_mesh"
    assert octomap_entry["frame"] == "map"
    assert octomap_entry["octomap"]["render_mode"] == "surface_mesh"
    assert octomap_entry["octomap"]["use_vertex_colors"] is False
    assert octomap_entry["octomap"]["alpha"] == 0.85
    assert octomap_entry["octomap"]["double_sided"] is False
    assert octomap_entry["octomap"]["max_triangles"] == 125000
    assert octomap_entry["octomap"]["source_coordinate_space"] == "optical"
    assert octomap_entry["octomap"]["native_topic"] == "/map_3d_octomap"
    assert octomap_entry["octomap"]["native_frame"] == "map"
    assert octomap_entry["octomap"]["native_binary_only"] is True


def test_octomap_defaults_are_emitted_without_render_options():
    robot = Robot(name="test_bot", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    dataviz.add_3d_octomap(topic="/map_3d_octomap_mesh", frame_id="map")

    client = _build_client()
    config = client._build_robot_config_dict(robot, dataviz)
    entry = next(
        item for item in config["global_visualizations"] if item.get("type") == "octomap"
    )
    octomap = entry["octomap"]
    assert octomap["render_mode"] == "surface_mesh"
    assert octomap["use_vertex_colors"] is True
    assert octomap["alpha"] == 1.0
    assert octomap["double_sided"] is False
    assert octomap["max_triangles"] == 60000
    assert octomap["source_coordinate_space"] == "enu"
    assert octomap["native_topic"] == "/map_3d_octomap"
    assert octomap["native_frame"] == "map"
    assert octomap["native_binary_only"] is True


def test_global_visualization_dedupes_octomap_across_multiple_robots():
    robot_a = Robot(name="bot_a", robot_type=RobotType.WHEELED)
    dataviz_a = robot_a.create_dataviz()
    dataviz_a.add_3d_octomap(topic="/map_3d_octomap_mesh", frame_id="map")

    robot_b = Robot(name="bot_b", robot_type=RobotType.WHEELED)
    dataviz_b = robot_b.create_dataviz()
    dataviz_b.add_3d_octomap(topic="/map_3d_octomap_mesh", frame_id="map")

    client = _build_client()
    global_payload = client._build_global_visualizations_payload([dataviz_a, dataviz_b])
    octomap_entries = [entry for entry in global_payload if entry.get("type") == "octomap"]
    assert len(octomap_entries) == 1


def test_semantic_box_serialized_as_global_visualization():
    robot = Robot(name="test_bot", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    dataviz.add_semantic_box(
        semantic_id="injured_person_1",
        label="Injured Person",
        center=(1.25, -0.45, 0.0),
        size=(0.8, 0.6, 1.7),
        frame_id="map",
        rotation_offset_euler=(0.0, 0.0, 12.0),
    )

    client = _build_client()
    config = client._build_robot_config_dict(robot, dataviz)
    semantic_entries = [
        entry for entry in config["global_visualizations"] if entry.get("type") == "semantic_box"
    ]
    assert len(semantic_entries) == 1

    semantic_entry = semantic_entries[0]
    assert semantic_entry["scope"] == "global"
    assert semantic_entry["topic"] == "/horus/semantic_boxes/injured_person_1"
    assert semantic_entry["frame"] == "map"
    assert semantic_entry["semantic_box"]["id"] == "injured_person_1"
    assert semantic_entry["semantic_box"]["label"] == "Injured Person"
    assert semantic_entry["semantic_box"]["center"] == {"x": 1.25, "y": -0.45, "z": 0.0}
    assert semantic_entry["semantic_box"]["size"] == {"x": 0.8, "y": 0.6, "z": 1.7}
    assert semantic_entry["semantic_box"]["rotation_offset_euler"] == {
        "x": 0.0,
        "y": 0.0,
        "z": 12.0,
    }


def test_global_visualization_dedupes_semantic_boxes_by_id_and_frame():
    robot_a = Robot(name="bot_a", robot_type=RobotType.WHEELED)
    dataviz_a = robot_a.create_dataviz()
    dataviz_a.add_semantic_box(
        semantic_id="stairs_1",
        label="Stairs",
        center=(0.0, 0.0, 0.0),
        size=(1.0, 2.0, 0.4),
        frame_id="map",
    )

    robot_b = Robot(name="bot_b", robot_type=RobotType.WHEELED)
    dataviz_b = robot_b.create_dataviz()
    dataviz_b.add_semantic_box(
        semantic_id="stairs_1",
        label="Stairs",
        center=(3.0, 2.0, 0.0),
        size=(1.2, 2.2, 0.45),
        frame_id="map",
    )

    client = _build_client()
    global_payload = client._build_global_visualizations_payload([dataviz_a, dataviz_b])
    semantic_entries = [entry for entry in global_payload if entry.get("type") == "semantic_box"]
    assert len(semantic_entries) == 1
