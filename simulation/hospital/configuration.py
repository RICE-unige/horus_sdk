"""Explicit local demo profiles; no dependence on a user's shell environment."""

from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROBOTS = ("carter1", "carter2", "carter3")
DOMAIN = 87


def prepare_description(source, output):
    """Normalize the cached visual shell for ROS without editing the asset cache."""
    tree = ET.parse(source)
    for joint in tree.getroot().findall("joint"):
        if joint.get("type") == "revolute" and joint.find("limit") is None:
            if joint.get("name") != "joint_caster_base":
                raise ValueError(f"Unexpected unbounded revolute joint: {joint.get('name')}")
            joint.set("type", "continuous")
    tree.write(output, encoding="unicode", xml_declaration=True)
    return output


def slam_parameters():
    return {"/**": {"ros__parameters": {
        "use_sim_time": True, "solver_plugin": "solver_plugins::CeresSolver",
        "ceres_linear_solver": "SPARSE_NORMAL_CHOLESKY", "ceres_preconditioner": "SCHUR_JACOBI",
        "ceres_trust_strategy": "LEVENBERG_MARQUARDT", "ceres_dogleg_type": "TRADITIONAL_DOGLEG",
        "ceres_loss_function": "None", "mode": "mapping", "base_frame": "base_link",
        "odom_frame": "global_odom", "map_frame": "map", "scan_topic": "slam/scan",
        "scan_share_topic": "/hospital/localized_scan", "resolution": 0.05,
        "transform_publish_period": 0.0333333, "map_update_interval": 1.0,
        "max_laser_range": 20.0, "minimum_time_interval": 0.1,
        "transform_timeout": 0.3, "tf_buffer_duration": 30.0,
        "stack_size_to_use": 40000000, "enable_interactive_mode": False,
        "throttle_scans": 1, "use_scan_matching": True, "use_scan_barycenter": True,
        "minimum_travel_distance": 0.15, "minimum_travel_heading": 0.15,
        "scan_buffer_size": 10, "scan_buffer_maximum_scan_distance": 10.0,
        "link_match_minimum_response_fine": 0.1, "link_scan_maximum_distance": 1.5,
        "loop_search_maximum_distance": 3.0, "do_loop_closing": True,
        "loop_match_minimum_chain_size": 10, "loop_match_maximum_variance_coarse": 3.0,
        "loop_match_minimum_response_coarse": 0.35, "loop_match_minimum_response_fine": 0.45,
        "correlation_search_space_dimension": 0.5, "correlation_search_space_resolution": 0.01,
        "correlation_search_space_smear_deviation": 0.1, "loop_search_space_dimension": 8.0,
        "loop_search_space_resolution": 0.05, "loop_search_space_smear_deviation": 0.03,
        "distance_variance_penalty": 0.5, "angle_variance_penalty": 1.0,
        "fine_search_angle_offset": 0.00349, "coarse_search_angle_offset": 0.349,
        "coarse_angle_resolution": 0.0349, "minimum_angle_penalty": 0.9,
        "minimum_distance_penalty": 0.5, "use_response_expansion": True,
    }}}


def costmap(global_map):
    parameters = {
        "use_sim_time": True, "global_frame": "map" if global_map else "odom",
        "robot_base_frame": "base_link", "resolution": 0.05,
        "update_frequency": 3.0 if global_map else 5.0, "publish_frequency": 1.0,
        "footprint": "[[0.17,0.27],[0.17,-0.27],[-0.61,-0.27],[-0.61,0.27]]",
        "footprint_padding": 0.02, "transform_tolerance": 0.3,
        "always_send_full_costmap": False,
        "plugins": (["static_layer"] if global_map else []) + ["obstacle_layer", "depth_layer", "inflation_layer"],
        "static_layer": {"plugin": "nav2_costmap_2d::StaticLayer", "map_subscribe_transient_local": True,
                         "map_topic": "map"},
        "obstacle_layer": {"plugin": "nav2_costmap_2d::ObstacleLayer", "enabled": True,
                           "observation_sources": "scan", "scan": {
                               "topic": "slam/scan", "data_type": "LaserScan", "clearing": True,
                               "marking": True, "max_obstacle_height": 2.0,
                               "obstacle_max_range": 10.0, "raytrace_max_range": 12.0,
                               "inf_is_valid": True}},
        "inflation_layer": {"plugin": "nav2_costmap_2d::InflationLayer", "inflation_radius": 0.55,
                            "cost_scaling_factor": 3.0},
        "depth_layer": {"plugin": "nav2_costmap_2d::VoxelLayer", "enabled": True,
            "publish_voxel_map": False,"origin_z": 0.0,"z_resolution": .15,"z_voxels":16,
            "min_obstacle_height":.05,"max_obstacle_height":1.8,"mark_threshold":0,
            "observation_sources":"depth","depth":{"topic":"camera/obstacles","data_type":"PointCloud2",
                "marking":True,"clearing":True,"min_obstacle_height":.05,"max_obstacle_height":1.8,
                "obstacle_max_range":6.0,"raytrace_max_range":8.0}},
    }
    if global_map:
        parameters["track_unknown_space"] = True
    else:
        parameters.update(rolling_window=True, width=6, height=6)
    return {"ros__parameters": parameters}


def navigation_parameters():
    result = {
        "controller_server": {"ros__parameters": {
            "use_sim_time": True, "controller_frequency": 15.0, "enable_stamped_cmd_vel": False,
            "odom_topic": "odom", "controller_plugins": ["FollowPath"],
            "progress_checker_plugins": ["progress_checker"], "goal_checker_plugins": ["goal_checker"],
            "progress_checker": {"plugin": "nav2_controller::SimpleProgressChecker",
                                 "required_movement_radius": 0.15, "movement_time_allowance": 20.0},
            "goal_checker": {"plugin": "nav2_controller::SimpleGoalChecker", "stateful": True,
                             "xy_goal_tolerance": 0.20, "yaw_goal_tolerance": 0.20},
            "FollowPath": {"plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
                           "desired_linear_vel": 0.35, "lookahead_dist": 0.7,
                           "min_lookahead_dist": 0.4, "max_lookahead_dist": 1.2,
                           "use_velocity_scaled_lookahead_dist": True, "lookahead_time": 1.5,
                           "rotate_to_heading_angular_vel": 0.6, "use_rotate_to_heading": True,
                           "allow_reversing": False, "transform_tolerance": 0.3,
                           "use_collision_detection": True, "max_allowed_time_to_collision_up_to_carrot": 1.0},
        }},
        "planner_server": {"ros__parameters": {"use_sim_time": True, "expected_planner_frequency": 1.0,
            "planner_plugins": ["GridBased"], "GridBased": {"plugin": "nav2_navfn_planner::NavfnPlanner",
                "tolerance": 0.3, "use_astar": True, "allow_unknown": True}}},
        "behavior_server": {"ros__parameters": {"use_sim_time": True, "local_frame": "odom",
            "global_frame": "map", "robot_base_frame": "base_link", "enable_stamped_cmd_vel": False,
            "local_costmap_topic": "local_costmap/costmap_raw", "global_costmap_topic": "global_costmap/costmap_raw",
            "local_footprint_topic": "local_costmap/published_footprint", "global_footprint_topic": "global_costmap/published_footprint",
            "behavior_plugins": ["spin", "backup", "drive_on_heading", "wait"],
            "spin": {"plugin": "nav2_behaviors::Spin"}, "backup": {"plugin": "nav2_behaviors::BackUp"},
            "drive_on_heading": {"plugin": "nav2_behaviors::DriveOnHeading"}, "wait": {"plugin": "nav2_behaviors::Wait"}}},
        "bt_navigator": {"ros__parameters": {"use_sim_time": True, "global_frame": "map",
            "robot_base_frame": "base_link", "odom_topic": "odom", "transform_tolerance": 0.3,
            "navigators": ["navigate_to_pose", "navigate_through_poses"],
            "navigate_to_pose": {"plugin": "nav2_bt_navigator::NavigateToPoseNavigator"},
            "navigate_through_poses": {"plugin": "nav2_bt_navigator::NavigateThroughPosesNavigator"}}},
        "local_costmap": {"local_costmap": costmap(False)},
        "global_costmap": {"global_costmap": costmap(True)},
    }
    return result


def navigation_topic_overrides(name):
    result = {"map_topic":f"/{name}/map","odom_topic":f"/{name}/odom"}
    for costmap_name in ("local_costmap","global_costmap"):
        prefix = f"{costmap_name}.{costmap_name}.ros__parameters"
        result[prefix+".obstacle_layer.scan.topic"] = f"/{name}/slam/scan"
        result[prefix+".depth_layer.depth.topic"] = f"/{name}/camera/obstacles"
    return result


def mapping_parameters(voxel_size=.05):
    return {"/**": {"ros__parameters": {
        "use_sim_time": True, "global_frame": "global_odom", "pose_frame": "carter1/base_link",
        "num_cameras": 3, "use_tf_transforms": True, "mapping_type": "static_tsdf",
        "voxel_size": voxel_size, "use_depth": True, "use_color": True, "use_lidar": False,
        "input_qos": "SYSTEM_DEFAULT", "maximum_input_queue_length": 6,
        "integrate_depth_rate_hz": 15.0, "integrate_color_rate_hz": 15.0,
        "update_mesh_rate_hz": 2.0, "publish_layer_rate_hz": 2.0,
        "update_esdf_rate_hz": 0.0, "publish_esdf_distance_slice": False,
        "decay_tsdf_rate_hz": 0.0, "clear_map_outside_radius_rate_hz": 0.0,
        "map_clearing_radius_m": -1.0, "map_clearing_frame_id": "carter1/base_link",
        "layer_visualization_exclusion_height_m": -1.0,
        "layer_visualization_exclusion_radius_m": -1.0, "layer_streamer_bandwidth_limit_mbps": 30.0,
        "print_rates_to_console": False, "print_timings_to_console": False,
        "static_mapper": {"do_depth_preprocessing": False, "workspace_bounds_type": "unbounded",
            "projective_integrator_max_integration_distance_m": 8.0,
            "projective_integrator_truncation_distance_vox": 4.0, "mesh_integrator_min_weight": 2.0,
            "mesh_integrator_weld_vertices": True},
    }}}


def write_configuration(directory, voxel_size=.05):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for name, parameters in (("slam.yaml", slam_parameters()), ("navigation.yaml", navigation_parameters()),
                             ("nvblox.yaml", mapping_parameters(voxel_size))):
        (directory / name).write_text(yaml.safe_dump(parameters, sort_keys=False))
