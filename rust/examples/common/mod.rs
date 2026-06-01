#![allow(dead_code)]

use horus::{
    bridge::RobotRegistryClient, Camera, DataViz, LaserScan, MinimapViewConfig,
    NavigationTaskConfig, ProjectedViewConfig, Robot, RobotDescriptionConfig, RobotDimensions,
    RobotManagerConfig, RobotType, TeleopConfig,
};
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;

pub struct RegistrationSet {
    pub robots: Vec<Robot>,
    pub datavizs: Vec<DataViz>,
}

pub fn topic(robot_name: &str, suffix: &str) -> String {
    format!("/{robot_name}/{suffix}")
}

pub fn options(items: &[(&str, serde_json::Value)]) -> HashMap<String, serde_json::Value> {
    items
        .iter()
        .map(|(key, value)| ((*key).to_string(), value.clone()))
        .collect()
}

pub fn standard_camera(robot_name: &str) -> Camera {
    let mut camera = Camera::try_new_with_profiles(
        "front_camera",
        format!("{robot_name}/camera_link"),
        topic(robot_name, "camera/image_raw/compressed"),
        "ros",
        "ros",
        "ros",
        "minimap",
    )
    .unwrap();
    camera.resolution = (160, 90);
    camera.fps = 6;
    camera.encoding = "jpeg".to_string();
    camera.minimap_image_type = "compressed".to_string();
    camera.teleop_image_type = "compressed".to_string();
    camera.add_metadata("image_type", serde_json::json!("compressed"));
    camera.configure_projected_view(ProjectedViewConfig {
        image_scale: Some(1.0),
        focal_length_scale: Some(0.55),
        show_frustum: Some(true),
        frustum_color: Some("#E6E6E0A0".to_string()),
        ..Default::default()
    });
    camera.configure_minimap_view(MinimapViewConfig {
        size: Some(10.0),
        position_offset: Some((0.0, 2.0, 0.0)),
        face_camera: Some(true),
        rotation_offset: Some((90.0, 0.0, 0.0)),
    });
    camera
}

pub fn stereo_camera(robot_name: &str, high_bandwidth: bool) -> Camera {
    let eye_resolution = if high_bandwidth {
        (1920, 1080)
    } else {
        (960, 540)
    };
    let mut camera = Camera::try_new_with_profiles(
        "front_stereo_camera",
        format!("{robot_name}/camera_link"),
        topic(robot_name, "camera/minimap/image_raw/compressed"),
        "ros",
        "ros",
        "webrtc",
        "minimap",
    )
    .unwrap();
    camera.is_stereo = true;
    camera.resolution = (eye_resolution.0 * 2, eye_resolution.1);
    camera.fps = if high_bandwidth { 90 } else { 10 };
    camera.encoding = "jpeg".to_string();
    camera.minimap_topic = topic(robot_name, "camera/minimap/image_raw/compressed");
    camera.teleop_topic = topic(robot_name, "camera/teleop/image_raw/compressed");
    camera.minimap_image_type = "compressed".to_string();
    camera.teleop_image_type = "compressed".to_string();
    camera.minimap_max_fps = 30;
    camera.stereo_layout = "side_by_side".to_string();
    camera.teleop_stereo_layout = "side_by_side".to_string();
    camera.configure_projected_view(ProjectedViewConfig {
        image_scale: Some(0.06),
        focal_length_scale: Some(0.09),
        ..Default::default()
    });
    camera
}

pub fn standard_laser_scan(name: &str, frame_id: String, topic: String) -> LaserScan {
    let mut scan = LaserScan::new(name, frame_id, topic);
    scan.color = "#6ED7FF".to_string();
    scan.point_size = 0.025;
    scan
}

pub fn standard_robot(
    name: &str,
    robot_type: RobotType,
    dimensions: RobotDimensions,
    profile: &str,
) -> Robot {
    standard_robot_with_base(name, robot_type, dimensions, profile, "base_link")
}

pub fn standard_robot_with_base(
    name: &str,
    robot_type: RobotType,
    dimensions: RobotDimensions,
    profile: &str,
    base_frame: &str,
) -> Robot {
    let mut robot = Robot::with_dimensions(name.to_string(), robot_type, dimensions);
    robot.configure_ros_binding("prefixed", "prefixed", Some(base_frame));
    robot.configure_robot_manager_with(RobotManagerConfig::default());
    robot.configure_teleop(TeleopConfig {
        command_topic: Some(topic(name, "cmd_vel")),
        robot_profile: Some(profile.to_string()),
        ..Default::default()
    });
    robot.configure_navigation_tasks(NavigationTaskConfig {
        goal_topic: Some(topic(name, "goal_pose")),
        cancel_topic: Some(topic(name, "goal_cancel")),
        goal_status_topic: Some(topic(name, "goal_status")),
        waypoint_path_topic: Some(topic(name, "waypoint_path")),
        waypoint_status_topic: Some(topic(name, "waypoint_status")),
        ..Default::default()
    });
    robot.add_sensor(Arc::new(standard_camera(name))).unwrap();
    robot
}

pub fn standard_dataviz(robot: &Robot) -> DataViz {
    let mut dataviz = robot.create_dataviz(None);
    let global_path = robot.resolve_topic("global_path");
    let local_path = robot.resolve_topic("local_path");
    robot.add_path_planning_to_dataviz(&mut dataviz, Some(&global_path), Some(&local_path), None);
    robot.add_navigation_safety_to_dataviz(&mut dataviz);
    dataviz
}

pub fn standard_fleet(
    names: &[&str],
    robot_type: RobotType,
    dimensions: RobotDimensions,
    profile: &str,
) -> RegistrationSet {
    let mut set = RegistrationSet {
        robots: Vec::new(),
        datavizs: Vec::new(),
    };
    for name in names {
        let robot = standard_robot(name, robot_type, dimensions.clone(), profile);
        let dataviz = standard_dataviz(&robot);
        set.robots.push(robot);
        set.datavizs.push(dataviz);
    }
    set
}

pub fn add_world_maps(dataviz: &mut DataViz) {
    dataviz.add_occupancy_grid(
        "/map",
        "map",
        Some(options(&[
            ("color_free", serde_json::json!("#222222")),
            ("color_occupied", serde_json::json!("#F4F4F4")),
            ("alpha", serde_json::json!(0.85)),
        ])),
    );
    dataviz.add_3d_map(
        "/map_3d",
        "map",
        Some(options(&[
            ("point_size", serde_json::json!(0.045)),
            ("max_points_per_frame", serde_json::json!(0)),
            ("render_all_points", serde_json::json!(true)),
            (
                "auto_point_size_by_workspace_scale",
                serde_json::json!(true),
            ),
            ("point_shape", serde_json::json!("circle")),
            ("color", serde_json::json!("#6ED7FF")),
        ])),
    );
    dataviz.add_3d_mesh(
        "/map_3d_mesh",
        "map",
        Some(options(&[
            ("max_triangles", serde_json::json!(60000)),
            ("use_vertex_colors", serde_json::json!(true)),
        ])),
    );
    dataviz.add_3d_octomap(
        "/map_3d_octomap_mesh",
        "map",
        Some(options(&[
            ("render_mode", serde_json::json!("surface_mesh")),
            ("native_topic", serde_json::json!("/map_3d_octomap")),
        ])),
    );
}

pub fn add_semantic_boxes(dataviz: &mut DataViz) {
    dataviz.add_semantic_box(
        "person_1",
        "person",
        (1.6, 0.8, 0.9),
        (0.45, 0.45, 1.8),
        "map",
        None,
    );
    dataviz.add_semantic_box(
        "person_2",
        "person",
        (-1.2, 1.4, 0.9),
        (0.45, 0.45, 1.8),
        "map",
        None,
    );
    dataviz.add_semantic_box(
        "equipment_1",
        "equipment",
        (0.3, -1.6, 0.4),
        (0.8, 0.6, 0.8),
        "map",
        None,
    );
}

fn asset_dir() -> PathBuf {
    PathBuf::from("../python/examples/.local_assets/robot_descriptions")
}

pub fn configure_robot_description(robot: &mut Robot, urdf_file: &str, base_frame: &str) {
    configure_robot_description_with(robot, urdf_file, base_frame, 90000, "preview_mesh");
}

pub fn configure_robot_description_with(
    robot: &mut Robot,
    urdf_file: &str,
    base_frame: &str,
    triangle_budget: i64,
    body_mesh_mode: &str,
) {
    robot.configure_ros_binding("prefixed", "prefixed", Some(base_frame));
    let mut config = RobotDescriptionConfig::new(
        asset_dir().join(urdf_file).display().to_string(),
        base_frame,
    );
    config.visual_mesh_triangle_budget = triangle_budget;
    config.body_mesh_mode = body_mesh_mode.to_string();
    robot.configure_robot_description(config);
}

pub fn robot_description_fleet() -> RegistrationSet {
    let models = [
        (
            "anymal_c",
            RobotType::Legged,
            RobotDimensions::new(0.95, 0.55, 0.70),
            "base",
            "anymal_c.urdf",
        ),
        (
            "jackal",
            RobotType::Wheeled,
            RobotDimensions::new(0.51, 0.43, 0.25),
            "base_link",
            "jackal.urdf",
        ),
        (
            "go1",
            RobotType::Legged,
            RobotDimensions::new(0.65, 0.32, 0.45),
            "base",
            "go1.urdf",
        ),
        (
            "h1",
            RobotType::Legged,
            RobotDimensions::new(0.55, 0.38, 1.25),
            "pelvis",
            "h1.urdf",
        ),
    ];
    let mut set = RegistrationSet {
        robots: Vec::new(),
        datavizs: Vec::new(),
    };
    for (name, robot_type, dimensions, base_frame, urdf_file) in models {
        let mut robot = standard_robot_with_base(
            name,
            robot_type,
            dimensions,
            robot_type.as_str(),
            base_frame,
        );
        configure_robot_description(&mut robot, urdf_file, base_frame);
        let dataviz = standard_dataviz(&robot);
        set.robots.push(robot);
        set.datavizs.push(dataviz);
    }
    set
}

pub fn register_or_fail(set: &mut RegistrationSet) {
    register_with(set, 0.1, true);
}

pub fn register_with(set: &mut RegistrationSet, workspace_scale: f64, keep_alive: bool) {
    let _ = keep_alive;
    if set.robots.is_empty() || set.robots.len() != set.datavizs.len() {
        panic!("HORUS native payload validation needs matching robots and DataViz entries");
    }
    let client = RobotRegistryClient::new();
    let global_visualizations = client.build_global_visualizations_payload(&set.datavizs);
    for (robot, dataviz) in set.robots.iter().zip(set.datavizs.iter()) {
        let payload = client.build_robot_config_dict(
            robot,
            dataviz,
            Some(global_visualizations.clone()),
            Some(workspace_scale),
        );
        if payload.robot_name.is_empty() || payload.robot_type.is_empty() {
            panic!("HORUS native payload validation failed");
        }
    }
    println!(
        "HORUS native payload validation OK. Live registration transport is Python-only for now."
    );
}

pub fn robot_manager_without_controls() -> RobotManagerConfig {
    RobotManagerConfig {
        teleop: false,
        tasks: false,
        ..Default::default()
    }
}
