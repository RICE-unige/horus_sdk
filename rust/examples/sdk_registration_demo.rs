use clap::Parser;
use horus::core::types::RobotType;
use horus::robot::{register_robots, Robot, RobotDimensions};
use horus::sensors::Camera;
use std::collections::HashMap;
use std::sync::Arc;

#[derive(Debug, Parser)]
#[command(about = "Register multiple robots with Horus (Rust parity demo).")]
struct Args {
    #[arg(long, default_value = "test_bot")]
    robot_name: String,
    #[arg(long, default_value = "")]
    robot_names: String,
    #[arg(long, default_value_t = 10)]
    robot_count: usize,
    #[arg(long, default_value_t = true)]
    keep_alive: bool,
    #[arg(long, default_value_t = 0.8)]
    length: f64,
    #[arg(long, default_value_t = 0.6)]
    width: f64,
    #[arg(long, default_value_t = 0.4)]
    height: f64,
    #[arg(long, default_value_t = true)]
    with_camera: bool,
    #[arg(long, default_value_t = true)]
    with_occupancy_grid: bool,
    #[arg(long, default_value = "compressed")]
    camera_image_type: String,
    #[arg(long)]
    camera_streaming_type: Option<String>,
    #[arg(long)]
    camera_minimap_streaming_type: Option<String>,
    #[arg(long)]
    camera_teleop_streaming_type: Option<String>,
    #[arg(long, default_value = "minimap")]
    camera_startup_mode: String,
    #[arg(long, default_value = "160x90,192x108,224x126,256x144,320x180,426x240")]
    camera_resolutions: String,
    #[arg(long, default_value_t = true)]
    vary_camera_resolution: bool,
    #[arg(long, default_value = "/map")]
    occupancy_topic: String,
    #[arg(long, default_value = "map")]
    occupancy_frame: String,
    #[arg(long)]
    occupancy_position_scale: Option<f64>,
    #[arg(long)]
    workspace_scale: Option<f64>,
}

fn parse_resolution_list(raw_value: &str) -> Vec<(u32, u32)> {
    raw_value
        .split(',')
        .filter_map(|token| {
            let item = token.trim().to_ascii_lowercase();
            let (left, right) = item.split_once('x')?;
            let width = left.parse::<u32>().ok()?.max(16);
            let height = right.parse::<u32>().ok()?.max(16);
            Some((width, height))
        })
        .collect()
}

fn resolve_robot_names(args: &Args) -> Vec<String> {
    if !args.robot_names.trim().is_empty() {
        let explicit: Vec<String> = args
            .robot_names
            .split(',')
            .map(str::trim)
            .filter(|name| !name.is_empty())
            .map(ToString::to_string)
            .collect();
        if !explicit.is_empty() {
            return explicit;
        }
    }
    if args.robot_count <= 1 {
        return vec![args.robot_name.clone()];
    }
    (0..args.robot_count)
        .map(|index| format!("{}_{}", args.robot_name, index + 1))
        .collect()
}

fn main() {
    let args = Args::parse();
    let robot_names = resolve_robot_names(&args);
    let mut resolutions = parse_resolution_list(&args.camera_resolutions);
    if resolutions.is_empty() {
        resolutions.push((160, 90));
    }

    let legacy_streaming = args
        .camera_streaming_type
        .clone()
        .unwrap_or_default()
        .trim()
        .to_ascii_lowercase();
    let minimap_streaming = args
        .camera_minimap_streaming_type
        .clone()
        .unwrap_or_else(|| legacy_streaming.clone());
    let minimap_streaming = if minimap_streaming.is_empty() {
        "ros".to_string()
    } else {
        minimap_streaming
    };
    let teleop_streaming = args
        .camera_teleop_streaming_type
        .clone()
        .unwrap_or_else(|| legacy_streaming.clone());
    let teleop_streaming = if teleop_streaming.is_empty() {
        "webrtc".to_string()
    } else {
        teleop_streaming
    };
    let startup_mode = {
        let normalized = args.camera_startup_mode.trim().to_ascii_lowercase();
        if normalized == "teleop" {
            "teleop".to_string()
        } else {
            "minimap".to_string()
        }
    };
    let legacy_payload_streaming = if legacy_streaming.is_empty() {
        minimap_streaming.clone()
    } else {
        legacy_streaming
    };

    let mut robots = Vec::new();
    let mut datavizs = Vec::new();

    for (index, name) in robot_names.iter().enumerate() {
        let dimensions = RobotDimensions::new(
            args.length + (index as f64 * 0.1),
            args.width + (index as f64 * 0.05),
            args.height + (index as f64 * 0.03),
        );
        let mut robot = Robot::with_dimensions(name.clone(), RobotType::Wheeled, dimensions);
        robot.configure_ros_binding("prefixed", "prefixed", Some("base_link"));
        robot.configure_robot_manager(true, true, true, true);
        robot.configure_workspace_compass(true, 8088, "auto");
        robot.configure_workspace_tutorial("multi_robot_intro");
        if index == 0 {
            robot.configure_local_body_model("rosbot", true);
        }
        robot.set_metadata(
            "teleop_config",
            serde_json::json!({
                "enabled": true,
                "robot_profile": "wheeled",
                "response_mode": "analog",
                "publish_rate_hz": 60.0,
                "axes": {
                    "deadzone": 0.12,
                    "linear_xy_max_mps": 1.1 + (index as f64 * 0.05),
                    "angular_z_max_rps": 1.4
                }
            }),
        );
        robot.set_metadata(
            "task_config",
            serde_json::json!({
                "go_to_point": {
                    "frame_id": "map",
                    "position_tolerance_m": 0.18
                },
                "waypoint": {
                    "frame_id": "map",
                    "position_tolerance_m": 0.25
                }
            }),
        );

        if args.with_camera {
            let image_type = args.camera_image_type.trim().to_ascii_lowercase();
            let (topic, encoding) = if image_type == "compressed" {
                (format!("/{name}/camera/image_raw/compressed"), "jpeg")
            } else {
                (format!("/{name}/camera/image_raw"), "rgb8")
            };

            let (width, height) = if args.vary_camera_resolution {
                resolutions[index % resolutions.len()]
            } else {
                resolutions[0]
            };

            let mut camera = Camera::try_new_with_profiles(
                "front_camera",
                format!("{name}/camera_link"),
                topic,
                &legacy_payload_streaming,
                &minimap_streaming,
                &teleop_streaming,
                &startup_mode,
            )
            .unwrap_or_else(|_| {
                Camera::new(
                    "front_camera",
                    format!("{name}/camera_link"),
                    format!("/{name}/camera/image_raw"),
                )
            });

            camera.resolution = (width, height);
            camera.fps = 6;
            camera.encoding = encoding.to_string();
            camera.minimap_topic = format!("/{name}/camera/minimap/compressed");
            camera.teleop_topic = format!("/{name}/camera/image_raw/compressed");
            camera.minimap_image_type = "compressed".to_string();
            camera.teleop_image_type = "compressed".to_string();
            camera.minimap_max_fps = 12;
            camera.add_metadata("image_type", serde_json::json!(image_type));
            camera.add_metadata(
                "streaming_type",
                serde_json::json!(legacy_payload_streaming),
            );
            camera.add_metadata(
                "minimap_streaming_type",
                serde_json::json!(minimap_streaming.clone()),
            );
            camera.add_metadata(
                "teleop_streaming_type",
                serde_json::json!(teleop_streaming.clone()),
            );
            camera.add_metadata("startup_mode", serde_json::json!(startup_mode.clone()));
            camera.add_metadata("display_mode", serde_json::json!("projected"));
            camera.add_metadata("use_tf", serde_json::json!(true));
            camera.add_metadata(
                "projection_target_frame",
                serde_json::json!(format!("{name}/camera_link")),
            );
            camera.add_metadata(
                "projected_scale_multiplier",
                serde_json::json!({"x": 1.0, "y": 1.0, "z": 1.0}),
            );

            if robot.add_sensor(Arc::new(camera)).is_err() {
                eprintln!("failed to add camera sensor for {}", name);
            }
        }

        let mut dataviz = robot.create_dataviz(None);
        if args.with_occupancy_grid {
            let mut render_options = std::collections::HashMap::new();
            if let Some(scale) = args.occupancy_position_scale {
                render_options.insert(
                    "position_scale".to_string(),
                    serde_json::json!(scale.max(0.01)),
                );
            }
            dataviz.add_occupancy_grid(
                &args.occupancy_topic,
                &args.occupancy_frame,
                Some(render_options),
            );
        }
        let global_path = robot.resolve_topic("global_path");
        let local_path = robot.resolve_topic("local_path");
        let trajectory = robot.resolve_topic("trajectory");
        robot.add_path_planning_to_dataviz(
            &mut dataviz,
            Some(&global_path),
            Some(&local_path),
            Some(&trajectory),
        );
        robot.add_navigation_safety_to_dataviz(&mut dataviz);

        let mut point_options = HashMap::new();
        point_options.insert("point_size".to_string(), serde_json::json!(0.01));
        point_options.insert("max_points_per_frame".to_string(), serde_json::json!(60000));
        point_options.insert(
            "enable_adaptive_quality".to_string(),
            serde_json::json!(true),
        );
        point_options.insert("render_mode".to_string(), serde_json::json!("opaque_fast"));
        dataviz.add_3d_map(&format!("/{name}/map_points"), "map", Some(point_options));

        let mut mesh_options = HashMap::new();
        mesh_options.insert("max_triangles".to_string(), serde_json::json!(200000));
        mesh_options.insert(
            "source_coordinate_space".to_string(),
            serde_json::json!("enu"),
        );
        dataviz.add_3d_mesh(&format!("/{name}/map_mesh"), "map", Some(mesh_options));

        let mut octomap_options = HashMap::new();
        octomap_options.insert(
            "native_topic".to_string(),
            serde_json::json!(format!("/{name}/octomap_binary")),
        );
        octomap_options.insert("native_frame".to_string(), serde_json::json!("map"));
        octomap_options.insert("render_mode".to_string(), serde_json::json!("surface_mesh"));
        dataviz.add_3d_octomap(
            &format!("/{name}/octomap_mesh"),
            "map",
            Some(octomap_options),
        );
        dataviz.add_semantic_box(
            &format!("{name}_dock"),
            "Dock",
            (index as f32, 0.0, 0.0),
            (0.6, 0.4, 0.3),
            "map",
            None,
        );

        datavizs.push(dataviz);
        robots.push(robot);
    }

    println!("Registering {} robot(s)...", robots.len());
    let (ok, result) = register_robots(
        &mut robots,
        Some(datavizs),
        args.keep_alive,
        true,
        10.0,
        args.workspace_scale,
    );
    if !ok {
        eprintln!("Registration failed: {result}");
        return;
    }
    println!("Registration result: {result}");
}
