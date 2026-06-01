mod common;

use horus::{
    Camera, ImmersiveViewConfig, LaserScan, MinimapViewConfig, ProjectedViewConfig, Robot,
    RobotDescriptionConfig, RobotDimensions, RobotManagerConfig, RobotType, TeleopConfig,
};
use std::sync::Arc;

fn main() {
    let robot_name = "unitree_go1";
    let mut robot = Robot::with_dimensions(
        robot_name,
        RobotType::Legged,
        RobotDimensions::new(0.54, 0.30, 0.18),
    );
    robot.configure_ros_binding("prefixed", "prefixed", Some("trunk"));

    let mut description = RobotDescriptionConfig::new(
        "/home/omotoye/Unitree_ros2_to_real/ros2_ws/src/go1_description/urdf/go1.urdf",
        "trunk",
    );
    description.chunk_size_bytes = 64000;
    description.visual_mesh_triangle_budget = 220000;
    description.body_mesh_mode = "runtime_high_mesh".to_string();
    robot.configure_robot_description(description);

    robot.configure_robot_manager_with(RobotManagerConfig::default());
    robot.configure_teleop(TeleopConfig {
        command_topic: Some(common::topic(robot_name, "cmd_vel")),
        robot_profile: Some("legged".to_string()),
        linear_xy_max_mps: Some(0.25),
        linear_z_max_mps: Some(0.0),
        angular_z_max_rps: Some(0.9),
        invert_angular_z: Some(true),
        ..Default::default()
    });

    let mut camera = Camera::new(
        "front_camera",
        "unitree_go1/camera_optical_left_face",
        "/unitree_go1/front_camera/left/color/image_rect/compressed",
    );
    camera.resolution = (928, 800);
    camera.fps = 30;
    camera.fov = 116.0;
    camera.encoding = "jpeg".to_string();
    camera.minimap_image_type = "compressed".to_string();
    camera.teleop_image_type = "compressed".to_string();
    camera.configure_projected_view(ProjectedViewConfig {
        image_scale: Some(0.0875),
        focal_length_scale: Some(0.0875),
        show_frustum: Some(true),
        ..Default::default()
    });
    camera.configure_immersive_view(ImmersiveViewConfig {
        ros_flip_x: Some(false),
        ros_flip_y: Some(true),
    });
    camera.configure_minimap_view(MinimapViewConfig {
        size: Some(1.0),
        ..Default::default()
    });
    robot.add_sensor(Arc::new(camera)).unwrap();

    let mut scan = LaserScan::new("front_laser_scan", "unitree_go1/laser", "/unitree_go1/scan");
    scan.min_angle = -2.3561945;
    scan.max_angle = 2.3561945;
    scan.angle_increment = 0.004363323;
    scan.min_range = 0.023;
    scan.max_range = 60.0;
    scan.color = "#6ED7FF".to_string();
    scan.point_size = 0.04;
    robot.add_sensor(Arc::new(scan)).unwrap();

    let mut dataviz = robot.create_dataviz(None);
    dataviz.add_occupancy_grid("/map", "map", None);
    dataviz.add_robot_velocity_data(robot_name, "/unitree_go1/odom", "map", None);
    dataviz.add_robot_odometry_trail(robot_name, "/unitree_go1/odom", "map", None);
    dataviz.add_robot_collision_risk(
        robot_name,
        "/unitree_go1/collision_risk",
        "unitree_go1/trunk",
        None,
    );

    let mut set = common::RegistrationSet {
        robots: vec![robot],
        datavizs: vec![dataviz],
    };
    common::register_or_fail(&mut set);
}
