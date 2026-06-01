mod common;

use horus::{
    NavigationTaskConfig, Robot, RobotDimensions, RobotManagerConfig, RobotType, TeleopConfig,
};

fn main() {
    let mut robot = Robot::with_dimensions(
        "arancino_uav",
        RobotType::Drone,
        RobotDimensions::new(0.46, 0.46, 0.18),
    );
    robot.configure_ros_binding("prefixed", "prefixed", Some("base_link"));
    robot.configure_robot_manager_with(RobotManagerConfig::default());
    robot.configure_teleop(TeleopConfig {
        command_topic: Some("/uav_sim/teleop/cmd_vel".to_string()),
        robot_profile: Some("drone".to_string()),
        ..Default::default()
    });
    robot.configure_navigation_tasks(NavigationTaskConfig {
        goal_topic: Some("/arancino_uav/goal".to_string()),
        cancel_topic: Some("/arancino_uav/goal_cancel".to_string()),
        goal_status_topic: Some("/uav_sim/status".to_string()),
        waypoint_path_topic: Some("/arancino_uav/waypoint_path".to_string()),
        waypoint_status_topic: Some("/uav_sim/status".to_string()),
        min_altitude_m: Some(0.0),
        max_altitude_m: Some(10.0),
        ..Default::default()
    });

    let mut dataviz = robot.create_dataviz(Some("arancino_uav_viz"));
    robot.add_path_planning_to_dataviz(&mut dataviz, Some("/uav_sim/path"), None, None);
    dataviz.add_3d_octomap(
        "/uav_sim/octomap_mesh",
        "map",
        Some(common::options(&[
            ("native_topic", serde_json::json!("/uav_sim/octomap_binary")),
            ("native_frame", serde_json::json!("map")),
        ])),
    );

    let mut set = common::RegistrationSet {
        robots: vec![robot],
        datavizs: vec![dataviz],
    };
    common::register_or_fail(&mut set);
}
