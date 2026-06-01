mod common;

use horus::{
    NavigationTaskConfig, Robot, RobotDimensions, RobotManagerConfig, RobotType, TeleopConfig,
};
use std::sync::Arc;

fn main() {
    let mut set = common::RegistrationSet {
        robots: Vec::new(),
        datavizs: Vec::new(),
    };

    for index in 1..=4 {
        let name = format!("stereo_bot_{index}");
        let mut robot = Robot::with_dimensions(
            name.clone(),
            RobotType::Wheeled,
            RobotDimensions::new(0.8, 0.55, 0.45),
        );
        robot.configure_ros_binding("prefixed", "prefixed", Some("base_link"));
        robot.configure_robot_manager_with(RobotManagerConfig::default());
        robot.configure_teleop(TeleopConfig {
            command_topic: Some(common::topic(&name, "cmd_vel")),
            robot_profile: Some("wheeled".to_string()),
            ..Default::default()
        });
        robot.configure_navigation_tasks(NavigationTaskConfig {
            goal_topic: Some(common::topic(&name, "goal_pose")),
            cancel_topic: Some(common::topic(&name, "goal_cancel")),
            goal_status_topic: Some(common::topic(&name, "goal_status")),
            waypoint_path_topic: Some(common::topic(&name, "waypoint_path")),
            waypoint_status_topic: Some(common::topic(&name, "waypoint_status")),
            ..Default::default()
        });
        robot
            .add_sensor(Arc::new(common::stereo_camera(&name, index == 1)))
            .unwrap();

        let dataviz = common::standard_dataviz(&robot);
        set.robots.push(robot);
        set.datavizs.push(dataviz);
    }

    common::register_or_fail(&mut set);
}
