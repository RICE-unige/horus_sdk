mod common;

use horus::{NavigationTaskConfig, RobotDimensions, RobotType};

fn main() {
    let mut set = common::standard_fleet(
        &["drone_1", "drone_2", "drone_3"],
        RobotType::Drone,
        RobotDimensions::new(0.46, 0.46, 0.18),
        "drone",
    );

    for robot in &mut set.robots {
        robot.configure_navigation_tasks(NavigationTaskConfig {
            goal_topic: Some(robot.resolve_topic("goal_pose")),
            cancel_topic: Some(robot.resolve_topic("goal_cancel")),
            goal_status_topic: Some(robot.resolve_topic("goal_status")),
            waypoint_path_topic: Some(robot.resolve_topic("waypoint_path")),
            waypoint_status_topic: Some(robot.resolve_topic("waypoint_status")),
            min_altitude_m: Some(0.0),
            max_altitude_m: Some(10.0),
            ..Default::default()
        });
    }

    common::register_or_fail(&mut set);
}
