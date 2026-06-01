mod common;

use horus::{RobotDimensions, RobotType};

fn main() {
    let mut set = common::standard_fleet(
        &["splat_rover_1", "splat_rover_2"],
        RobotType::Wheeled,
        RobotDimensions::new(0.70, 0.48, 0.30),
        "wheeled",
    );
    for robot in &mut set.robots {
        robot.configure_robot_manager_with(common::robot_manager_without_controls());
    }
    set.datavizs[0].add_gaussian_splat_map(
        "/horus/gaussian_splat/manifest",
        "map",
        Some(common::options(&[
            ("max_splats", serde_json::json!(350000)),
            ("render_mode", serde_json::json!("splats")),
        ])),
        "/map_gaussian_splat_preview",
    );
    common::register_or_fail(&mut set);
}
