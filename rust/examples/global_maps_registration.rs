mod common;

use horus::{RobotDimensions, RobotType};

fn main() {
    let mut set = common::standard_fleet(
        &["test_bot_1", "test_bot_2"],
        RobotType::Wheeled,
        RobotDimensions::new(0.8, 0.55, 0.45),
        "wheeled",
    );
    common::add_world_maps(&mut set.datavizs[0]);
    common::register_or_fail(&mut set);
}
