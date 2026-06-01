mod common;

use horus::{RobotDimensions, RobotType};

fn main() {
    let mut set = common::standard_fleet(
        &["legged_1", "legged_2", "legged_3"],
        RobotType::Legged,
        RobotDimensions::new(0.65, 0.32, 0.45),
        "legged",
    );
    common::register_or_fail(&mut set);
}
