mod common;

use horus::{RobotDimensions, RobotType};

fn main() {
    let mut set = common::standard_fleet(
        &["atlas", "nova", "orion"],
        RobotType::Wheeled,
        RobotDimensions::new(0.8, 0.55, 0.45),
        "wheeled",
    );
    common::register_or_fail(&mut set);
}
