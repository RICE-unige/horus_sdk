mod common;

use horus::{RobotDimensions, RobotType};

fn main() {
    let mut set = common::standard_fleet(
        &["atlas", "nova", "orion", "luna"],
        RobotType::Wheeled,
        RobotDimensions::new(0.8, 0.55, 0.45),
        "wheeled",
    );
    common::add_semantic_boxes(&mut set.datavizs[0]);
    common::register_or_fail(&mut set);
}
