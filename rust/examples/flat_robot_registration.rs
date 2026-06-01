mod common;

use horus::{RobotDimensions, RobotType};

fn main() {
    let mut set = common::RegistrationSet {
        robots: Vec::new(),
        datavizs: Vec::new(),
    };
    let mut robot = common::standard_robot(
        "flat_bot",
        RobotType::Wheeled,
        RobotDimensions::new(0.8, 0.55, 0.45),
        "wheeled",
    );
    robot.configure_ros_binding("flat", "flat", Some("base_link"));
    let dataviz = common::standard_dataviz(&robot);
    set.robots.push(robot);
    set.datavizs.push(dataviz);
    common::register_or_fail(&mut set);
}
