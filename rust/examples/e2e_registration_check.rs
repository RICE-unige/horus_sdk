use clap::Parser;
use horus::core::types::RobotType;
use horus::robot::Robot;
use horus::sensors::LaserScan;
use std::sync::Arc;

#[derive(Debug, Parser)]
#[command(about = "Minimal end-to-end registration check (Rust).")]
struct Args {
    #[arg(long, default_value = "SdkBot_E2E")]
    robot_name: String,
}

fn main() {
    let args = Args::parse();

    let mut robot = Robot::new(args.robot_name.clone(), RobotType::Wheeled);
    let mut lidar = LaserScan::new("Front Lidar", "laser_frame", "/scan");
    lidar.min_range = 0.1;
    lidar.max_range = 12.0;
    lidar.color = "#00FFFF".to_string();
    lidar.point_size = 0.1;
    if robot.add_sensor(Arc::new(lidar)).is_err() {
        eprintln!("failed to add lidar sensor");
        std::process::exit(1);
    }

    let dataviz = robot.create_dataviz(None);
    let (ok, result) = robot.register_with_horus(Some(dataviz), false, false, None);
    if !ok {
        eprintln!("Registration failed: {result}");
        std::process::exit(1);
    }
    println!("Registration OK: {result}");

    if robot.is_registered_with_horus() {
        println!("Metadata check OK (registered)");
        std::process::exit(0);
    }

    eprintln!("Metadata check failed");
    std::process::exit(2);
}

