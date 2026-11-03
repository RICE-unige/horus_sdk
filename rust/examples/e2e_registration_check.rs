use clap::Parser;
use horus::bridge::RobotRegistryClient;
use horus::core::types::RobotType;
use horus::robot::Robot;
use horus::sensors::LaserScan;
use std::sync::Arc;

#[derive(Debug, Parser)]
#[command(about = "Minimal end-to-end registration check (Rust).")]
struct Args {
    #[arg(long, default_value = "SdkBot_E2E")]
    robot_name: String,
    #[arg(long, default_value_t = 10.0)]
    timeout: f64,
    #[arg(long)]
    payload_only: bool,
    #[arg(long)]
    unregister: bool,
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
    let client = RobotRegistryClient::new();
    let payload = client.build_robot_config_dict(&robot, &dataviz, None, None);

    if args.payload_only {
        println!("{}", serde_json::to_string(&payload).unwrap());
        return;
    }
    let (ok, result) = client.register_robot(&mut robot, &dataviz, args.timeout, false, false, None);
    println!("{result}");
    if !ok { drop(client); std::process::exit(2); }
    if args.unregister {
        let (ok, result) = client.unregister_robot(&robot.name, args.timeout);
        println!("{result}");
        if !ok { drop(client); std::process::exit(3); }
    }
}
