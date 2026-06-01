mod common;

use horus::{RobotDimensions, RobotType, WebRtcTransportConfig};
use std::sync::Arc;

fn main() {
    let mut set = common::RegistrationSet {
        robots: Vec::new(),
        datavizs: Vec::new(),
    };

    for name in ["carter1", "carter2", "carter3"] {
        let mut robot = common::standard_robot(
            name,
            RobotType::Wheeled,
            RobotDimensions::new(0.82, 0.56, 0.60),
            "wheeled",
        );
        robot.configure_local_body_model("nova_carter", true);
        robot.remove_sensor("front_camera");

        let mut camera = common::standard_camera(name);
        camera.resolution = (1280, 720);
        camera.fps = 20;
        camera.teleop_streaming_type = "webrtc".to_string();
        camera.configure_webrtc_transport(WebRtcTransportConfig {
            bitrate_kbps: Some(2000),
            framerate: Some(20),
            ..Default::default()
        });
        robot.add_sensor(Arc::new(camera)).unwrap();
        robot
            .add_sensor(Arc::new(common::standard_laser_scan(
                "front_2d_lidar",
                format!("{name}/front_2d_lidar"),
                common::topic(name, "front_2d_lidar/scan"),
            )))
            .unwrap();

        let mut dataviz = robot.create_dataviz(None);
        dataviz.add_robot_global_path(name, &format!("/rviz/{name}/plan"), "global_odom", None);
        dataviz.add_robot_local_path(
            name,
            &format!("/rviz/{name}/local_plan"),
            "global_odom",
            None,
        );
        dataviz.add_robot_velocity_data(
            name,
            &common::topic(name, "chassis/odom"),
            "global_odom",
            None,
        );
        dataviz.add_robot_odometry_trail(
            name,
            &common::topic(name, "chassis/odom"),
            "global_odom",
            None,
        );

        set.robots.push(robot);
        set.datavizs.push(dataviz);
    }

    set.datavizs[0].add_occupancy_grid("/shared_map", "map", None);
    common::register_or_fail(&mut set);
}
