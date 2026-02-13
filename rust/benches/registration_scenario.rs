use criterion::{Criterion, criterion_group, criterion_main};
use horus::{Camera, Robot, RobotType};
use std::sync::Arc;

fn bench_registration_scenario(c: &mut Criterion) {
    c.bench_function("payload_build_ten_robots", |b| {
        b.iter(|| {
            for idx in 0..10 {
                let mut robot = Robot::new(format!("robot_{idx}"), RobotType::Wheeled);
                let camera = Camera::new(
                    format!("camera_{idx}"),
                    "camera_link",
                    format!("/robot_{idx}/camera/image_raw"),
                );
                robot.add_sensor(Arc::new(camera)).ok();
                let dataviz = robot.create_dataviz(None);
                let _ = horus::bridge::build_robot_config_dict(&robot, &dataviz, None);
            }
        });
    });
}

criterion_group!(benches, bench_registration_scenario);
criterion_main!(benches);
