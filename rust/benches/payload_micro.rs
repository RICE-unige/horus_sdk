use criterion::{Criterion, criterion_group, criterion_main};
use horus::{Camera, Robot, RobotType};
use std::sync::Arc;

fn bench_payload_build(c: &mut Criterion) {
    c.bench_function("payload_build_single_robot", |b| {
        b.iter(|| {
            let mut robot = Robot::new("bench_robot", RobotType::Wheeled);
            let cam = Camera::new("cam", "camera_link", "/camera/image_raw");
            robot.add_sensor(Arc::new(cam)).ok();
            let dataviz = robot.create_dataviz(None);
            let _ = horus::bridge::build_robot_config_dict(&robot, &dataviz, None);
        });
    });
}

criterion_group!(benches, bench_payload_build);
criterion_main!(benches);
