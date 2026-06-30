//! Cross-language serializer throughput benchmark (Rust).
//!
//! Measures how many registration payloads per second the Rust SDK builds,
//! single-threaded and across all cores. With no global interpreter lock,
//! payload building scales with cores -- the key advantage over the Python
//! reference. Run with an optional payload count:
//!
//!     cargo run --release --example throughput_benchmark -- 200000

use horus::bridge::RobotRegistryClient;
use horus::core::types::RobotType;
use horus::dataviz::DataViz;
use horus::robot::Robot;
use horus::sensors::Camera;
use std::sync::Arc;
use std::thread;
use std::time::Instant;

fn make_workload(idx: usize) -> (Robot, DataViz) {
    let mut robot = Robot::new(format!("robot_{idx}"), RobotType::Wheeled);
    let camera = Camera::new(
        format!("camera_{idx}"),
        "camera_link",
        format!("/robot_{idx}/camera/image_raw/compressed"),
    );
    robot.add_sensor(Arc::new(camera)).ok();
    let dataviz = robot.create_dataviz(None);
    (robot, dataviz)
}

fn main() {
    let total: usize = std::env::args()
        .nth(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(50_000);
    let threads = thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(4);

    let mut sink = 0usize;

    // Single-threaded.
    let client = RobotRegistryClient::new();
    let (robot, dataviz) = make_workload(0);
    let _ = client.build_robot_config_dict(&robot, &dataviz, None, None); // warmup
    let start = Instant::now();
    for _ in 0..total {
        let payload = client.build_robot_config_dict(&robot, &dataviz, None, None);
        sink = sink.wrapping_add(payload.sensors.len());
    }
    let single = total as f64 / start.elapsed().as_secs_f64();

    // Multi-threaded: per-thread client + workload, total work split across cores.
    let per_thread = total / threads;
    let start = Instant::now();
    let partials: Vec<usize> = thread::scope(|scope| {
        let handles: Vec<_> = (0..threads)
            .map(|t| {
                scope.spawn(move || {
                    let client = RobotRegistryClient::new();
                    let (robot, dataviz) = make_workload(t + 1);
                    let mut local = 0usize;
                    for _ in 0..per_thread {
                        let payload = client.build_robot_config_dict(&robot, &dataviz, None, None);
                        local = local.wrapping_add(payload.sensors.len());
                    }
                    local
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    });
    let multi = (per_thread * threads) as f64 / start.elapsed().as_secs_f64();
    sink = sink.wrapping_add(partials.iter().sum::<usize>());

    println!(
        "rust serializer throughput: single={single:.0} payloads/s, multi({threads} threads)={multi:.0} payloads/s, speedup={:.1}x (sink={sink})",
        multi / single
    );
}
