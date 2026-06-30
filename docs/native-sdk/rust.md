---
title: Rust SDK
sidebar_position: 2
---

# Rust SDK

The Rust SDK mirrors the Python registration payload with typed structs and `serde_json` serialization. Use it for native robot services that need predictable payload generation, low startup overhead, and build-time checks around the HORUS payload contract.

Live HORUS bridge registration, ACK handling, keep-alive, and dashboard monitoring are still Python SDK responsibilities. The Rust `register_robot`, `register_robots`, and `unregister_robot` helpers intentionally return an explicit unsupported-transport result instead of reporting fake local success. Use `RobotRegistryClient::build_robot_config_dict(...)` for native payload parity until a real Rust transport is implemented.

## What it sends

`RobotRegistryClient::build_robot_config_dict(...)` produces the same MR-facing registration fields as the Python SDK:

| Area | Rust support |
| --- | --- |
| Robot identity | name, type, dimensions |
| ROS binding | prefixed or flat topics and TF frames |
| Robot Manager | status, DataViz, teleop, and task panel sections |
| Controls | teleop defaults/overrides, go-to-point, waypoint topics |
| Cameras | minimap/teleop transport profiles, WebRTC settings, stereo fields, view/projection offsets |
| DataViz | robot transforms, paths, velocity text, odometry trails, collision risk, occupancy, pointcloud, mesh, octomap, Gaussian Splat fixtures, semantic boxes |
| Workspace | position scale, compass metadata, tutorial preset, local body model id |
| Field teammate | `entity_kind`, capability default-deny `capabilities`, versioned `field_teammate_config`, fail-closed `validate_field_teammate_safety` |

## Field teammate

A human field teammate is registered as a guidable, non-controllable entity, at parity with the Python SDK:

```rust
use horus::bridge::{build_robot_config_dict, validate_field_teammate_safety};
use horus::robot::Robot;

let teammate = Robot::field_teammate("field_teammate_1");
let dataviz = teammate.create_dataviz(None);
let payload = build_robot_config_dict(&teammate, &dataviz, None);

assert_eq!(payload.entity_kind, "field_teammate");
assert!(!payload.capabilities.controllable);
// A tampered payload that re-enables teleop/tasks/control is rejected here:
validate_field_teammate_safety(&payload).expect("default teammate is safe");
```

`Robot::field_teammate` (and `Robot::configure_field_teammate`) force the
robot-control capabilities off and disable teleop and the navigation tasks. The
payload carries the same `entity_kind` / `capabilities` / `field_teammate_config`
contract as Python, verified against `contracts/fixtures/field_teammate_hololens.json`.

## Other subsystems

Beyond registration payloads, the Rust SDK provides the HORUS experiments tooling
(`horus::experiments` — NDJSON/CSV metric writers, a monotonic-anchored clock, and
the field-teammate study reducer with dyad-level confidence intervals) and the
colour manager (`horus::color`). See **Implementation Status** for the full native
parity boundary.

## Build and test

```bash
cd ~/horus_sdk/rust
cargo test --all-targets
```

Run the native payload benchmarks when checking performance-sensitive changes:

```bash
cd ~/horus_sdk/rust
cargo bench --bench payload_micro
cargo bench --bench registration_scenario
```

## Minimal example

```rust
use horus::bridge::RobotRegistryClient;
use horus::core::types::RobotType;
use horus::robot::Robot;
use horus::sensors::Camera;
use std::sync::Arc;

fn main() {
    let mut robot = Robot::new("atlas", RobotType::Wheeled);
    robot.configure_ros_binding("prefixed", "prefixed", Some("base_link"));
    robot.configure_robot_manager(true, true, true, true);

    let mut camera = Camera::new(
        "front_camera",
        "atlas/camera_link",
        "/atlas/camera/image_raw/compressed",
    );
    camera.minimap_topic = "/atlas/camera/minimap/compressed".to_string();
    camera.teleop_topic = "/atlas/camera/image_raw/compressed".to_string();
    camera.minimap_image_type = "compressed".to_string();
    camera.teleop_image_type = "compressed".to_string();
    robot.add_sensor(Arc::new(camera)).unwrap();

    let mut dataviz = robot.create_dataviz(None);
    let global_path = robot.resolve_topic("global_path");
    let local_path = robot.resolve_topic("local_path");
    let trajectory = robot.resolve_topic("trajectory");
    robot.add_path_planning_to_dataviz(
        &mut dataviz,
        Some(&global_path),
        Some(&local_path),
        Some(&trajectory),
    );
    robot.add_navigation_safety_to_dataviz(&mut dataviz);
    dataviz.add_occupancy_grid("/map", "map", None);
    dataviz.add_3d_octomap("/atlas/octomap_mesh", "map", None);

    let client = RobotRegistryClient::new();
    let payload = client.build_robot_config_dict(&robot, &dataviz, None, Some(0.25));
    assert_eq!(payload.robot_name, "atlas");
}
```

## Payload Examples

The curated native examples live in `rust/examples/` and match the Python registration examples by basename: `ops_registration.rs`, `flat_robot_registration.rs`, `drone_registration.rs`, `legged_registration.rs`, `stereo_registration.rs`, the robot-description/map scenarios, Carter, Unitree Go1, and UAV sim payload checks.

```bash
cd ~/horus_sdk/rust
cargo run --example ops_registration
cargo run --example robot_description_registration
cargo run --example gaussian_splat_fixture_registration
```

`rust/examples/sdk_registration_demo.rs` remains the short native equivalent of the Python ops scenario at the payload layer; the larger scenario coverage is split into focused examples so payload construction stays concise.
The focused examples configure camera view profiles, teleop/task payloads, Robot Manager metadata, navigation safety visualizations, global 3D map payloads, semantic boxes, workspace compass/tutorial metadata, and local body model fields. They validate payload generation only; they do not contact a live HORUS bridge.
