# HORUS Rust SDK

Rust track for HORUS SDK parity with the implemented Python behavior.

## Current parity scope

- Robot model and metadata helpers:
  - `Robot::register_with_horus`
  - `Robot::unregister_from_horus`
  - `Robot::is_registered_with_horus`
  - `Robot::get_horus_id`
  - `Robot::get_horus_color`
  - `robot::register_robots`
- Camera transport profiles:
  - `streaming_type`
  - `minimap_streaming_type`
  - `teleop_streaming_type`
  - `startup_mode`
- DataViz parity methods:
  - sensor, transform, path, trajectory, occupancy, tf-tree, global navigation path
- Registration payload parity:
  - typed payload structs
  - global visualization dedupe
  - workspace scale include/omit rules
  - robot manager config defaults
- Utility parity baseline:
  - topic status board
  - topic monitor
  - rosout monitor parser
  - backend/CLI utility scaffolding

Contract freeze for both Rust/C++ tracks:
- `../contracts/sdk_payload_contract.md`
- `../contracts/fixtures/*.json`

## Build

```bash
cd rust
cargo check --all-targets
```

## Test

```bash
cd rust
cargo test --all-targets
```

Parity-focused integration tests are under `rust/tests/`:
- `camera_transport_profiles.rs`
- `global_visualizations_payload.rs`
- `workspace_scale_payload.rs`
- `topic_status_board.rs`
- `payload_snapshots.rs`
- `queued_status.rs`

## Benchmarks

```bash
cd rust
cargo bench --bench payload_micro
cargo bench --bench registration_scenario
```

## Examples

```bash
cd rust
cargo run --example sdk_registration_demo -- --robot-count 4 --with-occupancy-grid true
cargo run --example fake_tf_publisher -- --robot-name test_bot --rate-hz 10 --cycles 20
cargo run --example e2e_registration_check -- --robot-name SdkBot_E2E
```

## Notes

- ROS 2 runtime integration via `r2r` is feature-gated (`ros2`) and currently scaffolded.
- Registration/ack/heartbeat transport behavior is modeled for parity tests; full live ROS graph behavior is still evolving.
