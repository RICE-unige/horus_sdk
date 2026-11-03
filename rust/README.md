# HORUS Rust SDK

Rust SDK with native registration, models, Teach, mapping, remote rendering and experiment APIs. See `../docs/reference/implementation-status.md` for functional coverage, optional dependencies and qualification limits.

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
  - accepted/queued ACKs, replay, keep-alive, unregister and cancellation with `ros2`
  - explicit unsupported error in no-ROS builds
- Utility parity baseline:
  - topic status board
  - topic monitor
  - rosout monitor parser
  - owned bridge startup, paginated CLI dashboard and independent client lifecycle
  - batched `tf::Publisher` with ROS timestamps and retained static transforms

Contract freeze for both Rust/C++ tracks:
- `../contracts/sdk_payload_contract.md`
- `../contracts/fixtures/*.json`

## Build

```bash
cd rust
cargo check --all-targets
```

For live ROS registration on Ubuntu/Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/local_setup.bash
cargo test --features ros2 --all-targets --locked
cargo run --features ros2 --example ops_registration
```

The feature compiles the bundled C++ ROS runtime with CMake and links it through
a small C ABI. Install ROS development dependencies, a C++ compiler, CMake and
nlohmann-json before building. It does not launch the Python SDK. Only ROS's
build helpers need the system Python interpreter.

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
cargo run --example sdk_registration_demo
cargo run --features ros2 --example fake_tf_publisher -- --robot-name test_bot --rate-hz 10 --cycles 20
cargo run --example e2e_registration_check -- --robot-name SdkBot_E2E
```

## Notes

- ROS integration is opt-in (`ros2`), locally tested with ROS 2 Jazzy on Ubuntu.
- `RegistryOptions` controls owned bridge startup. Source the installed HORUS ROS overlay, or set an explicit executable and argument vector. Existing bridge processes are not adopted or stopped.
- Keep-alive blocks until `stop()`; share a client through `Arc` to cancel from another thread. The shared-registry accessor returns an `Arc`; `shutdown_robot_registry()` stops and resets that instance. Explicit stopped clients must be replaced.
- Runtime distribution must include `libhorus_native_ros.so.1` and ROS shared libraries. Build-directory RPATH alone is not a portable binary deployment.
- Native robot models, Teach/MCAP, costmap recovery, mesh source adapters, dashboard, collision-risk analysis and experiment helpers do not invoke the Python SDK. Point/mesh/Gaussian renderers support diagnostic PNG, RGB atlases and optional GPU-resident NVENC H.264. Dataset helpers cover catalogues, aligned full-resolution scan streaming, multiview atlases and explicit LOD export. See [Models and Teach](../docs/native-sdk/models-and-teach.md), [Mapping](../docs/native-sdk/mapping.md) and [Remote Rendering](../docs/native-sdk/remote-rendering.md) for build requirements and qualification boundaries.
- `LiveMeshPublisher` provides paced ROS delivery and late-join repair for already-paged geometry. See [Native Mapping](../docs/native-sdk/mapping.md).
