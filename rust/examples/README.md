# Rust Examples

## Available examples

- `sdk_registration_demo.rs`
  - Short ops-style native registration demo.
- `*_registration.rs`
  - Curated native registration examples that mirror the Python example basenames.
- `fake_tf_publisher.rs`
  - Publishes a moving `map -> ROBOT/base_link` and retained camera/lidar frames over ROS 2.
- `native_remote_render_agent.rs` and `remote_map_registration.rs`
  - Matched scene renderer and map-only registration; see [Remote Rendering](../../docs/native-sdk/remote-rendering.md).
- `e2e_registration_check.rs`
  - Live accepted-ACK and unregister check with `ros2`; `--payload-only` skips transport.

## Run

```bash
cd rust
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/local_setup.bash
cargo run --features ros2 --example sdk_registration_demo
```

```bash
cd rust
cargo run --features ros2 --example fake_tf_publisher -- --robot-name test_bot --rate-hz 10 --cycles 20
```

```bash
cd rust
cargo run --features ros2 --example e2e_registration_check -- --robot-name SdkBot_E2E
```

Without `ros2`, curated registration examples validate payloads only. With it,
they wait for real ACKs and keep accepted registrations alive until Ctrl+C.
Robot-description examples use native compilation and paced asset delivery.
The TF example requires `ros2` and generates transforms only, not camera images
or a simulator. Use a separate ROS domain when testing synthetic TF alongside a
real robot. Its two static frames remain available to subscribers that join
while the publisher is running; Ctrl+C stops only this publisher.
