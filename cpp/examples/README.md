# C++ Examples

## Available examples

- `sdk_registration_demo.cpp`
  - Short ops-style native registration demo.
- `*_registration.cpp`
  - Curated native registration examples that mirror the Python example basenames.
- `fake_tf_publisher.cpp`
  - Publishes a moving `map -> ROBOT/base_link` and retained camera/lidar frames over ROS 2.
- `native_remote_render_agent.cpp` and `remote_map_registration.cpp`
  - Matched scene renderer and map-only registration; see [Remote Rendering](../../docs/native-sdk/remote-rendering.md).
- `e2e_registration_check.cpp`
  - Register against a live app, check its ACK, then unregister. `--payload-only` skips transport.

## Build and run

```bash
cd cpp
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/local_setup.bash
cmake -S . -B build -DHORUS_ENABLE_ROS2=ON
cmake --build build --parallel
```

```bash
./build/examples/sdk_registration_demo
./build/examples/fake_tf_publisher --robot-name test_bot --rate-hz 10 --cycles 20
./build/examples/e2e_registration_check --robot-name SdkBot_E2E
```

ROS builds of the curated registration examples wait for real ACKs and keep
accepted registrations alive until Ctrl+C. No-ROS builds only inspect payloads.
Robot-description examples use native compilation and paced asset delivery.
The TF example requires a ROS-enabled build and generates transforms only, not
camera images or a simulator. Use a separate ROS domain when testing synthetic
TF alongside a real robot. Its two static frames remain available to subscribers
that join while the publisher is running; Ctrl+C stops only this publisher.
