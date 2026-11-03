# HORUS C++ SDK

C++20 SDK with native registration, models, Teach, mapping, remote rendering and experiment APIs. See `../docs/reference/implementation-status.md` for functional coverage, optional dependencies and qualification limits.

## Parity scope

- `horus::bridge::RobotRegistryClient`
  - `register_robot`
  - `register_robots`
  - `unregister_robot`
  - `check_backend_availability`
  - typed payload builders (`RobotRegistrationPayload`, camera/global/workspace rules)
- `horus::robot::register_robots` free function
- `horus::robot::Robot` HORUS registration helpers
- `horus::robot::Camera` transport profile fields:
  - `streaming_type`
  - `minimap_streaming_type`
  - `teleop_streaming_type`
  - `startup_mode`
- `horus::robot::DataViz` parity methods (sensor/transform/path/occupancy/tf-tree)
- In-memory TopicMap, priority EventBus, independent client lifecycle and ROS status dashboard
- Batched `horus::tf::Publisher` with ROS timestamps and retained static transforms

## Build

```bash
cd cpp
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/local_setup.bash
cmake -S . -B build -DHORUS_ENABLE_ROS2=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

Force no-ROS mode explicitly:

```bash
cd cpp
cmake -S . -B build_no_ros -DHORUS_ENABLE_ROS2=OFF
cmake --build build_no_ros --parallel
```

## Tests

```bash
cd cpp
ctest --test-dir build --output-on-failure
```

`cpp/tests/parity_tests.cpp` covers:
- camera profile defaults/validation,
- payload profile fallback behavior,
- global visualization + workspace scale parity rules.

## Benchmarks

```bash
cd cpp
./build/benchmarks/payload_micro
./build/benchmarks/registration_scenario
```

## Examples

See `cpp/examples/README.md`.

## Notes

- Build uses C++20.
- ROS-enabled build activates when `HORUS_ENABLE_ROS2=ON` and ROS dependencies are discoverable.
- No-ROS mode is supported for payload/model parity development.
- ROS mode uses a private context/executor, real accepted ACKs, timeouts, replay and explicit cancellation. `stop()` does not shut down the application's ROS context.
- Bridge auto-start resolves the installed bridge through the sourced ROS overlay. `RegistryOptions` can disable launch or supply an executable and argument vector. Only the owned child is stopped.
- Native robot models, Teach/MCAP, costmap recovery, mesh source adapters, dashboard, collision-risk analysis and experiment helpers are implemented. Point/mesh/Gaussian renderers support diagnostic PNG, RGB atlases and optional GPU-resident NVENC H.264. Dataset helpers cover catalogues, aligned full-resolution scan streaming, multiview atlases and explicit LOD export. See [Models and Teach](../docs/native-sdk/models-and-teach.md), [Mapping](../docs/native-sdk/mapping.md) and [Remote Rendering](../docs/native-sdk/remote-rendering.md) for build requirements and qualification boundaries.
- `LiveMeshPublisher` provides paced ROS delivery and late-join repair for already-paged geometry. See [Native Mapping](../docs/native-sdk/mapping.md).
- Installed CMake users link `horus::horus_cpp_sdk`. The native middleware shared library is installed beside the SDK library. Runtime ROS dependencies still require a sourced ROS environment.
