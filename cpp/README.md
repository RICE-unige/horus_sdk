# HORUS C++ SDK

C++ parity track for HORUS SDK, aligned to implemented Python behavior.

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
- Topic map, client, and utility scaffolding

## Build

```bash
cd cpp
cmake -S . -B build
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
