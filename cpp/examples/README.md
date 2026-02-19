# C++ Examples

## Available examples

- `sdk_registration_demo.cpp`
  - Multi-robot registration parity demo with camera + occupancy options.
- `fake_tf_publisher.cpp`
  - Fake TF loop scaffold.
- `e2e_registration_check.cpp`
  - Minimal register + metadata verification flow.

## Build and run

```bash
cd cpp
cmake -S . -B build
cmake --build build --parallel
```

```bash
./build/examples/sdk_registration_demo --robot-count 4 --with-occupancy-grid true
./build/examples/fake_tf_publisher --robot-name test_bot --rate-hz 10 --cycles 20
./build/examples/e2e_registration_check --robot-name SdkBot_E2E
```
