---
title: Native Performance
sidebar_position: 3
---

# Native Performance

The C++ and Rust SDKs are the preferred registration paths when payload generation runs in a native process. They avoid Python interpreter startup, dynamic object walking, and GIL contention while keeping the same HORUS MR payload contract.

## What to benchmark

Measure the SDK work that runs before the payload is published:

1. robot/sensor/DataViz object construction
2. registration payload serialization
3. multi-robot global visualization deduplication
4. camera topic profile extraction

Do not mix headset rendering, ROS transport latency, or WebRTC encode time into SDK serializer benchmarks. Those belong to bridge/runtime profiling.

## C++ checks

```bash
cd ~/horus_sdk/cpp
cmake -S . -B build_no_ros -DHORUS_ENABLE_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build_no_ros --parallel
./build_no_ros/benchmarks/payload_micro
./build_no_ros/benchmarks/registration_scenario
```

## Rust checks

```bash
cd ~/horus_sdk/rust
cargo bench --bench payload_micro
cargo bench --bench registration_scenario
```

## Guardrails

- Keep payload construction typed until the final JSON/value boundary.
- Keep per-robot camera and DataViz loops single-pass.
- Deduplicate global visualizations by stable type/topic/frame keys.
- Keep semantic boxes keyed by semantic id so repeated robots do not duplicate workspace annotations.
- Prefer explicit topic/TF binding helpers over repeated string manipulation in application code.

Use Python as the behavioral reference and native SDKs as the low-overhead implementation path.
