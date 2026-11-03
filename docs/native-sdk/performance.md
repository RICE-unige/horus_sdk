---
title: Native Performance
sidebar_position: 3
---

# Native Performance

C++ and Rust expose native registration and typed, buffer-oriented mapping APIs. A language choice alone does not establish better performance: NumPy kernels already run natively, and DDS, decoding and headset rendering can dominate end-to-end latency.

## What to benchmark

Measure the SDK work that runs before the payload is published:

1. robot/sensor/DataViz object construction
2. registration payload serialization
3. multi-robot global visualization deduplication
4. camera topic profile extraction

Do not mix headset rendering, ROS transport latency, or WebRTC encode time into SDK serializer benchmarks. Those belong to bridge/runtime profiling.

## Measured mapping kernels

Equivalent single-threaded workloads patch a 128 x 128 raw Nav2 region into a
1024 x 1024 grid 1,000 times, then encode a colored HMSH v2 page of 30,000 vertices
and 10,000 triangles 100 times. Each program reports five rounds after one warm-up.
Geometry and colors are unchanged. All three produce 600,052-byte pages with the
same checksum; a separate unit test checks an exact Python wire fixture.

Local development measurements on 2026-10-14, median time per operation:

| Implementation | Costmap patch | Mesh encoding |
| --- | ---: | ---: |
| Python / NumPy | 27.12 microseconds | 5,670.69 microseconds |
| C++ Release | 7.66 microseconds | 64.72 microseconds |
| Rust Release | 4.27 microseconds | 113.06 microseconds |

These are diagnostic measurements on one host, not a controlled publication
benchmark. CPU affinity, power policy and concurrent work were not controlled.
Encoding includes allocation and validation; patch timing excludes constructing
the full initial grid. No ROS, network or headset work is included. Re-run on
deployment hardware; do not infer robot capacity or latency from these timings.

```bash
cd ~/horus_sdk
cmake -S cpp -B build/native-release -DHORUS_ENABLE_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/native-release --target mapping_kernels --parallel 2
./build/native-release/benchmarks/mapping_kernels
PYTHONPATH=python python3 .github/scripts/benchmark_mapping_kernels.py
cd rust
cargo run --release --example mapping_benchmark --locked
```

The existing `throughput_benchmark` is a configuration/serialization benchmark.
Keep single-thread and multi-thread results separate and report hardware and
thread counts. It does not measure registration ACK time or ROS forwarding.

## C++ checks

```bash
cd ~/horus_sdk/cpp
cmake -S . -B build_no_ros -DHORUS_ENABLE_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build_no_ros --parallel
./build_no_ros/benchmarks/payload_micro
./build_no_ros/benchmarks/registration_scenario
./build_no_ros/benchmarks/throughput_benchmark 200000
```

## Rust checks

```bash
cd ~/horus_sdk/rust
cargo bench --bench payload_micro
cargo bench --bench registration_scenario
cargo run --release --example throughput_benchmark -- 200000
```

The Python baseline for the cross-language comparison:

```bash
cd ~/horus_sdk
PYTHONPATH=python python3 python/examples/tools/throughput_benchmark.py 50000
```

## Guardrails

- Keep payload construction typed until the final JSON/value boundary.
- Keep per-robot camera and DataViz loops single-pass.
- Deduplicate global visualizations by stable type/topic/frame keys.
- Keep semantic boxes keyed by semantic id so repeated robots do not duplicate workspace annotations.
- Prefer explicit topic/TF binding helpers over repeated string manipulation in application code.
- Patch only affected costmap rows. Native mesh encoders allocate one output buffer and retain finite-value, dimension, index and payload-budget checks.
- Keep one owned ROS executor per registry, not one process/node per robot.

Use Python as the behavioral reference and native SDKs as the low-overhead implementation path.
