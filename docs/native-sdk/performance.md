---
title: Native Performance
sidebar_position: 3
---

# Native Performance

The C++ and Rust SDKs are the preferred paths when registration payload generation runs in a native process. They avoid Python interpreter startup, dynamic object walking, and GIL contention while keeping the same HORUS MR payload contract. Live bridge registration is still handled by the Python SDK until native transports are implemented.

## What to benchmark

Measure the SDK work that runs before the payload is published:

1. robot/sensor/DataViz object construction
2. registration payload serialization
3. multi-robot global visualization deduplication
4. camera topic profile extraction

Do not mix headset rendering, ROS transport latency, or WebRTC encode time into SDK serializer benchmarks. Those belong to bridge/runtime profiling.

## Measured throughput

`throughput_benchmark` builds the same robot + camera + DataViz registration payload repeatedly and reports payloads/second, single-threaded and across every hardware thread. Because the native SDKs have no global interpreter lock, payload building scales with cores — the decisive advantage over the Python reference, which is pinned to a single core for CPU-bound work.

Representative run on a 32-core host (registration payload build, higher is better):

| SDK | Single-thread | Multi-thread (32) | vs Python (1 core) |
|-----|---------------|-------------------|--------------------|
| Python (reference) | ~18.3k payloads/s | — (GIL-bound) | 1× |
| C++ | ~81.7k payloads/s | ~767k payloads/s | 4.5× / **42×** |
| Rust | ~118k payloads/s | ~1.34M payloads/s | 6.5× / **73×** |

Even single-threaded the native SDKs are several times faster (typed structs and no per-object dictionary walking); with parallelism they pull an order of magnitude ahead. Numbers vary with CPU and core count — re-run locally for your hardware. The benchmark uses a per-thread registry client so threads never contend on shared state.

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

Use Python as the behavioral reference and native SDKs as the low-overhead implementation path.
