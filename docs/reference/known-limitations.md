---
title: Known Limitations
sidebar_position: 2
---

# Known Limitations

## Runtime constraints

- high-rate multi-robot camera streaming can still saturate bridge or headset budgets
- the most complete WebRTC validation path remains Jazzy-oriented
- workspace acceptance in MR is intentionally required before full activation
- Gaussian Splat rendering is experimental; use small fixtures and pointcloud fallback before dense Quest validation

## Example-model constraints

- the curated registration scripts are the primary references, but several paired fake runtimes still live under `python/examples/legacy/`
- some live workflows depend on assets or ROS graphs that are not bundled into the SDK repository itself

## API-surface constraints

- legacy transport fields still coexist with newer per-view camera transport fields for compatibility
- Gaussian Splat assets currently target standard 3DGS binary PLY fixtures; compressed SPZ-style paths are not the primary SDK contract yet
- Python remains the primary tutorial track; C++ and Rust now cover native registration payload parity but do not replace every Python operational helper

## Recommendation

Treat documentation updates as integration updates. Validate any meaningful change against:

1. the SDK serializer
2. a paired fake or live ROS runtime
3. `horus_ros2`
4. the HORUS MR workspace lifecycle
