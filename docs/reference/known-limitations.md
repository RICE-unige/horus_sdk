---
title: Known Limitations
sidebar_position: 2
---

# Known Limitations

## Runtime constraints

- high-rate multi-robot camera streaming can still saturate bridge or headset budgets
- the most complete WebRTC validation path remains Jazzy-oriented
- workspace acceptance in MR is intentionally required before full activation

## Example-model constraints

- the curated registration scripts are the primary references, but several paired fake runtimes still live under `python/examples/legacy/`
- some live workflows depend on assets or ROS graphs that are not bundled into the SDK repository itself

## API-surface constraints

- legacy transport fields still coexist with newer per-view camera transport fields for compatibility
- Python is the reference SDK; C++ and Rust are not the primary onboarding path yet

## Recommendation

Treat documentation updates as integration updates. Validate any meaningful change against:

1. the SDK serializer
2. a paired fake or live ROS runtime
3. `horus_ros2`
4. the HORUS MR workspace lifecycle
