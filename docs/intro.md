---
title: Intro
sidebar_position: 1
---

# HORUS SDK Documentation

This is the canonical documentation source for `horus_sdk`, aligned to the current implementation on `main`.

## Scope

- **Primary track:** Python SDK (`python/horus`)
- **Parity tracks:** C++ (`cpp/`) and Rust (`rust/`)
- **Integration surfaces:** `horus_ros2` backend runtime and `horus` MR app runtime

:::important
`horus_sdk` owns registration modeling, payload orchestration, and monitoring UX semantics.  
Bridge runtime internals are in [`horus_ros2`](https://github.com/RICE-unige/horus_ros2), and MR runtime UX is in [`horus`](https://github.com/RICE-unige/horus).
:::

## Documentation map

- [Installation](getting-started/installation.md)
- [Quickstart](getting-started/quickstart.md)
- [Installer Deep Dive](getting-started/installer.md)
- [System Boundary](architecture/system-boundary.md)
- [Runtime Flow](architecture/runtime-flow.md)

## Current-state highlights

- One-command installer for SDK + ROS2 workspace provisioning.
- Camera transport profiles support (`streaming_type` + `minimap/teleop/startup` fields).
- Global visualization payload support (`global_visualizations`) including occupancy grid.
- Workspace scale serialization support (`workspace_config.position_scale`).
- Dashboard link/data semantics integrated with topic monitoring.

## Research context

HORUS investigates scalable mixed-reality supervision and teleoperation for heterogeneous robot teams.  
The SDK focuses on the **contract layer** that binds backend/runtime systems with operator-facing MR workflows.
