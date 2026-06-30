---
title: Implementation Status
sidebar_position: 1
---

# Implementation Status

## Python track

Status: **Production baseline**

- curated registration examples for wheeled, drone, legged, stereo, robot-description, map, semantic, Carter, and Unitree workflows
- registration payload serialization with keep-alive and ACK handling
- camera transport profiles for ROS and WebRTC-backed flows
- robot-scoped and global DataViz support
- global map support for occupancy grids, pointclouds, meshes, octomaps, and experimental Gaussian splat fixtures
- topic monitoring and dashboard state
- field teammate (HoloLens-class) registration with a capability-driven, default-deny safety contract and a fail-closed serializer (`entity_kind`, `capabilities`, `field_teammate_config`)
- HRI study analysis scaffolding ("On the Map, In the Team") with a dyad-level, confidence-interval reducer

## Experimental areas

Status: **Active validation**

- Gaussian Splat DataViz registration, fixture publishing, ROS chunk transfer, cache validation, and pointcloud fallback are available for testing.
- Quest/XR Gaussian Splat rendering is still being validated; use the small fixture and diagnostic render modes before testing dense assets.

## C++ track

Status: **Native payload parity**

- C++ builds typed registration payloads for HORUS MR without the Python runtime
- live bridge registration, ACK handling, keep-alive, and dashboard monitoring remain Python-only
- camera transport profiles, teleop/task controls, ROS binding, workspace config, local body model metadata, and Robot Manager config are supported
- robot and global DataViz payloads cover transforms, paths, velocity, odometry trails, collision risk, occupancy, pointcloud, mesh, octomap, Gaussian Splat fixtures, and semantic boxes
- field teammate capability contract at parity: `make_field_teammate`, `entity_kind`/`capabilities`/`field_teammate_config` in the payload, and `validate_field_teammate_safety` as the fail-closed backstop (verified against the shared `field_teammate_hololens.json` fixture)
- `cpp/examples/` mirrors the curated Python scenarios by basename, with `sdk_registration_demo.cpp` kept as the short ops-style payload demo

## Rust track

Status: **Native payload parity**

- Rust builds typed registration payloads with `serde_json` while matching the Python payload contract
- live bridge registration, ACK handling, keep-alive, and dashboard monitoring remain Python-only
- camera transport profiles, teleop/task controls, ROS binding, workspace config, local body model metadata, and Robot Manager config are supported
- robot and global DataViz payloads cover transforms, paths, velocity, odometry trails, collision risk, occupancy, pointcloud, mesh, octomap, Gaussian Splat fixtures, and semantic boxes
- field teammate capability contract at parity: `Robot::field_teammate`, `entity_kind`/`capabilities`/`field_teammate_config` in the payload, and `validate_field_teammate_safety` as the fail-closed backstop (verified against the shared `field_teammate_hololens.json` fixture)
- `rust/examples/` mirrors the curated Python scenarios by basename, with `sdk_registration_demo.rs` kept as the short ops-style payload demo

## Native parity gaps (roadmap)

The native SDKs match the Python registration/DataViz/payload contract, including the field-teammate capability contract. These Python subsystems are **not yet ported** to the native SDKs and remain Python-only:

- **Robot description baking** (`python/horus/description/`): URDF/xacro resolution and STL/DAE/OBJ mesh baking. C++/Rust currently emit a minimal native description payload with a stable hash; full mesh baking is the largest remaining port.
- **Experiment analysis** (`python/horus/experiments/analysis.py`, `metrics.py`, `field_teammate_study.py`): characterization stats and the HRI study reducer.
- **Colour management** (`python/horus/color/`): present in the Rust SDK (`color.rs`), not yet in C++.
- **3D-map utilities** (`map_3d_workflow`, `voxel_mesh`): Python-only helper workflows.

These are tracked here intentionally rather than stubbed, so the parity surface is honest. The registration hot path — the part that benefits most from native performance — is fully ported and benchmarked (see Native Performance).

## Current known stub/non-primary areas

Status: Stub (not usable yet)

- `python/horus/bridge/ros2.py`
- `python/horus/bridge/unity_tcp.py`
- `python/horus/topics.py`
- `python/horus/robot/status.py`
- `python/horus/robot/teleop.py`
- `python/horus/robot/task.py`
- `python/horus/robot/dataviz.py`
- `python/horus/plugins/rosbot.py`
- `python/horus/core/exceptions.py`
