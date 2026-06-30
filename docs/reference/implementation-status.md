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
- experiments subsystem: NDJSON/CSV metric writers, a monotonic-anchored clock, and the field-teammate HRI study reducer (dyad-level aggregation with t-based confidence intervals)
- colour manager: per-scheme palettes with cached assignment and an MD5-deterministic fallback that matches Python/Rust byte-for-byte
- robot-description manifest at structural parity (link/joint/collision counts, base frame, stable hash, `body_mesh_mode`) plus native STL visual-mesh baking (binary + ASCII → deduplicated base64 mesh assets, `supports_visual_meshes`)
- `cpp/examples/` mirrors the curated Python scenarios by basename, with `sdk_registration_demo.cpp` kept as the short ops-style payload demo

## Rust track

Status: **Native payload parity**

- Rust builds typed registration payloads with `serde_json` while matching the Python payload contract
- live bridge registration, ACK handling, keep-alive, and dashboard monitoring remain Python-only
- camera transport profiles, teleop/task controls, ROS binding, workspace config, local body model metadata, and Robot Manager config are supported
- robot and global DataViz payloads cover transforms, paths, velocity, odometry trails, collision risk, occupancy, pointcloud, mesh, octomap, Gaussian Splat fixtures, and semantic boxes
- field teammate capability contract at parity: `Robot::field_teammate`, `entity_kind`/`capabilities`/`field_teammate_config` in the payload, and `validate_field_teammate_safety` as the fail-closed backstop (verified against the shared `field_teammate_hololens.json` fixture)
- experiments subsystem: NDJSON/CSV metric writers, a monotonic-anchored clock, and the field-teammate HRI study reducer (dyad-level aggregation with t-based confidence intervals); colour manager already present
- robot-description manifest at structural parity (link/joint/collision counts, base frame, stable hash, `body_mesh_mode`) plus native STL visual-mesh baking (binary + ASCII → deduplicated base64 mesh assets, `supports_visual_meshes`)
- `rust/examples/` mirrors the curated Python scenarios by basename, with `sdk_registration_demo.rs` kept as the short ops-style payload demo

## Native parity boundary (intentional)

The native SDKs now match the Python SDK across the registration / DataViz / payload contract, the field-teammate capability contract, the experiments subsystem (metric writers + HRI study reducer), the colour manager, and the structural robot-description manifest (link / joint / collision counts, base frame, stable hash, `body_mesh_mode`).

These Python capabilities are deliberately **not** ported. They are documented scope decisions, not stubs:

- **DAE/OBJ baking, decimation, and `package://` resolution** (`python/horus/description/robot_mesh_baker.py` and the mesh-resolution half of the resolver): the native SDKs **do** bake STL visual meshes (binary and ASCII) into deduplicated base64 mesh assets, so `supports_visual_meshes` and `mesh_assets` are populated for STL. The Python baker additionally converts DAE/OBJ via external tools, decimates to a triangle budget, derives texture-colour hints (Pillow), and resolves `package://` via a mesh root / ament index. Those paths use NumPy and external converters and remain Python-only; native mesh references that are DAE/OBJ-only or `package://`-only are skipped rather than baked.
- **Offline characterization analysis** (`python/horus/experiments/analysis.py`): a NumPy/Pandas i-RIM research-analysis script (latency percentiles, knee/saturation detection, N-run CI aggregation) run offline to produce paper figures. It is research tooling rather than SDK API, so it is not a native-SDK responsibility.
- **3D-map authoring helpers** (`map_3d_workflow`, `voxel_mesh`): Python-only offline workflows.

The registration hot path — the part that benefits most from native performance — is fully ported and benchmarked (see Native Performance).

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
