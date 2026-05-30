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

## Experimental areas

Status: **Active validation**

- Gaussian Splat DataViz registration, fixture publishing, ROS chunk transfer, cache validation, and pointcloud fallback are available for testing.
- Quest/XR Gaussian Splat rendering is still being validated; use the small fixture and diagnostic render modes before testing dense assets.

## C++ track

Status: **Native registration parity**

- C++ builds typed registration payloads for HORUS MR without the Python runtime
- camera transport profiles, teleop/task controls, ROS binding, workspace config, local body model metadata, and Robot Manager config are supported
- robot and global DataViz payloads cover transforms, paths, velocity, odometry trails, collision risk, occupancy, pointcloud, mesh, octomap, and semantic boxes
- `cpp/examples/sdk_registration_demo.cpp` is the native registration demo

## Rust track

Status: **Native registration parity**

- Rust builds typed registration payloads with `serde_json` while matching the Python payload contract
- camera transport profiles, teleop/task controls, ROS binding, workspace config, local body model metadata, and Robot Manager config are supported
- robot and global DataViz payloads cover transforms, paths, velocity, odometry trails, collision risk, occupancy, pointcloud, mesh, octomap, and semantic boxes
- `rust/examples/sdk_registration_demo.rs` is the native registration demo

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
