---
title: System Boundary
sidebar_position: 1
---

# System Boundary

HORUS is split across three actively coupled repositories. Understanding that split is the fastest way to avoid putting the wrong responsibility into the SDK.

## Repository ownership

| Repository | Owns | Does not own |
| --- | --- | --- |
| `horus_sdk` | robot models, payload serialization, curated examples, keep-alive, dashboard semantics | bridge internals, Unity interaction logic |
| `horus_ros2` | HORUS ROS 2 runtime, topic routing, bridge lifecycle, WebRTC support | robot-model authoring API, MR workspace UX |
| `horus` | mixed-reality workspace flow, Robot Manager, task UI, runtime policy | ROS-side payload construction |

## Contract surfaces

The SDK publishes registration payloads and runtime metadata that the other repositories consume:

- robot identity and dimensions
- base-frame and sensor-frame metadata
- teleop and task topics
- robot-scoped visualizations
- global visualizations such as maps and semantic layers
- keep-alive and dashboard state

## Why the split matters

- The SDK stays copyable into real robotics projects without depending on Unity internals.
- `horus_ros2` can evolve transport behavior without changing how a developer registers a robot.
- The MR app can tighten UX policy, multi-operator behavior, or task gating without changing the SDK object model.

## Practical rule

If the change is about how to describe a robot, sensor, topic, or visualization, it belongs in `horus_sdk`.

If the change is about how those things are transported across ROS and bridge runtime, it belongs in `horus_ros2`.

If the change is about how an operator sees or interacts with those things in-headset, it belongs in `horus`.
