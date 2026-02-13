---
title: System Boundary
sidebar_position: 1
---

# System Boundary

## Ownership model

| Layer | Repository | Responsibility |
|---|---|---|
| SDK contract + orchestration | `horus_sdk` | Robot/sensor models, registration payloads, observability semantics |
| ROS2 backend runtime | `horus_ros2` | Topic/service routing, bridge transport runtime (ROS/WebRTC), launch topology |
| MR app runtime | `horus` | Workspace flow, visualizers, operator interactions, runtime policy enforcement |

## Data boundary

`horus_sdk` emits registration payloads over `/horus/registration` and consumes:

- registration acknowledgments (`/horus/registration_ack`),
- heartbeat (`/horus/heartbeat`),
- topic monitor observations.

## Transport policy boundary

SDK payload can express desired camera transport profiles (`minimap_streaming_type`, `teleop_streaming_type`), but final runtime policy may be constrained by MR state and bridge availability.

## Why this split matters

- Keeps SDK independent from Unity scene internals.
- Enables Python/C++/Rust parity on shared payload contract fixtures.
- Allows backend and MR repositories to evolve runtime behavior while preserving contract compatibility.
