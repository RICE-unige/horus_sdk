---
title: Overview
sidebar_position: 1
---

# HORUS SDK

`horus_sdk` is the registration and contract layer for HORUS. It lets you describe robots, sensors, teleop topics, task topics, robot-description assets, and DataViz layers, then publish that configuration into the HORUS ROS 2 runtime and mixed-reality application.

## Who this documentation is for

- robotics developers integrating ROS 2 robots into HORUS MR
- research teams validating fleet supervision, teleop, and task flows
- engineers who need a concrete example to register a new robot quickly

## What the SDK owns

| Capability | What you define here |
| --- | --- |
| Robot identity | Name, type, dimensions, base frame, local-body or URDF metadata |
| Control surfaces | Robot Manager availability, teleop topics, navigation-task topics |
| Sensor registration | Camera, lidar, and per-view transport or rendering metadata |
| DataViz contract | Robot-scoped overlays, global maps, semantic layers, safety signals |
| Runtime observability | Registration ACK handling, keep-alive, topic and dashboard state |

## What the SDK does not own

- `horus_ros2` owns the HORUS ROS 2 runtime, bridge lifecycle, and WebRTC-capable transport.
- `horus` owns workspace placement, Robot Manager UI, task authoring UX, and mixed-reality runtime policy.
- Backend perception or copilot services are outside the SDK contract layer.

## Recommended reading order

1. [Installation](getting-started/installation.md)
2. [Quickstart](getting-started/quickstart.md)
3. [Tutorial summary](tutorials/summary.md)
4. [Curated examples](examples/registration-flows.md)
5. [Python SDK reference](python-sdk/robot-model.md)
6. [Integration](integration/horus-ros2.md)

## Project stance

The root `python/examples/` scripts are the primary copyable onboarding material. The tutorial track is the primary teaching path. The `python/examples/legacy/` folder is still useful for paired fake runtimes, stress tests, and deep validation, but it is no longer the main documentation path.

