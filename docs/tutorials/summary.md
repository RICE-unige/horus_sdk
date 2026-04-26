---
title: Tutorial Summary
sidebar_position: 1
toc_max_heading_level: 2
---

# Tutorial Summary

This tutorial track teaches the SDK in the order a robotics developer actually needs it.

:::tip[Use the tutorial track to learn the model]

The tutorial pages explain how to assemble a registration from first principles. The example catalog is for the later moment when you already understand the contract and want a known-good file to copy from.

:::

The goal is not to walk line by line through the example scripts. The goal is to teach the contract layers that make a registration work in HORUS MR.

## What you build by the end

By the end of the track, you should be able to build a clean registration for a new robot by composing these layers in order:

1. robot identity and body bounds
2. operator-facing Robot Manager availability
3. camera streams and view behavior
4. teleop and task contracts
5. robot-scoped DataViz layers
6. URDF-backed robot description
7. workspace-level map layers
8. live-robot topic and frame substitution

## How to use this track

1. finish the [Quickstart](../getting-started/quickstart.md) once
2. read the tutorial page before opening any example script
3. copy only the contract fragment you actually need
4. use the paired fake runtime as validation, not as the main teaching source
5. verify the result in HORUS MR before moving to the next layer

## Tutorial order

| Tutorial | What you learn | Validate with |
| --- | --- | --- |
| [Tutorial 1: First ground robot](first-ground-robot.md) | the smallest useful `Robot` registration | `python/examples/ops_registration.py` |
| [Tutorial 2: Cameras and views](cameras-and-views.md) | how one camera definition becomes projected, minimap, and teleop-facing views | `python/examples/ops_registration.py` |
| [Tutorial 3: Teleop and tasks](operator-controls.md) | how Robot Manager controls are declared from topic contracts | `python/examples/ops_registration.py`, `drone_registration.py`, `legged_registration.py` |
| [Tutorial 4: DataViz layers](dataviz-layers.md) | how to add paths, odometry, safety, and semantic layers | `python/examples/ops_registration.py`, `semantic_perception_registration.py` |
| [Tutorial 5: Robot description](robot-description.md) | how to move from primitive bounds to URDF-backed bodies | `python/examples/robot_description_registration.py` |
| [Tutorial 6: Global maps](global-maps.md) | how occupancy, pointcloud, dense mesh, and octomap layers are declared | `python/examples/occupancy_map_registration.py` and related map registrations |
| [Tutorial 7: Live robot checklist](live-robot-checklist.md) | how to substitute real topics and frames without changing the SDK model | `python/examples/carter_registration.py`, `python/examples/unitree_go1_registration.py` |

## What this track assumes

- you can source ROS 2 and the `horus_ros2` workspace
- you can run the Python examples from the repository root
- you have access to the HORUS MR app or at least the ROS runtime needed to receive registration ACKs

## What this track is not

- it is not a full API reference
- it is not the legacy fake-runtime catalog
- it is not a line-by-line commentary on the example scripts

If you already know the SDK model and only need a runnable reference, use the [example catalog](../examples/registration-flows.md).
