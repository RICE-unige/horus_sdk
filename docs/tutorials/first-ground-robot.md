---
title: "Tutorial 1: First Ground Robot"
sidebar_position: 2
toc_max_heading_level: 2
---

# Tutorial 1: First Ground Robot

This tutorial explains the smallest useful registration shape for a ground robot in HORUS MR.

:::note[Design rule]

Start by making one robot correct. Only after that should you scale the same definition to a fleet.

:::

## Goal

By the end of this step, you should understand:

- what a `Robot` object is representing
- which fields are identity versus behavior
- why `workspace_scale` and `keep_alive` matter at registration time
- how one robot scales cleanly into a fleet

## Build the smallest useful robot

Start with identity and body bounds only:

```python
from horus.robot import Robot, RobotDimensions, RobotType

robot = Robot(
    name="atlas",
    robot_type=RobotType.WHEELED,
    dimensions=RobotDimensions(length=0.8, width=0.55, height=0.45),
)
```

Then opt into the operator-facing management surface:

```python
robot.configure_robot_manager()
```

Finally, register the robot into the workspace:

```python
from horus.robot import register_robots

success, result = register_robots(
    [robot],
    workspace_scale=0.1,
    keep_alive=True,
)
```

## What each part means

- `name`: This is the runtime identity of the robot in HORUS. Use a stable name that matches the rest of your ROS graph. Topic naming, TF naming, and operator-facing selection all become easier when the SDK name matches the robot namespace you already use.
- `robot_type`: HORUS uses this to decide which operator actions make sense. `WHEELED` exposes the normal ground-robot flow, `DRONE` keeps `Take Off` and `Land`, and `LEGGED` maps those controls to `Stand Up` and `Sit Down`.
- `dimensions`: These are the simplest body bounds the app can use before you move to a URDF-backed robot description. They affect interaction volume, visibility, and basic spatial reasoning in the workspace.
- `configure_robot_manager()`: This declares that the robot should expose the normal Robot Manager panel in HORUS MR.
- `workspace_scale=0.1`: This tells HORUS how large the workspace should be relative to the source ROS world. In the curated examples, `0.1` is the standard mini-map scale.
- `keep_alive=True`: This keeps the registration client alive after publishing. In practice, this is the normal mode because it supports session continuity, ACK tracking, and multi-operator replay semantics.

## Assembly order that scales well

Use this order when building a new registration from scratch:

1. define `Robot(name, robot_type, dimensions)`
2. enable Robot Manager if operators should interact with it
3. add cameras and other sensors
4. add teleop and task contracts
5. add DataViz layers
6. move to URDF only after the simpler version is already correct

That order keeps failures easy to isolate.

## How to validate this step

Use the full reference workflow only after you understand the contract fragments above:

```bash
# Terminal A
python3 python/examples/legacy/fake_tf_ops_suite.py

# Terminal B
python3 python/examples/ops_registration.py
```

## What you should see in HORUS MR

- the robots appear after workspace acceptance
- selecting one robot opens one Robot Manager, not all of them
- the robot identity is stable across reopen or reconnect cycles

## What comes next

The robot exists now, but it is still not useful for sensing or operator intervention. Continue with [Tutorial 2: Cameras and views](cameras-and-views.md).
