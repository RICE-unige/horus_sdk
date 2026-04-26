---
title: "Tutorial 3: Teleop and Tasks"
sidebar_position: 4
toc_max_heading_level: 2
---

# Tutorial 3: Teleop and Tasks

This tutorial explains how Robot Manager controls are declared from the SDK side.

:::tip[Think in contracts, not buttons]

The SDK does not draw the Robot Manager. It declares the topics and semantics the Robot Manager should expose.

:::

## Goal

By the end of this step, you should understand:

- how teleop topics become operator controls
- how navigation task topics become `Go-To Point` and `Draw Waypoint`
- why `robot_type` and `robot_profile` are separate but related
- how drone and legged action buttons differ from the ground-robot flow

## Step 1: declare teleop

```python
robot.configure_teleop(
    command_topic="/atlas/cmd_vel",
    robot_profile="wheeled",
)
```

This tells HORUS:

- which command topic to publish for operator motion
- which teleop profile should shape the UI and control mapping

Use the profile that matches the robot's actual motion model:

- `wheeled`
- `drone`
- `legged`

## Step 2: declare navigation tasks

```python
robot.configure_navigation_tasks(
    goal_topic="/atlas/goal_pose",
    cancel_topic="/atlas/goal_cancel",
    goal_status_topic="/atlas/goal_status",
    waypoint_path_topic="/atlas/waypoint_path",
    waypoint_status_topic="/atlas/waypoint_status",
    frame_id="map",
)
```

This is what makes the ground-navigation task tools work:

- `Go-To Point`
- `Draw Waypoint`
- goal canceling
- goal and waypoint status tracking

The key idea is that task authoring in HORUS is spatial, but task execution is still topic-driven. The SDK tells HORUS where those task messages should go.

## Robot type versus robot profile

These are related, but they are not identical.

### `robot_type`

This describes what kind of robot the operator is dealing with overall:

- wheeled
- drone
- legged

### `robot_profile`

This tells the teleop and task subsystems which operator behavior model to use for that control flow.

In the curated examples they usually match, but it is still useful to understand that one is robot identity and the other is operator-control behavior.

## How special action buttons are decided

The curated examples make the distinction clear:

- `python/examples/drone_registration.py` uses `RobotType.DRONE`
- `python/examples/legged_registration.py` uses `RobotType.LEGGED`

That difference is what drives the extra action buttons:

- drones keep `Take Off` and `Land`
- legged robots get `Stand Up` and `Sit Down`

This is why getting robot type right matters. It is not cosmetic.

## Validation path

Ground robot:

```bash
# Terminal A
python3 python/examples/legacy/fake_tf_ops_suite.py

# Terminal B
python3 python/examples/ops_registration.py
```

Drone:

```bash
# Terminal A
python3 python/examples/legacy/fake_tf_drone_ops_suite.py

# Terminal B
python3 python/examples/drone_registration.py
```

Legged:

```bash
# Terminal A
python3 python/examples/legacy/fake_tf_legged_ops_suite.py

# Terminal B
python3 python/examples/legged_registration.py
```

## What you should see in HORUS MR

- wheeled robots show normal teleop and ground-navigation task controls
- drones expose `Take Off` and `Land`
- legged robots expose `Stand Up` and `Sit Down`
- task panels are grounded in the topics you declared, not hardcoded per demo

## What comes next

Now that the operator can command the robot, the next layer is operator awareness. Continue with [Tutorial 4: DataViz layers](dataviz-layers.md).
