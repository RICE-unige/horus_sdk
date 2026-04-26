---
title: Registration
sidebar_position: 4
---

# Registration

## When to use it

Use registration when you are ready to publish robot metadata into HORUS, keep the session alive, and monitor whether the app acknowledged and activated the payload.

## Minimal example

```python
from horus.robot import Robot, RobotType, register_robots

robot = Robot(name="atlas", robot_type=RobotType.WHEELED)

success, result = register_robots(
    [robot],
    keep_alive=False,
)
print(success, result)
```

## Realistic example

```python
from horus.robot import Robot, RobotDimensions, RobotType, register_robots

robot = Robot(
    name="atlas",
    robot_type=RobotType.WHEELED,
    dimensions=RobotDimensions(length=0.80, width=0.55, height=0.45),
)
robot.configure_robot_manager()
robot.configure_teleop(
    command_topic="/atlas/cmd_vel",
    robot_profile="wheeled",
)

dataviz = robot.create_dataviz()

success, result = register_robots(
    [robot],
    datavizs=[dataviz],
    workspace_scale=0.1,
    compass_enabled=False,
    keep_alive=True,
)
print(success, result)
```

In the curated examples, `keep_alive=True` is the normal default because the process also supports multi-operator replay and dashboard presence semantics.

## Common failure and fix

- **Failure:** the call returns, but the app never acknowledges the robots.
- **Fix:** check both runtime and environment assumptions. The SDK shell must have ROS and the `horus_ros2` workspace sourced, the bridge must be reachable or auto-startable, and the MR app must be connected far enough to emit `/horus/registration_ack`.
