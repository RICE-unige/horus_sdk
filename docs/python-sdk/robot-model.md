---
title: Robot Model
sidebar_position: 1
---

# Robot Model

## When to use it

Use the `Robot` model when you are defining what HORUS MR should instantiate for a robot: its identity, type, dimensions, base frame, control surfaces, and optional local-body or URDF metadata.

## Minimal example

```python
from horus.robot import Robot, RobotType

robot = Robot(name="atlas", robot_type=RobotType.WHEELED)
```

## Realistic example

```python
from horus.robot import Robot, RobotDimensions, RobotType

robot = Robot(
    name="atlas",
    robot_type=RobotType.WHEELED,
    dimensions=RobotDimensions(length=0.80, width=0.55, height=0.45),
)

robot.configure_ros_binding(base_frame="base_link")
robot.configure_robot_manager()
robot.configure_teleop(
    command_topic="/atlas/cmd_vel",
    robot_profile="wheeled",
)
robot.configure_navigation_tasks(
    goal_topic="/atlas/goal_pose",
    cancel_topic="/atlas/goal_cancel",
    goal_status_topic="/atlas/goal_status",
    waypoint_path_topic="/atlas/waypoint_path",
    waypoint_status_topic="/atlas/waypoint_status",
    frame_id="map",
)
```

You can extend the same model with:

- `configure_local_body_model(...)` for a built-in local robot body
- `configure_robot_description(...)` for URDF-backed visual meshes
- robot-type-specific behavior such as drone `Take Off` / `Land` or legged `Stand Up` / `Sit Down`

## Common failure and fix

- **Failure:** the robot registers but the MR runtime behaves inconsistently or task/camera overlays attach to the wrong transform.
- **Fix:** verify that the robot name is unique within the session and that `base_frame` matches the real ROS TF tree. Wrong names break payload dedupe, and wrong base frames break tasking and sensor alignment.
