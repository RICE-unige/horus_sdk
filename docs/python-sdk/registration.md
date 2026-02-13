---
title: Registration
sidebar_position: 4
---

# Registration

## When to use it

Use registration APIs when pushing robot configuration payloads to HORUS and tracking ACK/heartbeat state transitions.

## Minimal example

```python
from horus.robot import Robot, RobotType

robot = Robot(name="sdk_bot", robot_type=RobotType.WHEELED)
ok, result = robot.register_with_horus(keep_alive=False, show_dashboard=False)
print(ok, result)
```

## Realistic example

```python
from horus.robot import Robot, RobotType, register_robots

robots = []
datavizs = []
for i in range(4):
    robot = Robot(name=f"team_bot_{i+1}", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    dataviz.add_occupancy_grid(topic="/map", frame_id="map")
    robots.append(robot)
    datavizs.append(dataviz)

success, result = register_robots(
    robots,
    datavizs=datavizs,
    keep_alive=True,
    workspace_scale=0.1
)
print(success, result)
```

## Common failure and fix

- **Failure:** no ACK or stale status after registration push.
- **Fix:** verify backend launch, app connectivity, and workspace acceptance sequence before concluding SDK-side failure.
