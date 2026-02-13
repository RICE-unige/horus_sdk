---
title: Robot Model
sidebar_position: 1
---

# Robot Model

## When to use it

Use `Robot` and related enums/classes to define registration identity, dimensions, metadata, and attached sensors before publishing to HORUS runtime.

## Minimal example

```python
from horus.robot import Robot, RobotType

robot = Robot(name="test_bot", robot_type=RobotType.WHEELED)
```

## Realistic example

```python
from horus.robot import Robot, RobotType, Dimensions

robot = Robot(name="atlas_01", robot_type=RobotType.LEGGED)
robot.set_dimensions(Dimensions(length=0.9, width=0.55, height=1.0))
robot.add_metadata("operator_group", "alpha")
robot.add_metadata("robot_manager_config", {
    "enabled": True,
    "prefab_asset_path": "Assets/Prefabs/UI/RobotManager.prefab",
    "sections": {"status": True, "data_viz": True, "teleop": True, "tasks": True}
})
```

## Common failure and fix

- **Failure:** duplicate robot names produce unstable registration updates.
- **Fix:** enforce unique names per session and normalize naming policy before batch registration.
