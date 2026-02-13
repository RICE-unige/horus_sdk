---
title: HORUS ROS2 Integration
sidebar_position: 1
---

# HORUS ROS2 Integration

## Role of `horus_ros2`

`horus_ros2` provides backend runtime, routing, and bridge-level transport execution.  
The SDK emits contracts; ROS2 runtime executes them.

## Typical startup

```bash
cd ~/horus/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch horus_backend horus_complete_backend.launch.py
```

Backend-only fallback:

```bash
ros2 launch horus_backend horus_backend.launch.py
```

## Integration contract touchpoints

- `/horus/registration`
- `/horus/registration_ack`
- `/horus/heartbeat`

WebRTC signaling topics are serialized through camera metadata fields and consumed by runtime bridge components.

## Validation commands

```bash
ros2 topic list | grep horus
ros2 topic echo /horus/registration_ack
```

## Notes

- On Humble, current main may operate in backend-only mode for installer flow.
- Jazzy is recommended for full bridge validation.
