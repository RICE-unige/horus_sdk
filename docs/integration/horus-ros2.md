---
title: HORUS ROS2 Integration
sidebar_position: 1
---

# HORUS ROS2 Integration

`horus_ros2` is the runtime side of HORUS. It is not a generic ROS bridge wrapper around somebody else's transport stack. It is the HORUS-specific ROS 2 runtime that carries registration payloads, commands, camera data, and optional WebRTC sessions between robotics code and the MR app.

## What the SDK expects

- a ROS 2 shell with `rclpy` and message packages available
- a built `horus_ros2` workspace or an installer-managed HORUS layout
- `horus_unity_bridge` discoverable from the current shell if SDK bridge auto-start is enabled

## Common integration pattern

The common source-checkout workflow is:

```bash
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python:$PYTHONPATH
python3 python/examples/ops_registration.py
```

If the bridge is not already running, the SDK registration client will try to start it.

## Manual bridge launch

Use this when you are debugging startup or intentionally bypassing SDK auto-start:

```bash
cd ~/horus_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

## Topics that matter most

- `/horus/registration`
- `/horus/registration_ack`
- `/horus/heartbeat`
- `/horus/webrtc/client_signal`
- `/horus/webrtc/server_signal`
- `/horus/multi_operator/sdk_registration_replay_*`

## Operational guidance

- Use Jazzy when you need the most complete bridge and WebRTC validation path.
- Treat the SDK shell and the bridge shell as one runtime boundary: if `source ~/horus_ws/install/setup.bash` is missing, the SDK may serialize correctly but still fail to start the bridge automatically.
