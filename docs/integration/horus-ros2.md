---
title: HORUS ROS2 Integration
sidebar_position: 1
---

# HORUS ROS2 Integration

## Role of `horus_ros2`

`horus_ros2` is the ROS 2 runtime and transport layer used by HORUS. It is not a generic ROS TCP bridge: it provides the HORUS Unity bridge, topic routing, WebRTC signaling support, and bridge-level control/lease behavior used by the MR app.

The SDK emits robot, sensor, task, teleop, and DataViz contracts. `horus_ros2` transports those contracts and runtime topics between ROS 2 and the HORUS MR application.

## Recommended startup

For local development, the repository is normally checked out at `~/horus_ws/src/horus_ros2` and built from the workspace root:

```bash
cd ~/horus_ws
source /opt/ros/humble/setup.bash  # or jazzy
colcon build --symlink-install --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

If the optional backend package is present and needed for your test, launch the complete backend instead:

```bash
cd ~/horus_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch horus_backend horus_complete_backend.launch.py
```

## Integration contract touchpoints

Core registration topics:

- `/horus/registration`
- `/horus/registration_ack`
- `/horus/heartbeat`

Camera/WebRTC signaling topics are configured through camera metadata and commonly use:

- `/horus/webrtc/client_signal`
- `/horus/webrtc/server_signal`

Multi-operator control coordination uses bridge-visible lease state and command protection paths, including:

- `/horus/multi_operator/control_lease_request`
- `/horus/multi_operator/control_lease_state`

Robot command, teleop, task, camera, and DataViz topics are authored by SDK metadata and forwarded by the runtime bridge.

## Validation commands

```bash
ros2 topic list | grep horus
ros2 topic echo /horus/registration_ack
ros2 topic echo /horus/multi_operator/control_lease_state
```

Use the SDK quickstart fake runtime and registration commands to verify the bridge with live robot data.

## Notes

- Humble is the default development target in the current local workflow; Jazzy is also supported when your ROS 2 workspace is built for it.
- Build with `-DENABLE_WEBRTC=ON` when validating teleop camera streaming.
- Workspace sharing mode is chosen in the MR app; `horus_ros2` remains the shared runtime transport and command-arbitration layer.
