---
title: Troubleshooting
sidebar_position: 1
---

# Troubleshooting

## No registration ACK

Check the runtime topics first:

```bash
ros2 topic list | grep horus
ros2 topic echo /horus/registration_ack
```

If nothing appears:

- confirm the app is connected to the bridge host
- confirm `horus_unity_bridge` is running
- confirm the SDK shell sourced `~/horus_ws/install/setup.bash`

## Bridge auto-start does not work

Try the registration script from a shell where both ROS and the workspace are active:

```bash
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
```

If you still need to debug startup, launch the bridge manually and rerun the SDK example.

## Camera panel is blank

Check:

- the camera topic exists and is publishing the expected image type
- the `frame_id` matches the real camera frame
- `image_type`, `minimap_image_type`, and `teleop_image_type` match the actual ROS message content

For live systems, also check whether the example expects WebRTC for teleop or ROS-only transport.

## Robot appears but does not move

Verify the command and task topics that the example registered:

```bash
ros2 topic list | grep cmd_vel
ros2 topic list | grep goal_pose
ros2 topic list | grep waypoint_path
```

The SDK can register a correct robot while the paired fake runtime or live robot graph is still missing the command subscribers.

## Global map or semantic layer is missing

Check that:

- the first `DataViz` instance in the batch is carrying the intended global layer
- the required topic is publishing
- the MR workspace has already been accepted

## WSL2 networking

If the headset cannot reach the bridge ports from WSL, use mirrored networking:

```ini
[wsl2]
networkingMode=mirrored
```

Then restart WSL:

```powershell
wsl --shutdown
```
