---
title: Runtime Flow
sidebar_position: 2
---

# Runtime Flow

## End-to-end sequence

1. SDK creates robot, sensor, task, teleop, and DataViz objects.
2. SDK builds registration payloads containing robot-scoped configuration and global visualization fields.
3. Payloads are published to `/horus/registration`.
4. `horus_ros2` transports registration, command, camera, task, and DataViz topics between ROS 2 and the MR app.
5. The MR app accepts a workspace, then activates runtime entities.
6. SDK dashboard state reconciles link/data state from registration ACKs, monitor subscriptions, publisher activity, and app connectivity signals.

## Canonical flow command set

```bash
# terminal 1: HORUS ROS 2 bridge
cd ~/horus_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash
ros2 launch horus_unity_bridge unity_bridge.launch.py

# terminal 2: fake robot runtime
cd ~/horus_sdk
source /opt/ros/humble/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python
python3 python/examples/fake_tf_ops_suite.py --robot-count 6 --rate 30 --static-camera --publish-compressed-images --task-path-publish-rate 5

# terminal 3: SDK registration
cd ~/horus_sdk
source /opt/ros/humble/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python
python3 python/examples/sdk_typical_ops_demo.py --robot-count 6 --workspace-scale 0.1
```

## Failure domains

- **SDK model/serialization issue:** malformed fields, missing task metadata, or mismatched robot names in the registration payload.
- **Runtime bridge issue:** transport setup, topic routing, WebRTC signaling, or control lease enforcement fails.
- **MR lifecycle issue:** workspace is not accepted yet, the operator lacks control authority, or a panel/task is intentionally gated.

## Operational truth source

Dashboard status is derived from the combined signal path:

- app connectivity state,
- registration ACK state,
- monitor subscription linkage,
- publisher presence/activity,
- task/teleop command activity.

This avoids false-positive ?active? states during pre-workspace, disconnected, or blocked-control phases.
