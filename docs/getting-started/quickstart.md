---
title: Quickstart
sidebar_position: 2
---

# Quickstart

This quickstart validates the current HORUS integration path from a source checkout:

1. Start `horus_ros2` bridge runtime.
2. Publish fake TF, camera, teleop, and task data.
3. Register robots using the SDK demo.
4. Open the HORUS MR app and accept the workspace.

The commands below assume the local development layout used by the project:

- `~/horus_ws/src/horus_ros2` for the ROS 2 bridge/runtime repository.
- `~/horus_sdk` for the SDK repository.
- ROS 2 Humble or Jazzy installed.

## 1) Start the HORUS ROS 2 bridge

Build and launch the Unity bridge from the ROS 2 workspace root:

```bash
cd ~/horus_ws
source /opt/ros/humble/setup.bash  # use jazzy if that is your active ROS distro
colcon build --symlink-install --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

If your workspace includes the optional backend package and you want the full backend launch instead, use:

```bash
cd ~/horus_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch horus_backend horus_complete_backend.launch.py
```

## 2) Start the fake robot runtime

In a second terminal, start the unified fake runtime. This publishes TF, camera streams, teleop command handling, task status, and optional global visualization data.

```bash
cd ~/horus_sdk
source /opt/ros/humble/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python

python3 python/examples/fake_tf_ops_suite.py \
  --robot-count 4 \
  --rate 30 \
  --static-camera \
  --publish-compressed-images \
  --task-path-publish-rate 5 \
  --publish-collision-risk \
  --collision-threshold-m 1.2
```

## 3) Register robots with the SDK

In a third terminal, register matching robot metadata with HORUS:

```bash
cd ~/horus_sdk
source /opt/ros/humble/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python

python3 python/examples/sdk_typical_ops_demo.py \
  --robot-count 4 \
  --workspace-scale 0.1
```

Keep this process running while testing. It keeps the SDK dashboard alive and lets the MR app receive registration updates and status changes.

## 4) Open HORUS MR

Start the HORUS app on the headset, connect it to the machine running `horus_ros2`, then accept the workspace when prompted.

Expected MR behavior:

- Registration ACK arrives for each robot.
- Four robots appear after workspace acceptance.
- Robot Manager opens for selected robots.
- Camera panels and DataViz rows move to active states when data is flowing.
- Teleop and task controls publish through the bridge instead of local-only mock paths.

## Optional occupancy-grid check

Use this when validating global map visualization behavior:

```bash
# terminal 2: fake runtime with occupancy grid
cd ~/horus_sdk
source /opt/ros/humble/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python

python3 python/examples/fake_tf_ops_suite.py \
  --robot-count 6 \
  --static-camera \
  --publish-compressed-images \
  --publish-occupancy-grid \
  --occupancy-rate 1.0

# terminal 3: registration with global occupancy visualization metadata
cd ~/horus_sdk
source /opt/ros/humble/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python

python3 python/examples/sdk_registration_demo.py \
  --robot-count 6 \
  --with-camera \
  --with-occupancy-grid \
  --workspace-scale 0.1
```

Expected occupancy behavior:

- Occupancy metadata is emitted under `global_visualizations`.
- The MR runtime applies the map only after workspace acceptance.
- Robot registration should not be rebuilt just because global visualization data updates.

## Smoke check command

Run this when you only need payload-level validation without launching the full MR app:

```bash
cd ~/horus_sdk
export PYTHONPATH=python
python3 python/examples/e2e_registration_check.py
```
