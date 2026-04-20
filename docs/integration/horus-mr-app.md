---
title: HORUS MR Integration
sidebar_position: 2
---

# HORUS MR Integration

## Role of the `horus` MR app

The HORUS MR app consumes SDK registration payloads, enforces workspace-gated activation, and maps runtime robot state to mixed-reality interaction. The app owns workspace placement, Robot Manager panels, task authoring UI, teleoperation UI, shared/private workspace modes, and operator-facing control feedback.

## End-to-end flow

```bash
# 1. HORUS ROS 2 bridge
cd ~/horus_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash
ros2 launch horus_unity_bridge unity_bridge.launch.py

# 2. fake robot runtime
cd ~/horus_sdk
source /opt/ros/humble/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python
python3 python/examples/fake_tf_ops_suite.py --robot-count 4 --rate 30 --static-camera --publish-compressed-images --task-path-publish-rate 5

# 3. SDK registration
cd ~/horus_sdk
source /opt/ros/humble/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python
python3 python/examples/sdk_typical_ops_demo.py --robot-count 4 --workspace-scale 0.1
```

After these commands are running, launch the HORUS app on the headset, connect to the bridge host, and accept the workspace.

## Runtime semantics expected in MR

- Startup camera mode policy is MiniMap-first by default.
- MiniMap camera transport normally uses ROS image topics.
- Teleop camera transport can use WebRTC when the bridge is built with WebRTC support.
- Occupancy maps and other global visualizations remain hidden until workspace acceptance.
- Robot Manager interaction uses the robot's selected runtime panel and command authority state.
- Multi-operator shared/private modes are chosen in the app, while command arbitration is enforced through runtime bridge state.

## Integration checklist

- Registration ACK arrives per robot.
- Robots appear only after workspace acceptance.
- First frame appears for active camera streams.
- Robot Manager opens for the selected robot only.
- Dashboard and MR visual state agree after workspace acceptance.
- Blocked controls provide unavailable/disabled feedback instead of silently sending commands.
