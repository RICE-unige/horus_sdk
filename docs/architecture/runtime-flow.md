---
title: Runtime Flow
sidebar_position: 2
---

# Runtime Flow

## End-to-end sequence

1. Your Python code builds `Robot`, `Sensor`, and `DataViz` objects.
2. `register_robots(...)` serializes them into SDK payloads.
3. The SDK publishes registration to `/horus/registration`.
4. `horus_ros2` receives the payload and forwards runtime topics between ROS 2 and the app.
5. HORUS MR accepts a workspace and activates the registered runtime entities.
6. ACK, keep-alive, and topic-monitoring signals feed the SDK dashboard state.

## Canonical runnable flow

```bash
# terminal A: paired fake runtime
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python:$PYTHONPATH
python3 python/examples/legacy/fake_tf_ops_suite.py

# terminal B: curated registration example
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python:$PYTHONPATH
python3 python/examples/ops_registration.py
```

## Runtime decision points

- **Bridge startup**: the SDK can auto-start `horus_unity_bridge` when the ROS shell is prepared correctly.
- **Workspace acceptance**: robots and global layers do not become fully active in MR until the workspace is accepted.
- **Transport policy**: camera transport preferences come from the SDK, but the final behavior still depends on runtime capability and app mode.
- **Dashboard truth**: link and data state are derived from ACKs, app connectivity, subscriptions, and publisher activity together.

## Failure domains

- SDK issue: wrong names, frames, topics, or payload fields
- bridge issue: runtime transport not running, WebRTC unavailable, or routing broken
- MR issue: workspace not accepted, operator control unavailable, or UI intentionally gated
