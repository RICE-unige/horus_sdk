---
title: Quickstart
sidebar_position: 2
---

# Quickstart

This flow validates the real integration path:

1. Start bridge runtime.
2. Publish fake TF/camera/occupancy data.
3. Register robots using SDK demo.

## 1) Start backend/bridge

```bash
cd ~/horus/ros2
source /opt/ros/jazzy/setup.bash   # or humble
source install/setup.bash
ros2 launch horus_backend horus_complete_backend.launch.py
```

If running backend-only install:

```bash
ros2 launch horus_backend horus_backend.launch.py
```

## 2) Start fake publishers

```bash
cd ~/horus/sdk
python3 python/examples/fake_tf_publisher.py \
  --robot-count 4 \
  --with-camera \
  --publish-occupancy-grid
```

## 3) Register robots

```bash
cd ~/horus/sdk
python3 python/examples/sdk_registration_demo.py \
  --robot-count 4 \
  --with-camera \
  --with-occupancy-grid \
  --workspace-scale 0.1
```

## Expected behavior

- Registration ACK arrives for each robot.
- Dashboard core rows move to active link/data states when app is connected.
- Occupancy map payload appears under global visualizations.
- MR runtime applies map only after workspace acceptance.

## Smoke check command

```bash
python3 python/examples/e2e_registration_check.py
```
