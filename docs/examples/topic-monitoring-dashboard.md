---
title: Live Robot Workflows
sidebar_position: 4
---

# Live Robot Workflows

:::warning[Use this after the tutorial track]

This page is for live-system references. If you are not already comfortable with the SDK shape, work through [Tutorial 7: Live robot checklist](../tutorials/live-robot-checklist.md) first.

:::

These are the curated examples that are meant to mirror real robot graphs instead of synthetic paired runtimes.

## Carter fleet

Use `python/examples/carter_registration.py` when your ROS graph provides:

- `/tf`
- `/tf_static`
- `/shared_map`
- `/<robot>/cmd_vel`
- `/<robot>/chassis/odom`
- `/<robot>/front_2d_lidar/scan`
- `/<robot>/front_stereo_camera/left/image_raw/compressed`
- `/rviz/<robot>/plan`
- `/rviz/<robot>/local_plan`

Launch:

```bash
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python:$PYTHONPATH
python3 python/examples/carter_registration.py
```

## Unitree Go1

Use `python/examples/unitree_go1_registration.py` when you have the real Go1 ROS graph plus the Go1 description package available locally.

Run the relay in one terminal and the registration in another:

```bash
# terminal A
python3 python/examples/tools/unitree_go1_high_mode_relay.py

# terminal B
python3 python/examples/unitree_go1_registration.py
```

This workflow covers:

- URDF-backed body visualization
- front compressed camera registration
- LaserScan registration
- collision alert DataViz
- stand/sit relay behavior for HORUS MR controls

## Observability expectations

For both live workflows, validate:

- registration ACK appears
- the expected sensor topics are present
- Robot Manager opens for the intended robot only
- camera, task, and DataViz layers align to the real TF tree
