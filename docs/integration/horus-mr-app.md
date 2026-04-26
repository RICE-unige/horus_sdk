---
title: HORUS MR Integration
sidebar_position: 2
---

# HORUS MR Integration

The SDK does not render anything itself. It feeds HORUS MR with the metadata the app needs to create robots, camera panels, task flows, and DataViz layers in the workspace.

## What the app consumes from the SDK

- robot identity, type, dimensions, and base-frame metadata
- teleop and task topics for Robot Manager
- camera definitions and per-view transport policy
- robot-scoped DataViz such as paths and collision alerts
- global visualizations such as occupancy grids, point clouds, meshes, octomaps, and semantic boxes

## What the app decides at runtime

- workspace placement and acceptance
- panel placement and visibility
- mixed-reality interaction rules
- tutorial or multi-operator gating
- authority and control presentation

## Expected integration sequence

```bash
# terminal A
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python:$PYTHONPATH
python3 python/examples/legacy/fake_tf_ops_suite.py

# terminal B
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python:$PYTHONPATH
python3 python/examples/ops_registration.py
```

In the headset:

1. launch HORUS
2. connect to the bridge host
3. accept the workspace
4. open Robot Manager for a registered robot

## Behaviors to validate

- robot spawn happens after workspace acceptance
- Robot Manager opens only for the selected robot
- minimap camera works before teleop and immersive/teleop panels follow the declared transport policy
- global layers such as maps or semantic boxes appear without forcing full robot rebuilds
- host/join sessions can reconstruct SDK registration state through replay topics
