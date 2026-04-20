---
title: Occupancy Grid Workflow
sidebar_position: 4
---

# Occupancy Grid Workflow

## Goal

Validate global occupancy-grid payload wiring and workspace scale behavior end-to-end.

Use this from a source checkout:

```bash
cd ~/horus_sdk
source /opt/ros/humble/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python
```

## 1) Publish fake occupancy grid

```bash
python3 python/examples/fake_tf_ops_suite.py \
  --robot-count 6 \
  --static-camera \
  --publish-compressed-images \
  --publish-occupancy-grid \
  --occupancy-rate 1.0 \
  --occupancy-resolution 0.10 \
  --occupancy-width 220 \
  --occupancy-height 220
```

## 2) Register global visualization

```bash
python3 python/examples/sdk_registration_demo.py \
  --robot-count 6 \
  --with-camera \
  --with-occupancy-grid \
  --occupancy-topic /map \
  --occupancy-frame map \
  --workspace-scale 0.1
```

## Expected runtime behavior

- Occupancy config is emitted in `global_visualizations`.
- The map is deduped across robot registrations.
- The MR runtime applies the map only after workspace acceptance.
- Unknown-space rendering follows the payload option.
- Global visualization updates should not force existing robots to be reconfigured.

## Regression checks

- No per-robot occupancy payload spam.
- Map topic switch reconfigures cleanly.
- No stale map activation before workspace acceptance.
- Robot Manager and teleop panels remain stable when global visualization data changes.
