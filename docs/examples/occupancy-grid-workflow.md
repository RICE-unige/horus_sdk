---
title: Occupancy Grid Workflow
sidebar_position: 4
---

# Occupancy Grid Workflow

## Goal

Validate global occupancy-grid payload wiring and workspace scale behavior end-to-end.

## 1) Publish fake occupancy grid

```bash
python3 python/examples/fake_tf_publisher.py \
  --robot-count 6 \
  --with-camera \
  --publish-occupancy-grid \
  --occupancy-topic /map \
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

- occupancy config emitted in `global_visualizations`
- deduped across robot registrations
- applied in MR only after workspace acceptance
- unknown-space rendering follows payload option

## Regression checks

- no per-robot occupancy payload spam
- map topic switch reconfigures cleanly
- no stale map activation before workspace acceptance
