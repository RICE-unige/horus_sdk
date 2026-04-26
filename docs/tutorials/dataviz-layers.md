---
title: "Tutorial 4: DataViz Layers"
sidebar_position: 5
toc_max_heading_level: 2
---

# Tutorial 4: DataViz Layers

This tutorial explains how to declare what the operator should see around a robot, not just what they can command.

In HORUS, the robot body and the operator controls are only part of the story. Paths, odometry, collision risk, and semantic layers are what turn the workspace into a useful supervisory interface.

## Goal

By the end of this step, you should understand:

- the difference between robot-scoped overlays and workspace-level overlays
- how `DataViz` is attached to a robot
- why path, odometry, and safety signals are modeled separately
- how semantic boxes fit into the current visualization contract

## Step 1: create a robot-scoped layer

```python
dataviz = robot.create_dataviz()
```

This is the container that carries operator-facing overlays for one robot registration.

## Step 2: add motion context

```python
robot.add_path_planning_to_dataviz(
    dataviz,
    global_path_topic="/atlas/global_path",
    local_path_topic="/atlas/local_path",
)
```

These helpers encode the common path overlays most operators expect:

- global path
- local path

## Step 3: add safety context

```python
robot.add_navigation_safety_to_dataviz(
    dataviz,
    odom_topic="/atlas/odom",
    collision_risk_topic="/atlas/collision_risk",
)
```

This adds the normal safety-aware overlays:

- odometry trail
- collision alert

## Step 4: add shared semantic context when needed

```python
semantic_layer = datavizs[0]
semantic_layer.add_semantic_box(
    "person_1",
    "person",
    center=(1.6, 0.8, 0.9),
    size=(0.45, 0.45, 1.8),
    render_options={"color": "#00C853", "confidence": 0.92},
)
```

This is a good example of the difference between robot-scoped and workspace-scoped meaning:

- path and odometry are usually attached to one robot
- semantic boxes usually describe the shared world

The semantic example keeps the layer in one `DataViz` object, but the meaning is still workspace-global from the operator's point of view.

:::note[Current contract shape]

The current SDK surface is still helper-driven rather than a generic visualization-entity graph. That is why the common path and safety overlays appear as explicit helper methods.

:::

## What to validate

Base robot awareness:

```bash
# Terminal A
python3 python/examples/legacy/fake_tf_ops_suite.py

# Terminal B
python3 python/examples/ops_registration.py
```

Semantic layer:

```bash
# Terminal A
python3 python/examples/legacy/fake_tf_ops_suite.py --robot-count 4

# Terminal B
python3 python/examples/semantic_perception_registration.py
```

In HORUS MR, verify:

- global and local paths render on the right robot
- odometry and collision overlays are spatially aligned
- semantic boxes appear in the shared world rather than behaving like per-robot UI

## What comes next

At this point you understand the core registration model for basic operations. The next step is visual fidelity: [Tutorial 5: Robot description](robot-description.md).
