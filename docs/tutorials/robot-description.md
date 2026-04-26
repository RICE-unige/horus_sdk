---
title: "Tutorial 5: Robot Description"
sidebar_position: 6
toc_max_heading_level: 2
---

# Tutorial 5: Robot Description

The earlier tutorials used primitive dimensions as the robot body. This tutorial explains when and how to move to a URDF-backed robot description.

The reference file is `python/examples/robot_description_registration.py`.

## Goal

By the end of this step, you should understand:

- why `dimensions` are still useful even when a URDF exists
- what `configure_ros_binding(...)` is solving
- how `configure_robot_description(...)` turns a URDF into a HORUS-visible body
- which settings matter for visual meshes and runtime budgets

## The transition point

```python
robot.configure_ros_binding(base_frame="base_link")
robot.configure_robot_description(
    urdf_path=str(require_urdf("jackal.urdf")),
    base_frame="base_link",
    source="ros",
    include_visual_meshes=True,
    visual_mesh_triangle_budget=90000,
    body_mesh_mode="preview_mesh",
)
```

## What changed from the earlier tutorials

### `configure_ros_binding(...)`

This makes the base TF assumptions explicit. It is the point where you stop relying on simple naming conventions and start declaring exactly which frame HORUS should treat as the robot root.

### `configure_robot_description(...)`

This tells the SDK to send a robot-description contract instead of relying only on primitive box bounds.

The important fields are:

- `urdf_path`: where the robot description comes from
- `base_frame`: which frame anchors the body
- `include_visual_meshes`: whether HORUS should request the visual meshes
- `visual_mesh_triangle_budget`: how much geometry you are willing to spend
- `body_mesh_mode`: how aggressively the runtime should render the body

## Why dimensions are still present

The curated example still creates the robot with `RobotDimensions(...)` first. That is deliberate. Dimensions still matter for:

- interaction volumes
- fallback body bounds
- simple geometric reasoning before the full mesh path is active

Think of URDF and primitive dimensions as complementary, not mutually exclusive.

## Run path

Fetch the sample assets once:

```bash
python3 python/examples/tools/fetch_robot_description_assets.py
```

Then run:

```bash
# Terminal A
python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models

# Terminal B
python3 python/examples/robot_description_registration.py
```

## What you should see in HORUS MR

- robot bodies look like the real platforms instead of generic boxes
- the body anchor matches the real base frame
- camera panels and task overlays still attach correctly after the body upgrade

## What comes next

Once URDF-backed bodies are working, the next natural step is shared environment context. Continue with [Tutorial 6: Global maps](global-maps.md).
