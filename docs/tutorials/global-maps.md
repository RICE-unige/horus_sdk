---
title: "Tutorial 6: Global Maps"
sidebar_position: 7
toc_max_heading_level: 2
---

# Tutorial 6: Global Maps

This tutorial explains how shared world layers are added on top of the robot-description flow.

The curated SDK now keeps each major map family in its own registration example so the behavior is easy to isolate:

- occupancy grid
- pointcloud
- dense mesh
- octomap

## Goal

By the end of this step, you should understand:

- why map layers are documented separately from the base robot-description registration
- which map family matches which operator need
- how the SDK expresses these as workspace-level DataViz layers

## Occupancy grid

Use this when you want the clearest 2D navigation context:

```python
world_layers.add_occupancy_grid(
    "/map",
    frame_id="map",
    render_options={"color_free": "#222222", "color_occupied": "#F4F4F4", "alpha": 0.85},
)
```

Reference script:

- `python/examples/occupancy_map_registration.py`

## Pointcloud

Use this when the source data is a dense point set and you want direct spatial evidence instead of a meshed surface:

```python
world_layers.add_3d_map(
    "/map_3d",
    frame_id="map",
    render_options={
        "point_size": 0.055,
        "render_all_points": True,
        "visible_points_budget": 180000,
        "color": "#6ED7FF",
    },
)
```

Reference script:

- `python/examples/pointcloud_map_registration.py`

## Dense mesh

Use this when you want a more solid environmental surface:

```python
world_layers.add_3d_mesh(
    "/map_3d_mesh",
    frame_id="map",
    render_options={"max_triangles": 220000, "use_vertex_colors": True},
)
```

Reference script:

- `python/examples/mesh_map_registration.py`

## Octomap

Use this when the runtime already provides an octomap-oriented representation and you want HORUS to mirror that layer directly:

```python
world_layers.add_3d_octomap(
    "/map_3d_octomap_mesh",
    frame_id="map",
    render_options={
        "render_mode": "surface_mesh",
        "native_topic": "/map_3d_octomap",
        "max_triangles": 120000,
    },
)
```

Reference script:

- `python/examples/octomap_registration.py`

## Why these are separate examples

This is one of the places where the curated examples are intentionally not over-generalized.

The point is not to build one script with ten flags. The point is to give you one clean registration per map family so you can copy the one that matches your runtime.

## Validation path

All of these builds on the same robot-description fake runtime:

```bash
python3 python/examples/tools/fetch_robot_description_assets.py
```

Then choose one:

```bash
python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models --publish-occupancy-grid
python3 python/examples/occupancy_map_registration.py
```

```bash
python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models --map-3d-mode pointcloud --map-3d-profile realistic
python3 python/examples/pointcloud_map_registration.py
```

## What comes next

The final tutorial moves from simulated infrastructure to real ROS graphs. Continue with [Tutorial 7: Live robot checklist](live-robot-checklist.md).
