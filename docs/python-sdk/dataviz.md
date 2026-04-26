---
title: DataViz
sidebar_position: 3
---

# DataViz

## When to use it

Use `DataViz` when you need to declare what an operator should see in HORUS MR beyond the robot body itself: paths, odometry trails, collision alerts, maps, and semantic layers.

## Minimal example

```python
dataviz = robot.create_dataviz()
robot.add_path_planning_to_dataviz(
    dataviz,
    global_path_topic="/atlas/global_path",
    local_path_topic="/atlas/local_path",
)
```

## Realistic example

```python
dataviz = robot.create_dataviz()

robot.add_navigation_safety_to_dataviz(
    dataviz,
    odom_topic="/atlas/odom",
    collision_risk_topic="/atlas/collision_risk",
)

dataviz.add_occupancy_grid(
    "/map",
    frame_id="map",
    render_options={
        "color_free": "#222222",
        "color_occupied": "#F4F4F4",
        "alpha": 0.85,
    },
)

dataviz.add_semantic_box(
    "person_1",
    "person",
    center=(1.6, 0.8, 0.9),
    size=(0.45, 0.45, 1.8),
    render_options={"color": "#00C853", "confidence": 0.92},
)
```

Use robot convenience helpers for common per-robot overlays, and direct `DataViz` methods for global layers such as occupancy, point clouds, meshes, octomaps, and semantic entities.

## Common failure and fix

- **Failure:** maps or semantic layers appear duplicated or do not appear consistently across operators.
- **Fix:** keep workspace-level layers in global DataViz scope and make sure the intended topic is published by the runtime. For point clouds and dense maps, also verify render options such as point size, budgets, and culling policy.
