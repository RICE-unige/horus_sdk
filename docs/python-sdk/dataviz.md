---
title: DataViz
sidebar_position: 3
---

# DataViz

## When to use it

Use DataViz definitions to declare visualization intent for robot-scoped and global layers (for example occupancy maps).

## Minimal example

```python
dataviz = robot.create_dataviz()
dataviz.add_tf(topic="/tf", frame_id="map")
```

## Realistic example

```python
dataviz = robot.create_dataviz()
dataviz.add_laserscan(topic="/scan", frame_id="laser_link", color="#00FFFF")
dataviz.add_pointcloud(topic="/points_raw", frame_id="lidar_link", color="#7DD3FC")
dataviz.add_occupancy_grid(
    topic="/map",
    frame_id="map",
    render_options={
        "show_unknown_space": True,
        "position_scale": 0.1,
        "position_offset": {"x": 0.0, "y": 0.0, "z": 0.0},
        "rotation_offset_euler": {"x": 0.0, "y": 0.0, "z": 0.0}
    }
)
```

## Common failure and fix

- **Failure:** occupancy grid config duplicated across robots causing payload bloat.
- **Fix:** rely on global visualization dedupe path; keep occupancy in global scope and robot-specific layers in `visualizations`.
