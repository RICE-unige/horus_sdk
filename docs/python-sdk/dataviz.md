---
title: DataViz
sidebar_position: 3
---

# DataViz

## When to use it

Use `DataViz` when you need to declare what an operator should see in HORUS MR beyond the robot body itself: paths, odometry trails, collision alerts, maps, and semantic layers.

Keep two scopes separate:

- robot-scoped overlays belong on the robot's own `DataViz`
- shared world layers belong on one global `DataViz`, usually the first one passed to `register_robots(...)`

That separation matters in MR because global maps are anchored to the accepted workspace, while robot-scoped layers follow robot identity and TF.

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

## Global map families

| Method | Primary topic type | Best for |
| --- | --- | --- |
| `add_occupancy_grid(...)` | `nav_msgs/OccupancyGrid` | 2D navigation context |
| `add_3d_map(...)` | `sensor_msgs/PointCloud2` | direct 3D evidence and scan-derived maps |
| `add_3d_mesh(...)` | mesh marker or mesh map topic | solid world surfaces |
| `add_3d_octomap(...)` | octomap mesh/native octomap topics | octomap-style map runtimes |
| `add_gaussian_splat_map(...)` | HORUS Gaussian splat manifest plus ROS chunks | experimental 3DGS visualization with pointcloud fallback |

Use the smallest map family that answers the operator's question. A dense visual layer is useful for inspection, but it costs more headset memory, GPU time, and registration debugging time than an occupancy grid.

## Pointcloud rendering options

PointCloud2 layers should normally use workspace-scaled point sizing so the same registration behaves correctly at tabletop and room scale:

```python
world_layers.add_3d_map(
    "/map_3d",
    frame_id="map",
    render_options={
        "point_size": 0.035,
        "auto_point_size_by_workspace_scale": True,
        "min_point_size": 0.0015,
        "max_point_size": 0.012,
        "point_shape": "circle",
        "render_mode": "opaque_fast",
        "render_all_points": True,
        "visible_points_budget": 120000,
        "max_visible_points_budget": 180000,
        "enable_view_frustum_culling": False,
        "map_static_mode": True,
    },
)
```

Use `point_shape="circle"` when square billboard corners overlap and make the map look like flat panels. Keep `render_mode="opaque_fast"` unless you specifically need transparency; it is the preferred Quest path.

## Gaussian splat maps

Gaussian splat support is a HORUS DataViz path, not a separate viewer. The SDK registers a small manifest topic and receives the asset over ROS chunk topics, matching the rest of the HORUS registration workflow.

```python
world_layers.add_gaussian_splat_map(
    manifest_topic="/horus/gaussian_splat/manifest",
    frame_id="map",
    preview_topic="/map_gaussian_splat_preview",
    render_options={
        "render_mode": "splats",
        "max_splats": 350000,
        "render_scale": 0.5,
        "sh_order": 2,
        "half_precision_sh": True,
        "adaptive_sort": True,
        "sort_passes": 2,
        "splat_scale": 1.0,
        "contribution_cull_threshold": 0.1,
        "pointcloud_fallback": True,
    },
)
```

`render_mode` accepts:

| Value | Purpose |
| --- | --- |
| `splats` | normal Gaussian splat rendering |
| `debug_points` | fixed-size points for workspace anchoring checks |
| `mono_center_eye` | Quest/XR diagnostic mode for isolating stereo matrix issues |
| `no_covariance` | fixed-size splats for checking whether covariance expansion is causing distortion |

Keep `pointcloud_fallback=True` while developing. The fallback preview lets HORUS MR show the same map frame if splat loading, caching, or rendering fails.

## Common failure and fix

- **Failure:** maps or semantic layers appear duplicated or do not appear consistently across operators.
- **Fix:** keep workspace-level layers in global DataViz scope and make sure the intended topic is published by the runtime. For point clouds and dense maps, also verify render options such as point size, budgets, and culling policy.
- **Failure:** pointcloud maps look like large overlapping panels.
- **Fix:** use workspace-scaled sizing, smaller `min_point_size` / `max_point_size` clamps, and `point_shape="circle"`.
- **Failure:** a Gaussian splat layer does not appear, but the fallback pointcloud does.
- **Fix:** check the `/horus/gaussian_splat/manifest`, chunk topics, cache/hash logs in HORUS MR, and temporarily set `render_mode="debug_points"` to isolate loading from covariance and stereo rendering.
