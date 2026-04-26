---
title: Map Workflows
sidebar_position: 3
---

# Map Workflows

Map layers in HORUS are registered through SDK DataViz as global visualizations. The curated examples separate the registration script from the paired runtime that publishes the map data.

## Shared prerequisite

For the robot-description-backed map examples:

```bash
python3 python/examples/tools/fetch_robot_description_assets.py
```

## Occupancy grid

```bash
# terminal A
python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models --publish-occupancy-grid

# terminal B
python3 python/examples/occupancy_map_registration.py
```

## PointCloud2

```bash
# terminal A
python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models --map-3d-mode pointcloud --map-3d-profile realistic

# terminal B
python3 python/examples/pointcloud_map_registration.py
```

## Dense mesh

```bash
# terminal A
python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models --map-3d-mode mesh --map-3d-profile realistic --map-3d-mesh-voxel-size 0.07 --map-3d-mesh-max-voxels 220000 --map-3d-mesh-max-triangles 220000 --map-3d-mesh-update-policy snapshot

# terminal B
python3 python/examples/mesh_map_registration.py
```

## Octomap-style mesh

```bash
# terminal A
python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models --map-3d-mode octomap --map-3d-profile realistic

# terminal B
python3 python/examples/octomap_registration.py
```

## Operational notes

- `global_maps_registration.py` remains useful when you want one registration payload carrying multiple map families at once.
- Point cloud and mesh examples are the most sensitive to render-option tuning, workspace scale, and performance budgets.
- In MR, global layers stay workspace-gated; they are not the trigger for full robot rebuilds.
