---
title: Curated Examples
sidebar_position: 1
---

# Curated Examples

:::note[Reference, not tutorial]

This page is a catalog, not the main teaching path. If you are learning the SDK for the first time, start with the [tutorial summary](../tutorials/summary.md) and come back here when you need a known-good script to copy.

:::

The primary example catalog lives in `python/examples/`. These are the scripts new users should copy from when integrating a robot into HORUS.

The paired fake runtimes may still live under `python/examples/legacy/`, but they are supporting tools, not the main SDK surface.

## Core registration workflows

| Script | Use when | Pair with |
| --- | --- | --- |
| `ops_registration.py` | you want the standard wheeled fleet workflow | `python/examples/legacy/fake_tf_ops_suite.py` |
| `flat_robot_registration.py` | your robot graph is flat, not namespaced | `python/examples/legacy/fake_tf_single_flat.py` |
| `drone_registration.py` | you need drone `Take Off` / `Land` behavior | `python/examples/legacy/fake_tf_drone_ops_suite.py` |
| `legged_registration.py` | you need legged `Stand Up` / `Sit Down` behavior | `python/examples/legacy/fake_tf_legged_ops_suite.py` |
| `stereo_registration.py` | you want minimap mono plus teleop stereo transport | `python/examples/legacy/fake_stereo_camera_multi.py` |
| `semantic_perception_registration.py` | you want default-visible semantic boxes in the workspace | `python/examples/legacy/fake_tf_ops_suite.py` |

## Robot description and map workflows

| Script | Use when | Pair with |
| --- | --- | --- |
| `robot_description_registration.py` | you want URDF-backed robot bodies | `python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models` |
| `occupancy_map_registration.py` | you want a 2D occupancy layer | same robot-description runtime with `--publish-occupancy-grid` |
| `pointcloud_map_registration.py` | you want a global PointCloud2 layer | same robot-description runtime with `--map-3d-mode pointcloud` |
| `mesh_map_registration.py` | you want a dense mesh layer | same robot-description runtime with `--map-3d-mode mesh` |
| `octomap_registration.py` | you want the octomap-style mesh path | same robot-description runtime with `--map-3d-mode octomap` |
| `global_maps_registration.py` | you want one registration that carries all map families together | dedicated validation only |
| `gaussian_splat_fixture_registration.py` | you want to validate experimental Gaussian Splat DataViz with a PointCloud2 fallback | `python/examples/tools/publish_gaussian_splat_fixture.py --profile small` |
| `robot_description_compass_registration.py` | you want robot-description registration plus workspace compass metadata | project-specific compass validation |

## Live robot workflows

| Script | Use when | Notes |
| --- | --- | --- |
| `carter_registration.py` | you are integrating a live Carter fleet | includes occupancy map, lidar, compressed cameras, plans, and teleop |
| `unitree_go1_registration.py` | you are integrating a live Unitree Go1 | includes URDF mesh, compressed front camera, LaserScan, collision alert, and stand/sit relay |

## Native SDK demos

The C++ and Rust SDKs mirror the curated registration examples above by basename at the payload layer. These are native payload examples, not copies of the legacy fake runtimes or replacements for Python live bridge registration.

| Python script | C++ example | Rust example |
| --- | --- | --- |
| `ops_registration.py` | `cpp/examples/ops_registration.cpp` | `rust/examples/ops_registration.rs` |
| `flat_robot_registration.py` | `cpp/examples/flat_robot_registration.cpp` | `rust/examples/flat_robot_registration.rs` |
| `drone_registration.py` | `cpp/examples/drone_registration.cpp` | `rust/examples/drone_registration.rs` |
| `legged_registration.py` | `cpp/examples/legged_registration.cpp` | `rust/examples/legged_registration.rs` |
| `stereo_registration.py` | `cpp/examples/stereo_registration.cpp` | `rust/examples/stereo_registration.rs` |
| `semantic_perception_registration.py` | `cpp/examples/semantic_perception_registration.cpp` | `rust/examples/semantic_perception_registration.rs` |
| `robot_description_registration.py` | `cpp/examples/robot_description_registration.cpp` | `rust/examples/robot_description_registration.rs` |
| `robot_description_compass_registration.py` | `cpp/examples/robot_description_compass_registration.cpp` | `rust/examples/robot_description_compass_registration.rs` |
| `occupancy_map_registration.py` | `cpp/examples/occupancy_map_registration.cpp` | `rust/examples/occupancy_map_registration.rs` |
| `pointcloud_map_registration.py` | `cpp/examples/pointcloud_map_registration.cpp` | `rust/examples/pointcloud_map_registration.rs` |
| `mesh_map_registration.py` | `cpp/examples/mesh_map_registration.cpp` | `rust/examples/mesh_map_registration.rs` |
| `octomap_registration.py` | `cpp/examples/octomap_registration.cpp` | `rust/examples/octomap_registration.rs` |
| `global_maps_registration.py` | `cpp/examples/global_maps_registration.cpp` | `rust/examples/global_maps_registration.rs` |
| `gaussian_splat_fixture_registration.py` | `cpp/examples/gaussian_splat_fixture_registration.cpp` | `rust/examples/gaussian_splat_fixture_registration.rs` |
| `carter_registration.py` | `cpp/examples/carter_registration.cpp` | `rust/examples/carter_registration.rs` |
| `unitree_go1_registration.py` | `cpp/examples/unitree_go1_registration.cpp` | `rust/examples/unitree_go1_registration.rs` |
| `uav_sim_horus_registration.py` | `cpp/examples/uav_sim_horus_registration.cpp` | `rust/examples/uav_sim_horus_registration.rs` |

`cpp/examples/sdk_registration_demo.cpp` and `rust/examples/sdk_registration_demo.rs` remain the short native equivalent of `ops_registration.py` for payload construction.

## Recommended reading order

Start here in order:

1. `ops_registration.py`
2. `flat_robot_registration.py`
3. `robot_description_registration.py`
4. the map showcase registrations
5. `gaussian_splat_fixture_registration.py` only after pointcloud map behavior is stable
6. live Carter or Unitree only after the first three are stable

## Where legacy still matters

The legacy example folder is still valuable for:

- fake runtimes that simulate topics and command subscribers
- advanced validation variants
- historical workflows that are not the primary onboarding path anymore

Use the root examples first. Drop into `python/examples/legacy/` only when you need a paired simulator or a deeper diagnostic flow.
