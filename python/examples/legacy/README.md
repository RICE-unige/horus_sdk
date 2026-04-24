# Legacy Example Catalog

This folder contains the preserved HORUS SDK demo suite. These examples are intentionally practical: each primary workflow tells you which fake ROS data source to run, which SDK registration script to run, and what should appear in HORUS MR.

The new curated examples will live in `python/examples/`. This folder remains the reference catalog for older, full-system validation scenarios.

## How These Examples Work

Most HORUS MR examples are two-part workflows:
- Terminal A publishes runtime data: TF, camera images, maps, odometry, task status, or fake robot motion.
- Terminal B runs SDK registration: robot identities, sensors, Robot Manager controls, tasks, and DataViz metadata.

Running only Terminal A is not enough because HORUS MR will see ROS topics but may not know what robots or UI controls to create. Running only Terminal B is also not enough because HORUS MR may create robots and panels, but the data streams will be missing or stale.

Most runtime scripts are ROS 2 scripts. Run them from Linux/WSL with ROS sourced, not from a plain Windows shell.

## Quick Chooser

| Goal | Run This Workflow |
|---|---|
| First complete ground-robot test | Recommended first workflow: `fake_tf_ops_suite.py` + `sdk_typical_ops_demo.py` |
| Teleop, Go-To Point, waypoint, nav paths, odometry, collision risk | Recommended first workflow |
| Occupancy grid | Occupancy Map Registration |
| PointCloud2 3D map | 3D Map PointCloud Profiles |
| Quest-friendly mesh map | 3D Mesh Map Pipeline |
| Octomap-style global map | Octomap / Octomap Mesh Pipeline |
| Host/join multi-operator validation | Multi-Operator Host Validation |
| Flat root ROS topics like `/cmd_vel` and `/goal_pose` | Flat Single-Robot ROS Binding |
| Drone takeoff/land and 3D tasks | Drone Takeoff, Land, and 3D Task Flow |
| Legged stand/sit actions | Legged Stand and Sit Flow |
| URDF / robot-description rendering | Robot Description Demo |
| First-time onboarding tutorial | Workspace Tutorial Demo |
| Robot description plus managed 3D map helpers | Robot Description With Managed 3D Map |
| Stereo immersive teleop | Stereo Camera Immersive Teleop |
| Semantic boxes / perception layer | Semantic Perception Layer |
| Live Carter system | Carter Live Integration |
| Diagnostics and support tools | Script Coverage table |

## Common Setup

Use this in each WSL terminal:

```bash
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash  # or your active ROS distro
export PYTHONPATH=python:$PYTHONPATH
```

Start the HORUS bridge in a separate terminal:

```bash
cd ~/horus_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

Then connect HORUS MR, draw or join a workspace, accept it, and run one workflow below.

Support utilities outside this legacy folder:
- `python/examples/tools/fetch_robot_description_assets.py`
- `python/examples/.local_assets/robot_descriptions/`

If you installed HORUS via `install.sh`, translate `~/horus_sdk/...` to `~/horus/sdk/...` and run the same script paths from there.

## Recommended First Workflow

Use this when you want the most complete fake ground-robot scene: TF, cameras, teleop, Go-To Point, waypoint, nav-path overlays, odometry trail, and collision-risk data.

Terminal A, fake robot data:

```bash
python3 python/examples/legacy/fake_tf_ops_suite.py \
  --robot-count 10 \
  --rate 30 \
  --static-camera \
  --publish-compressed-images \
  --task-path-publish-rate 5 \
  --publish-collision-risk \
  --collision-threshold-m 1.2
```

Terminal B, SDK registration for the same robots:

```bash
python3 python/examples/legacy/sdk_typical_ops_demo.py \
  --robot-count 10 \
  --workspace-scale 0.1
```

Expected in HORUS MR:
- 10 wheeled robots appear after workspace acceptance.
- Robot Manager exposes camera, teleop, Go-To Point, Draw Waypoint, Draw Nav Path, odometry trail, and collision-risk DataViz.
- Robots remain idle until teleop or task commands are sent.
- Use this workflow for normal ground teleop, Go-To Point, waypoint, and task-status validation.

## Paired Workflows

### Occupancy Map Registration

Use this when you only need a simple 2D occupancy-grid layer with basic robot registration.

Terminal A, fake TF plus occupancy grid:

```bash
python3 python/examples/legacy/fake_tf_publisher.py \
  --robot-count 6 \
  --publish-occupancy-grid \
  --occupancy-rate 1.0
```

Terminal B, SDK registration with global occupancy metadata:

```bash
python3 python/examples/legacy/sdk_registration_demo.py \
  --robot-count 6 \
  --with-occupancy-grid \
  --workspace-scale 0.1
```

Expected in HORUS MR:
- Six robots appear from TF.
- A global occupancy layer appears in the accepted workspace.

### 3D Map PointCloud Profiles

The old separate basic, compact-house, and realistic map scripts are consolidated into `fake_3d_map_publisher.py`:
- `--profile basic`: lightweight room and simple obstacles.
- `--profile compact_house`: Quest-friendly colorful indoor scene with a hard point cap.
- `--profile realistic`: dense indoor stress-test scene with replay bursts for multi-operator sessions.

Use these as three-terminal workflows: fake robot TF, fake map data, then SDK registration.

Terminal A1, fake robot TF/camera data:

```bash
python3 python/examples/legacy/fake_tf_publisher.py \
  --robot-count 4 \
  --static-camera \
  --publish-compressed-images
```

Terminal A2, choose one map profile.

Basic map:

```bash
python3 python/examples/legacy/fake_3d_map_publisher.py \
  --profile basic \
  --topic /map_3d \
  --frame map
```

Compact Quest-friendly map:

```bash
python3 python/examples/legacy/fake_3d_map_publisher.py \
  --profile compact_house \
  --topic /map_3d \
  --frame map \
  --max-points 100000 \
  --publish-mode on_change
```

Dense stress-test map:

```bash
python3 python/examples/legacy/fake_3d_map_publisher.py \
  --profile realistic \
  --topic /map_3d \
  --frame map \
  --max-points 100000 \
  --publish-mode on_change
```

Terminal B, SDK registration with PointCloud2 map metadata:

```bash
python3 python/examples/legacy/sdk_registration_demo.py \
  --robot-count 4 \
  --with-camera \
  --with-3d-map \
  --map-3d-topic /map_3d \
  --map-3d-frame map \
  --workspace-scale 0.1
```

Expected in HORUS MR:
- Robots appear from the TF stream.
- A global 3D point-cloud layer appears from `/map_3d`.
- The selected profile changes map density/content only; the SDK registration path stays the same.

### 3D Mesh Map Pipeline

Use this when validating the Quest-friendly mesh map path instead of raw point-cloud rendering.

Terminal A1, fake robot TF/camera data:

```bash
python3 python/examples/legacy/fake_tf_publisher.py \
  --robot-count 4 \
  --static-camera \
  --publish-compressed-images
```

Terminal A2, source point cloud:

```bash
python3 python/examples/legacy/fake_3d_map_publisher.py \
  --profile compact_house \
  --topic /map_3d \
  --frame map \
  --publish-mode on_change
```

Terminal A3, point-cloud-to-mesh converter:

```bash
python3 python/examples/legacy/pointcloud_to_voxel_mesh_marker.py \
  --cloud-topic /map_3d \
  --mesh-topic /map_3d_mesh \
  --voxel-size 0.10 \
  --max-voxels 60000 \
  --max-triangles 60000 \
  --update-policy snapshot
```

Terminal B, SDK registration with mesh metadata:

```bash
python3 python/examples/legacy/sdk_registration_demo.py \
  --robot-count 4 \
  --with-camera \
  --with-3d-mesh \
  --map-3d-mesh-topic /map_3d_mesh \
  --workspace-scale 0.1
```

Expected in HORUS MR:
- Robots appear from TF.
- The global map renders through the mesh marker path instead of raw `PointCloud2`.
- This is the preferred manual 3D map test when Quest performance matters.

### Octomap / Octomap Mesh Pipeline

Use this when validating the octomap-style global visualization path.

Terminal A1, fake robot TF/camera data:

```bash
python3 python/examples/legacy/fake_tf_publisher.py \
  --robot-count 4 \
  --static-camera \
  --publish-compressed-images
```

Terminal A2, fake octomap and mesh marker data:

```bash
python3 python/examples/legacy/fake_octomap_publisher.py \
  --octomap-topic /map_3d_octomap \
  --mesh-topic /map_3d_octomap_mesh \
  --frame map \
  --max-voxels 60000 \
  --max-triangles 60000
```

Terminal B, SDK registration with octomap metadata:

```bash
python3 python/examples/legacy/sdk_robot_description_demo.py \
  --robot-profile real_models \
  --workspace-scale 0.1 \
  --collision-opaque \
  --map-3d-mode octomap
```

Expected in HORUS MR:
- Robot-description robots are registered.
- Octomap mesh data appears through the global 3D map path.

### Multi-Operator Host Validation

Use this with the MR host/join workflow. The fake data source is the standard ops suite; the SDK registration is host-aware and supports joiner registry replay.

Terminal A, fake robot data:

```bash
python3 python/examples/legacy/fake_tf_ops_suite.py \
  --robot-count 10 \
  --rate 30 \
  --static-camera \
  --publish-compressed-images
```

Terminal B, SDK host registration:

```bash
python3 python/examples/legacy/sdk_multi_operator_host_demo.py \
  --robot-count 10 \
  --workspace-scale 0.1
```

Expected in HORUS MR:
- The host creates the workspace and registers the robot team.
- Joiners receive the SDK registry replay after the shared workspace baseline is available.
- Dashboard operator visibility and registration replay state should update.

### Flat Single-Robot ROS Binding

Use this when a robot publishes root topics such as `/cmd_vel`, `/goal_pose`, `/waypoint_path`, `/odom`, and flat TF frames such as `base_link` and `camera_link`.

Terminal A, flat fake runtime:

```bash
python3 python/examples/legacy/fake_tf_single_flat.py \
  --map-frame map \
  --base-frame base_link \
  --camera-frame camera_link \
  --rate 30
```

Terminal B, flat SDK registration:

```bash
python3 python/examples/legacy/sdk_single_robot_flat_demo.py \
  --robot-name robot \
  --base-frame base_link \
  --camera-frame camera_link \
  --camera-topic /camera/image_raw \
  --workspace-scale 0.1
```

Expected in HORUS MR:
- One robot named `robot` appears.
- Control topics resolve to flat roots instead of `/<robot>/...`.
- Ambiguous multi-robot flat-root registration is intentionally rejected by the MR side.

### Drone Takeoff, Land, and 3D Task Flow

Use this for aerial command and 3D goal validation.

Terminal A, fake drone runtime:

```bash
python3 python/examples/legacy/fake_tf_drone_ops_suite.py \
  --robot-count 3 \
  --robot-names drone_1,drone_2,drone_3 \
  --min-altitude 0.0 \
  --max-altitude 25.0 \
  --takeoff-altitude 1.2
```

Terminal B, SDK registration:

```bash
python3 python/examples/legacy/sdk_typical_ops_demo.py \
  --robot-names drone_1,drone_2,drone_3 \
  --teleop-profile drone \
  --go-to-min-altitude 0.0 \
  --go-to-max-altitude 25.0 \
  --workspace-scale 0.1
```

Expected in HORUS MR:
- Drone `Take Off` and `Land` controls are active.
- Go-To Point, waypoint, and draw-path use 3D authoring with altitude limits.
- Multi-Robot Go-To Point can dispatch to active aerial robots.

### Legged Stand and Sit Flow

Use this for legged Robot Manager actions. Stand/sit are action commands only; they do not take height arguments.

Terminal A, fake legged runtime:

```bash
python3 python/examples/legacy/fake_tf_legged_ops_suite.py \
  --robot-names legged_1,legged_2,legged_3 \
  --min-height 0.0 \
  --max-height 0.6
```

Terminal B, SDK registration:

```bash
python3 python/examples/legacy/sdk_legged_ops_demo.py
```

Expected in HORUS MR:
- `TakeOffButton` is relabeled to `Stand Up` and publishes an empty action on `/<robot>/stand_up`.
- `LandButton` is relabeled to `Sit Down` and publishes an empty action on `/<robot>/sit_down`.
- Wheeled robots still show those two controls disabled.

### Robot Description Demo

Use this for URDF-driven robot visualization.

Fetch URDF assets first:

```bash
python3 python/examples/tools/fetch_robot_description_assets.py --force
```

Terminal A, fake runtime for description-driven robots:

```bash
python3 python/examples/legacy/fake_tf_robot_description_suite.py \
  --robot-profile real_models \
  --map-3d-mode off
```

Terminal B, SDK registration:

```bash
python3 python/examples/legacy/sdk_robot_description_demo.py \
  --robot-profile real_models \
  --workspace-scale 0.1 \
  --collision-opaque \
  --map-3d-mode off
```

Expected in HORUS MR:
- Description-driven body, collision, joint, and optional visual-mesh data are available.
- Robot-description DataViz channels appear separately from normal sensor layers.

### Workspace Tutorial Demo

Use this to enable the robot-description onboarding tutorial. The normal robot-description demo does not show the tutorial; this registration path opts into it.

Fetch URDF assets first:

```bash
python3 python/examples/tools/fetch_robot_description_assets.py --force
```

Terminal A, fake runtime:

```bash
python3 python/examples/legacy/fake_tf_robot_description_suite.py \
  --robot-profile real_models \
  --map-3d-mode off
```

Terminal B, tutorial-enabled SDK registration:

```bash
python3 python/examples/legacy/sdk_robot_description_tutorial_demo.py \
  --robot-profile real_models \
  --workspace-scale 0.1 \
  --collision-opaque \
  --map-3d-mode off
```

Expected in HORUS MR:
- The normal robot-description demo remains unchanged.
- The tutorial panel appears only for this tutorial-enabled registration.
- The preset id is `robot_description_onboarding_v1`.

### Robot Description With Managed 3D Map

The robot-description fake runtime can auto-start the consolidated point-cloud publisher and mesh converter. Use this when you want one fake runtime terminal rather than manually launching map helper terminals.

Mesh mode, recommended for Quest 3:

```bash
python3 python/examples/legacy/fake_tf_robot_description_suite.py \
  --robot-profile real_models \
  --map-3d-mode mesh \
  --map-3d-profile compact_house
```

```bash
python3 python/examples/legacy/sdk_robot_description_demo.py \
  --robot-profile real_models \
  --workspace-scale 0.1 \
  --collision-opaque \
  --map-3d-mode mesh
```

Dense mesh stress mode:

```bash
python3 python/examples/legacy/fake_tf_robot_description_suite.py \
  --robot-profile real_models \
  --map-3d-mode mesh \
  --map-3d-profile realistic
```

```bash
python3 python/examples/legacy/sdk_robot_description_demo.py \
  --robot-profile real_models \
  --workspace-scale 0.1 \
  --collision-opaque \
  --map-3d-mode mesh \
  --map-3d-profile realistic
```

Octomap mode:

```bash
python3 python/examples/legacy/fake_tf_robot_description_suite.py \
  --robot-profile real_models \
  --map-3d-mode octomap
```

```bash
python3 python/examples/legacy/sdk_robot_description_demo.py \
  --robot-profile real_models \
  --workspace-scale 0.1 \
  --collision-opaque \
  --map-3d-mode octomap
```

Expected in HORUS MR:
- Robot-description robots appear.
- The fake runtime manages the selected map helper pipeline.
- The SDK registration advertises the matching global visualization metadata.

### Stereo Camera Immersive Teleop

Use this for dual-stream camera metadata, MiniMap/immersive camera switching, and stereo teleop rendering.

Terminal A, fake TF plus dual-stream camera data:

```bash
python3 python/examples/legacy/fake_stereo_camera_multi.py \
  --robot-count 4 \
  --stereo-robot-count 4 \
  --stereo-input-mode sbs \
  --stereo-eye-resolution 960x540 \
  --stereo-baseline 0.12 \
  --image-rate 10 \
  --highres-robot stereo_bot_1 \
  --highres-eye-resolution 1920x1080 \
  --highres-image-rate 90 \
  --static-camera \
  --publish-compressed-images
```

Terminal B, matching SDK camera metadata:

```bash
python3 python/examples/legacy/sdk_stereo_registration_demo.py \
  --robot-count 4 \
  --stereo-robot-count 4 \
  --stereo-input-mode sbs \
  --stereo-eye-resolution 960x540 \
  --mono-resolution 960x540 \
  --highres-robot stereo_bot_1 \
  --highres-eye-resolution 1920x1080 \
  --highres-camera-fps 90 \
  --workspace-scale 0.1
```

Expected in HORUS MR:
- MiniMap camera view stays mono.
- Immersive teleop view uses the stereo stream.
- Use `--stereo-input-mode dual_topic` in both terminals to validate left/right topic mode.

### Semantic Perception Layer

Use this when validating semantic boxes with the standard ops-suite robots.

Terminal A, fake robot data:

```bash
python3 python/examples/legacy/fake_tf_ops_suite.py \
  --robot-count 4 \
  --static-camera \
  --publish-compressed-images
```

Terminal B, SDK registration with semantic layer metadata:

```bash
python3 python/examples/legacy/sdk_fake_semantic_perception_demo.py
```

Expected in HORUS MR:
- Robots and semantic boxes appear together in the same workspace.
- Semantic boxes are treated as global visualization data rather than robot re-registration events.
- In multi-operator sessions, the semantic layer is visible to participants by default while local close/hide behavior remains local.

### Carter Live Integration

This is not a fake-data pair. Run it only when the Carter navigation stack is already publishing live TF, camera, map, odometry, and Nav2 topics.

Live system must provide:
- `/tf`
- `/tf_static`
- `/shared_map`
- per-robot camera topics
- per-robot odometry, path, and Nav2 action topics

SDK registration:

```bash
python3 python/examples/legacy/sdk_hospital_carter_live_demo.py \
  --robot-names carter1,carter2,carter3 \
  --workspace-scale 0.1 \
  --tf-topic /tf \
  --tf-static-topic /tf_static \
  --shared-map-topic /shared_map \
  --body-mesh-mode runtime_high_mesh
```

Expected in HORUS MR:
- Live Carter robots register against the shared SLAM/Nav graph.
- Runtime camera, map, odometry, and navigation topics come from the live system, not fake publishers.

## Script Coverage

Every tracked legacy script is represented below. Scripts marked as primary workflows are intended to be run directly as part of a HORUS MR test. Utility scripts are support tools or diagnostics.

| Script | Role | Where It Is Used |
|---|---|---|
| `fake_tf_ops_suite.py` | Primary runtime | Recommended first workflow, multi-operator, semantic layer |
| `sdk_typical_ops_demo.py` | Primary SDK registration | Recommended first workflow, drone registration base |
| `fake_tf_publisher.py` | Primary runtime | Occupancy, manual 3D map, mesh, octomap base TF |
| `sdk_registration_demo.py` | Primary SDK registration | Occupancy, manual PointCloud2 map, manual mesh map |
| `fake_3d_map_publisher.py` | Primary/utility runtime | PointCloud2 map profiles and mesh source cloud |
| `pointcloud_to_voxel_mesh_marker.py` | Utility runtime | Mesh map conversion pipeline |
| `fake_octomap_publisher.py` | Primary/utility runtime | Octomap / octomap mesh pipeline |
| `sdk_multi_operator_host_demo.py` | Primary SDK registration | Host/join multi-operator validation |
| `fake_tf_single_flat.py` | Primary runtime | Flat single-robot ROS binding |
| `sdk_single_robot_flat_demo.py` | Primary SDK registration | Flat single-robot ROS binding |
| `fake_tf_drone_ops_suite.py` | Primary runtime | Drone takeoff, land, and 3D tasks |
| `fake_tf_legged_ops_suite.py` | Primary runtime | Legged Stand Up / Sit Down action flow |
| `sdk_legged_ops_demo.py` | Primary SDK registration | Legged Stand Up / Sit Down action flow |
| `fake_tf_robot_description_suite.py` | Primary runtime | Robot description, tutorial, managed 3D maps |
| `sdk_robot_description_demo.py` | Primary SDK registration | Robot description and managed 3D maps |
| `sdk_robot_description_tutorial_demo.py` | Primary SDK registration | Workspace tutorial opt-in |
| `fake_stereo_camera_multi.py` | Primary runtime | Stereo camera immersive teleop |
| `sdk_stereo_registration_demo.py` | Primary SDK registration | Stereo camera immersive teleop |
| `sdk_fake_semantic_perception_demo.py` | Primary SDK registration | Semantic perception layer |
| `sdk_hospital_carter_live_demo.py` | Live-system registration | Carter live integration |
| `robot_description_rviz_validation_demo.py` | Diagnostic utility | RViz-first URDF and TF validation |
| `e2e_registration_check.py` | Diagnostic utility | Minimal bridge/app registration smoke test |
| `fake_tf_teleop_common.py` | Internal helper | Shared by fake ops runtimes; do not launch directly |

## Validation Expectations

- Start with the recommended first workflow unless you need a specific subsystem.
- Keep Terminal A and Terminal B robot names/counts aligned.
- Use `python3 <script> --help` before adapting a legacy script for a new experiment.
- Treat this folder as stable legacy reference material, not the final shape of the next example generation.
- If HORUS MR shows robots but no data, inspect Terminal A first.
- If ROS topics exist but HORUS MR shows no robots or controls, inspect Terminal B and registration acknowledgements.
