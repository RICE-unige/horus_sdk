<p align="center">
  <img src="docs/horus_logo_black.svg#gh-light-mode-only" alt="HORUS logo" height="90">
  <img src="docs/horus_logo_white.svg#gh-dark-mode-only" alt="HORUS logo" height="90">
</p>

<p align="center"><em>Holistic Operational Reality for Unified Systems</em></p>

[![CI](https://github.com/RICE-unige/horus_sdk/actions/workflows/ci.yml/badge.svg)](https://github.com/RICE-unige/horus_sdk/actions/workflows/ci.yml)
[![Release](https://github.com/RICE-unige/horus_sdk/actions/workflows/release.yml/badge.svg)](https://github.com/RICE-unige/horus_sdk/actions/workflows/release.yml)
![Python](https://img.shields.io/badge/Python-3.10%2B-blue)
![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-22314E)
[![License](https://img.shields.io/badge/License-Apache--2.0-green.svg)](LICENSE)

> [!IMPORTANT]
> This repository owns the SDK/client orchestration layer of HORUS.
> The ROS 2 bridge runtime is maintained in [`horus_ros2`](https://github.com/RICE-unige/horus_ros2), and the MR release/distribution repository is [`horus`](https://github.com/RICE-unige/horus).

## Research Focus

HORUS investigates scalable mixed-reality **multi-robot management by an operator**, with support for **heterogeneous robot teams** (aerial, legged, and wheeled) across supervision and teleoperation workflows.

`horus_sdk` focuses on:
- robot/sensor registration models,
- SDK-side orchestration and connection lifecycle,
- dashboard and topic-state observability.

## Ownership Boundary

| Layer | Repository | Responsibility |
|---|---|---|
| SDK + registration payloads | `horus_sdk` | Robot config modeling, metadata, monitor UX |
| ROS 2 bridge runtime | `horus_ros2` | TCP/WebRTC bridge, ROS topic/service routing |
| MR app runtime | `horus` | Unity Quest scene, workspace flow, in-headset UX |

## Repository Map

| Path | Description |
|---|---|
| `python/horus/` | Main SDK implementation (bridge, sensors, dataviz, utils, plugins) |
| `python/examples/` | Operational demos (`sdk_registration_demo.py`, `sdk_typical_ops_demo.py`, `sdk_multi_operator_host_demo.py`, fake publishers, e2e checks) |
| `python/tests/` | SDK tests (serialization/state/dashboard behavior) |
| `cpp/` | C++ SDK parity track (paused) |
| `rust/` | Rust SDK parity track (paused) |

> [!NOTE]
> Python remains the most complete and actively used track for current experiments.

## Rust/C++ Parity Status

- Canonical parity contract and fixtures remain in:
  - `contracts/sdk_payload_contract.md`
  - `contracts/fixtures/*.json`
- C++ and Rust parity tracks are intentionally paused while Python SDK reaches full teleoperation feature depth.
- CI coverage for C++/Rust SDK test jobs is temporarily disabled in this phase and will be restored during parity bring-up.

## Requirements

- Python **3.10+**
- ROS 2 **Humble** or **Jazzy** environment available (`rclpy` + message packages)
- Running bridge from `horus_ros2` (`horus_unity_bridge`)

> [!WARNING]
> `horus_ros2/main` currently includes `GenericClient`-based bridge code that is not available in ROS 2 Humble headers.
> For this reason, `horus_sdk` CI on Humble validates `horus_interfaces` and `horus_backend` from `horus_ros2` and skips `horus_unity_bridge` packages.
> Full bridge build validation should be run on Jazzy or a Humble-compatible `horus_ros2` revision.

## One-Command Install

Install HORUS SDK + ROS2 backend with a standardized local layout:

```bash
curl -fsSL https://raw.githubusercontent.com/RICE-unige/horus_sdk/main/install.sh | bash
```

Default install root:
- `~/horus/sdk`
- `~/horus/ros2`
- `~/horus/bin`

Post-install helpers:
- `horus-status`
- `horus-start`
- `horus-stop`
- `horus-update`
- `horus-python`
- `horus-uninstall`

See `docs/INSTALLER.md` for flags and non-interactive usage.

The installer keeps your shell clean (no global venv auto-activation). Use:
- `python3 <~/horus/sdk/...>` directly (shim auto-routes HORUS scripts to HORUS env)
- `horus-python <script.py>` for SDK scripts
- `source ~/horus/bin/horus-env` for a session-scoped HORUS environment

Install (development):

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

## End-to-End Quick Start

> [!NOTE]
> Commands below use repository-clone paths (`~/horus_sdk`, `~/horus_ws/src/horus_ros2`).
> If you installed with `install.sh`, adapt paths to `~/horus/sdk` and `~/horus/ros2`.

### 1) Start bridge (`horus_ros2`)

Follow the setup/install instructions in the [`horus_ros2` README](https://github.com/RICE-unige/horus_ros2), then run:

```bash
cd ~/horus_ws
source /opt/ros/humble/setup.bash  # or jazzy
colcon build --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

### 2) Start unified fake ops runtime (TF + camera + teleop + tasks)

```bash
cd ~/horus_sdk
python3 python/examples/fake_tf_ops_suite.py --robot-count 10 --rate 30 --static-camera --publish-compressed-images --task-path-publish-rate 5 --publish-collision-risk --collision-threshold-m 1.2
```

### 3) Run typical SDK registration demo

```bash
cd ~/horus_sdk
python3 python/examples/sdk_typical_ops_demo.py --robot-count 10 --workspace-scale 0.1
```

### 4) Multi-operator host/join validation (SDK host-side)

```bash
cd ~/horus_sdk
python3 python/examples/sdk_multi_operator_host_demo.py --robot-count 10 --workspace-scale 0.1
```

Use this together with the MR runtime host/join workflow to validate dashboard operator visibility and joiner registry replay behavior.

### 5) Flat single-robot validation (no ROS namespace / no TF prefix)

Use this when a robot publishes root topics such as `/cmd_vel`, `/goal_pose`, `/waypoint_path`, `/odom`, and flat TF frames such as `base_link` / `camera_link`.

Fake runtime:

```bash
cd ~/horus_sdk
python3 python/examples/fake_tf_single_flat.py \
  --map-frame map \
  --base-frame base_link \
  --camera-frame camera_link \
  --rate 30
```

SDK registration demo:

```bash
cd ~/horus_sdk
python3 python/examples/sdk_single_robot_flat_demo.py \
  --robot-name robot \
  --base-frame base_link \
  --camera-frame camera_link \
  --camera-topic /camera/image_raw \
  --workspace-scale 0.1
```

Flat single-robot notes:
- This flow uses the SDK ROS-binding layer instead of requiring a robot namespace or TF prefix.
- HORUS logical robot identity stays separate from ROS naming; `robot_name` remains the display/runtime identity in MR.
- Registered default control topics resolve to flat roots in this mode: `/cmd_vel`, `/goal_pose`, `/goal_cancel`, `/goal_status`, `/waypoint_path`, `/waypoint_status`.
- The fake runtime covers teleop, go-to-point, waypoint, odometry, collision-risk, nav-path, camera, and flat TF validation in one workflow.
- Ambiguous multi-robot flat-root registration is intentionally rejected on the MR side instead of being guessed.

### 6) Hospital Carter live demo (shared nav graph + shared map + description body mesh)

Use this with the live Carter navigation stack when it already publishes the shared world view:
- `/tf`
- `/tf_static`
- `/shared_map`
- per-robot Nav2 actions and path topics under `/<robot>/...`

The current demo is nav-aware. It consumes the shared TF/map graph directly, registers one shared occupancy grid from `/shared_map`, bridges HORUS go-to/waypoint topics into Nav2 actions, and uses the description-driven robot body path instead of local prefabs.

```bash
cd ~/horus_sdk
python3 python/examples/sdk_hospital_carter_live_demo.py \
  --robot-names carter1,carter2,carter3 \
  --workspace-scale 0.1 \
  --tf-topic /tf \
  --tf-static-topic /tf_static \
  --shared-map-topic /shared_map \
  --body-mesh-mode runtime_high_mesh
```

Quick verification:

```bash
ros2 topic echo /carter1/front_stereo_camera/left/image_raw/compressed --once
ros2 topic echo /carter1/front_stereo_camera/left/camera_info --once
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
ros2 topic echo /shared_map --once
ros2 action list | grep navigate
```

Expected HORUS-facing task topics created by the demo:
- `/<robot>/goal_pose`
- `/<robot>/goal_cancel`
- `/<robot>/goal_status`
- `/<robot>/waypoint_path`
- `/<robot>/waypoint_status`

Expected live topics consumed by the demo:
- `/tf`
- `/tf_static`
- `/shared_map`
- `/carter1/front_stereo_camera/left/image_raw/compressed`
- `/carter2/front_stereo_camera/left/image_raw/compressed`
- `/carter3/front_stereo_camera/left/image_raw/compressed`
- `/carter1/front_stereo_camera/left/camera_info`
- `/carter2/front_stereo_camera/left/camera_info`
- `/carter3/front_stereo_camera/left/camera_info`
- `/carter1/chassis/odom`
- `/carter2/chassis/odom`
- `/carter3/chassis/odom`
- `/carter1/plan_smoothed`
- `/carter2/plan_smoothed`
- `/carter3/plan_smoothed`
- `/carter1/received_global_plan`
- `/carter2/received_global_plan`
- `/carter3/received_global_plan`
- `/<robot>/navigate_to_pose`
- `/<robot>/navigate_through_poses`

Body mesh modes:
- `--body-mesh-mode collision_only`: collision/joint overlays only
- `--body-mesh-mode preview_mesh`: Quest-friendly preview body
- `--body-mesh-mode runtime_high_mesh`: highest Quest-safe body quality
- `--body-mesh-mode max_quality_mesh`: compatibility alias for `runtime_high_mesh`

Navigation DataViz quick notes:
- `sdk_typical_ops_demo.py` registers nav path + motion safety DataViz metadata (velocity/odometry trail/collision risk) by default.
- `GoalMarkerData` and `WayPointQueue` visibility toggles are runtime-controlled in MR during active go-to/waypoint tasks.
- Robot-description DataViz channels are exposed separately in MR (`CollisionMeshData`, `JointAxesData`) and use manifest support flags.

Robot description demo quick start (collision + joints + visual meshes):

```bash
cd ~/horus_sdk
python3 python/examples/tools/fetch_robot_description_assets.py
python3 python/examples/fake_tf_robot_description_suite.py
python3 python/examples/sdk_robot_description_demo.py --workspace-scale 0.1 --collision-opaque
```

Notes:
- `fake_tf_robot_description_suite.py` runs with fixed TF scale `1.0`; MR `workspace_scale` remains the only global shrink.
- Local demo URDF assets are stored under `python/examples/.local_assets/robot_descriptions/` and are gitignored.
- Current fetched assets include `go1.urdf`, `jackal.urdf(.xacro)`, `anymal_c.urdf`, and `h1.urdf`.
- Use `--wheeled-urdf` / `--legged-urdf` to override the resolved Jackal/Go1 paths.
- Use `--robot-profile real_models` with `--anymal-urdf` / `--h1-urdf` (plus existing `--wheeled-urdf` / `--legged-urdf`) for real-model profile overrides.
- Collision-body transparency is SDK-driven for V1 (`is_transparent` in manifest). Demo defaults to opaque; use `--collision-transparent` to switch.
- Visual body meshes are now delivered through the same description transport with body mesh modes:
  - `collision_only`
  - `preview_mesh`
  - `runtime_high_mesh`

Real-model profile demo (Anymal C + Jackal + Go1 + Unitree H1, 3D map off by default):

```bash
cd ~/horus_sdk
python3 python/examples/tools/fetch_robot_description_assets.py --force
python3 python/examples/fake_tf_robot_description_suite.py --robot-profile real_models --map-3d-mode off
python3 python/examples/sdk_robot_description_demo.py --robot-profile real_models --workspace-scale 0.1 --collision-opaque --map-3d-mode off
```

Enable 3D map in mesh mode (recommended for Quest 3):

```bash
cd ~/horus_sdk
python3 python/examples/fake_tf_robot_description_suite.py --robot-profile real_models --map-3d-mode mesh
python3 python/examples/sdk_robot_description_demo.py --robot-profile real_models --workspace-scale 0.1 --collision-opaque --map-3d-mode mesh
```

High-detail mesh stress test (snapshot-first, no periodic keepalive):

```bash
cd ~/horus_sdk
python3 python/examples/fake_tf_robot_description_suite.py --robot-profile real_models --map-3d-mode mesh --map-3d-detailed --map-3d-mesh-update-policy snapshot --map-3d-mesh-republish-interval 0
python3 python/examples/sdk_robot_description_demo.py --robot-profile real_models --workspace-scale 0.1 --collision-opaque --map-3d-mode mesh
```

Enable 3D map in octomap mode (Quest-first hybrid source):

```bash
cd ~/horus_sdk
python3 python/examples/fake_tf_robot_description_suite.py --robot-profile real_models --map-3d-mode octomap
python3 python/examples/sdk_robot_description_demo.py --robot-profile real_models --workspace-scale 0.1 --collision-opaque --map-3d-mode octomap
```

High-detail octomap stress test (100k voxel/triangle caps):

```bash
cd ~/horus_sdk
python3 python/examples/fake_tf_robot_description_suite.py --robot-profile real_models --map-3d-mode octomap --map-3d-detailed --map-3d-octomap-max-voxels 100000 --map-3d-octomap-max-triangles 100000 --map-3d-octomap-republish-interval 0
python3 python/examples/sdk_robot_description_demo.py --robot-profile real_models --workspace-scale 0.1 --collision-opaque --map-3d-mode octomap
```

3D map CLI notes:
- `--map-3d-mode {off,pointcloud,mesh,octomap}` is the primary switch for this real-model demo pair.
- Topic/frame overrides are available via `--map-3d-topic`, `--map-3d-frame`, `--map-3d-mesh-topic`, and `--map-3d-mesh-frame`.
- Marker-only rollback is active for mesh transport in this branch:
  - `--map-3d-mesh-transport`, `--map-3d-mesh-array-topic`, and `--map-3d-mesh-chunk-max-triangles` are accepted as compatibility aliases but ignored with warning logs.
  - Effective mesh topic is always marker-based (`/map_3d_mesh`).
- Octomap controls are exposed through: `--map-3d-octomap-topic`, `--map-3d-octomap-mesh-topic`, `--map-3d-octomap-frame`, `--map-3d-octomap-max-voxels`, `--map-3d-octomap-max-triangles`, and `--map-3d-octomap-republish-interval`.
- Legacy aliases `--with-3d-map` and `--with-3d-mesh` are retained for compatibility and print deprecation warnings.
- In mesh mode, the fake runtime auto-starts both the fake pointcloud publisher and pointcloud-to-mesh converter pipeline.
- In octomap mode, the fake runtime auto-starts a Quest-first octomap publisher that always outputs mesh markers and optionally emits native `octomap_msgs/Octomap` metadata when dependencies are available.
- Mesh converter controls are exposed through: `--map-3d-mesh-voxel-size`, `--map-3d-mesh-max-voxels`, `--map-3d-mesh-max-triangles`, `--map-3d-mesh-update-policy {snapshot,periodic,continuous}`, and `--map-3d-mesh-republish-interval`.
- Real-model mesh flow is currently marker-only (`/map_3d_mesh`) for runtime stability.
- Quest-focused default is snapshot-first mesh conversion (`snapshot` policy with no periodic republish) to avoid repeated heavy marker rebuilds.
- Replay-time bounded republish bursts are enabled for map publishers/converters so late joiners reliably receive cached map data during SDK replay windows.

> [!WARNING]
> Pointcloud mode is retained for diagnostics but is not recommended for regular Meta Quest 3 operation. Use mesh mode for stable runtime behavior.

RViz-first TF validation (robot_state_publisher from real URDF):

```bash
cd ~/horus_sdk
python3 python/examples/tools/fetch_robot_description_assets.py
python3 python/examples/robot_description_rviz_validation_demo.py
```

This publishes:
- `/<robot>/joint_states` (zeroed joint states),
- `/<robot>/robot_description` (URDF text),
- TF tree from `robot_state_publisher` using URDF joints/collisions (with `frame_prefix=<robot>/`),
- optional `map -> <robot>/<base_frame>` placement transforms.

## Camera Registration Model

Camera payloads support legacy and profile-based transport fields:
- `streaming_type` (legacy fallback)
- `minimap_streaming_type`
- `teleop_streaming_type`
- `startup_mode`
- `is_stereo`, `stereo_layout`, `right_topic` (base stereo capability/source descriptors)
- `minimap_topic`, `minimap_image_type`, `minimap_max_fps`
- `teleop_topic`, `teleop_image_type`
- `teleop_stereo_layout`, `teleop_right_topic` (for dual-topic stereo)

Typical policy:
- MiniMap -> ROS
- Teleop -> WebRTC

Dual-stream policy used by the stereo demos:
- MiniMap topic is mono and capped at 30 FPS from source.
- Teleop topic is high-rate (SBS via WebRTC, dual-topic via ROS).

> [!TIP]
> Keep legacy `streaming_type` populated for compatibility with older MR clients while using profile fields for new behavior.

## Teleoperation SDK Baseline

Current SDK-side teleoperation support includes:
- `control.teleop` payload serialization (`command_topic`, `raw_input_topic`, `head_pose_topic`, profile/response/deadman/axes/discrete settings),
- teleop fake TF test scenarios (`fake_tf_teleop_single.py`, `fake_tf_teleop_multi.py`) for command-flow validation,
- dashboard control-topic visibility for `cmd_vel`, `joy`, and `head_pose`,
- runtime transport observability via `/horus/teleop/runtime_state` (ROS/WebRTC active path highlight).

## Robot Task Contract Baseline

Current SDK-side robot task support includes:
- `control.tasks.go_to_point` payload serialization (`goal_topic`, `cancel_topic`, `status_topic`, `frame_id`, tolerances, `min_altitude_m`, `max_altitude_m`),
- `control.tasks.waypoint` payload serialization (`path_topic`, `status_topic`, `frame_id`, tolerances),
- metadata override source `robot.metadata["task_config"]["go_to_point"]`,
- metadata override source `robot.metadata["task_config"]["waypoint"]`,
- go-to-point cancel contract on `/<robot>/goal_cancel` using `std_msgs/String` payload `"cancel"`,
- fake goal-navigation simulator `python/examples/fake_tf_go_to_point.py` for send/cancel/reached cycle tests,
- fake waypoint simulator `python/examples/fake_tf_waypoint.py` for ordered path/status cycle tests,
- fake aerial ops simulator `python/examples/fake_tf_drone_ops_suite.py` for drone takeoff/land + 3D go-to/waypoint validation.

## Global Visualization and Workspace Config Model

Registration payloads can now include:
- `global_visualizations` (deduped, robot-independent visual layers such as occupancy grid),
- `workspace_config.position_scale` (global scale hint consumed by MR runtime).

This enables 2D occupancy-map wiring without duplicating map config in each robot-scoped visualization block.

## Dashboard Semantics

Topic dashboard rows are grouped as:
- Core topics: registration/ack/heartbeat
- Data topics: TF/camera/robot streams

Link/Data interpretation:

| Field | Meaning |
|---|---|
| `Link` | Whether bridge-side subscription/publisher linkage is established |
| `Data=ACTIVE` | Link exists and data flow is observed |
| `Data=STALE` | Link exists but expected data publisher is silent |
| `Data=IDLE` | App disconnected or no active bridge linkage |

> [!WARNING]
> Topic truth depends on both ROS graph state and app connection phase. Validate with full workflow (connect -> workspace accept -> register).

Camera dashboard notes:
- Camera transport badge is resolved from active runtime mode (`ROS` or `WEBRTC`) with profile fallback.
- For camera rows on active `WEBRTC`, `Link` is app-session/runtime-state based (not ROS-subscriber based).
- For camera rows on active `ROS`, `Link` follows backend subscriber presence.
- Runtime transport overrides have expiry/refresh handling to reduce stale mode indicators.

## Multi-Operator SDK Baseline

Current SDK support for the multi-operator runtime includes:
- dashboard `Operators` summary and presence-stage visibility for shared-host, shared-join, and private-workspace progression,
- dedicated host-side test flow via `python/examples/sdk_multi_operator_host_demo.py`,
- SDK registry replay protocol publishing for joiner baseline reconstruction:
  - `/horus/multi_operator/sdk_registration_replay_request`
  - `/horus/multi_operator/sdk_registry_replay_begin`
  - `/horus/multi_operator/sdk_registry_replay_item`
  - `/horus/multi_operator/sdk_registry_replay_end`
- private-operator presence reporting and direct collaborative registration/replay support without shared workspace sync,
- bridge auto-start strategy hardening that prefers the current shell ROS 2 workspace before helper fallback (with mismatch diagnostics).

> [!NOTE]
> The SDK is not the source of truth for host-authored workspace geometry. Workspace baseline still comes from the MR runtime (`horus`).

## Practical Validation Workflows

### All-in-one 10-robot ops test (no occupancy map)

```bash
# Terminal A: unified fake runtime (robots stay static until teleop/task commands)
python3 python/examples/fake_tf_ops_suite.py --robot-count 10 --rate 30 --static-camera --publish-compressed-images

# Terminal B: typical SDK registration flow (camera + teleop + go-to-point + waypoint)
python3 python/examples/sdk_typical_ops_demo.py --robot-count 10 --workspace-scale 0.1
```

Notes:
- this scenario intentionally does not publish occupancy grid,
- robots move only from `/<robot>/cmd_vel`, `/<robot>/goal_pose`, or `/<robot>/waypoint_path`,
- active teleop input cancels active go-to-point/waypoint motion and requires task resend.

Command examples while the suite is running:

```bash
# Teleop (atlas): move forward while turning
ros2 topic pub -r 10 /atlas/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.35}, angular: {z: 0.30}}"

# Go-to-point (nova)
ros2 topic pub --once /nova/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 1.5, y: -0.8, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

# Waypoint path (orion)
ros2 topic pub --once /orion/waypoint_path nav_msgs/msg/Path "{header: {frame_id: map}, poses: [{header: {frame_id: map}, pose: {position: {x: 0.8, y: 0.8, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: 1.6, y: 0.2, z: 0.0}, orientation: {w: 1.0}}}]}"
```

### Registration smoke test

```bash
python3 python/examples/e2e_registration_check.py
```

### Multi-robot camera stress test

```bash
python3 python/examples/fake_tf_publisher.py --robot-count 10
python3 python/examples/sdk_registration_demo.py --robot-count 10
```

### Occupancy-map integration test

```bash
python3 python/examples/fake_tf_publisher.py --robot-count 6 --publish-occupancy-grid --occupancy-rate 1.0
python3 python/examples/sdk_registration_demo.py --robot-count 6 --with-occupancy-grid --workspace-scale 0.1
```

### Teleop command-flow fake TF tests

```bash
# Single robot: remains static until /test_bot/cmd_vel receives Twist
python3 python/examples/fake_tf_teleop_single.py --robot-name test_bot --static-camera --publish-compressed-images

# Multi-robot: all stay static until per-robot cmd_vel commands arrive
python3 python/examples/fake_tf_teleop_multi.py --robot-count 4 --robot-name test_bot --static-camera --publish-compressed-images
```

### Go-to-point task fake TF test

```bash
# Robots stay idle until /<robot>/goal_pose is published.
# Cancel with:  ros2 topic pub --once /<robot>/goal_cancel std_msgs/msg/String '{data: "cancel"}'
# Goal completion/cancel status is published on /<robot>/goal_status.
python3 python/examples/fake_tf_go_to_point.py --robot-count 3 --robot-name test_bot --static-camera
```

### Waypoint task fake TF test

```bash
# Send nav_msgs/Path to /<robot>/waypoint_path.
# Execution progress/status is emitted on /<robot>/waypoint_status.
python3 python/examples/fake_tf_waypoint.py --robot-count 3 --robot-name test_bot --static-camera
```

### Drone task flow test (takeoff/land + 3D go-to/waypoint)

```bash
# Terminal A: drone fake runtime (3 robots, extended altitude envelope)
python3 python/examples/fake_tf_drone_ops_suite.py \
  --robot-count 3 \
  --robot-names drone_1,drone_2,drone_3 \
  --min-altitude 0.0 \
  --max-altitude 25.0 \
  --takeoff-altitude 1.2

# Terminal B: SDK registration metadata with aerial profile + matching altitude bounds
python3 python/examples/sdk_typical_ops_demo.py \
  --robot-names drone_1,drone_2,drone_3 \
  --teleop-profile aerial \
  --go-to-min-altitude 0.0 \
  --go-to-max-altitude 25.0 \
  --workspace-scale 0.1
```

Expected MR behavior for this scenario:
- Drone `TakeOffButton` / `LandButton` commands are active.
- Drone go-to, waypoint, and draw-path use 3D authoring with altitude gating.
- Multi-robot go-to-point V1 can dispatch to all active aerial robots from one authored anchor.

### Stereo camera immersive test (dual-stream minimap + teleop)

```bash
# Terminal A: fake TF + dual-stream camera generator
python3 python/examples/fake_stereo_camera_multi.py \
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

# Terminal B: register matching mono/stereo camera metadata
python3 python/examples/sdk_stereo_registration_demo.py \
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

Dual-topic stereo (left + right topics) is also supported by using `--stereo-input-mode dual_topic` in both commands.

- Stereo depth rendering is applied in immersive teleop view.
- MiniMap camera view stays mono by design.
- `sbs` mode uses immersive teleop WebRTC transport.
- `dual_topic` mode keeps immersive teleop on ROS transport (left/right topics).
- Source topics are split by mode: `/camera/minimap/*` (mono) and `/camera/teleop/*` (teleop stream).
- If `Stereo` is missing in CameraData dropdown, verify that robot camera registration sets `is_stereo=true`.
- Stopping the registration demo with `Ctrl+C` in keep-alive mode is treated as user cancellation, not a registration failure.

Troubleshooting:
- If immersive SBS appears frozen, look for `WebRTC frame stall detected` in Unity logs; the client will auto-restart the session.
- If dashboard camera transport/link status looks stale, wait one dashboard refresh cycle or confirm `/horus/teleop_runtime_state` is still publishing active updates.

## Known Constraints

- Multi-robot high-rate camera streams can saturate bridge/headset resources before network limits.
- WebRTC and ROS transport trade-offs should be selected per use-case (awareness vs low-latency control).
- Startup behavior is intentionally conservative in MR (MiniMap-first policy).

## MR Integration Status (from `horus`)

Current Unity MR baseline relevant to SDK integration:
- Workspace-gated registration is active and expected before full robot activation.
- Camera runtime follows MiniMap/Teleop transport profiles (MiniMap-first startup policy).
- Occupancy map is consumed from global visualization payload and remains hidden until workspace accept.
- Registration-path batching/deduping exists on MR side to reduce workspace-accept stalls.
- Multi-operator host/join workspace baseline is active in MR (`Join Workspace` gating after alignment).
- Joiner robot baseline reconstruction uses SDK replay begin/item/end topics (workspace baseline remains MR-owned).
- Per-robot lease-based multi-operator control foundation is integrated on the MR/bridge path (SDK currently provides observability and replay support, not arbitration).

> [!NOTE]
> When validating SDK changes, always test against the full MR flow:
> connect -> draw workspace -> accept -> register -> observe dashboard/topic transitions.

## Cross-Repo Alignment Priorities

The MR roadmap introduces upcoming requirements that depend on SDK payload and orchestration maturity:
- 2D/3D tasking workflows (go-to-point, waypoints, path drawing, go-to-label, multi-robot go-to-point, follow-leader teleop).
- Expanded sensor visualization requirements (battery, velocity, LaserScan, PointCloud).
- Robot description evolution from runtime visual-mesh delivery toward known-robot local hero meshes and unknown-robot ingestion policy.
- Session recording + after-action replay data contracts.
- Resource-aware streaming policy signals (quality tiers, priority, stream caps).
- Persistent mission objects (pins/annotations/evidence/task assignment).
- Safety and semantic perception signals for teleop and supervision.
- Multi-operator and copilot-oriented orchestration scenarios.

SDK roadmap and examples should evolve to provide the metadata, presets, and validation scripts needed for these MR milestones.

## Roadmap

> [!NOTE]
> SDK roadmap items are scoped to payload schemas, orchestration policies, and validation workflows that unlock MR/runtime features; current baseline includes description-driven visual mesh bodies with quality modes, Carter nav shared-map integration, private-operator presence tracking, marker-only mesh stability flow, octomap mode integration, and flat single-robot ROS-binding compatibility.

| Track | Status | SDK Baseline | Next Milestone |
|---|---|---|---|
| Robot Manager Contracts | :large_orange_diamond: In progress | `robot_manager_config` payload support and demo defaults are in place. | Extend schema for status/task bindings and section-level runtime options. |
| Teleoperation Contracts | :large_orange_diamond: In progress | `control.teleop` contract, teleop fake TF scenarios, control-topic dashboard rows, and runtime transport-state signaling are integrated. Follow-leader teleop V1 in MR reuses this contract with no schema change. | Add subset/handoff metadata and manipulator-capability descriptors. |
| ROS Binding Compatibility | :large_orange_diamond: In progress | Optional ROS-binding metadata is integrated for prefixed and flat single-robot workflows, with resolved registration payloads, dashboard topic ownership overrides for root topics, and explicit flat demo scripts. | Extend example coverage beyond flat single-robot ground flow and document backend metadata conventions more formally. |
| 2D Map Contracts | :white_check_mark: Foundation complete | Global occupancy-grid visualization payload and workspace scale forwarding are integrated. | Add richer map overlay contracts (goals, nav path layers, region semantics). |
| 3D Map Contracts | :large_orange_diamond: In progress | Shared `--map-3d-mode` workflow is integrated in the real-model demo pair with pointcloud, mesh, and octomap global visualization payload wiring, greedy voxel meshing, snapshot-first Quest defaults, marker-only mesh transport coercion for runtime stability, and replay-time bounded republish bursts for deterministic late-join map delivery. | Expand native octomap interoperability beyond metadata-only publish path and add renderer policy hints (decimation/quality tiers) for runtime tuning. |
| Robot Description Contracts (V1) | :large_orange_diamond: In progress | URDF resolver + compiled collision/joint/visual-mesh schema, manifest hashing, chunked request/reply transport, body mesh modes (`collision_only`, `preview_mesh`, `runtime_high_mesh`), source-color recovery, and demo/validation scripts are integrated. | Add known-robot local hero-mesh fallback policy and extend validation to mixed mesh+collision deployments and more source mesh formats. |
| Tasking (2D/3D) | :large_orange_diamond: In progress | Typed Go-To/Waypoint schemas are integrated with altitude-bounds support for aerial robots, plus fake TF validation scripts for ground and drone ops suites. | Add explicit payload contracts for Draw Path, Go-To Label, and subset-based Multi-Robot Go-To policies. |
| Session Recording | :white_circle: Planned | - | Add mission/session record contract (events, commands, timeline references). |
| After-Action Replay | :white_circle: Planned | - | Add replay manifest schema and deterministic timeline reconstruction inputs. |
| Adaptive Streaming Policies | :white_circle: Planned | Static transport profiles exist, but no load-aware policy orchestration. | Add policy payload for per-robot FPS/resolution tiers, priority (teleop-first), transport fallback, and dynamic max-stream guardrails. |
| Operator Safety Contracts | :white_circle: Planned | - | Add deadman/e-stop/geofence/risk-confirm metadata channels with policy defaults. |
| Persistent Mission Objects | :white_circle: Planned | - | Add shared mission-object schema (pins, notes, attachments, assignees, lifecycle). |
| Manipulator Teleoperation | :white_circle: Planned | - | Add manipulator capability descriptors (joint/EEF/gripper limits, home poses, safety envelopes). |
| Mobile Manipulator Coordination | :white_circle: Planned | Base and manipulator are modeled independently today. | Add combined base+arm action primitives and coordination metadata. |
| Semantic Perception Layers | :white_circle: Planned | - | Add semantic layer payloads with confidence, uncertainty, and spatial anchoring metadata. |
| Multi-Operator Orchestration | :large_orange_diamond: In progress | SDK dashboard presence visibility now tracks shared-host, shared-join, and private-workspace operators; multi-operator host demo workflow, bridge auto-start hardening, SDK registry replay protocol publishing, replay-triggered map republish burst hooks for joiner map reliability, and direct private-operator replay support are integrated. | Add richer operator identity/lease observability summaries, explicit ownership metadata schemas, and stronger rejoin/replay validation suites. |
| AI Copilot Orchestration | :white_circle: Planned | - | Define copilot action-scoping contracts, approval/guardrail metadata, and operator-visible intervention traces. |

## 📖 Citation

If you use HORUS or ideas from this work in your research, please cite:

O. S. Adekoya, A. Sgorbissa, C. T. Recchiuto. HORUS: A Mixed Reality Interface for Managing Teams of Mobile Robots. arXiv preprint arXiv:2506.02622, 2025.

```bibtex
@misc{adekoya2025horus,
  title         = {HORUS: A Mixed Reality Interface for Managing Teams of Mobile Robots},
  author        = {Adekoya, Omotoye Shamsudeen and Sgorbissa, Antonio and Recchiuto, Carmine Tommaso},
  year          = {2025},
  eprint        = {2506.02622},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO},
  url           = {https://github.com/RICE-unige/horus},
  pdf           = {https://arxiv.org/abs/2506.02622},
  note          = {arXiv preprint arXiv:2506.02622}
}
```

## 📬 Contact

For questions or support:

- Omotoye Shamsudeen Adekoya  
- Email: `omotoye.adekoya@edu.unige.it`

## 💡 Acknowledgments

This project is part of PhD research at the University of Genoa, under the supervision of:

- Prof. Carmine Recchiuto
- Prof. Antonio Sgorbissa

Developed by **RICE Lab**, University of Genoa.

## Contributing

1. Reproduce behavior with explicit command lines.
2. Include before/after impact on registration and dashboard states.
3. Add or update focused tests when changing serialization/state logic.

## License

Apache-2.0. See [`LICENSE`](LICENSE).
