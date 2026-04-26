<p align="center">
  <img src="static/img/horus_logo_black.svg#gh-light-mode-only" alt="HORUS logo" height="90">
  <img src="static/img/horus_logo_white.svg#gh-dark-mode-only" alt="HORUS logo" height="90">
</p>

<p align="center"><em>Holistic Operational Reality for Unified Systems</em></p>

[![CI](https://github.com/RICE-unige/horus_sdk/actions/workflows/ci.yml/badge.svg)](https://github.com/RICE-unige/horus_sdk/actions/workflows/ci.yml)
[![Release](https://github.com/RICE-unige/horus_sdk/actions/workflows/release.yml/badge.svg)](https://github.com/RICE-unige/horus_sdk/actions/workflows/release.yml)
[![Docs CI](https://github.com/RICE-unige/horus_sdk/actions/workflows/docs-ci.yml/badge.svg)](https://github.com/RICE-unige/horus_sdk/actions/workflows/docs-ci.yml)
![Python](https://img.shields.io/badge/Python-3.10%2B-blue)
![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-22314E)
[![License](https://img.shields.io/badge/License-Apache--2.0-green.svg)](LICENSE)

## Documentation

- Site: <https://rice-unige.github.io/horus_sdk/>
- Source branch: `main`
- Staging branch for large docs refactors: `docs/site-v1`
- Local commands:

```bash
npm ci
npm run docs:check
npm run docs:build
npm run docs:start
```

> [!IMPORTANT]
> This repository owns the SDK/client orchestration layer of HORUS.
> The ROS 2 bridge runtime is maintained in [`horus_ros2`](https://github.com/RICE-unige/horus_ros2), and the MR release/distribution repository is [`horus`](https://github.com/RICE-unige/horus).
> Adjacent future services for copilot orchestration and scene understanding are now being developed separately in [`compass`](https://github.com/Omotoye/compass) and [`lenses`](https://github.com/Omotoye/lenses).

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
| Copilot/orchestration service | `compass` | Future conversational planning, approvals, and backend execution orchestration |
| Scene understanding service | `lenses` | Future perception, scene graph generation, and semantic context |

## Repository Map

| Path | Description |
|---|---|
| `python/horus/` | Main SDK implementation (bridge, sensors, dataviz, utils, plugins) |
| `python/examples/` | Curated, no-CLI SDK registration examples for normal user workflows |
| `python/examples/legacy/` | Full legacy fake-runtime and SDK demo catalog preserved for validation and advanced variants |
| `python/examples/tools/` | Support utilities for legacy demos (for example asset fetchers) |
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
- ROS packages are installed through the ROS/apt environment, not as portable PyPI dependencies.
- Installed/buildable `horus_ros2` bridge runtime (`horus_unity_bridge`); SDK registration auto-starts it when needed

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

See `docs/getting-started/installer.md` for flags and non-interactive usage.

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
> Commands below use repository-clone paths (`~/horus_sdk`, `~/horus_ws`).
> If you installed with `install.sh`, adapt paths to `~/horus/sdk` and `~/horus/ros2`.

### 1) Prepare the SDK shell

The SDK registration client manages the HORUS bridge for the normal curated examples. When registration starts, it checks port `10000`; if the bridge is not already running, it auto-starts `horus_unity_bridge` from the current ROS shell first and then falls back to the installer `horus-start` helper when available.

Manual bridge launch is therefore not part of the normal quick start. Keep manual `ros2 launch horus_unity_bridge unity_bridge.launch.py` usage for legacy validation, custom bridge debugging, or cases where `HORUS_SDK_BRIDGE_AUTOSTART_MODE=off`.

When running from a source checkout, keep your ROS environment and SDK path active:

```bash
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash  # or humble when using a Humble-compatible bridge revision
source ~/horus_ws/install/setup.bash  # makes horus_unity_bridge discoverable for auto-start
export PYTHONPATH=python:$PYTHONPATH
```

### 2) Use the curated SDK examples

The root `python/examples/` scripts are intentionally small: no CLI flags, no option matrix, just realistic SDK registrations that match one useful HORUS MR workflow.

Each workflow uses one fake/runtime terminal and one SDK registration terminal unless noted otherwise. The SDK registration terminal is also responsible for bridge auto-start.

### Ground robot ops fleet

Use this first. It covers the normal ground-robot workflow: Robot Manager, camera, minimap/immersive teleop, go-to point, waypoint, paths, odometry, and collision-risk overlays.

```bash
# Terminal A: fake robot data and task subscribers
python3 python/examples/legacy/fake_tf_ops_suite.py

# Terminal B: SDK registration
python3 python/examples/ops_registration.py
```

### Flat single-robot ROS binding

Use this when a robot publishes flat ROS topics such as `/cmd_vel`, `/goal_pose`, `/global_path`, `/local_path`, and `/camera/image_raw` instead of namespaced topics. The fake runtime publishes a front raw camera stream and continuously refreshes nav paths while goals or waypoints are active.

```bash
# Terminal A
python3 python/examples/legacy/fake_tf_single_flat.py

# Terminal B
python3 python/examples/flat_robot_registration.py
```

### Drone registration

Use this for aerial robots. HORUS MR keeps the drone `Take Off` and `Land` controls enabled for this robot type.

```bash
# Terminal A
python3 python/examples/legacy/fake_tf_drone_ops_suite.py

# Terminal B
python3 python/examples/drone_registration.py
```

### Legged robot registration

Use this for legged robots. HORUS MR presents the action controls as `Stand Up` and `Sit Down` instead of drone takeoff/land controls.

```bash
# Terminal A
python3 python/examples/legacy/fake_tf_legged_ops_suite.py

# Terminal B
python3 python/examples/legged_registration.py
```

### Live Carter registration

Use this for the live NVIDIA Carter multi-robot setup. The curated example registers three Carter robots, the shared occupancy map, front compressed cameras, 2D lidar, teleop, Robot Manager task topics, odometry trails, and RViz-relayed global/local plans.

Expected live topics include `/tf`, `/tf_static`, `/shared_map`, `/<robot>/cmd_vel`, `/<robot>/chassis/odom`, `/<robot>/front_2d_lidar/scan`, `/<robot>/front_stereo_camera/left/image_raw/compressed`, `/rviz/<robot>/plan`, and `/rviz/<robot>/local_plan` for `carter1`, `carter2`, and `carter3`.

```bash
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python:$PYTHONPATH

python3 python/examples/carter_registration.py
```

For advanced Carter validation with topic probing, Nav2 action bridging, and AprilTag semantic overlays, use the legacy live Carter demo documented in [python/examples/legacy/README.md](python/examples/legacy/README.md).

### Live Unitree Go1 registration

Use this for the real Unitree Go1 ROS graph. The registration includes TF, the front compressed camera, LaserScan, collision alert DataViz, Robot Manager, legged teleop on `/unitree_go1/cmd_vel`, and the real Go1 URDF visual mesh from `/home/omotoye/Unitree_ros2_to_real/ros2_ws/src/go1_description`.

The HORUS MR legged action buttons publish `/unitree_go1/stand_up` and `/unitree_go1/sit_down`. The support relay below maps those to the Unitree SDK high-mode service `/unitree_go1/legged_sdk/set_high_mode` with `mode=10` for stand and `mode=20` for sit. The same relay also converts `/unitree_go1/scan` into `/unitree_go1/collision_risk` so the HORUS collision alert layer can render live obstacle risk.

```bash
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python:$PYTHONPATH

# Required robot-description source for visual mesh registration:
# /home/omotoye/Unitree_ros2_to_real/ros2_ws/src/go1_description/urdf/go1.urdf

# Terminal A: relay HORUS legged buttons to Unitree SetHighMode service
python3 python/examples/tools/unitree_go1_high_mode_relay.py

# Terminal B: SDK registration
python3 python/examples/unitree_go1_registration.py
```

### Robot-description registration

Use this when you want HORUS MR to load URDF-backed robot bodies instead of simple bounding boxes.

```bash
# One-time asset fetch
python3 python/examples/tools/fetch_robot_description_assets.py

# Terminal A
python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models

# Terminal B
python3 python/examples/robot_description_registration.py
```

### Stereo camera registration

Use this for the dual camera policy used by teleop: mono minimap stream plus side-by-side stereo teleop stream.

```bash
# Terminal A
python3 python/examples/legacy/fake_stereo_camera_multi.py

# Terminal B
python3 python/examples/stereo_registration.py
```

### Occupancy Grid Map

Use this to showcase a 2D occupancy-grid map layer. This workflow is paired with the real-model robot-description fake runtime so the map appears with realistic robot bodies, TF, cameras, and task data.

```bash
# One-time asset fetch
python3 python/examples/tools/fetch_robot_description_assets.py

# Terminal A: real-model robots plus occupancy grid
python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models --publish-occupancy-grid

# Terminal B: SDK registration
python3 python/examples/occupancy_map_registration.py
```

### PointCloud Map

Use this to showcase a global PointCloud2 map layer. This workflow is paired with the real-model robot-description fake runtime so the map appears with realistic robot bodies, TF, cameras, and task data.

```bash
# Terminal A: real-model robots plus PointCloud2 map
python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models --map-3d-mode pointcloud --map-3d-profile realistic

# Terminal B: SDK registration
python3 python/examples/pointcloud_map_registration.py
```

### Dense Mesh Map

Use this to showcase the highest-detail synthetic mesh map currently exposed by the examples. This workflow is paired with the real-model robot-description fake runtime, but the focus is the dense mesh map layer. It is intentionally dense and should be used for visual quality/performance validation, not as the first Quest stress test.

```bash
# Terminal A: real-model robots plus ultra-dense mesh map
python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models --map-3d-mode mesh --map-3d-profile realistic --map-3d-mesh-voxel-size 0.07 --map-3d-mesh-max-voxels 220000 --map-3d-mesh-max-triangles 220000 --map-3d-mesh-update-policy snapshot

# Terminal B: SDK registration
python3 python/examples/mesh_map_registration.py
```

### Octomap Map

Use this to showcase the octomap-style global map layer. This workflow is paired with the real-model robot-description fake runtime so the map appears with realistic robot bodies, TF, cameras, and task data.

```bash
# Terminal A: real-model robots plus octomap mesh layer
python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models --map-3d-mode octomap --map-3d-profile realistic

# Terminal B: SDK registration
python3 python/examples/octomap_registration.py
```

### Semantic perception registration

Use this to register global semantic boxes, such as detected people or equipment, as default-visible DataViz layers.

```bash
# Terminal A
python3 python/examples/legacy/fake_tf_ops_suite.py --robot-count 4

# Terminal B
python3 python/examples/semantic_perception_registration.py
```

### Multi-operator note

The curated registration examples all use `keep_alive=True`, so they remain suitable for host/join and private multi-operator validation. Start any example above before or during the shared HORUS MR session; joiners can receive the SDK registration replay from the active host-side SDK process.

### Legacy catalog

The full legacy command catalog remains in [python/examples/legacy/README.md](python/examples/legacy/README.md). Use it for stress tests, advanced flag variants, Carter/live integrations, tutorial-specific flows, and lower-level validation.

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
- unified fake ops runtime (`python/examples/legacy/fake_tf_ops_suite.py`) for command-flow validation across teleop, go-to-point, waypoint, paths, odometry, and collision-risk data,
- dashboard control-topic visibility for `cmd_vel`, `joy`, and `head_pose`,
- runtime transport observability via `/horus/teleop/runtime_state` (ROS/WebRTC active path highlight).

## Robot Task Contract Baseline

Current SDK-side robot task support includes:
- `control.tasks.go_to_point` payload serialization (`goal_topic`, `cancel_topic`, `status_topic`, `frame_id`, tolerances, `min_altitude_m`, `max_altitude_m`),
- `control.tasks.waypoint` payload serialization (`path_topic`, `status_topic`, `frame_id`, tolerances),
- metadata override source `robot.metadata["task_config"]["go_to_point"]`,
- metadata override source `robot.metadata["task_config"]["waypoint"]`,
- go-to-point cancel contract on `/<robot>/goal_cancel` using `std_msgs/String` payload `"cancel"`,
- unified fake ops runtime `python/examples/legacy/fake_tf_ops_suite.py` for ground teleop, go-to-point, waypoint, task path, and status cycle tests,
- fake aerial ops simulator `python/examples/legacy/fake_tf_drone_ops_suite.py` for drone takeoff/land + 3D go-to/waypoint validation.

## Global Visualization and Workspace Config Model

Registration payloads can now include:
- `global_visualizations` (deduped, robot-independent visual layers such as occupancy grid, 3D maps, octomap mesh layers, and semantic boxes),
- `workspace_config.position_scale` (global scale hint consumed by MR runtime).

This enables workspace-level visualization wiring without duplicating map or semantic-layer config in each robot-scoped visualization block. In current MR builds, global semantic layers are default-visible for all operators, while a user closing/hiding a layer is treated as a local display choice.

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
- dedicated host-side test flow via `python/examples/legacy/sdk_multi_operator_host_demo.py`,
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

The runnable command catalog for the preserved demo suite now lives in [python/examples/legacy/README.md](python/examples/legacy/README.md).

Use that document for:
- registration smoke tests and multi-robot camera stress,
- teleop, go-to-point, waypoint, drone, and legged action validation,
- robot-description, tutorial, RViz, and Carter live integrations,
- stereo camera and immersive transport coverage,
- synthetic 3D map, octomap, and mesh-conversion support flows.

Recommended first checks:
- `python/examples/legacy/e2e_registration_check.py`
- `python/examples/legacy/fake_tf_ops_suite.py`
- `python/examples/legacy/sdk_typical_ops_demo.py`
- `python/examples/legacy/sdk_multi_operator_host_demo.py`

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
- Safety and semantic-perception signals for teleop and supervision, with future scene-context sources expected from `lenses`.
- Multi-operator and copilot-oriented orchestration scenarios, with future copilot workflows expected from `compass`.

SDK roadmap and examples should evolve to provide the metadata, presets, and validation scripts needed for these MR milestones.

## Roadmap

> [!NOTE]
> SDK roadmap items are scoped to payload schemas, orchestration policies, and validation workflows that unlock MR/runtime features. Current baseline includes compatibility guardrail tests, curated no-CLI examples, preserved legacy examples, semantic configuration helpers, description-driven visual mesh bodies with Collada/DAE baking, Carter and Unitree live registrations, global map/semantic visualization examples, flat single-robot ROS-binding compatibility, multi-operator registration replay, and the separate initialization of `compass` and `lenses` as adjacent future services.

| Track | Status | SDK Baseline | Next Milestone |
|---|---|---|---|
| Robot Manager Contracts | :large_orange_diamond: In progress | `robot_manager_config` payload support, typed semantic configuration helpers, demo defaults, and robot-type action semantics are in place. Drone robots keep Take Off/Land, wheeled robots disable those actions, and legged robots map them to Stand Up/Sit Down action commands. | Extend schema for richer status bindings, task availability reasons, and section-level runtime options. |
| Teleoperation Contracts | :large_orange_diamond: In progress | `control.teleop` contract, teleop fake TF scenarios, control-topic dashboard rows, runtime transport-state signaling, curated teleop examples, and live Unitree Go1 conservative speed defaults are integrated. Follow-leader teleop V1 in MR reuses this contract with no schema change. | Add subset/handoff metadata, safety-speed profile presets, and manipulator-capability descriptors. |
| ROS Binding Compatibility | :large_orange_diamond: In progress | Optional ROS-binding metadata is integrated for prefixed and flat single-robot workflows, with resolved registration payloads, dashboard topic ownership overrides for root topics, curated flat registration examples, and compatibility tests. | Document backend metadata conventions more formally and add diagnostics for frame/topic mismatches before registration. |
| 2D Map Contracts | :white_check_mark: Foundation complete | Global occupancy-grid visualization payload and workspace scale forwarding are integrated. | Add richer map overlay contracts (goals, nav path layers, region semantics). |
| 3D Map Contracts | :large_orange_diamond: In progress | Point-cloud, mesh, and octomap global visualization payload wiring is integrated with greedy voxel meshing, snapshot-first Quest defaults, marker-only mesh transport coercion for runtime stability, replay-time bounded republish bursts, consolidated legacy 3D-map helpers, and a curated global-map registration example. | Expand native octomap interoperability beyond metadata-only publish path and add renderer policy hints (decimation/quality tiers) for runtime tuning. |
| DataViz API Structure | :large_orange_diamond: In progress | Current fixed DataViz helpers remain compatible, while internal visualization models, global visualization builders, semantic helpers, and curated examples now provide a cleaner v1 foundation. | Introduce an additive entity-based v2 DataViz contract with annotation contexts, batch updates, temporal metadata, and compatibility adapters back to v1. |
| Examples and Compatibility Guardrails | :large_orange_diamond: In progress | Public import/payload guardrail tests, curated no-CLI examples, a preserved legacy catalog, and legacy help/command documentation are in place. | Add automated launch-smoke coverage for curated examples and payload snapshot coverage for every new contract family. |
| Robot Description Contracts (V1) | :large_orange_diamond: In progress | URDF resolver + compiled collision/joint/visual-mesh schema, manifest hashing, chunked request/reply transport, body mesh modes (`collision_only`, `preview_mesh`, `runtime_high_mesh`), source-color recovery, Collada/DAE mesh baking, curated robot-description examples, and Unitree Go1 visual mesh registration are integrated. | Add known-robot local hero-mesh fallback policy, package/material diagnostics, and validation for mixed mesh+collision deployments across more real robot descriptions. |
| Tasking (2D/3D) | :large_orange_diamond: In progress | Typed Go-To/Waypoint schemas are integrated with altitude-bounds support for aerial robots, legged stand/sit action commands, and fake TF validation scripts for ground, drone, and legged ops suites. | Add explicit payload contracts for Draw Path, Label Pose, Go-To Labeled Pose, and subset-based Multi-Robot Go-To policies. |
| Session Recording | :white_circle: Planned | - | Add mission/session record contract (events, commands, timeline references). |
| After-Action Replay | :white_circle: Planned | - | Add replay manifest schema and deterministic timeline reconstruction inputs. |
| Adaptive Streaming Policies | :white_circle: Planned | Static transport profiles exist, but no load-aware policy orchestration. | Add policy payload for per-robot FPS/resolution tiers, priority (teleop-first), transport fallback, and dynamic max-stream guardrails. |
| Operator Safety Contracts | :white_circle: Planned | - | Add deadman/e-stop/geofence/risk-confirm metadata channels with policy defaults. |
| Persistent Mission Objects | :white_circle: Planned | - | Add shared mission-object schema (pins, notes, attachments, assignees, lifecycle). |
| Manipulator Teleoperation | :white_circle: Planned | - | Add manipulator capability descriptors (joint/EEF/gripper limits, home poses, safety envelopes). |
| Mobile Manipulator Coordination | :white_circle: Planned | Base and manipulator are modeled independently today. | Add combined base+arm action primitives and coordination metadata. |
| Semantic Perception Layers | :large_orange_diamond: In progress | Semantic box payloads and a curated semantic perception registration example are integrated as global visualization layers; `lenses` remains the future richer scene-understanding producer. | Add semantic tracks, class dictionaries, confidence/staleness styling, segmentation/depth overlays, uncertainty, spatial anchoring, and scene-context references aligned with `lenses` outputs. |
| Multi-Operator Orchestration | :large_orange_diamond: In progress | SDK dashboard presence visibility now tracks shared-host, shared-join, and private-workspace operators; multi-operator host demo workflow, bridge auto-start hardening, SDK registry replay protocol publishing, replay-triggered map/semantic layer replay behavior, and direct private-operator replay support are integrated. | Add richer operator identity/lease observability summaries, explicit ownership metadata schemas, layer visibility ownership semantics, and stronger rejoin/replay validation suites. |
| AI Copilot Orchestration | :white_circle: Planned | No copilot contract is integrated into `horus_sdk` yet; `compass` now exists separately as the future copilot/orchestration service. | Define copilot action-scoping contracts, approval/guardrail metadata, operator-visible intervention traces, and the first stable `horus_sdk <-> compass` contract boundary. |

## Citation

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

## Contact

For questions or support:

- Omotoye Shamsudeen Adekoya
- Email: `omotoye.adekoya@edu.unige.it`

## Acknowledgments

This project is part of PhD research at the University of Genoa, under the supervision of:

- Prof. Carmine Recchiuto
- Prof. Antonio Sgorbissa

Developed by **RICE Lab**, University of Genoa.

## Contributing

1. Reproduce behavior with explicit command lines.
2. Include before/after impact on registration and dashboard states.
3. Add or update focused tests when changing serialization/state logic.

## Related Repositories

- MR runtime: <https://github.com/RICE-unige/horus>
- ROS 2 bridge runtime: <https://github.com/RICE-unige/horus_ros2>
- Copilot/orchestration service: <https://github.com/Omotoye/compass>
- Scene-understanding service: <https://github.com/Omotoye/lenses>

## License

Apache-2.0. See [`LICENSE`](LICENSE).
