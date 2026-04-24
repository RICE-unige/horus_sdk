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
| `python/examples/legacy/` | Current operational demo catalog preserved as legacy reference while the next curated example set is prepared |
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
> Current operational demos now live under `python/examples/legacy/`.
> The root `python/examples/` folder is being kept clean for the next curated example set.

> [!NOTE]
> Commands below use repository-clone paths (`~/horus_sdk`, `~/horus_ws`).
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
python3 python/examples/legacy/fake_tf_ops_suite.py --robot-count 10 --rate 30 --static-camera --publish-compressed-images --task-path-publish-rate 5 --publish-collision-risk --collision-threshold-m 1.2
```

### 3) Run the typical SDK registration demo

```bash
cd ~/horus_sdk
python3 python/examples/legacy/sdk_typical_ops_demo.py --robot-count 10 --workspace-scale 0.1
```

### 4) Validate the MR flow

- connect the MR client,
- draw and accept the workspace,
- confirm robots appear in the workspace,
- open a Robot Manager,
- verify dashboard topic rows move from registration-only into active runtime state.

### Additional workflows

The full command catalog for the preserved demo suite now lives in [python/examples/legacy/README.md](python/examples/legacy/README.md).

Use that catalog for:
- multi-operator host and join validation,
- flat single-robot ROS-binding validation,
- Carter live navigation integration,
- robot-description and workspace-tutorial demos,
- stereo camera and immersive transport validation,
- drone and legged action-flow validation,
- synthetic 3D map, mesh, and octomap publishers.

Support utilities remain outside the legacy folder:
- `python/examples/tools/fetch_robot_description_assets.py`
- `python/examples/.local_assets/robot_descriptions/`

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
> SDK roadmap items are scoped to payload schemas, orchestration policies, and validation workflows that unlock MR/runtime features; current baseline includes description-driven visual mesh bodies with quality modes, Carter nav shared-map integration, private-operator presence tracking, marker-only mesh stability flow, octomap mode integration, flat single-robot ROS-binding compatibility, and the separate initialization of `compass` and `lenses` as adjacent future services.

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
| Semantic Perception Layers | :white_circle: Planned | No semantic scene-context contract is integrated into `horus_sdk` yet; `lenses` now exists separately as the future scene-understanding producer. | Add semantic layer payloads with confidence, uncertainty, spatial anchoring, and scene-context references aligned with `lenses` outputs. |
| Multi-Operator Orchestration | :large_orange_diamond: In progress | SDK dashboard presence visibility now tracks shared-host, shared-join, and private-workspace operators; multi-operator host demo workflow, bridge auto-start hardening, SDK registry replay protocol publishing, replay-triggered map republish burst hooks for joiner map reliability, and direct private-operator replay support are integrated. | Add richer operator identity/lease observability summaries, explicit ownership metadata schemas, and stronger rejoin/replay validation suites. |
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
