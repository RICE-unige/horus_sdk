<p align="center">
  <img src="docs/horus_logo_black.svg#gh-light-mode-only" alt="HORUS logo" height="90">
  <img src="docs/horus_logo_white.svg#gh-dark-mode-only" alt="HORUS logo" height="90">
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
- Docs branch: `docs/site-v1`
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
| `python/examples/` | Operational demos (`sdk_registration_demo.py`, `sdk_typical_ops_demo.py`, fake publishers, e2e checks) |
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

### 1) Start bridge (`horus_ros2`)

Follow the setup/install instructions in the [`horus_ros2` README](https://github.com/RICE-unige/horus_ros2), then run:

```bash
cd ~/horus/ros2
source /opt/ros/humble/setup.bash  # or jazzy
colcon build --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

### 2) Start unified fake ops runtime (TF + camera + teleop + tasks)

```bash
cd ~/horus/sdk
python3 python/examples/fake_tf_ops_suite.py --robot-count 10 --rate 30 --static-camera --publish-compressed-images
```

### 3) Run typical SDK registration demo

```bash
cd ~/horus/sdk
python3 python/examples/sdk_typical_ops_demo.py --robot-count 10 --workspace-scale 0.1
```

## Camera Registration Model

Camera payloads support legacy and profile-based transport fields:
- `streaming_type` (legacy fallback)
- `minimap_streaming_type`
- `teleop_streaming_type`
- `startup_mode`

Typical policy:
- MiniMap -> ROS
- Teleop -> WebRTC

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
- `control.tasks.go_to_point` payload serialization (`goal_topic`, `cancel_topic`, `status_topic`, `frame_id`, tolerances),
- `control.tasks.waypoint` payload serialization (`path_topic`, `status_topic`, `frame_id`, tolerances),
- metadata override source `robot.metadata["task_config"]["go_to_point"]`,
- metadata override source `robot.metadata["task_config"]["waypoint"]`,
- go-to-point cancel contract on `/<robot>/goal_cancel` using `std_msgs/String` payload `"cancel"`,
- fake goal-navigation simulator `python/examples/fake_tf_go_to_point.py` for send/cancel/reached cycle tests,
- fake waypoint simulator `python/examples/fake_tf_waypoint.py` for ordered path/status cycle tests.

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

> [!NOTE]
> When validating SDK changes, always test against the full MR flow:
> connect -> draw workspace -> accept -> register -> observe dashboard/topic transitions.

## Cross-Repo Alignment Priorities

The MR roadmap introduces upcoming requirements that depend on SDK payload and orchestration maturity:
- 2D/3D tasking workflows (go-to-point, waypoints, path drawing, go-to-label, multi-robot go-to-point, follow-lead teleop).
- Expanded sensor visualization requirements (battery, velocity, LaserScan, PointCloud).
- Session recording + after-action replay data contracts.
- Resource-aware streaming policy signals (quality tiers, priority, stream caps).
- Persistent mission objects (pins/annotations/evidence/task assignment).
- Safety and semantic perception signals for teleop and supervision.
- Multi-operator and copilot-oriented orchestration scenarios.

SDK roadmap and examples should evolve to provide the metadata, presets, and validation scripts needed for these MR milestones.

## Roadmap

> [!NOTE]
> SDK roadmap items are scoped to payload schemas, orchestration policies, and validation workflows that unlock MR/runtime features.

| Track | Status | SDK Baseline | Next Milestone |
|---|---|---|---|
| Robot Manager Contracts | :large_orange_diamond: In progress | `robot_manager_config` payload support and demo defaults are in place. | Extend schema for status/task bindings and section-level runtime options. |
| Teleoperation Contracts | :large_orange_diamond: In progress | `control.teleop` contract, teleop fake TF scenarios, control-topic dashboard rows, and runtime transport-state signaling are integrated. | Add follow-leader/advanced handoff metadata and manipulator-capability descriptors. |
| 2D Map Contracts | :white_check_mark: Foundation complete | Global occupancy-grid visualization payload and workspace scale forwarding are integrated. | Add richer map overlay contracts (goals, nav path layers, region semantics). |
| 3D Map Contracts | :white_circle: Planned | - | Define 3D map source descriptors and rendering policy hints. |
| Tasking (2D/3D) | :large_orange_diamond: In progress | Typed schemas for Go-To Point and Waypoint are integrated, with fake TF validation scripts and fixtures/tests. | Add Path Drawing + Go-To Label + Multi-Robot Go-To Point contracts and validation presets. |
| Session Recording | :white_circle: Planned | - | Add mission/session record contract (events, commands, timeline references). |
| After-Action Replay | :white_circle: Planned | - | Add replay manifest schema and deterministic timeline reconstruction inputs. |
| Adaptive Streaming Policies | :white_circle: Planned | Static transport profiles exist, but no load-aware policy orchestration. | Add policy payload for per-robot FPS/resolution tiers, priority (teleop-first), transport fallback, and dynamic max-stream guardrails. |
| Operator Safety Contracts | :white_circle: Planned | - | Add deadman/e-stop/geofence/risk-confirm metadata channels with policy defaults. |
| Persistent Mission Objects | :white_circle: Planned | - | Add shared mission-object schema (pins, notes, attachments, assignees, lifecycle). |
| Manipulator Teleoperation | :white_circle: Planned | - | Add manipulator capability descriptors (joint/EEF/gripper limits, home poses, safety envelopes). |
| Mobile Manipulator Coordination | :white_circle: Planned | Base and manipulator are modeled independently today. | Add combined base+arm action primitives and coordination metadata. |
| Semantic Perception Layers | :white_circle: Planned | - | Add semantic layer payloads with confidence, uncertainty, and spatial anchoring metadata. |
| Copilot and Multi-Operator | :white_circle: Planned | Single-operator path is primary baseline. | Add operator identity/session ownership metadata and copilot action-scoping contracts. |

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
