| **HORUS SDK** | <h1> Holistic Operational Reality for Unified Systems </h1> |
|---|---|
| SDK Layer | Registration, orchestration, and observability |

<div align="center">

```text
██╗  ██╗ ██████╗ ██████╗ ██╗   ██╗███████╗
██║  ██║██╔═══██╗██╔══██╗██║   ██║██╔════╝
███████║██║   ██║██████╔╝██║   ██║███████╗
██╔══██║██║   ██║██╔══██╗██║   ██║╚════██║
██║  ██║╚██████╔╝██║  ██║╚██████╔╝███████║
╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═╝ ╚═════╝ ╚══════╝
```
</div>

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
| `python/examples/` | Operational demos (`sdk_registration_demo.py`, fake publishers, e2e checks) |
| `python/tests/` | SDK tests (serialization/state/dashboard behavior) |
| `cpp/` | C++ SDK track |
| `rust/` | Rust SDK track |

> [!NOTE]
> Python remains the most complete and actively used track for current experiments.

## Rust/C++ Parity Workstreams

- Canonical parity contract and fixtures:
  - `contracts/sdk_payload_contract.md`
  - `contracts/fixtures/*.json`
- Branches:
  - `feature/sdk-rust-parity`
  - `feature/sdk-cpp-parity`

Rust parity branch currently includes:
- typed registration payload builders,
- camera transport profile fallback/default parity,
- global visualization dedupe parity,
- workspace scale serialization parity,
- robot manager config defaults parity,
- Rust integration tests mirroring Python parity tests under `rust/tests/`.

C++ parity branch now also includes:
- `feature/sdk-cpp-parity` implementation of parity APIs and typed payload builders,
- C++ examples parity (`sdk_registration_demo`, `fake_tf_publisher`, `e2e_registration_check`),
- C++ parity tests and benchmark binaries,
- ROS/no-ROS C++ CI coverage through the matrix workflow.

## Requirements

- Python **3.10+**
- ROS 2 **Humble** or **Jazzy** environment available (`rclpy` + message packages)
- Running bridge from `horus_ros2` (`horus_unity_bridge`)

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
cd ~/horus_ws/src/horus_ros2
source /opt/ros/humble/setup.bash  # or jazzy
colcon build --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

### 2) Start fake data (TF + camera + occupancy map)

```bash
cd ~/horus_sdk
python3 python/examples/fake_tf_publisher.py --robot-count 4 --with-camera --publish-occupancy-grid
```

### 3) Run SDK registration demo

```bash
cd ~/horus_sdk
python3 python/examples/sdk_registration_demo.py --robot-count 4 --with-camera --with-occupancy-grid --workspace-scale 0.1
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

### Registration smoke test

```bash
python3 python/examples/e2e_registration_check.py
```

### Multi-robot camera stress test

```bash
python3 python/examples/fake_tf_publisher.py --robot-count 10 --with-camera
python3 python/examples/sdk_registration_demo.py --robot-count 10 --with-camera
```

### Occupancy-map integration test

```bash
python3 python/examples/fake_tf_publisher.py --robot-count 6 --with-camera --publish-occupancy-grid --occupancy-rate 1.0
python3 python/examples/sdk_registration_demo.py --robot-count 6 --with-camera --with-occupancy-grid --workspace-scale 0.1
```

### Targeted tests

```bash
pytest python/tests/test_camera_transport_profiles.py
pytest python/tests/test_global_visualizations_payload.py
pytest python/tests/test_workspace_scale_payload.py
pytest python/tests
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
| Teleoperation Contracts | :large_orange_diamond: In progress | MiniMap/Teleop transport profile fields are active in camera config payloads. | Add explicit teleop mode capability descriptors and handoff metadata. |
| 2D Map Contracts | :white_check_mark: Foundation complete | Global occupancy-grid visualization payload and workspace scale forwarding are integrated. | Add richer map overlay contracts (goals, nav path layers, region semantics). |
| 3D Map Contracts | :white_circle: Planned | - | Define 3D map source descriptors and rendering policy hints. |
| Tasking (2D/3D) | :white_circle: Planned | Generic task hooks exist; structured task catalogs are not finalized. | Publish typed task schema for Go to Point, Waypoint, Path Drawing, Go to Label, Multi-Robot Go to Point, and Follow-Lead Teleop. |
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
