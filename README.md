# HORUS SDK

| ![horus_logo_medium](https://github.com/user-attachments/assets/895961b0-c4b5-4f20-994f-be4ad20efe7f) | <h1 align="center"><a href="https://rice.dibris.unige.it/">Holistic Operational Reality for Unified Systems – SDK</a></h1> |
| :-----------------------------------------------------------------------------------------------------: | :--------------------------------------------------------------------------------------------------------------------------------------------- |

<div align="center">

[![CI](https://github.com/RICE-unige/horus_sdk/actions/workflows/ci.yml/badge.svg)](https://github.com/RICE-unige/horus_sdk/actions/workflows/ci.yml)
[![Release](https://github.com/RICE-unige/horus_sdk/actions/workflows/release.yml/badge.svg)](https://github.com/RICE-unige/horus_sdk/actions/workflows/release.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/downloads/)
[![ROS2 Humble](https://img.shields.io/badge/ros2-humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Documentation](https://img.shields.io/badge/docs-mkdocs-green.svg)](https://rice-unige.github.io/horus_sdk/)

</div>

> [!IMPORTANT]
> This repo combines the Python SDK, the C++ reference implementation, and the ROS 2 backend that interfaces with Unity’s ROS-TCP-Endpoint. Clone with `--recursive` (or run `git submodule update --init --recursive`) to ensure the Unity bridge is present.

---

## 📚 Table of Contents

- [Overview](#-overview)
  - [Current Version](#current-version-010-alpha)
- [Features](#-features)
- [Install & Build](#-install--build)
  - [Python SDK](#python-sdk)
  - [ROS 2 Backend](#ros-2-backend)
  - [C++ SDK](#c-sdk)
- [Quick Start](#-quick-start)
- [Architecture](#-architecture)
- [Roadmap](#-roadmap)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🔍 Overview

HORUS-SDK unifies ROS robots with the HORUS mixed-reality application on Meta Quest headsets. It auto-discovers robots, publishes a JSON handshake for the MR client, and mirrors telemetry/commands over TCP.

| Directory        | Highlights                                                                 |
| ---------------- | ---------------------------------------------------------------------------- |
| `python/`        | Primary SDK (client orchestration, robot & sensor models, monitoring utils) |
| `cpp/`           | High-performance SDK mirroring the Python API surface                        |
| `horus_ros2_ws/` | ROS 2 Humble workspace with the backend nodes and Unity ROS-TCP-Endpoint     |

### Current Version: `0.1.0-alpha`

> [!NOTE]
> The API is still evolving. Expect occasional breaking changes while we finalize the TopicMap/EventBus contracts.

---

## 🌟 Features

| Done ✔ | Planned 🛠 | Description                                                                   |
| :----: | :--------: | ----------------------------------------------------------------------------- |
| ✔      |            | Modular Python SDK (client, bridge, sensors, dataviz, plugins, utils)         |
| ✔      |            | Full robot registration + color assignment + dataviz wiring                   |
| ✔      |            | Topic subscription monitoring + TTY-friendly dashboards                       |
| ✔      |            | Managed ROS backend processes & TCP bridge integration                        |
| ✔      |            | Reference C++ SDK and ROS 2 backend workspace                                 |
|        | 🛠         | Async ROS 1/ROS 2 bridge parity in Python                                     |
|        | 🛠         | Plugin presets (Rosbot, Spot) + teleop profile mapping                        |
|        | 🛠         | JSON handshake generator + MR telemetry replay tooling                        |

---

## 🛠 Install & Build

### Python SDK

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .\.venv\Scripts\activate
pip install --upgrade pip
pip install -e ".[dev]"
pytest python/tests
```

The editable install exposes the `horus-sdk` console entry point so you can orchestrate robots or run the monitoring utilities directly.

### ROS 2 Backend

```bash
cd horus_ros2_ws
rosdep install --from-paths src -y --rosdistro humble
colcon build
source install/setup.bash
ros2 launch horus_backend horus_backend.launch.py
```

This spins up the backend node, Unity bridge (`ROS-TCP-Endpoint` submodule), and TCP server that the Quest headset connects to.

### C++ SDK

```bash
cmake -S cpp -B build/cpp
cmake --build build/cpp
```

Link against headers in `cpp/include/horus` to embed HORUS primitives directly inside your ROS nodes or robotics middleware.

---

## 🚀 Quick Start

```bash
git clone --recursive https://github.com/RICE-unige/horus_sdk.git
cd horus_sdk

# Backend
cd horus_ros2_ws && colcon build && source install/setup.bash
ros2 launch horus_backend horus_backend.launch.py

# Python client (separate shell)
cd ../
python -m venv .venv && source .venv/bin/activate
pip install -e .
python examples/quick_test.py
```

> [!TIP]
> When iterating on robot configs, use `python/tests/test_topic_status_*.py` as a fast signal that topic subscriptions are being announced correctly without a running ROS graph.

---

## 📐 Architecture

```text
Python / C++ SDKs  <-->  HORUS Backend (ROS 2)  <-->  ROS-TCP-Endpoint  <-->  HORUS MR App
        (8080)                       (10000)                        (Unity TCP bridge)
```

- **SDK Client** – CLI orchestrator that brings up backend processes, monitors Unity presence, and registers robots.
- **Robot & Sensor Models** – Declarative descriptions of robots, sensors, and visualization streams.
- **DataViz** – Builds the JSON payload consumed by the MR app with color-coded overlays.
- **Topic Monitor** – Watches ROS graph activity and renders TTY or non-TTY dashboards.
- **Backend** – ROS 2 nodes manage TCP connections, robot registry, and relay telemetry to the headset.

---

## 🗺 Roadmap

| Milestone | Target                                           | Status |
| --------- | ------------------------------------------------ | ------ |
| M0        | Real-time MR connection monitoring               | ✅     |
| M1        | TopicMap & EventBus parity across Python/C++     | 🔄     |
| M2        | TCP bridge + JSON handshake tooling              | ✅     |
| M3        | ROS 2 bridge + closed-loop control demo          | ✅     |
| M4        | Plugin presets (Rosbot, Spot)                    | 🔄     |
| M5        | ROS 1 bridge & docs site refresh                 | 📋     |

---

## 🤝 Contributing

PRs and discussions are welcome! Please include tests (or manual repro steps) for any behavior changes and run `pytest python/tests` before submitting.

---

## 📝 License

Distributed under the [Apache 2.0](LICENSE) license.
