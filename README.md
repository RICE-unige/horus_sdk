# HORUS SDK

| ![horus_logo_medium](https://github.com/user-attachments/assets/895961b0-c4b5-4f20-994f-be4ad20efe7f) | <h1 align="center"><a href="https://rice.dibris.unige.it/">Holistic Operational Reality for Unified Systems – SDK</a></h1> |
| :-----------------------------------------------------------------------------------------------------: | :--------------------------------------------------------------------------------------------------------------------------------------------- |

<div align="center">

[![CI](https://github.com/RICE-unige/horus_sdk/actions/workflows/ci.yml/badge.svg)](https://github.com/RICE-unige/horus_sdk/actions/workflows/ci.yml)
[![Release](https://github.com/RICE-unige/horus_sdk/actions/workflows/release.yml/badge.svg)](https://github.com/RICE-unige/horus_sdk/actions/workflows/release.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/downloads/)
[![ROS2 Humble](https://img.shields.io/badge/ros2-humble-blue.svg)](https://docs.ros.org/en/humble/)


</div>

> [!IMPORTANT]
> The SDK manages the Client-side logic. The ROS 2 Bridge Infrastructure has moved to a separate repository: [`horus_ros2`](https://github.com/RICE-unige/horus_ros2).

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

### Python SDK (Build from Source)

**For Development (Editable):**
```bash
python -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

**For Release (Wheel Generation):**
```bash
pip install build
python3 -m build
# Output in dist/ (e.g., horus_sdk-0.1.0-py3-none-any.whl)
```

### ROS 2 Bridge Infrastructure

The infrastructure (Bridge + Request Handlers) lives in `horus_ros2`.

```bash
git clone https://github.com/RICE-unige/horus_ros2.git
cd horus_ros2

# Install dependencies
rosdep install --from-paths . -y --ignore-src

# Optional WebRTC camera streaming prerequisites (bridge host)
sudo apt-get update
sudo apt-get install -y \
  pkg-config \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libopencv-dev

# Build
colcon build --symlink-install --packages-select horus_unity_bridge --cmake-args -DENABLE_WEBRTC=ON
source install/setup.bash

# Run
ros2 launch horus_unity_bridge unity_bridge.launch.py
```

> [!NOTE]
> WebRTC support in `horus_unity_bridge` is enabled only when CMake finds GStreamer + OpenCV.
> The bridge fetches `libdatachannel` during configure, so internet access is required on first build.
> In the Unity MR project, keep `com.unity.webrtc` in `Packages/manifest.json` (current tested entry: `"com.unity.webrtc": "3.0.0-pre.8"`).

### C++ SDK

```bash
cmake -S cpp -B build/cpp
cmake --build build/cpp
```

Link against headers in `cpp/include/horus` to embed HORUS primitives directly inside your ROS nodes or robotics middleware.

---

## 🚀 Quick Start

```bash
git clone https://github.com/RICE-unige/horus_sdk.git
cd horus_sdk

# 1. Start Infrastructure (in horus_ros2 repo)
cd horus_ros2
source install/setup.bash
ros2 launch horus_unity_bridge unity_bridge.launch.py

# 2. Run SDK Client (in horus_sdk repo)
python3 python/examples/sdk_registration_demo.py
```

> [!TIP]
> When iterating on robot configs, use `python/tests/test_topic_status_*.py` as a fast signal that topic subscriptions are being announced correctly without a running ROS graph.

### Smoke Test (Registration + Ack + Heartbeat)

```bash
python3 python/examples/e2e_registration_check.py
```

---

## 📐 Architecture

```text
Python / C++ SDKs  <-->  HORUS Backend (ROS 2)  <-->  HORUS Unity Bridge  <-->  HORUS MR App
        (8080)                       (10000)                        (Unity TCP client)
```

- **SDK Client** – CLI orchestrator that brings up backend processes, monitors Unity presence, and registers robots.
- **Robot & Sensor Models** – Declarative descriptions of robots, sensors, and visualization streams.
- **DataViz** – Builds the JSON payload consumed by the MR app with color-coded overlays.
- **Topic Monitor** – Watches ROS graph activity and renders TTY or non-TTY dashboards.
- **Backend** – ROS 2 nodes manage TCP connections, robot registry, and relay telemetry to the headset.
- **Unity Bridge** – `horus_unity_bridge` hosts the TCP server on port 10000; the HORUS MR app connects as a client.

---

## 🗺 Roadmap

| Milestone | Target                                           | Status |
| --------- | ------------------------------------------------ | ------ |
| M0        | Real-time MR connection monitoring               | ✅     |
| M1        | TopicMap & EventBus parity across Python/C++     | 🔄     |
| M2        | TCP bridge + JSON handshake tooling              | ✅     |
| M3        | ROS 2 bridge + closed-loop control demo          | ✅     |
| M4        | Plugin presets (Rosbot, Spot)                    | 🔄     |
| M5        | Docs site refresh                                | 📋     |

---

## 🤝 Contributing

PRs and discussions are welcome! Please include tests (or manual repro steps) for any behavior changes and run `pytest python/tests` before submitting.

---

## 📝 License

Distributed under the [Apache 2.0](LICENSE) license.
