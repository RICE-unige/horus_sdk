| ![horus\_logo\_medium](https://github.com/user-attachments/assets/895961b0-c4b5-4f20-994f-be4ad20efe7f) | <h1 align="center"><a href="https://rice.dibris.unige.it/">**H**olistic **O**perational **R**eality for **U**nified **S**ystems – SDK</a></h1> |
| :-----------------------------------------------------------------------------------------------------: | :--------------------------------------------------------------------------------------------------------------------------------------------- |

---

## 📚 Table of Contents

* [🔍 Overview](#-overview)

  * [**Current Version:** `0.1.0-alpha`](#current-version-010-alpha)
* [🌟 Features](#-features)
* [🛠 Installation](#-installation)

  * [From PyPI *(soon)*](#from-pypi)
  * [Editable (Dev) Install](#editable-dev-install)
* [🚀 Quick Start](#-quickstart)
* [📐 Architecture Glimpse](#-architecture-glimpse)
* [🗺 Roadmap](#-roadmap)
* [🤝 Contributing](#-contributing)
* [📝 License](#-license)
* [📖 Citation](#-citation)
* [📬 Contact](#-contact)
* [💡 Acknowledgments](#-acknowledgments)

---

## 🔍 Overview

**Horus-SDK** is the official toolkit that glues your ROS robots to the **HORUS Mixed-Reality application** running on Meta Quest 3. It discovers robots, builds a JSON *Initial Payload* for the headset, and relays traffic both ways.

This repository contains two parallel SDK implementations:

*   **`./python`**: A flexible and easy-to-use Python SDK.
*   **`./cpp`**: A high-performance C++ SDK for production environments.

Both SDKs are being developed in parallel and aim for feature parity.

### **Current Version:** `0.1.0-alpha`

> **🎉 Latest Update**: Python SDK reorganized with dedicated modules for better architecture!
> Complete robot registration system with comprehensive examples and improved backend integration.
> Ready for advanced robot management and mixed reality visualization.

---

## 🌟 Features

| Done ✔ | Planned 🛠 | Description                                                                     |
| :----: | :--------: | ------------------------------------------------------------------------------- |
|    ✔   |            | **Modular Python SDK** with dedicated robot, sensor, dataviz, and color modules |
|    ✔   |            | **Complete robot registration system** with ROS2 service integration          |
|    ✔   |            | **Comprehensive sensor support** (Camera, LiDAR, LaserScan) with auto-config  |
|    ✔   |            | **Advanced data visualization** with automatic color assignment for MR        |
|    ✔   |            | **Complete ROS2 backend infrastructure** with C++ implementation                |
|    ✔   |            | **Professional SDK initialization** with animated spinners and status checking  |
|    ✔   |            | **Real-time MR app monitoring** with instant Quest 3 connection detection      |
|    ✔   |            | **TCP bridge integration** via ROS-TCP-Endpoint for Quest 3 communication     |
|    ✔   |            | **Continuous monitoring mode** with clean Ctrl+C shutdown and process cleanup  |
|    ✔   |            | **Development testing framework** with interactive test launcher               |
|    ✔   |            | **Automatic backend management** with comprehensive process lifecycle control  |
|    ✔   |            | **Comprehensive examples** (Carter robot, live integration, sensor demos)      |
|        |     🛠     | Async **ROS 2 bridge** (`rclpy`) + ROS 1 (`roslibpy`)                           |
|        |     🛠     | JSON handshake generator for the MR headset                                     |
|        |     🛠     | Plugin system (`plugins.Rosbot`, `plugins.Spot`)                                |
|        |     🛠     | Teleop profile mapping (axes / buttons → `/cmd_vel`)                            |

---

## 🚀 Getting Started

### Quick Test
```bash
# Clone and test the SDK
git clone --recursive https://github.com/RICE-unige/horus_sdk.git
cd horus_sdk

# Build ROS2 workspace
cd horus_ros2_ws
colcon build
source install/setup.bash
cd ..

# Test SDK with quick test
python3 examples/quick_test.py
```

### Full Documentation

*   [**Python SDK**](./python/README.md)
*   [**C++ SDK**](./cpp/README.md)
*   [**ROS2 Backend**](./horus_ros2_ws/README.md)
*   [**Testing Guide**](./TESTING.md)

---

## 📐 Architecture Overview

### System Architecture
```text
Python/C++ SDK ←→ HORUS Backend (C++) ←→ ROS-TCP-Endpoint ←→ HORUS MR App (Quest 3)
    (port 8080)                            (port 10000)
```

### SDK Structure
```text
client        ← orchestrator
 ├─ core/        ← pure logic (TopicMap, EventBus, exceptions)
 ├─ robot/       ← robot management and control
 ├─ sensors/     ← sensor modeling and management
 ├─ dataviz/     ← data visualization for MR
 ├─ color/       ← color management for multi-robot
 ├─ bridge/      ← IO adapters (ros1, ros2, unity_tcp)
 ├─ plugins/     ← ready-made robot presets
 └─ utils/       ← supporting infrastructure
```

### Key Components
- **HORUS Backend**: C++ ROS2 node with TCP server and robot registration system
- **ROS-TCP-Endpoint**: Quest 3 integration bridge (as git submodule)
- **SDK Client**: Professional initialization with real-time MR app monitoring
- **Robot Management**: Complete robot modeling with sensor integration
- **Data Visualization**: Advanced MR visualization with automatic color assignment
- **Connection Monitor**: Live detection of Quest 3 device connections/disconnections
- **Process Manager**: Comprehensive cleanup and lifecycle management
- **Examples Framework**: Comprehensive robot integration examples and demos

Everything outside **`bridge/`** is ROS-agnostic and therefore unit-testable
without running a roscore.

---

## 🗺 Roadmap

| Milestone | Target                                             | Status |
| --------- | -------------------------------------------------- | ------ |
| **M0**    | Real-time MR app connection monitoring             | ✅ Complete |
| **M0.5**  | Python SDK reorganization and robot registration   | ✅ Complete |
| **M1**    | Implement `TopicSpec`, `TopicMap`, and `EventBus`  | 🔄 Ready for implementation |
| **M2**    | TCP bridge + JSON handshake, full unit tests       | ✅ Infrastructure complete |
| **M3**    | ROS 2 bridge + first working control loop          | ✅ Backend ready |
| **M4**    | Plugins: Rosbot (diff-drive), Spot (legged)        | 🔄 Awaiting M1 completion |
| **M5**    | ROS 1 bridge & docs deployment                     | 📋 Planned |

---

## 🤝 Contributing

PRs are welcome!
See **`CONTRIBUTING.md`** (coming soon) for coding style, branching model, and how to run the test suite.

---

## 📝 License

Released under the [Apache 2.0](LICENSE) license.

---

## 📖 Citation

If you use HORUS or the SDK in academic work, please cite:

```bibtex
@misc{adekoya2025horus,
  title   = {HORUS: A Mixed Reality Interface for Managing Teams of Mobile Robots},
  author  = {Adekoya, Omotoye Shamsudeen and Sgorbissa, Antonio and Recchiuto, Carmine T.},
  year    = {2025},
  eprint  = {2506.02622},
  archivePrefix = {arXiv},
  primaryClass  = {cs.RO}
}
```

---

## 📬 Contact

|                        |                                                                                                                               |
| :--------------------- | :---------------------------------------------------------------------------------------------------------------------------- |
| **Omotoye S. Adekoya** | [homepage](https://rubrica.unige.it/personale/UkFEXVhg) · [omotoye.adekoya@edu.unige.it](mailto:omotoye.adekoya@edu.unige.it) |

---

## 💡 Acknowledgments

Developed at the **[RICE Lab](https://rice.dibris.unige.it/)**,
[University of Genoa](https://unige.it/en), under
Prof. Carmine Recchiuto & Prof. Antonio Sgorbissa.
