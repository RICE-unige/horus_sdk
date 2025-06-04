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

**Horus-SDK** is the official Python toolkit that glues your ROS robots to the **HORUS Mixed-Reality application** running on Meta Quest 3.
It discovers robots, builds a JSON *Initial Payload* for the headset, and relays traffic both ways—so you write **high-level Python**, not raw ROS.

### **Current Version:** `0.1.0-alpha`

> *Skeleton only*: all modules import, but most are still stubs.
> First real code will land in the `core/` package.

---

## 🌟 Features

| Done ✔ | Planned 🛠 | Description                                                                     |
| :----: | :--------: | ------------------------------------------------------------------------------- |
|    ✔   |            | **Typed topic keys** (`Status.BATTERY`, `DataViz.LIDAR`) — no more string typos |
|        |     🛠     | Async **ROS 2 bridge** (`rclpy`) + ROS 1 (`roslibpy`)                           |
|        |     🛠     | JSON handshake generator for the MR headset                                     |
|        |     🛠     | Plugin system (`plugins.Rosbot`, `plugins.Spot`)                                |
|        |     🛠     | Teleop profile mapping (axes / buttons → `/cmd_vel`)                            |

---

## 🛠 Installation

### From PyPI *(soon)*

```bash
pip install horus-sdk         # will ship once M2 is done
```

### Editable (Dev) Install

```bash
# clone
git clone https://github.com/RICE-unige/horus_sdk.git && cd horus_sdk

# Python ≥ 3.11
python3.11 -m venv ~/.venvs/horus
source ~/.venvs/horus/bin/activate

pip install --upgrade pip setuptools wheel
pip install -e .              # editable install
```

Check:

```bash
python - <<'PY'
import horus, sys, pathlib
print("HORUS", horus.__version__ if hasattr(horus, "__version__") else "dev",
      "→", pathlib.Path(horus.__file__).parent)
PY
```

---

## 🚀 Quick Start

```python
import asyncio, horus as hr
from horus.plugins.rosbot import Rosbot          # stub for now

async def main():
    client = hr.Client(transport="ros2",
                       headset="tcp://0.0.0.0:6000")

    # build a robot descriptor fluently
    client.add_robot(
        Rosbot("bot1").with_namespace("/bot1")
    )

    await client.handshake()   # emits InitialPayload → headset
    await client.spin()        # pumps EventBus + bridges

asyncio.run(main())
```

*(Until the bridges are finished this will just run without doing much.)*

---

## 📐 Architecture Glimpse

```text
client.py        ← orchestrator
 ├─ core/        ← pure logic (TopicMap, EventBus…)
 ├─ bridge/      ← IO adapters (ros1, ros2, unity_tcp)
 ├─ robot/       ← façade for users (status, task, teleop)
 └─ plugins/     ← ready-made robot presets
```

Everything outside **`bridge/`** is ROS-agnostic and therefore unit-testable
without running a roscore.

---

## 🗺 Roadmap

| Milestone | Target                                             |
| --------- | -------------------------------------------------- |
| **M1**    | Implement `TopicSpec`, `TopicMap`, and `EventBus`  |
| **M2**    | Unity-TCP bridge + JSON handshake, full unit tests |
| **M3**    | ROS 2 bridge + first working control loop          |
| **M4**    | Plugins: Rosbot (diff-drive), Spot (legged)        |
| **M5**    | ROS 1 bridge & docs deployment                     |

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
