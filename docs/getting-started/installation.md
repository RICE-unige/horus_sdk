---
title: Installation
sidebar_position: 1
---

# Installation

## Supported targets

- Ubuntu 24.04 (default ROS2: Jazzy)
- Ubuntu 22.04 (default ROS2: Humble)
- WSL2 Ubuntu (with mirrored networking recommended for Meta Quest connectivity)

## One-command bootstrap

```bash
curl -fsSL https://raw.githubusercontent.com/RICE-unige/horus_sdk/main/install.sh | bash
```

Default install root:

- `~/horus/sdk`
- `~/horus/ros2`
- `~/horus/bin`
- `~/horus/logs`
- `~/horus/state`

## Non-interactive install

```bash
bash install.sh --yes --channel stable --ros-distro jazzy --webrtc on
```

## Development install (repo-local)

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

## Verify installation

```bash
horus-status
```

Expected:

- `horus_backend package: OK`
- `horus_unity_bridge package: OK` on Jazzy/full bridge builds

:::warning
On ROS2 Humble, the installer may complete in **backend-only mode** due to missing GenericClient headers required by current `horus_unity_bridge`.
:::
