---
title: Installation
sidebar_position: 1
---

# Installation

## Supported environments

- Ubuntu 24.04 with ROS 2 Jazzy
- Ubuntu 22.04 with ROS 2 Humble
- WSL2 Ubuntu when Quest connectivity is handled correctly

## Choose an installation path

### Installer workflow

Use this when you want the standard HORUS layout and helper commands:

```bash
curl -fsSL https://raw.githubusercontent.com/RICE-unige/horus_sdk/main/install.sh | bash
```

This creates:

- `~/horus/sdk`
- `~/horus/ros2`
- `~/horus/bin`
- `~/horus/logs`
- `~/horus/state`

### Source checkout workflow

Use this when you are developing the SDK itself:

```bash
cd ~/horus_sdk
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
npm ci
```

## ROS 2 prerequisites

- `rclpy` and the standard ROS 2 message packages must come from the ROS installation, not from PyPI.
- `horus_ros2` should be available in `~/horus_ws/src/horus_ros2` or installed via the HORUS installer.
- For bridge auto-start from a source checkout, `~/horus_ws/install/setup.bash` must exist and be sourceable.

## Verify the installation

Installer-based layout:

```bash
horus-status
```

Source checkout:

```bash
cd ~/horus_sdk
PYTHONPATH=python python3 -m pytest -q
```

## Humble note

ROS 2 Humble remains supported for the SDK, but full bridge validation is still safer on Jazzy when you need current WebRTC-capable `horus_unity_bridge` behavior.
