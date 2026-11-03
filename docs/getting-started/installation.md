---
title: Installation
sidebar_position: 1
---

# Installation

## Supported environments

- Ubuntu 24.04 with ROS 2 Jazzy
- WSL2 Ubuntu 24.04 when Quest connectivity is handled correctly

The full installer targets Jazzy. Using the Python SDK with a separate, older
ROS bridge is a manual compatibility task, not a supported full Humble install.

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
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

Run `npm ci` separately only when developing the documentation website.

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

The current full bridge uses `rclcpp::GenericClient`, which is unavailable in
Humble. The installer rejects that configuration before changing the system;
it does not silently install a backend without the MR bridge.
