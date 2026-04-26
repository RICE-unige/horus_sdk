---
title: Installer
sidebar_position: 3
---

# Installer

Use the installer when you want a reproducible HORUS SDK plus `horus_ros2` setup without manually shaping the workspace.

## Bootstrap

```bash
curl -fsSL https://raw.githubusercontent.com/RICE-unige/horus_sdk/main/install.sh | bash
```

## Common flags

```bash
bash install.sh --help
```

Flags you will use most often:

- `--yes`
- `--channel stable|main`
- `--install-root <path>`
- `--ros-distro auto|jazzy|humble`
- `--webrtc on|off`
- `--shell-config auto|manual`

## Installed helper commands

- `horus-status`
- `horus-start`
- `horus-stop`
- `horus-update`
- `horus-python`
- `horus-uninstall`

## How it fits the curated examples

The examples documented on this site do not require you to launch the bridge manually in the common case. The registration client can auto-start `horus_unity_bridge` from the active ROS shell or from the installed HORUS helper environment.

Use manual bridge launch when:

- you are debugging bridge startup
- you want to pin a custom launch file
- you intentionally disabled SDK bridge auto-start

## WSL2 note

If the headset cannot reach WSL-hosted ports, use mirrored networking:

```ini
[wsl2]
networkingMode=mirrored
```

Then restart WSL:

```powershell
wsl --shutdown
```
