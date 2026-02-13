---
title: Installer
sidebar_position: 3
---

# Installer Deep Dive

This page documents the curated installer flow for reproducible SDK + ROS2 runtime setup.

## Bootstrap

```bash
curl -fsSL https://raw.githubusercontent.com/RICE-unige/horus_sdk/main/install.sh | bash
```

## Flags

```bash
bash install.sh --help
```

Main options:

- `--yes`
- `--channel stable|main`
- `--install-root <path>`
- `--ros-distro auto|jazzy|humble`
- `--webrtc on|off`
- `--shell-config auto|manual`
- `--log-file <path>`

## Runtime helpers installed

- `horus-status`
- `horus-start`
- `horus-stop`
- `horus-update`
- `horus-python`
- `horus-uninstall`

## Humble behavior

When ROS2 Humble is selected, installer may switch to **backend-only build mode** if the Unity bridge cannot be built with current headers.

You will see:

- warning during build phase,
- warning-aware completion summary,
- runtime note explaining that Unity bridge was not built.

For full backend + Unity bridge support, use ROS2 Jazzy.

## WSL2 networking note (Meta Quest)

If Meta Quest cannot reach bridge ports from WSL:

```ini
[wsl2]
networkingMode=mirrored
```

Then run from Windows PowerShell:

```powershell
wsl --shutdown
```

Restart WSL and re-test connectivity.
