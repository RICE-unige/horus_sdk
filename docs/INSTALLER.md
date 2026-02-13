# HORUS Installer

This document defines the one-command installer flow for HORUS SDK + ROS2 backend.

## Bootstrap command

```bash
curl -fsSL https://raw.githubusercontent.com/RICE-unige/horus_sdk/main/install.sh | bash
```

## Supported hosts

- Ubuntu 24.04 (ROS 2 Jazzy default)
- Ubuntu 22.04 (ROS 2 Humble default)
- WSL2 Ubuntu with the same mapping

## Default install layout

- `~/horus/sdk` -> `horus_sdk`
- `~/horus/ros2` -> `horus_ros2`
- `~/horus/bin` -> helper commands
- `~/horus/logs` -> installer/runtime logs
- `~/horus/state` -> installer state

## Default behavior

- Channel: `stable` (pinned refs from `install/manifest/releases.json`)
- Bridge mode: WebRTC ON
- Startup mode: manual (`horus-start`)
- Python setup: virtualenv in `~/horus/sdk/.venv` with `--system-site-packages` so apt ROS Python modules are available
- Shell integration: idempotent `~/.bashrc` block that sets `HORUS_HOME` and adds `~/horus/bin` to `PATH` (no global venv activation)
- Python UX: `python3` is shimmed via `~/horus/bin/python3` to auto-use HORUS env for scripts under `~/horus/sdk`, while forwarding non-HORUS usage to system Python

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

## Helper commands

After install:

- `horus-status`
- `horus-start`
- `horus-stop`
- `horus-update`
- `horus-python`
- `horus-uninstall`

Uninstall usage:

```bash
horus-uninstall
# non-interactive
horus-uninstall --yes
# optional: also purge ROS apt packages installed by installer
horus-uninstall --yes --purge-ros-packages
```

Use environment on-demand:

```bash
# one command with HORUS env
horus-python path/to/script.py
# or manual session-scoped activation
source ~/horus/bin/horus-env
```

## Notes

- If target repos contain local uncommitted changes, installer aborts safely.
- If Humble build hits known `GenericClient` incompatibility for bridge package, installer retries backend build without unity bridge packages and prints warnings.
