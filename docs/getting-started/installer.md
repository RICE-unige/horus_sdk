---
title: Installer
sidebar_position: 3
---

# Installer

The installer sets up the Python SDK, the ROS 2 bridge and a small set of helper
commands in a dedicated directory. It does not start a robot, launch a bridge,
change network settings or activate a Python environment in every terminal.

## Requirements

- Ubuntu 24.04, native or WSL2, with an amd64 or arm64 CPU.
- A normal user account with sudo access.
- Bash, Git, curl and Ubuntu's `python3`.
- Internet access to Ubuntu/ROS repositories, GitHub and PyPI.
- Free disk and memory for a C++ Release build. Compilation defaults to two jobs.

The full bridge requires **ROS 2 Jazzy**. This installer rejects Ubuntu 22.04 /
Humble rather than reporting success with an incomplete, backend-only runtime.
It installs ROS Base, not the RViz/desktop metapackage.

## Install

Download the entry point and inspect the proposed configuration first:

```bash
curl -fL --connect-timeout 15 --max-time 120 \
  https://raw.githubusercontent.com/RICE-unige/horus_sdk/main/install.sh \
  -o install-horus.sh
bash install-horus.sh --check
bash install-horus.sh
```

For unattended installation:

```bash
bash install-horus.sh --yes --install-root "$HOME/horus" --shell-config manual
```

Do not run the installer itself with `sudo`. It requests elevated privileges only
for system dependencies. Without a terminal, `--yes` is required.

The stable channel selects the exact source commits in the release manifest.
The main channel tracks development heads and is not a release guarantee.
A downloaded **release installer artifact** pins the installer and SDK to that
release's commit; it also uses the manifest's pinned ROS bridge revision.
System apt packages and Python dependencies are resolved at installation time,
so these are source-pinned installations, not fully hermetic builds.

Managed installs build a regular Python package inside their dedicated environment,
not an editable source import. After editing SDK source, rerun the installer/update
workflow to replace the installed package. For deliberate development checkouts,
`python -m pip install -e .` remains available in your own environment.

The release pipeline gates packaging on SDK regressions for Python 3.10 and 3.12.
It separately installs the wheel and source distribution into clean temporary
environments outside the checkout, verifies bundled CUDA sources and the typing
marker, and runs public API, costmap and Teach recording tests against those
installed packages. Installer validation runs with WebRTC enabled and disabled.
These checks do not substitute for device and physical-robot qualification.

## Options

| Option | Meaning |
| --- | --- |
| `--check` | Validate platform, paths, manifest and local checkout state without installing. Does not verify remote refs, build the bridge or test connectivity. |
| `--yes` | Use the supplied/default configuration without interactive questions. |
| `--install-root PATH` | Dedicated installation directory; default `~/horus`. |
| `--channel stable\|main` | Pinned release-manifest sources or development heads. |
| `--ros-distro auto\|jazzy` | Jazzy on Ubuntu 24.04. |
| `--webrtc on\|off` | Include media dependencies and compile WebRTC support; default on. |
| `--shell-config auto\|manual` | Add helpers to Bash PATH, or leave shell startup unchanged. |
| `--log-file PATH` | Append output to a chosen log file. |
| `--help` | Print usage. |

Set `HORUS_BUILD_JOBS` to a positive integer to change C++ build parallelism.
Existing virtual environments are reused only when compatible with Ubuntu's
Python 3.12 and ROS packages; incompatible environments are not erased.

## Layout and verification

| Path under the installation root | Contents |
| --- | --- |
| `sdk/` | SDK checkout and project-specific `.venv/` |
| `ros2/` | ROS source checkout and colcon build/install/log outputs |
| `bin/` | HORUS helper commands |
| `lib/installer/` | Helper implementation |
| `logs/` | Installation logs, including failing command output |
| `state/` | Configuration, last successful install's commit IDs, current `install.status` and process ownership |

After installation:

```bash
"$HOME/horus/bin/horus-status"
"$HOME/horus/bin/horus-python" -c 'import horus, rclpy; print("SDK and ROS imports OK")'
```

With automatic shell integration, open a new terminal to use the short command
names. Normal `python3` remains unchanged. Use `horus-python` for SDK scripts, or
explicitly activate the environment:

```bash
source "$HOME/horus/bin/horus-env"
```

The installer checks SDK/ROS imports, package locations and bridge executables.
With WebRTC enabled, it also checks the required media plugins. This verifies
installation, **not** headset connectivity, video latency or a robot's interfaces.

## Helper commands

| Command | Behavior |
| --- | --- |
| `horus-python ...` | Run the SDK Python with its ROS environment. |
| `horus-status` | Check imports and package locations; fail if the installation is incomplete. Report the helper-owned launch separately. |
| `horus-start [launch arguments]` | Run the backend launch in the foreground. Reject a second helper launch in the same installation. |
| `horus-stop` | Stop only the launch process group created by this installation's `horus-start`. |
| `horus-update` | Fetch the current installer/manifest and update the selected channel. Refuse dirty, divergent or ahead checkouts. |
| `horus-uninstall [--yes]` | Remove this installation's Bash integration and rename its directory to a sibling backup. Preserve all files and shared apt packages. |

Normal registration examples **auto-start the bridge**. Do not also start it
manually unless debugging or intentionally managing the backend separately.
Examples still need their documented robot or simulation publishers.

## Updates and recovery

Stop running registrations and helper launches before updating. The installer
does not kill an unrelated bridge or service just to make an installation succeed.

```bash
horus-stop
horus-update
```

Local commits, modified files and a changed Git remote are never reset or replaced.
Keep development checkouts separate from the installed runtime copies. If the
installer refuses an update, preserve your work and use a new installation root
or reconcile the checkout yourself.

Interrupted installations retain their logs and an incomplete status. Rerunning
the installer against the same managed root retries the setup; it does not delete
the virtual environment or colcon outputs. Helpers refuse an incomplete runtime.
Installation is not an atomic rollback of apt packages or both repositories.

Shell changes are backed up alongside `~/.bashrc`. An incomplete or malformed
HORUS block is reported rather than deleting the remainder of the file.

To remove the helper installation:

```bash
horus-stop
horus-uninstall
```

The command prints the retained backup directory. Review it before deleting it,
especially if you stored assets, experiments or local code inside the install
root. Apt/ROS packages may be shared by other projects and are not removed.

## WSL2 and networking

The Quest must be able to reach the bridge on the configured host/port (normally
TCP 10000). NAT and mirrored WSL networking need different routing arrangements;
mirrored networking is an option only on Windows/WSL versions that support it,
not an installer requirement.

The installer does not change firewall rules, networking mode, `ROS_DOMAIN_ID`,
`ROS_LOCALHOST_ONLY`, VPNs or DDS settings. Select these for your deployment.
See the [official ROS installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
for the supported Ubuntu/ROS pairing and repository setup.
