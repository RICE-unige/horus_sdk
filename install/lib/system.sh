#!/usr/bin/env bash
# =============================================================================
# HORUS Installer - System Detection & Dependencies
# =============================================================================

OS_ID=""
OS_VERSION=""
OS_CODENAME=""
IS_WSL=0

BASE_PACKAGES=(
  software-properties-common
  curl
  git
  ca-certificates
  gnupg
  lsb-release
  build-essential
  cmake
  pkg-config
  python3
  python3-pip
  python3-venv
  python3-rosdep
  python3-colcon-common-extensions
  python3-vcstool
  nlohmann-json3-dev
  libunwind-dev
)

ROS_RUNTIME_PACKAGES=()
WEBRTC_PACKAGES=(
  libgstreamer1.0-dev
  libgstreamer-plugins-base1.0-dev
  libgstreamer-plugins-bad1.0-dev
  gstreamer1.0-plugins-base
  gstreamer1.0-plugins-good
  gstreamer1.0-plugins-bad
  gstreamer1.0-plugins-ugly
  gstreamer1.0-libav
  libopencv-dev
)

# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

_apt_update_silent() {
  run_silent "Updating apt package index" \
    sudo env DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get update -qq
}

_apt_install_silent() {
  local description="$1"
  shift
  run_silent "$description" \
    sudo env DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get install -y -qq "$@"
}

# ---------------------------------------------------------------------------
# Platform detection
# ---------------------------------------------------------------------------

detect_platform() {
  [ -f /etc/os-release ] || die "Missing /etc/os-release; unsupported host"

  # shellcheck disable=SC1091
  . /etc/os-release

  OS_ID="${ID,,}"
  OS_VERSION="${VERSION_ID:-unknown}"
  OS_CODENAME="${VERSION_CODENAME:-${UBUNTU_CODENAME:-}}"

  if is_wsl; then
    IS_WSL=1
  else
    IS_WSL=0
  fi

  if [ "$OS_ID" != "ubuntu" ]; then
    die "Unsupported OS '$OS_ID'. Installer currently supports Ubuntu/Ubuntu-on-WSL2 only."
  fi

  if [ "$OS_CODENAME" != "noble" ] && [ "$OS_CODENAME" != "jammy" ]; then
    die "Unsupported Ubuntu codename '$OS_CODENAME'. Supported: noble (24.04), jammy (22.04)."
  fi
}

resolve_ros_distro() {
  local requested="$1"
  case "$requested" in
    jazzy|humble)
      printf "%s" "$requested"
      return 0
      ;;
    auto)
      if [ "$OS_CODENAME" = "noble" ]; then
        printf "jazzy"
      elif [ "$OS_CODENAME" = "jammy" ]; then
        printf "humble"
      else
        die "Cannot auto-resolve ROS distro for codename '$OS_CODENAME'"
      fi
      return 0
      ;;
    *)
      die "Invalid ROS distro '$requested'"
      ;;
  esac
}

# ---------------------------------------------------------------------------
# Sudo
# ---------------------------------------------------------------------------

ensure_sudo() {
  if ! command_exists sudo; then
    die "sudo is required but not installed"
  fi
  if ! sudo -v; then
    die "Failed to obtain sudo access"
  fi
}

# ---------------------------------------------------------------------------
# ROS apt repository
# ---------------------------------------------------------------------------

setup_ros_apt_repository() {
  local codename="$1"
  local keyring="/usr/share/keyrings/ros-archive-keyring.gpg"
  local list_file="/etc/apt/sources.list.d/ros2.list"
  local ros_uri="packages.ros.org/ros2/ubuntu"

  _apt_update_silent
  _apt_install_silent "Installing apt prerequisites" software-properties-common

  run_silent "Adding universe repository" \
    bash -c 'sudo env DEBIAN_FRONTEND=noninteractive add-apt-repository -y universe >/dev/null 2>&1 || true'

  run_silent "Downloading ROS 2 GPG key" \
    sudo bash -c "mkdir -p /usr/share/keyrings && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o $keyring"

  # Older/hand-written ROS source entries can conflict on Signed-By and break apt update.
  # Remove only ROS 2 source stanzas, then write one canonical entry.
  run_silent "Cleaning conflicting ROS 2 apt source entries" \
    sudo python3 - "$ros_uri" "$list_file" <<'PY'
import pathlib
import re
import shutil
import sys
import time

ros_uri = sys.argv[1]
target_list = pathlib.Path(sys.argv[2]).resolve()
timestamp = time.strftime("%Y%m%d%H%M%S")

def backup(path: pathlib.Path) -> None:
    backup_path = path.with_name(f"{path.name}.bak.horus.{timestamp}")
    shutil.copy2(path, backup_path)

def clean_list_file(path: pathlib.Path) -> None:
    if not path.exists() or not path.is_file():
        return

    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines(keepends=True)
    changed = False
    out = []
    for line in lines:
        stripped = line.lstrip()
        if stripped.startswith("#"):
            out.append(line)
            continue
        if ros_uri in line:
            changed = True
            continue
        out.append(line)

    if changed:
        backup(path)
        path.write_text("".join(out), encoding="utf-8")

def clean_sources_file(path: pathlib.Path) -> None:
    if not path.exists() or not path.is_file():
        return

    content = path.read_text(encoding="utf-8", errors="ignore")
    if ros_uri not in content:
        return

    paragraphs = re.split(r"\n\s*\n", content.strip(), flags=re.MULTILINE)
    kept = []
    changed = False
    for p in paragraphs:
        if ros_uri in p:
            changed = True
            continue
        kept.append(p.strip())

    if changed:
        backup(path)
        if kept:
            path.write_text("\n\n".join(kept) + "\n", encoding="utf-8")
        else:
            path.unlink()

def glob_paths(pattern: str):
    return sorted(pathlib.Path("/").glob(pattern))

clean_list_file(pathlib.Path("/etc/apt/sources.list"))
for p in glob_paths("etc/apt/sources.list.d/*.list"):
    if p.resolve() == target_list:
        if p.exists():
            p.unlink()
        continue
    clean_list_file(p)

for p in glob_paths("etc/apt/sources.list.d/*.sources"):
    clean_sources_file(p)
PY

  local arch
  arch="$(dpkg --print-architecture)"
  printf "deb [arch=%s signed-by=%s] http://packages.ros.org/ros2/ubuntu %s main\n" \
    "$arch" "$keyring" "$codename" | sudo tee "$list_file" >/dev/null

  _apt_update_silent
}

# ---------------------------------------------------------------------------
# Package installation (split into separate callable functions)
# ---------------------------------------------------------------------------

build_ros_runtime_packages() {
  local distro="$1"
  ROS_RUNTIME_PACKAGES=(
    "ros-${distro}-desktop"
    "ros-${distro}-rclpy"
    "ros-${distro}-std-msgs"
    "ros-${distro}-geometry-msgs"
    "ros-${distro}-sensor-msgs"
    "ros-${distro}-nav-msgs"
    "ros-${distro}-tf2-ros"
    "ros-${distro}-tf2-msgs"
    "ros-${distro}-std-srvs"
    "ros-${distro}-example-interfaces"
    "ros-${distro}-ament-cmake"
    "ros-${distro}-rclcpp"
  )
}

install_base_dependencies() {
  _apt_install_silent "Installing base development tools" "${BASE_PACKAGES[@]}"
}

install_ros_dependencies() {
  local distro="$1"
  build_ros_runtime_packages "$distro"
  _apt_install_silent "Installing ROS 2 runtime packages" "${ROS_RUNTIME_PACKAGES[@]}"
}

install_webrtc_dependencies() {
  _apt_install_silent "Installing GStreamer and media packages" "${WEBRTC_PACKAGES[@]}"
}

# ---------------------------------------------------------------------------
# rosdep
# ---------------------------------------------------------------------------

init_rosdep() {
  local distro="$1"
  sudo rosdep init >/dev/null 2>&1 || true
  run_silent "Updating rosdep index" rosdep update --rosdistro "$distro"
}

# ---------------------------------------------------------------------------
# WSL guidance
# ---------------------------------------------------------------------------

print_wsl_guidance() {
  if [ "$IS_WSL" -eq 1 ]; then
    printf '\n'
    log_warn "WSL2 detected: Meta Quest-to-WSL networking may require explicit host/IP routing."
    log_warn "If Meta Quest cannot reach port 10000, enable mirrored networking in %USERPROFILE%\\.wslconfig:"
    log_warn "  [wsl2]"
    log_warn "  networkingMode=mirrored"
    log_warn "Then run 'wsl --shutdown' from Windows PowerShell and restart WSL."
    printf '\n'
  fi
}
