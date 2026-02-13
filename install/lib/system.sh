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

  _apt_update_silent
  _apt_install_silent "Installing apt prerequisites" software-properties-common

  run_silent "Adding universe repository" \
    bash -c 'sudo env DEBIAN_FRONTEND=noninteractive add-apt-repository -y universe >/dev/null 2>&1 || true'

  run_silent "Downloading ROS 2 GPG key" \
    sudo bash -c "mkdir -p /usr/share/keyrings && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o $keyring"


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
    log_warn "WSL2 detected: Unity-to-WSL networking may require explicit host/IP routing."
    log_warn "If Unity cannot reach port 10000, use WSL host IP instead of localhost."
    printf '\n'
  fi
}
