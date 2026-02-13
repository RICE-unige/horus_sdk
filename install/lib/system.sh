#!/usr/bin/env bash

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

sudo_noninteractive() {
  sudo env DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC "$@"
}

apt_update() {
  run_cmd sudo_noninteractive apt-get update
}

apt_install() {
  run_cmd sudo_noninteractive apt-get install -y "$@"
}

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

  log_info "Detected platform: Ubuntu $OS_VERSION ($OS_CODENAME)"
  if [ "$IS_WSL" -eq 1 ]; then
    log_info "Detected environment: WSL2"
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

ensure_sudo() {
  if ! command_exists sudo; then
    die "sudo is required but not installed"
  fi
  log_info "Validating sudo access"
  if ! sudo -v; then
    die "Failed to obtain sudo access"
  fi
}

setup_ros_apt_repository() {
  local codename="$1"
  local keyring="/usr/share/keyrings/ros-archive-keyring.gpg"
  local list_file="/etc/apt/sources.list.d/ros2.list"

  log_info "Configuring ROS 2 apt repository"
  apt_update
  apt_install software-properties-common
  run_cmd sudo_noninteractive add-apt-repository -y universe >/dev/null 2>&1 || true

  run_cmd sudo_noninteractive mkdir -p /usr/share/keyrings
  run_cmd sudo_noninteractive curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$keyring"

  local arch
  arch="$(dpkg --print-architecture)"
  printf "deb [arch=%s signed-by=%s] http://packages.ros.org/ros2/ubuntu %s main\n" \
    "$arch" "$keyring" "$codename" | sudo tee "$list_file" >/dev/null

  apt_update
}

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

install_dependencies() {
  local distro="$1"
  local webrtc="$2"

  build_ros_runtime_packages "$distro"

  log_info "Installing system dependencies"
  apt_install "${BASE_PACKAGES[@]}"
  apt_install "${ROS_RUNTIME_PACKAGES[@]}"

  if [ "$webrtc" = "on" ]; then
    log_info "Installing WebRTC/media dependencies"
    apt_install "${WEBRTC_PACKAGES[@]}"
  fi

  log_success "Dependency installation completed"
}

init_rosdep() {
  local distro="$1"
  log_info "Initializing rosdep"
  sudo rosdep init >/dev/null 2>&1 || true
  run_cmd rosdep update --rosdistro "$distro"
}

print_wsl_guidance() {
  if [ "$IS_WSL" -eq 1 ]; then
    log_warn "WSL2 detected: Unity-to-WSL networking may require explicit host/IP routing."
    log_warn "If Unity cannot reach port 10000, use WSL host IP instead of localhost from Windows apps."
  fi
}
