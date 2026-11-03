#!/usr/bin/env bash
# Platform details are also consumed by install/install.sh.
# shellcheck disable=SC2034
OS_ID=""
OS_VERSION=""
OS_CODENAME=""
IS_WSL=0
BASE_PACKAGES=(
  git curl ca-certificates build-essential cmake pkg-config
  python3 python3-pip python3-venv python3-rosdep
  python3-colcon-common-extensions python3-vcstool
  nlohmann-json3-dev libunwind-dev libssl-dev
)
ROS_RUNTIME_PACKAGES=()
WEBRTC_PACKAGES=(
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
  libgstreamer-plugins-bad1.0-dev gstreamer1.0-tools
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
  gstreamer1.0-libav gstreamer1.0-nice libopencv-dev
)

_apt_update_silent() {
  run_silent "Updating apt index" sudo env DEBIAN_FRONTEND=noninteractive \
    apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30 \
    -o APT::Update::Error-Mode=any update -qq
}

_apt_install_silent() {
  local description="$1"
  shift
  run_silent "$description" sudo env DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC \
    apt-get -o DPkg::Lock::Timeout=120 -o Acquire::Retries=3 \
    -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30 install -y --no-install-recommends "$@"
}

detect_platform() {
  [ -f /etc/os-release ] || die "Missing /etc/os-release"
  # shellcheck disable=SC1091
  . /etc/os-release
  OS_ID="${ID,,}"
  OS_VERSION="${VERSION_ID:-unknown}"
  OS_CODENAME="${VERSION_CODENAME:-${UBUNTU_CODENAME:-}}"
  if is_wsl; then IS_WSL=1; fi
  [ "$OS_ID" = ubuntu ] && [ "$OS_CODENAME" = noble ] ||
    die "Full HORUS installation requires Ubuntu 24.04 (Noble) with ROS 2 Jazzy. Humble lacks the bridge GenericClient API; no backend-only fallback is installed."
  case "$(dpkg --print-architecture)" in
    amd64|arm64) ;;
    *) die "ROS 2 Jazzy binary installation requires amd64 or arm64" ;;
  esac
}

resolve_ros_distro() {
  [ "$OS_CODENAME" = noble ] || die "Jazzy requires Ubuntu Noble"
  case "$1" in
    auto|jazzy) printf jazzy ;;
    *) die "The full HORUS bridge requires Jazzy; '$1' is unsupported by this installer." ;;
  esac
}

ensure_sudo() {
  command_exists sudo || die "Install sudo before running the installer"
  sudo -v || die "Could not obtain sudo access"
}

setup_ros_apt_repository() (
  set -e
  local codename="$1" version tmp
  _apt_update_silent
  _apt_install_silent "Installing repository prerequisites" curl ca-certificates software-properties-common
  run_silent "Enabling Ubuntu Universe" sudo env DEBIAN_FRONTEND=noninteractive add-apt-repository -y universe
  _apt_update_silent
  # A working existing ROS repository is left unchanged, including its keys.
  if apt-cache policy ros-jazzy-ros-base | grep -Eq 'Candidate: [0-9]'; then
    _log_to_file "Using the existing ROS apt repository"
    return 0
  fi
  # Follow ROS's maintained source-package setup instead of rewriting apt files.
  tmp="$(mktemp -d -t horus-ros-source.XXXXXXXX)"
  trap 'rm -rf -- "$tmp"' EXIT
  run_silent "Resolving ROS repository package" curl --fail --show-error --location \
    --connect-timeout 15 --max-time 120 --retry 2 \
    https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest -o "$tmp/release.json"
  version="$(/usr/bin/python3 -c 'import json,sys,re; v=json.load(open(sys.argv[1]))["tag_name"]; assert re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+",v),v; print(v)' "$tmp/release.json")"
  run_silent "Downloading ROS repository package $version" curl --fail --show-error --location \
    --connect-timeout 15 --max-time 120 --retry 2 \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$version/ros2-apt-source_$version.${codename}_all.deb" \
    -o "$tmp/ros2-apt-source.deb"
  run_silent "Installing ROS repository configuration" sudo dpkg -i "$tmp/ros2-apt-source.deb"
  _apt_update_silent
)

build_ros_runtime_packages() {
  local distro="$1"
  ROS_RUNTIME_PACKAGES=(
    "ros-$distro-ros-base" "ros-$distro-rclpy" "ros-$distro-std-msgs"
    "ros-$distro-geometry-msgs" "ros-$distro-sensor-msgs" "ros-$distro-nav-msgs"
    "ros-$distro-tf2-ros" "ros-$distro-tf2-msgs" "ros-$distro-std-srvs"
    "ros-$distro-example-interfaces" "ros-$distro-ament-cmake" "ros-$distro-rclcpp"
  )
}

install_base_dependencies() {
  _apt_install_silent "Installing development tools" "${BASE_PACKAGES[@]}"
}

install_ros_dependencies() {
  build_ros_runtime_packages "$1"
  _apt_install_silent "Installing ROS runtime" "${ROS_RUNTIME_PACKAGES[@]}"
}

install_webrtc_dependencies() {
  _apt_install_silent "Installing WebRTC/media dependencies" "${WEBRTC_PACKAGES[@]}"
}

init_rosdep() {
  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    run_silent "Initializing rosdep" sudo rosdep init
  fi
  run_silent "Updating rosdep" rosdep update --rosdistro "$1"
}

print_wsl_guidance() {
  if [ "$IS_WSL" -eq 1 ]; then
    log_warn "WSL2: the Quest must reach the bridge at the Windows/WSL LAN address."
    log_warn "Routing depends on your WSL version (NAT or mirrored). See the installer networking guide."
    log_info "No firewall, network mode, ROS_DOMAIN_ID or ROS_LOCALHOST_ONLY setting was changed."
  fi
}
