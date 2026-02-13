#!/usr/bin/env bash
# =============================================================================
# HORUS Installer - Runtime Setup & Workspace Build
# =============================================================================

HORUS_INSTALL_RUNTIME_MODE="full"
HORUS_INSTALL_RUNTIME_REASON=""

source_setup_file() {
  local setup_file="$1"
  local had_nounset=0

  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi

  # shellcheck disable=SC1090
  source "$setup_file"

  if [ "$had_nounset" -eq 1 ]; then
    set -u
  fi
}

setup_python_sdk_runtime() {
  local sdk_dir="$1"

  [ -d "$sdk_dir" ] || die "SDK directory missing: $sdk_dir"

  run_silent "Creating Python virtual environment" \
    python3 -m venv --clear --system-site-packages "$sdk_dir/.venv"

  local pip_bin="$sdk_dir/.venv/bin/pip"
  local python_bin="$sdk_dir/.venv/bin/python"

  run_silent "Upgrading pip and setuptools" \
    "$pip_bin" install --upgrade pip "setuptools<80" wheel

  run_silent "Installing Python dependencies" \
    "$pip_bin" install \
      "numpy>=1.21.0" \
      "dataclasses-json>=0.5.0" \
      "rich>=13.0.0"

  run_silent "Installing HORUS SDK package" \
    "$pip_bin" install -e "$sdk_dir" --no-deps

  run_silent "Verifying HORUS import" \
    "$python_bin" -c "import horus; print('horus import OK')"
}

_clear_colcon_artifacts() {
  local ros2_dir="$1"
  rm -rf "$ros2_dir/build" "$ros2_dir/install" "$ros2_dir/log"
}

_humble_has_generic_client() {
  local distro="$1"
  [ -f "/opt/ros/$distro/include/rclcpp/rclcpp/generic_client.hpp" ]
}

build_ros2_backend_workspace() {
  local ros2_dir="$1"
  local distro="$2"
  local webrtc="$3"

  HORUS_INSTALL_RUNTIME_MODE="full"
  HORUS_INSTALL_RUNTIME_REASON=""

  [ -d "$ros2_dir" ] || die "ROS2 directory missing: $ros2_dir"
  [ -f "/opt/ros/$distro/setup.bash" ] || die "ROS setup missing: /opt/ros/$distro/setup.bash"

  source_setup_file "/opt/ros/$distro/setup.bash"

  local webrtc_flag="OFF"
  if [ "$webrtc" = "on" ]; then
    webrtc_flag="ON"
  fi

  run_silent "Installing ROS workspace dependencies via rosdep" \
    bash -c "cd '$ros2_dir' && rosdep install --from-paths . -y --ignore-src --rosdistro '$distro' --skip-keys 'nlohmann-json3-dev'"

  # ROS 2 Humble does not ship rclcpp GenericClient headers used by horus_unity_bridge.
  # Build backend-only on Humble to avoid a noisy fail->retry flow and stale setup artifacts.
  if [ "$distro" = "humble" ] && ! _humble_has_generic_client "$distro"; then
    HORUS_INSTALL_RUNTIME_MODE="backend_only"
    HORUS_INSTALL_RUNTIME_REASON="ROS 2 Humble is missing rclcpp GenericClient headers required by horus_unity_bridge."
    log_warn "$HORUS_INSTALL_RUNTIME_REASON"
    log_warn "Proceeding with backend-only workspace build on Humble."

    _clear_colcon_artifacts "$ros2_dir"
    run_silent "Compiling ROS 2 workspace (Humble backend-only)" \
      bash -c "cd '$ros2_dir' && colcon build --symlink-install --event-handlers console_direct+ --packages-skip horus_unity_bridge horus_unity_bridge_test --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_WEBRTC=$webrtc_flag" \
      || die "Humble backend-only build failed"
    return 0
  fi

  if ! run_silent "Compiling ROS 2 workspace (ENABLE_WEBRTC=$webrtc_flag)" \
    bash -c "cd '$ros2_dir' && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_WEBRTC=$webrtc_flag"; then

    # Humble fallback: retry without unity bridge if GenericClient is missing
    if [ "$distro" = "humble" ] && ! _humble_has_generic_client "$distro"; then
      HORUS_INSTALL_RUNTIME_MODE="backend_only"
      HORUS_INSTALL_RUNTIME_REASON="ROS 2 Humble is missing rclcpp GenericClient headers required by horus_unity_bridge."
      log_warn "Humble lacks GenericClient headers. Retrying without unity bridge packages."
      _clear_colcon_artifacts "$ros2_dir"
      run_silent "Compiling ROS 2 workspace (Humble fallback)" \
        bash -c "cd '$ros2_dir' && colcon build --symlink-install --event-handlers console_direct+ --packages-skip horus_unity_bridge horus_unity_bridge_test --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_WEBRTC=$webrtc_flag" \
        || die "Fallback Humble build failed"
    else
      die "ROS2 workspace build failed"
    fi
  fi
}

validate_runtime_install() {
  local install_root="$1"
  local distro="$2"
  local runtime_mode="${3:-full}"

  local ros2_setup="$install_root/ros2/install/setup.bash"
  [ -f "$ros2_setup" ] || die "Expected ROS2 install setup not found: $ros2_setup"

  source_setup_file "/opt/ros/$distro/setup.bash"
  source_setup_file "$ros2_setup"

  if ! ros2 pkg prefix horus_backend >/dev/null 2>&1; then
    die "horus_backend package not found after build"
  fi

  if [ "$runtime_mode" = "full" ]; then
    if ! ros2 pkg prefix horus_unity_bridge >/dev/null 2>&1; then
      die "horus_unity_bridge package not found after build"
    fi
  else
    log_warn "Unity bridge is unavailable in this install (backend-only mode)."
  fi
}
