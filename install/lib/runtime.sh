#!/usr/bin/env bash

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

  log_info "Setting up Python virtual environment"
  run_cmd python3 -m venv "$sdk_dir/.venv"

  local pip_bin="$sdk_dir/.venv/bin/pip"
  local python_bin="$sdk_dir/.venv/bin/python"

  run_cmd "$pip_bin" install --upgrade pip setuptools wheel
  run_cmd "$pip_bin" install -e "$sdk_dir" --no-deps
  run_cmd "$python_bin" -c "import horus; print('horus import OK')"

  log_success "Python SDK runtime is ready"
}

build_ros2_backend_workspace() {
  local ros2_dir="$1"
  local distro="$2"
  local webrtc="$3"

  [ -d "$ros2_dir" ] || die "ROS2 directory missing: $ros2_dir"
  [ -f "/opt/ros/$distro/setup.bash" ] || die "ROS setup missing: /opt/ros/$distro/setup.bash"

  source_setup_file "/opt/ros/$distro/setup.bash"

  local webrtc_flag="OFF"
  if [ "$webrtc" = "on" ]; then
    webrtc_flag="ON"
  fi

  log_info "Installing ROS package dependencies via rosdep"
  (
    cd "$ros2_dir"
    run_cmd rosdep install --from-paths . -y --ignore-src --rosdistro "$distro" --skip-keys "nlohmann-json3-dev"
  )

  log_info "Building HORUS ROS2 workspace (ENABLE_WEBRTC=$webrtc_flag)"
  if ! (
    cd "$ros2_dir"
    colcon build --symlink-install --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release "-DENABLE_WEBRTC=$webrtc_flag"
  ); then
    if [ "$distro" = "humble" ] && [ ! -f "/opt/ros/$distro/include/rclcpp/rclcpp/generic_client.hpp" ]; then
      log_warn "Humble lacks GenericClient headers used by horus_unity_bridge/main."
      log_warn "Retrying build without unity bridge packages on Humble."
      (
        cd "$ros2_dir"
        colcon build --symlink-install --event-handlers console_direct+ \
          --packages-skip horus_unity_bridge horus_unity_bridge_test \
          --cmake-args -DCMAKE_BUILD_TYPE=Release "-DENABLE_WEBRTC=$webrtc_flag"
      ) || die "Fallback Humble build failed"
    else
      die "ROS2 workspace build failed"
    fi
  fi

  log_success "ROS2 workspace build completed"
}

validate_runtime_install() {
  local install_root="$1"
  local distro="$2"

  local ros2_setup="$install_root/ros2/install/setup.bash"
  [ -f "$ros2_setup" ] || die "Expected ROS2 install setup not found: $ros2_setup"

  source_setup_file "/opt/ros/$distro/setup.bash"
  source_setup_file "$ros2_setup"

  if ! ros2 pkg prefix horus_backend >/dev/null 2>&1; then
    die "horus_backend package not found after build"
  fi

  log_success "Runtime validation checks passed"
}
