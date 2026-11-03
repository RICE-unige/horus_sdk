#!/usr/bin/env bash
# Build with Ubuntu's ROS-compatible Python, not an activated Conda/other ROS overlay.
clean_ros_command() {
  env -u PYTHONPATH -u PYTHONHOME -u VIRTUAL_ENV -u CONDA_PREFIX \
    -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    PYTHONNOUSERSITE=1 PIP_DISABLE_PIP_VERSION_CHECK=1 PIP_DEFAULT_TIMEOUT=60 \
    "$@"
}

source_setup_file() {
  local setup_file="$1" had_nounset=0 status=0
  if [[ $- == *u* ]]; then had_nounset=1; set +u; fi
  # shellcheck disable=SC1090
  source "$setup_file" || status=$?
  if [ "$had_nounset" -eq 1 ]; then set -u; fi
  return "$status"
}

setup_python_sdk_runtime() {
  local sdk_dir="$1" venv="$1/.venv"
  [ -f "$sdk_dir/pyproject.toml" ] || die "SDK packaging metadata missing: $sdk_dir"
  if [ -e "$venv" ]; then
    [ -x "$venv/bin/python" ] || die "Incomplete venv at $venv; rename it before retrying (it will not be deleted)."
    clean_ros_command "$venv/bin/python" -c \
      'import sys; assert sys.version_info[:2] == (3,12), "Jazzy requires Python 3.12"; assert sys.base_prefix == "/usr", "Use the Ubuntu Python runtime"' ||
      die "Existing venv is incompatible with Ubuntu ROS; choose another install root."
    grep -q '^include-system-site-packages = true$' "$venv/pyvenv.cfg" ||
      die "Existing venv cannot access ROS system packages; choose another install root."
  else
    run_silent "Creating Python environment" clean_ros_command /usr/bin/python3 -m venv --system-site-packages "$venv"
  fi
  run_silent "Installing SDK and declared dependencies" clean_ros_command "$venv/bin/python" \
    -m pip install --retries 2 "$sdk_dir"
  run_silent "Verifying SDK and ROS Python imports" clean_ros_command bash --noprofile --norc -e -c \
    'source "/opt/ros/$1/setup.bash"; "$2" -c "import horus, rclpy; from sensor_msgs.msg import Image"' _ "$ROS_DISTRO" "$venv/bin/python"
}

build_ros2_backend_workspace() {
  local ros2_dir="$1" distro="$2" webrtc="$3" jobs="${HORUS_BUILD_JOBS:-2}" flag=OFF
  [[ "$jobs" =~ ^[1-9][0-9]*$ ]] || die "HORUS_BUILD_JOBS must be a positive integer"
  [ -f "/opt/ros/$distro/include/rclcpp/rclcpp/generic_client.hpp" ] ||
    die "Selected ROS installation lacks GenericClient; a full bridge cannot be built."
  [ "$webrtc" != on ] || flag=ON
  run_silent "Installing workspace dependencies" clean_ros_command bash --noprofile --norc -e -c \
    'source "/opt/ros/$1/setup.bash"; cd "$2"; rosdep install --from-paths horus_interfaces horus_backend horus_unity_bridge --ignore-src -y --rosdistro "$1"' _ "$distro" "$ros2_dir"
  # The source checkout pins/verifies libdatachannel; enable that fallback explicitly.
  run_silent "Building ROS bridge ($jobs jobs, WebRTC $flag)" clean_ros_command bash --noprofile --norc -e -c \
    'source "/opt/ros/$1/setup.bash"; cd "$2"; export CMAKE_BUILD_PARALLEL_LEVEL="$3"; export MAKEFLAGS="-j$3"; /usr/bin/colcon build --symlink-install --executor sequential --event-handlers console_direct+ --packages-up-to horus_backend horus_unity_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "-DENABLE_WEBRTC=$4" "-DHORUS_ALLOW_LIBDATACHANNEL_FETCH=$4"' \
    _ "$distro" "$ros2_dir" "$jobs" "$flag"
}

validate_runtime_install() {
  local root="$1" distro="$2"
  [ -f "$root/ros2/install/setup.bash" ] || die "ROS workspace setup missing"
  run_silent "Checking installed package locations and executables" clean_ros_command bash --noprofile --norc -e -c '
    source "/opt/ros/$1/setup.bash"
    source "$2/ros2/install/local_setup.bash"
    for package in horus_backend horus_unity_bridge; do
      prefix="$(ros2 pkg prefix "$package")"
      case "$prefix" in "$2"/ros2/install/*) ;; *) echo "Wrong overlay: $prefix" >&2; exit 1 ;; esac
      test -n "$(ros2 pkg executables "$package")"
    done
    "$2/sdk/.venv/bin/python" -c "import horus, rclpy; from horus_interfaces.msg import RobotState"
  ' _ "$distro" "$root"
  if [ "$WEBRTC" = on ]; then
    local plugin
    for plugin in nicesrc x264enc rtph264pay; do
      run_silent "Checking media plugin $plugin" gst-inspect-1.0 "$plugin"
    done
  fi
}
