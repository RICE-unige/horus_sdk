#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/common.sh"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/args.sh"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/system.sh"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/repos.sh"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/runtime.sh"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/helpers.sh"

bootstrap_log() {
  local early_log="/tmp/horus-installer-$(date +%Y%m%d-%H%M%S).log"
  if [ -z "$LOG_FILE" ]; then
    LOG_FILE="$early_log"
  fi
}

ask_interactive_defaults() {
  if [ "$ASSUME_YES" -eq 1 ]; then
    return 0
  fi

  log_info "Interactive setup: confirm installation options"

  prompt_with_default_into INSTALL_ROOT "Install root" "$INSTALL_ROOT"
  INSTALL_ROOT="$(normalize_path "$INSTALL_ROOT")"

  prompt_with_default_into CHANNEL "Channel (stable/main)" "$CHANNEL"
  CHANNEL="${CHANNEL,,}"

  prompt_with_default_into ROS_DISTRO "ROS distro (auto/jazzy/humble)" "$ROS_DISTRO"
  ROS_DISTRO="${ROS_DISTRO,,}"

  prompt_with_default_into WEBRTC "Build WebRTC bridge (on/off)" "$WEBRTC"
  WEBRTC="${WEBRTC,,}"

  prompt_with_default_into SHELL_CONFIG "Shell integration (auto/manual)" "$SHELL_CONFIG"
  SHELL_CONFIG="${SHELL_CONFIG,,}"

  case "$CHANNEL" in
    stable|main) ;;
    *) die "Invalid channel: $CHANNEL" ;;
  esac
  case "$ROS_DISTRO" in
    auto|jazzy|humble) ;;
    *) die "Invalid ROS distro: $ROS_DISTRO" ;;
  esac
  case "$WEBRTC" in
    on|off) ;;
    *) die "Invalid WebRTC value: $WEBRTC" ;;
  esac
  case "$SHELL_CONFIG" in
    auto|manual) ;;
    *) die "Invalid shell config: $SHELL_CONFIG" ;;
  esac
}

setup_logging() {
  mkdir -p "$(dirname "$LOG_FILE")"
  exec > >(tee -a "$LOG_FILE") 2>&1
  log_info "Installer version: $HORUS_INSTALLER_VERSION"
  log_info "Log file: $LOG_FILE"
}

main() {
  parse_args "$@"
  bootstrap_log

  detect_platform
  ROS_DISTRO="$(resolve_ros_distro "$ROS_DISTRO")"
  ask_interactive_defaults
  ROS_DISTRO="$(resolve_ros_distro "$ROS_DISTRO")"

  prepare_install_root "$INSTALL_ROOT" "$ASSUME_YES"

  if [ -z "$LOG_FILE" ] || [[ "$LOG_FILE" == /tmp/horus-installer-* ]]; then
    LOG_FILE="$INSTALL_ROOT/logs/install-$(date +%Y%m%d-%H%M%S).log"
  fi
  setup_logging

  log_info "Resolved settings"
  log_info "  install_root: $INSTALL_ROOT"
  log_info "  channel:      $CHANNEL"
  log_info "  ros_distro:   $ROS_DISTRO"
  log_info "  webrtc:       $WEBRTC"
  log_info "  shell_config: $SHELL_CONFIG"

  ensure_sudo
  setup_ros_apt_repository "$OS_CODENAME"
  install_dependencies "$ROS_DISTRO" "$WEBRTC"
  init_rosdep "$ROS_DISTRO"

  local manifest_path="$REPO_ROOT/install/manifest/releases.json"
  parse_manifest_channel "$manifest_path" "$CHANNEL"

  if [ -n "$MANIFEST_NOTES" ]; then
    log_info "Channel notes: $MANIFEST_NOTES"
  fi

  local sdk_dir="$INSTALL_ROOT/sdk"
  local ros2_dir="$INSTALL_ROOT/ros2"

  sync_repo_to_ref "$MANIFEST_SDK_URL" "$MANIFEST_SDK_REF" "$sdk_dir" "horus_sdk"
  sync_repo_to_ref "$MANIFEST_ROS2_URL" "$MANIFEST_ROS2_REF" "$ros2_dir" "horus_ros2"

  setup_python_sdk_runtime "$sdk_dir"
  build_ros2_backend_workspace "$ros2_dir" "$ROS_DISTRO" "$WEBRTC"
  validate_runtime_install "$INSTALL_ROOT" "$ROS_DISTRO"

  write_helper_scripts "$INSTALL_ROOT" "$ROS_DISTRO"
  configure_shell_integration "$INSTALL_ROOT" "$SHELL_CONFIG"
  write_install_state \
    "$INSTALL_ROOT" \
    "$CHANNEL" \
    "$ROS_DISTRO" \
    "$WEBRTC" \
    "$SHELL_CONFIG" \
    "$MANIFEST_SDK_REF" \
    "$MANIFEST_ROS2_REF"
  write_install_summary \
    "$INSTALL_ROOT" \
    "$LOG_FILE" \
    "$CHANNEL" \
    "$ROS_DISTRO" \
    "$WEBRTC" \
    "$MANIFEST_SDK_REF" \
    "$MANIFEST_ROS2_REF"

  print_wsl_guidance

  log_success "HORUS installation completed"
  cat <<EOM

Next steps:
  1) Open a new shell (or run: source ~/.bashrc)
  2) Check status: horus-status
  3) Start backend: horus-start
  4) Run demo (new shell):
     python3 "$INSTALL_ROOT/sdk/python/examples/sdk_registration_demo.py" --robot-count 4 --with-camera --with-occupancy-grid
  5) Uninstall later if needed: horus-uninstall

EOM
}

main "$@"
