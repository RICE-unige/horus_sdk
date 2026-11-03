#!/usr/bin/env bash
# Full SDK + ROS bridge installation. No daemon is started by the installer.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for library in common ui args system repos runtime helpers; do
  # shellcheck disable=SC1090
  source "$SCRIPT_DIR/lib/$library.sh"
done

ask_interactive_defaults() {
  [ "$ASSUME_YES" -eq 0 ] || return 0
  prompt_with_default_into INSTALL_ROOT "Install root" "$INSTALL_ROOT"
  INSTALL_ROOT="$(normalize_path "$INSTALL_ROOT")"
  prompt_with_default_into CHANNEL "Channel (stable/main)" "$CHANNEL"
  prompt_with_default_into WEBRTC "Build WebRTC bridge (on/off)" "$WEBRTC"
  prompt_with_default_into SHELL_CONFIG "Shell integration (auto/manual)" "$SHELL_CONFIG"
  # Apply the same validation to interactive input and command-line input.
  parse_args --install-root "$INSTALL_ROOT" --channel "$CHANNEL" \
    --webrtc "$WEBRTC" --shell-config "$SHELL_CONFIG"
}

installer_cleanup() {
  local result="$?"
  _ui_cleanup
  if [ -n "${INSTALL_STARTED:-}" ] && [ "$result" -ne 0 ]; then
    printf 'failed\n' >"$INSTALL_ROOT/state/install.status"
    printf '\nInstallation incomplete. No services were started. Log: %s\n' "$LOG_FILE" >&2
  fi
  if [ -n "${INSTALLER_SNAPSHOT:-}" ]; then rm -rf -- "$INSTALLER_SNAPSHOT"; fi
}

main() {
  parse_args "$@"
  if [ "$CHECK_ONLY" -eq 1 ]; then LOG_FILE=""; fi
  detect_platform
  ROS_DISTRO="$(resolve_ros_distro "$ROS_DISTRO")"
  ask_interactive_defaults
  for command in git realpath flock /usr/bin/python3; do
    command_exists "$command" || die "Missing prerequisite: $command"
  done
  parse_manifest_channel "$SCRIPT_DIR/manifest/releases.json" "$CHANNEL"
  check_install_root "$INSTALL_ROOT"
  check_legacy_python_shim "$INSTALL_ROOT"
  /usr/bin/python3 "$SCRIPT_DIR/lib/manage.py" shell-check "$INSTALL_ROOT" "$SHELL_CONFIG"
  if [ -f "$INSTALL_ROOT/state/horus-start.pid" ]; then
    local legacy_pid
    legacy_pid="$(cat "$INSTALL_ROOT/state/horus-start.pid")"
    if [[ "$legacy_pid" =~ ^[0-9]+$ ]] && [ -d "/proc/$legacy_pid" ]; then
      die "A legacy launcher PID is still present ($legacy_pid); stop that launch before updating."
    fi
  fi
  # Read-only preflight checks both repositories before creating files or using sudo.
  local check="$CHECK_ONLY"
  CHECK_ONLY=1
  preflight_repo "$MANIFEST_SDK_URL" "$MANIFEST_SDK_REF" "$INSTALL_ROOT/sdk" horus_sdk
  preflight_repo "$MANIFEST_ROS2_URL" "$MANIFEST_ROS2_REF" "$INSTALL_ROOT/ros2" horus_ros2
  CHECK_ONLY="$check"
  print_config_summary "$INSTALL_ROOT" "$CHANNEL" "$ROS_DISTRO" "$WEBRTC" "$SHELL_CONFIG" "Ubuntu $OS_VERSION ($OS_CODENAME)"
  log_info "SDK revision: $MANIFEST_SDK_REF"
  log_info "ROS bridge revision: $MANIFEST_ROS2_REF"
  if [ -n "$MANIFEST_NOTES" ]; then log_info "$MANIFEST_NOTES"; fi
  if [ "$CHECK_ONLY" -eq 1 ]; then
    log_success "Configuration checks passed. No installation performed; remote refs/build/runtime have not been tested."
    return 0
  fi
  [ "$EUID" -ne 0 ] || die "Run as your normal user, not sudo/root; individual apt steps use sudo."
  export GIT_TERMINAL_PROMPT=0 GIT_HTTP_LOW_SPEED_LIMIT=1000 GIT_HTTP_LOW_SPEED_TIME=60
  prepare_install_root "$INSTALL_ROOT"
  exec {RUNTIME_LOCK_FD}>"$INSTALL_ROOT/state/runtime.lock"
  flock -n "$RUNTIME_LOCK_FD" || die "Stop horus-start before updating this installation"
  if [ -z "$LOG_FILE" ]; then
    LOG_FILE="$(mktemp "$INSTALL_ROOT/logs/install-$(date -u +%Y%m%dT%H%M%S).XXXXXXXX.log")"
  fi
  mkdir -p "$(dirname "$LOG_FILE")"
  touch "$LOG_FILE"  # Append to an explicit user log; never truncate it.
  setup_ui_traps
  trap installer_cleanup EXIT
  INSTALL_STARTED=1
  printf 'installing\n' >"$INSTALL_ROOT/state/install.status"
  # Keep this version's templates/tools intact if its own SDK checkout is updated.
  INSTALLER_SNAPSHOT="$(mktemp -d -t horus-installer.XXXXXXXX)"
  cp -a "$SCRIPT_DIR/." "$INSTALLER_SNAPSHOT/"
  SCRIPT_DIR="$INSTALLER_SNAPSHOT"
  _log_to_file "Installer $HORUS_INSTALLER_VERSION on $OS_ID $OS_VERSION; root=$INSTALL_ROOT; channel=$CHANNEL"
  print_banner
  local total=11
  if [ "$WEBRTC" = on ]; then total=12; fi
  step_init "$total"

  # Fetch both targets before checkout so local ahead/divergent work cannot be lost.
  preflight_repo "$MANIFEST_SDK_URL" "$MANIFEST_SDK_REF" "$INSTALL_ROOT/sdk" horus_sdk
  preflight_repo "$MANIFEST_ROS2_URL" "$MANIFEST_ROS2_REF" "$INSTALL_ROOT/ros2" horus_ros2
  step_start "Syncing SDK"
  sync_repo_to_ref "$MANIFEST_SDK_URL" "$MANIFEST_SDK_REF" "$INSTALL_ROOT/sdk" horus_sdk
  step_done
  step_start "Syncing ROS bridge"
  sync_repo_to_ref "$MANIFEST_ROS2_URL" "$MANIFEST_ROS2_REF" "$INSTALL_ROOT/ros2" horus_ros2
  step_done
  step_start "Validating sudo access"
  ensure_sudo
  step_done
  step_start "Configuring ROS apt repository"
  setup_ros_apt_repository "$OS_CODENAME"
  step_done
  step_start "Installing development tools"
  install_base_dependencies
  step_done
  step_start "Installing ROS $ROS_DISTRO"
  install_ros_dependencies "$ROS_DISTRO"
  step_done
  if [ "$WEBRTC" = on ]; then
    step_start "Installing media dependencies"
    install_webrtc_dependencies
    step_done
  fi
  step_start "Preparing rosdep"
  init_rosdep "$ROS_DISTRO"
  step_done
  step_start "Installing SDK Python runtime"
  setup_python_sdk_runtime "$INSTALL_ROOT/sdk"
  step_done
  step_start "Building ROS bridge"
  build_ros2_backend_workspace "$INSTALL_ROOT/ros2" "$ROS_DISTRO" "$WEBRTC"
  step_done
  step_start "Validating SDK and bridge"
  validate_runtime_install "$INSTALL_ROOT" "$ROS_DISTRO"
  step_done
  step_start "Installing helper commands"
  write_helper_scripts "$INSTALL_ROOT" "$ROS_DISTRO"
  configure_shell_integration "$INSTALL_ROOT" "$SHELL_CONFIG"
  write_install_state "$INSTALL_ROOT" "$CHANNEL" "$ROS_DISTRO" "$WEBRTC" "$SHELL_CONFIG" "$MANIFEST_SDK_REF" "$MANIFEST_ROS2_REF"
  printf 'complete\n' >"$INSTALL_ROOT/state/install.status"
  step_done
  print_wsl_guidance
  print_completion_summary "$INSTALL_ROOT"
}
main "$@"
