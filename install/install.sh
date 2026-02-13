#!/usr/bin/env bash
# =============================================================================
# HORUS Installer - Main Entry Point
#
# Installs the HORUS Mixed Reality Robot Management Platform:
#   - horus_sdk (Python SDK)
#   - horus_ros2 (ROS 2 bridge)
#   - Helper scripts and shell integration
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/common.sh"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib/ui.sh"
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

# ---------------------------------------------------------------------------
# Bootstrap log (temporary until install root is ready)
# ---------------------------------------------------------------------------

bootstrap_log() {
  local early_log="/tmp/horus-installer-$(date +%Y%m%d-%H%M%S).log"
  if [ -z "$LOG_FILE" ]; then
    LOG_FILE="$early_log"
  fi
}

# ---------------------------------------------------------------------------
# Interactive configuration prompts
# ---------------------------------------------------------------------------

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

# ---------------------------------------------------------------------------
# Logging setup (writes to log file only - no tee to stdout)
# ---------------------------------------------------------------------------

setup_logging() {
  mkdir -p "$(dirname "$LOG_FILE")"
  : > "$LOG_FILE"
  _log_to_file "=========================================="
  _log_to_file "HORUS Installer v$HORUS_INSTALLER_VERSION"
  _log_to_file "Date: $(date -u)"
  _log_to_file "Platform: $OS_ID $OS_VERSION ($OS_CODENAME)"
  _log_to_file "Config: install_root=$INSTALL_ROOT channel=$CHANNEL ros_distro=$ROS_DISTRO webrtc=$WEBRTC"
  _log_to_file "=========================================="
}

# ---------------------------------------------------------------------------
# Main installation flow
# ---------------------------------------------------------------------------

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
  setup_ui_traps

  # --- Banner ---
  print_banner
  printf '    %bInstaller v%s%b\n' "${C_DIM}" "$HORUS_INSTALLER_VERSION" "${C_RESET}"

  # --- Configuration summary ---
  local os_info="Ubuntu $OS_VERSION ($OS_CODENAME)"
  if [ "$IS_WSL" -eq 1 ]; then
    os_info="$os_info / WSL2"
  fi
  print_config_summary \
    "$INSTALL_ROOT" "$CHANNEL" "$ROS_DISTRO" "$WEBRTC" "$SHELL_CONFIG" "$os_info"

  # --- Compute step total ---
  # Steps:
  #  1. Sudo access
  #  2. ROS apt repository
  #  3. Base system packages
  #  4. ROS 2 packages
  #  5. WebRTC dependencies      (conditional)
  #  6. rosdep init
  #  7. Sync horus_sdk
  #  8. Sync horus_ros2
  #  9. Python SDK runtime
  # 10. ROS 2 workspace build
  # 11. Runtime validation
  # 12. Helper scripts & shell integration
  local total=11
  if [ "$WEBRTC" = "on" ]; then
    total=12
  fi
  step_init "$total"

  # --- Step 1: Sudo ---
  step_start "Validating sudo access"
  ensure_sudo
  step_done

  # --- Step 2: ROS apt repo ---
  step_start "Configuring ROS 2 apt repository"
  setup_ros_apt_repository "$OS_CODENAME"
  step_done

  # --- Step 3: Base packages ---
  step_start "Installing base system packages"
  install_base_dependencies
  step_done

  # --- Step 4: ROS packages ---
  step_start "Installing ROS 2 packages ($ROS_DISTRO)"
  install_ros_dependencies "$ROS_DISTRO"
  step_done

  # --- Step 5: WebRTC (conditional) ---
  if [ "$WEBRTC" = "on" ]; then
    step_start "Installing WebRTC/media dependencies"
    install_webrtc_dependencies
    step_done
  fi

  # --- Step 6: rosdep ---
  step_start "Initializing rosdep"
  init_rosdep "$ROS_DISTRO"
  step_done

  # --- Parse manifest (silent, no step needed) ---
  local manifest_path="$REPO_ROOT/install/manifest/releases.json"
  parse_manifest_channel "$manifest_path" "$CHANNEL"

  if [ -n "$MANIFEST_NOTES" ]; then
    _log_to_file "Channel notes: $MANIFEST_NOTES"
  fi

  # --- Step 7: Sync SDK ---
  local sdk_dir="$INSTALL_ROOT/sdk"
  step_start "Syncing horus_sdk repository"
  sync_repo_to_ref "$MANIFEST_SDK_URL" "$MANIFEST_SDK_REF" "$sdk_dir" "horus_sdk"
  step_done

  # --- Step 8: Sync ROS2 ---
  local ros2_dir="$INSTALL_ROOT/ros2"
  step_start "Syncing horus_ros2 repository"
  sync_repo_to_ref "$MANIFEST_ROS2_URL" "$MANIFEST_ROS2_REF" "$ros2_dir" "horus_ros2"
  step_done

  # --- Step 9: Python SDK ---
  step_start "Setting up Python SDK runtime"
  setup_python_sdk_runtime "$sdk_dir"
  step_done

  # --- Step 10: Build ROS2 workspace ---
  step_start "Building ROS 2 workspace (this may take several minutes)"
  build_ros2_backend_workspace "$ros2_dir" "$ROS_DISTRO" "$WEBRTC"
  step_done

  # --- Step 11: Validate ---
  step_start "Validating runtime installation"
  validate_runtime_install "$INSTALL_ROOT" "$ROS_DISTRO" "$HORUS_INSTALL_RUNTIME_MODE"
  step_done

  # --- Step 12: Helpers and shell ---
  step_start "Writing helper scripts and shell integration"
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
    "$MANIFEST_ROS2_REF" \
    "$HORUS_INSTALL_RUNTIME_MODE" \
    "$HORUS_INSTALL_RUNTIME_REASON"
  step_done

  # --- WSL guidance ---
  print_wsl_guidance

  # --- Completion ---
  print_completion_summary "$INSTALL_ROOT" "$HORUS_INSTALL_RUNTIME_MODE" "$HORUS_INSTALL_RUNTIME_REASON"
}

main "$@"
