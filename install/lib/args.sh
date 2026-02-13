#!/usr/bin/env bash
# =============================================================================
# HORUS Installer - Argument Parsing
# =============================================================================

INSTALL_ROOT_DEFAULT="$HOME/horus"
CHANNEL_DEFAULT="stable"
ROS_DISTRO_DEFAULT="auto"
WEBRTC_DEFAULT="on"
SHELL_CONFIG_DEFAULT="auto"

INSTALL_ROOT="$INSTALL_ROOT_DEFAULT"
CHANNEL="$CHANNEL_DEFAULT"
ROS_DISTRO="$ROS_DISTRO_DEFAULT"
WEBRTC="$WEBRTC_DEFAULT"
SHELL_CONFIG="$SHELL_CONFIG_DEFAULT"
ASSUME_YES=0
LOG_FILE=""

print_usage() {
  if [ -t 1 ]; then
    printf '\n'
    printf '  %bHORUS Installer%b v%s\n' '\033[1;34m' '\033[0m' "${HORUS_INSTALLER_VERSION:-0.1.0}"
    printf '  %bHolistic Operational Reality for Unified Systems%b\n' '\033[2m' '\033[0m'
    printf '\n'
  fi

  cat <<'USAGE'
Usage:
  install.sh [options]

Options:
  --yes                         Non-interactive mode (accept all defaults)
  --channel <stable|main>       Source channel [default: stable]
  --install-root <path>         Installation directory [default: ~/horus]
  --ros-distro <auto|jazzy|humble>
                                ROS 2 distribution [default: auto-detect]
  --webrtc <on|off>             WebRTC bridge support [default: on]
  --shell-config <auto|manual>  Shell integration mode [default: auto]
  --log-file <path>             Custom log file path
  --help                        Show this help message

Examples:
  install.sh                              Interactive installation
  install.sh --yes                        Accept all defaults
  install.sh --yes --channel main         Track development branch
  install.sh --yes --webrtc off           Skip WebRTC dependencies

Documentation:
  https://github.com/RICE-unige/horus_sdk

USAGE
}

parse_args() {
  while [ "$#" -gt 0 ]; do
    case "$1" in
      --yes)
        ASSUME_YES=1
        shift
        ;;
      --channel)
        [ "$#" -ge 2 ] || die "Missing value for --channel"
        CHANNEL="$2"
        shift 2
        ;;
      --install-root)
        [ "$#" -ge 2 ] || die "Missing value for --install-root"
        INSTALL_ROOT="$2"
        shift 2
        ;;
      --ros-distro)
        [ "$#" -ge 2 ] || die "Missing value for --ros-distro"
        ROS_DISTRO="$2"
        shift 2
        ;;
      --webrtc)
        [ "$#" -ge 2 ] || die "Missing value for --webrtc"
        WEBRTC="$2"
        shift 2
        ;;
      --shell-config)
        [ "$#" -ge 2 ] || die "Missing value for --shell-config"
        SHELL_CONFIG="$2"
        shift 2
        ;;
      --log-file)
        [ "$#" -ge 2 ] || die "Missing value for --log-file"
        LOG_FILE="$2"
        shift 2
        ;;
      --help|-h)
        print_usage
        exit 0
        ;;
      *)
        die "Unknown argument: $1"
        ;;
    esac
  done

  CHANNEL="${CHANNEL,,}"
  ROS_DISTRO="${ROS_DISTRO,,}"
  WEBRTC="${WEBRTC,,}"
  SHELL_CONFIG="${SHELL_CONFIG,,}"

  case "$CHANNEL" in
    stable|main) ;;
    *) die "Invalid --channel '$CHANNEL' (use stable|main)" ;;
  esac

  case "$ROS_DISTRO" in
    auto|jazzy|humble) ;;
    *) die "Invalid --ros-distro '$ROS_DISTRO' (use auto|jazzy|humble)" ;;
  esac

  case "$WEBRTC" in
    on|off) ;;
    *) die "Invalid --webrtc '$WEBRTC' (use on|off)" ;;
  esac

  case "$SHELL_CONFIG" in
    auto|manual) ;;
    *) die "Invalid --shell-config '$SHELL_CONFIG' (use auto|manual)" ;;
  esac

  INSTALL_ROOT="$(normalize_path "$INSTALL_ROOT")"
  if [ -n "$LOG_FILE" ]; then
    LOG_FILE="$(normalize_path "$LOG_FILE")"
  fi
}
