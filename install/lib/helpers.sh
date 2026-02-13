#!/usr/bin/env bash

write_helper_scripts() {
  local install_root="$1"
  local ros_distro="$2"

  local bin_dir="$install_root/bin"
  local state_dir="$install_root/state"
  mkdir -p "$bin_dir" "$state_dir"

  cat > "$bin_dir/horus-env" <<EOM
#!/usr/bin/env bash
export HORUS_HOME="$install_root"
export HORUS_ROS_DISTRO="$ros_distro"

_horus_source_setup() {
  local setup_file="\$1"
  local had_nounset=0

  if [[ \$- == *u* ]]; then
    had_nounset=1
    set +u
  fi

  # shellcheck disable=SC1090
  source "\$setup_file"

  if [ "\$had_nounset" -eq 1 ]; then
    set -u
  fi
}

if [ -f "/opt/ros/\$HORUS_ROS_DISTRO/setup.bash" ]; then
  _horus_source_setup "/opt/ros/\$HORUS_ROS_DISTRO/setup.bash"
fi

if [ -f "\$HORUS_HOME/ros2/install/setup.bash" ]; then
  _horus_source_setup "\$HORUS_HOME/ros2/install/setup.bash"
fi

if [ -f "\$HORUS_HOME/sdk/.venv/bin/activate" ]; then
  # shellcheck disable=SC1090
  source "\$HORUS_HOME/sdk/.venv/bin/activate"
fi
EOM

  cat > "$bin_dir/horus-python" <<'EOM'
#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1090
source "$SCRIPT_DIR/horus-env"
exec "$HORUS_HOME/sdk/.venv/bin/python3" "$@"
EOM

  cat > "$bin_dir/python3" <<'EOM'
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HORUS_HOME="${HORUS_HOME:-$(cd "$SCRIPT_DIR/.." && pwd)}"
SYSTEM_PYTHON="/usr/bin/python3"

use_horus_python=0

if [ $# -gt 0 ]; then
  first_arg="$1"

  # Absolute path under installed SDK tree.
  if [[ "$first_arg" == "$HORUS_HOME"/sdk/* ]]; then
    use_horus_python=1
  fi

  # Relative script path that resolves under installed SDK tree.
  if [ "$use_horus_python" -eq 0 ] && [[ "$first_arg" != -* ]] && [ -e "$first_arg" ]; then
    resolved="$(readlink -f "$first_arg" 2>/dev/null || true)"
    if [[ "$resolved" == "$HORUS_HOME"/sdk/* ]]; then
      use_horus_python=1
    fi
  fi
fi

# Running from SDK root with `-m`, `-c`, or no args: prefer HORUS env.
if [ "$use_horus_python" -eq 0 ] && [[ "$(pwd)" == "$HORUS_HOME"/sdk* ]]; then
  case "${1-}" in
    -m|-c|'') use_horus_python=1 ;;
  esac
fi

if [ "$use_horus_python" -eq 1 ] && [ -x "$HORUS_HOME/sdk/.venv/bin/python3" ]; then
  if [ -f "$HORUS_HOME/bin/horus-env" ]; then
    # shellcheck disable=SC1090
    source "$HORUS_HOME/bin/horus-env"
  fi
  exec "$HORUS_HOME/sdk/.venv/bin/python3" "$@"
fi

exec "$SYSTEM_PYTHON" "$@"
EOM

  cat > "$bin_dir/horus-start" <<'EOM'
#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1090
source "$SCRIPT_DIR/horus-env"

if ros2 pkg prefix horus_unity_bridge >/dev/null 2>&1; then
  echo "Starting full HORUS backend stack (backend + unity bridge)"
  exec ros2 launch horus_backend horus_complete_backend.launch.py
else
  echo "horus_unity_bridge not available in current workspace. Starting backend-only launch."
  exec ros2 launch horus_backend horus_backend.launch.py
fi
EOM

  cat > "$bin_dir/horus-stop" <<'EOM'
#!/usr/bin/env bash
set -euo pipefail

pkill -f horus_backend_node 2>/dev/null || true
pkill -f horus_unity_bridge_node 2>/dev/null || true
pkill -f horus_unity_bridge 2>/dev/null || true

if command -v lsof >/dev/null 2>&1; then
  lsof -ti :8080,10000 | xargs -r kill -TERM 2>/dev/null || true
fi

echo "HORUS backend processes stopped (if running)."
EOM

  cat > "$bin_dir/horus-status" <<'EOM'
#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1090
source "$SCRIPT_DIR/horus-env"

echo "HORUS_HOME=$HORUS_HOME"
echo "HORUS_ROS_DISTRO=$HORUS_ROS_DISTRO"

if command -v ros2 >/dev/null 2>&1; then
  echo "ros2: OK"
else
  echo "ros2: MISSING"
fi

if [ -x "$HORUS_HOME/sdk/.venv/bin/python" ]; then
  echo "python venv: OK"
else
  echo "python venv: MISSING"
fi

if ros2 pkg prefix horus_backend >/dev/null 2>&1; then
  echo "horus_backend package: OK"
else
  echo "horus_backend package: MISSING"
fi

if ros2 pkg prefix horus_unity_bridge >/dev/null 2>&1; then
  echo "horus_unity_bridge package: OK"
else
  echo "horus_unity_bridge package: MISSING"
fi

if command -v ss >/dev/null 2>&1; then
  if ss -ltn | grep -q ':8080 '; then
    echo "port 8080: LISTEN"
  else
    echo "port 8080: FREE"
  fi
  if ss -ltn | grep -q ':10000 '; then
    echo "port 10000: LISTEN"
  else
    echo "port 10000: FREE"
  fi
fi
EOM

  cat > "$bin_dir/horus-update" <<'EOM'
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HORUS_HOME="${HORUS_HOME:-$(cd "$SCRIPT_DIR/.." && pwd)}"
STATE_FILE="$HORUS_HOME/state/install-config.env"

if [ ! -f "$STATE_FILE" ]; then
  echo "Missing installer state file: $STATE_FILE" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$STATE_FILE"

INSTALL_SCRIPT="$HORUS_HOME/sdk/install/install.sh"
if [ ! -x "$INSTALL_SCRIPT" ]; then
  echo "Installer script not found at $INSTALL_SCRIPT" >&2
  exit 1
fi

exec bash "$INSTALL_SCRIPT" \
  --yes \
  --install-root "$HORUS_HOME" \
  --channel "${HORUS_CHANNEL:-stable}" \
  --ros-distro "${HORUS_ROS_DISTRO:-auto}" \
  --webrtc "${HORUS_WEBRTC:-on}" \
  --shell-config "${HORUS_SHELL_CONFIG:-auto}"
EOM

  cat > "$bin_dir/horus-uninstall" <<'EOM'
#!/usr/bin/env bash
set -euo pipefail

ASSUME_YES=0
PURGE_ROS_PACKAGES=0
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HORUS_HOME="${HORUS_HOME:-$(cd "$SCRIPT_DIR/.." && pwd)}"
STATE_FILE="$HORUS_HOME/state/install-config.env"
ROS_DISTRO="${HORUS_ROS_DISTRO:-}"

usage() {
  cat <<USAGE
HORUS uninstall helper

Usage:
  horus-uninstall [options]

Options:
  --yes                  Non-interactive mode
  --purge-ros-packages   Also remove installer ROS apt packages for this distro
  --help                 Show this help
USAGE
}

log() { echo "[horus-uninstall] $*"; }
warn() { echo "[horus-uninstall][warn] $*" >&2; }
die() { echo "[horus-uninstall][error] $*" >&2; exit 1; }

while [ "$#" -gt 0 ]; do
  case "$1" in
    --yes)
      ASSUME_YES=1
      shift
      ;;
    --purge-ros-packages)
      PURGE_ROS_PACKAGES=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      die "Unknown argument: $1"
      ;;
  esac
done

if [ -f "$STATE_FILE" ]; then
  # shellcheck disable=SC1090
  source "$STATE_FILE"
  if [ -n "${HORUS_INSTALL_ROOT:-}" ]; then
    HORUS_HOME="$HORUS_INSTALL_ROOT"
  fi
  if [ -z "$ROS_DISTRO" ] && [ -n "${HORUS_ROS_DISTRO:-}" ]; then
    ROS_DISTRO="$HORUS_ROS_DISTRO"
  fi
fi

if [ -z "$ROS_DISTRO" ]; then
  if [ -d /opt/ros/jazzy ]; then
    ROS_DISTRO="jazzy"
  elif [ -d /opt/ros/humble ]; then
    ROS_DISTRO="humble"
  fi
fi

if [ "$ASSUME_YES" -ne 1 ] && ! tty -s; then
  die "No interactive TTY detected. Re-run with --yes."
fi

if [ "$ASSUME_YES" -ne 1 ]; then
  echo "This will remove HORUS files from: $HORUS_HOME"
  if [ "$PURGE_ROS_PACKAGES" -eq 1 ]; then
    echo "It will also purge selected ROS apt packages for distro: ${ROS_DISTRO:-unknown}"
  fi
  printf "Continue? [y/N]: "
  read -r answer || true
  answer="${answer,,}"
  case "$answer" in
    y|yes) ;;
    *) echo "Aborted."; exit 0 ;;
  esac
fi

log "Stopping running HORUS processes (if any)"
pkill -f horus_backend_node 2>/dev/null || true
pkill -f horus_unity_bridge_node 2>/dev/null || true
pkill -f horus_unity_bridge 2>/dev/null || true

if command -v lsof >/dev/null 2>&1; then
  lsof -ti :8080,10000 | xargs -r kill -TERM 2>/dev/null || true
fi

bashrc="$HOME/.bashrc"
marker_start="# >>> HORUS installer >>>"
marker_end="# <<< HORUS installer <<<"
if [ -f "$bashrc" ] && grep -Fq "$marker_start" "$bashrc"; then
  log "Removing HORUS shell integration block from ~/.bashrc"
  tmp_bashrc="$(mktemp)"
  awk -v start="$marker_start" -v end="$marker_end" '
    $0 == start { skip = 1; next }
    $0 == end { skip = 0; next }
    skip != 1 { print }
  ' "$bashrc" > "$tmp_bashrc"
  cat "$tmp_bashrc" > "$bashrc"
  rm -f "$tmp_bashrc"
fi

if [ "$PURGE_ROS_PACKAGES" -eq 1 ]; then
  [ -n "$ROS_DISTRO" ] || die "Cannot infer ROS distro for package purge."
  if ! command -v sudo >/dev/null 2>&1; then
    die "sudo is required for --purge-ros-packages"
  fi
  log "Purging ROS apt packages for distro '$ROS_DISTRO'"
  sudo env DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get remove --purge -y \
    "ros-${ROS_DISTRO}-desktop" \
    "ros-${ROS_DISTRO}-rclpy" \
    "ros-${ROS_DISTRO}-std-msgs" \
    "ros-${ROS_DISTRO}-geometry-msgs" \
    "ros-${ROS_DISTRO}-sensor-msgs" \
    "ros-${ROS_DISTRO}-nav-msgs" \
    "ros-${ROS_DISTRO}-tf2-ros" \
    "ros-${ROS_DISTRO}-tf2-msgs" \
    "ros-${ROS_DISTRO}-std-srvs" \
    "ros-${ROS_DISTRO}-example-interfaces" \
    "ros-${ROS_DISTRO}-ament-cmake" \
    "ros-${ROS_DISTRO}-rclcpp" || warn "ROS package purge reported issues."
  sudo env DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get autoremove -y || true
fi

if [ -e "$HORUS_HOME" ]; then
  case "$PWD" in
    "$HORUS_HOME"|"$HORUS_HOME"/*)
      cd "$HOME"
      ;;
  esac
  log "Removing install root: $HORUS_HOME"
  rm -rf "$HORUS_HOME"
else
  warn "Install root does not exist: $HORUS_HOME"
fi

log "Uninstall complete."
echo "Restart your shell (or run: source ~/.bashrc)."
EOM

  chmod +x \
    "$bin_dir/horus-env" \
    "$bin_dir/python3" \
    "$bin_dir/horus-python" \
    "$bin_dir/horus-start" \
    "$bin_dir/horus-stop" \
    "$bin_dir/horus-status" \
    "$bin_dir/horus-update" \
    "$bin_dir/horus-uninstall"

  log_success "Helper commands created in $bin_dir"
}

configure_shell_integration() {
  local install_root="$1"
  local shell_config="$2"

  if [ "$shell_config" != "auto" ]; then
    log_info "Shell integration set to manual; skipping ~/.bashrc changes"
    return 0
  fi

  local bashrc="$HOME/.bashrc"
  local marker_start="# >>> HORUS installer >>>"
  local marker_end="# <<< HORUS installer <<<"

  if [ -f "$bashrc" ] && grep -Fq "$marker_start" "$bashrc"; then
    log_info "~/.bashrc already has HORUS integration block; refreshing"
    local tmp_bashrc
    tmp_bashrc="$(mktemp)"

    awk -v start="$marker_start" -v end="$marker_end" '
      $0 == start { skip = 1; next }
      $0 == end { skip = 0; next }
      skip != 1 { print }
    ' "$bashrc" > "$tmp_bashrc"

    cat "$tmp_bashrc" > "$bashrc"
    rm -f "$tmp_bashrc"
  fi

  cat >> "$bashrc" <<EOM

$marker_start
export HORUS_HOME="$install_root"
if [ -d "\$HORUS_HOME/bin" ] && [[ ":\$PATH:" != *":\$HORUS_HOME/bin:"* ]]; then
  export PATH="\$HORUS_HOME/bin:\$PATH"
fi
$marker_end
EOM

  log_success "Added HORUS integration block to ~/.bashrc"
}

write_install_state() {
  local install_root="$1"
  local channel="$2"
  local ros_distro="$3"
  local webrtc="$4"
  local shell_config="$5"
  local sdk_ref="$6"
  local ros2_ref="$7"

  local state_file="$install_root/state/install-config.env"
  cat > "$state_file" <<EOM
HORUS_INSTALL_ROOT="$install_root"
HORUS_CHANNEL="$channel"
HORUS_ROS_DISTRO="$ros_distro"
HORUS_WEBRTC="$webrtc"
HORUS_SHELL_CONFIG="$shell_config"
HORUS_SDK_REF="$sdk_ref"
HORUS_ROS2_REF="$ros2_ref"
HORUS_INSTALLED_AT="$(timestamp_utc)"
EOM

  log_success "Installer state written to $state_file"
}

write_install_summary() {
  local install_root="$1"
  local log_file="$2"
  local channel="$3"
  local ros_distro="$4"
  local webrtc="$5"
  local sdk_ref="$6"
  local ros2_ref="$7"
  local runtime_mode="${8:-full}"
  local runtime_reason="${9:-}"

  local summary_file="$install_root/state/install-summary.json"

  if ! python3 - "$summary_file" "$install_root" "$log_file" "$channel" "$ros_distro" "$webrtc" "$sdk_ref" "$ros2_ref" "$runtime_mode" "$runtime_reason" "$(timestamp_utc)" "${HORUS_INSTALLER_VERSION:-unknown}" <<'PY'
import json
import sys

summary_file, install_root, log_file, channel, ros_distro, webrtc, sdk_ref, ros2_ref, runtime_mode, runtime_reason, installed_at, version = sys.argv[1:]
payload = {
    "installer_version": version,
    "installed_at": installed_at,
    "install_root": install_root,
    "log_file": log_file,
    "channel": channel,
    "ros_distro": ros_distro,
    "webrtc": webrtc,
    "refs": {
        "horus_sdk": sdk_ref,
        "horus_ros2": ros2_ref,
    },
    "runtime_mode": runtime_mode,
}
if runtime_reason:
    payload["runtime_reason"] = runtime_reason

with open(summary_file, "w", encoding="utf-8") as fh:
    json.dump(payload, fh, indent=2)
    fh.write("\n")
PY
  then
    die "Failed to write install summary JSON"
  fi

  log_success "Install summary written to $summary_file"
}
