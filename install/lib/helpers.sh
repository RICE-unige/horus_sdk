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

if [ -f "/opt/ros/\$HORUS_ROS_DISTRO/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/\$HORUS_ROS_DISTRO/setup.bash"
fi

if [ -f "\$HORUS_HOME/ros2/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "\$HORUS_HOME/ros2/install/setup.bash"
fi

if [ -f "\$HORUS_HOME/sdk/.venv/bin/activate" ]; then
  # shellcheck disable=SC1090
  source "\$HORUS_HOME/sdk/.venv/bin/activate"
fi
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

HORUS_HOME_DEFAULT="$HOME/horus"
HORUS_HOME="${HORUS_HOME:-$HORUS_HOME_DEFAULT}"
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

  chmod +x \
    "$bin_dir/horus-env" \
    "$bin_dir/horus-start" \
    "$bin_dir/horus-stop" \
    "$bin_dir/horus-status" \
    "$bin_dir/horus-update"

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
    log_info "~/.bashrc already has HORUS integration block"
    return 0
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

  local summary_file="$install_root/state/install-summary.json"

  if ! python3 - "$summary_file" "$install_root" "$log_file" "$channel" "$ros_distro" "$webrtc" "$sdk_ref" "$ros2_ref" "$(timestamp_utc)" "${HORUS_INSTALLER_VERSION:-unknown}" <<'PY'
import json
import sys

summary_file, install_root, log_file, channel, ros_distro, webrtc, sdk_ref, ros2_ref, installed_at, version = sys.argv[1:]
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
}

with open(summary_file, "w", encoding="utf-8") as fh:
    json.dump(payload, fh, indent=2)
    fh.write("\n")
PY
  then
    die "Failed to write install summary JSON"
  fi

  log_success "Install summary written to $summary_file"
}
