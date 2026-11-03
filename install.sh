#!/usr/bin/env bash
set -euo pipefail

# Release packaging replaces these defaults with immutable repository commits.
BOOTSTRAP_DEFAULT_REF="main"
BOOTSTRAP_SDK_REF=""
BOOTSTRAP_REPO_URL="${HORUS_INSTALLER_REPO_URL:-https://github.com/RICE-unige/horus_sdk.git}"
BOOTSTRAP_REF="${HORUS_INSTALLER_REF:-$BOOTSTRAP_DEFAULT_REF}"
export HORUS_INSTALLER_REPO_URL="$BOOTSTRAP_REPO_URL"
export HORUS_INSTALLER_REF="$BOOTSTRAP_REF"
if [ -n "$BOOTSTRAP_SDK_REF" ]; then
  export HORUS_RELEASE_SDK_REF="$BOOTSTRAP_SDK_REF"
fi

SCRIPT_SOURCE="${BASH_SOURCE[0]-}"
if [ -n "$SCRIPT_SOURCE" ] && [ -f "$SCRIPT_SOURCE" ]; then
  SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_SOURCE")" && pwd)"
  if [ -f "$SCRIPT_DIR/install/install.sh" ]; then
    exec bash "$SCRIPT_DIR/install/install.sh" "$@"
  fi
fi

command -v git >/dev/null || { printf "%s\n" "git is required; install git before running this bootstrap." >&2; exit 1; }
TMP_DIR="$(mktemp -d -t horus-bootstrap.XXXXXXXX)"
trap 'rm -rf -- "$TMP_DIR"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
export GIT_TERMINAL_PROMPT=0
export GIT_HTTP_LOW_SPEED_LIMIT=1000
export GIT_HTTP_LOW_SPEED_TIME=60

printf "[horus] Downloading installer (%s)\n" "$BOOTSTRAP_REF"
CLONE_PATH="$TMP_DIR/installer"
# Fetch branches, tags and commit IDs without retrying into partial clones.
git init -q "$CLONE_PATH"
git -C "$CLONE_PATH" remote add origin "$BOOTSTRAP_REPO_URL"
git -C "$CLONE_PATH" fetch --depth 1 origin "$BOOTSTRAP_REF"
git -C "$CLONE_PATH" checkout -q --detach FETCH_HEAD
[ -f "$CLONE_PATH/install/install.sh" ] || { printf "%s\n" "Selected revision has no HORUS installer." >&2; exit 1; }
# Keep the parent alive so the EXIT trap removes the downloaded source.
bash "$CLONE_PATH/install/install.sh" "$@"
