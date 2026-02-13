#!/usr/bin/env bash

set -euo pipefail

SCRIPT_SOURCE="${BASH_SOURCE[0]-}"
if [ -n "$SCRIPT_SOURCE" ] && [ -f "$SCRIPT_SOURCE" ]; then
  SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_SOURCE")" && pwd)"
  LOCAL_INSTALLER="$SCRIPT_DIR/install/install.sh"

  # Prefer local installer when running from a checked-out repo.
  if [ -f "$LOCAL_INSTALLER" ]; then
    exec bash "$LOCAL_INSTALLER" "$@"
  fi
fi

BOOTSTRAP_REPO_URL="${HORUS_INSTALLER_REPO_URL:-https://github.com/RICE-unige/horus_sdk.git}"
BOOTSTRAP_REF="${HORUS_INSTALLER_REF:-main}"

if ! command -v git >/dev/null 2>&1; then
  echo "[error] git is required to bootstrap installer" >&2
  exit 1
fi

TMP_DIR="$(mktemp -d)"
cleanup() {
  rm -rf "$TMP_DIR"
}
trap cleanup EXIT

echo "[horus] Bootstrapping installer from ${BOOTSTRAP_REPO_URL} (${BOOTSTRAP_REF})"

CLONE_PATH="$TMP_DIR/horus_sdk_installer"
if ! git clone --depth 1 --branch "$BOOTSTRAP_REF" "$BOOTSTRAP_REPO_URL" "$CLONE_PATH"; then
  echo "[warn] shallow clone by branch failed, retrying full clone + checkout"
  git clone "$BOOTSTRAP_REPO_URL" "$CLONE_PATH"
  git -C "$CLONE_PATH" checkout "$BOOTSTRAP_REF"
fi

if [ ! -f "$CLONE_PATH/install/install.sh" ]; then
  echo "[error] installer payload not found at '$CLONE_PATH/install/install.sh'" >&2
  echo "[error] the selected ref '$BOOTSTRAP_REF' may not include installer sources yet." >&2
  echo "[hint] try: HORUS_INSTALLER_REF=<branch-or-commit> bash install.sh" >&2
  exit 1
fi

exec bash "$CLONE_PATH/install/install.sh" "$@"
