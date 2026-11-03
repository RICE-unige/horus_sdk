#!/usr/bin/env bash
# Manifest values are consumed by the main entry point.
# shellcheck disable=SC2034
# Release selection and non-destructive repository updates.
MANIFEST_SDK_URL=""
MANIFEST_SDK_REF=""
MANIFEST_ROS2_URL=""
MANIFEST_ROS2_REF=""
MANIFEST_NOTES=""

parse_manifest_channel() {
  local manifest_path="$1" channel="$2" result
  local -a lines
  result="$(/usr/bin/python3 - "$manifest_path" "$channel" "${HORUS_RELEASE_SDK_REF:-}" <<'PY'
import json
import re
import sys
from pathlib import Path

data = json.loads(Path(sys.argv[1]).read_text())
if data.get("schema_version") != 1:
    raise SystemExit("Unsupported release manifest schema")
selected = data["channels"][sys.argv[2]]
sdk_ref = selected["sdk_ref"]
if sys.argv[3] and sys.argv[2] == "stable":
    sdk_ref = sys.argv[3]
values = [data["repos"]["horus_sdk"], sdk_ref,
          data["repos"]["horus_ros2"], selected["ros2_ref"]]
for value in values:
    if not isinstance(value, str) or not value or any(c in value for c in "\n\r\0"):
        raise SystemExit("Manifest URLs and refs must be non-empty single-line strings")
for ref in values[1::2]:
    if sys.argv[2] == "stable":
        valid = re.fullmatch(r"[0-9a-f]{40}", ref)
    else:
        valid = re.fullmatch(r"[0-9a-f]{40}|refs/heads/[\w./-]+|refs/tags/[\w./-]+", ref)
    if not valid:
        raise SystemExit("Invalid ref (stable requires full commit IDs): " + ref)
notes = selected.get("notes", "")
if not isinstance(notes, str) or "\n" in notes or "\r" in notes:
    raise SystemExit("Invalid channel notes")
print("\n".join(values + [notes]))
PY
)" || die "Invalid release manifest: $manifest_path ($channel)"
  mapfile -t lines <<<"$result"
  MANIFEST_SDK_URL="${lines[0]}"
  MANIFEST_SDK_REF="${lines[1]}"
  MANIFEST_ROS2_URL="${lines[2]}"
  MANIFEST_ROS2_REF="${lines[3]}"
  MANIFEST_NOTES="${lines[4]:-}"
}

ensure_git_repo_clean() {
  local repo_path="$1" repo_label="$2" status
  [ -e "$repo_path/.git" ] || die "$repo_label is not a Git checkout: $repo_path"
  status="$(git -C "$repo_path" status --porcelain)"
  [ -z "$status" ] || die "$repo_label has local changes. Preserve/commit them yourself or use another install root:
$status"
}

preflight_repo() {
  local url="$1" ref="$2" path="$3" label="$4"
  [ -e "$path" ] || return 0
  ensure_git_repo_clean "$path" "$label"
  [ "$(git -C "$path" remote get-url origin)" = "$url" ] ||
    die "$label origin differs from the manifest; it will not be overwritten."
  [ "$CHECK_ONLY" -eq 0 ] || return 0
  run_silent "Fetching $label target" git -C "$path" fetch --no-tags origin "$ref"
  # Refuse downgrades, divergent branches and unpublished commits.
  git -C "$path" merge-base --is-ancestor HEAD FETCH_HEAD ||
    die "$label contains commits absent from the requested revision. No checkout/reset performed; use another install root."
  if [[ "$ref" == refs/heads/* ]] && git -C "$path" show-ref --verify --quiet "$ref"; then
    git -C "$path" merge-base --is-ancestor "$ref" FETCH_HEAD ||
      die "$label requested local branch is ahead/diverged; it will not be reset."
  fi
}

sync_repo_to_ref() {
  local url="$1" ref="$2" path="$3" label="$4" target tmp
  if [ -e "$path" ]; then
    preflight_repo "$url" "$ref" "$path" "$label"
    target="$(git -C "$path" rev-parse FETCH_HEAD)"
    if [[ "$ref" == refs/heads/* ]]; then
      local branch="${ref#refs/heads/}"
      if git -C "$path" show-ref --verify --quiet "$ref"; then
        git -C "$path" merge-base --is-ancestor "$ref" "$target" ||
          die "$label local branch $branch is ahead/diverged; not resetting it."
        run_silent "Selecting $label branch" git -C "$path" checkout --no-overwrite-ignore "$branch"
        run_silent "Fast-forwarding $label" git -C "$path" merge --ff-only --no-autostash --no-overwrite-ignore "$target"
      else
        run_silent "Selecting $label branch" git -C "$path" checkout --no-overwrite-ignore -b "$branch" "$target"
      fi
    else
      run_silent "Selecting $label commit" git -C "$path" checkout --no-overwrite-ignore --detach "$target"
    fi
  else
    tmp="$(mktemp -d "${path}.clone.XXXXXXXX")"
    if ! (
      set -e
      git init -q "$tmp"
      git -C "$tmp" remote add origin "$url"
      git -C "$tmp" fetch --no-tags origin "$ref"
      if [[ "$ref" == refs/heads/* ]]; then
        git -C "$tmp" checkout -q -b "${ref#refs/heads/}" FETCH_HEAD
      else
        git -C "$tmp" checkout -q --detach FETCH_HEAD
      fi
    ) >>"$LOG_FILE" 2>&1; then
      rm -rf -- "$tmp"
      die "Could not fetch $label at $ref. See $LOG_FILE"
    fi
    mv -T -- "$tmp" "$path"
  fi
  _log_to_file "$label at $(git -C "$path" rev-parse HEAD)"
}

check_install_root() {
  local root="$1"
  case "$root" in
    /|/usr|/usr/*|/etc|/etc/*|/var|/var/*|/opt|/home|"$HOME")
      die "Choose a dedicated user-owned directory, not $root" ;;
  esac
  [ ! -e "$root" ] || [ -d "$root" ] || die "Install root is not a directory: $root"
  if [ -d "$root" ] && [ -n "$(find "$root" -mindepth 1 -maxdepth 1 -print -quit)" ]; then
    [ -f "$root/state/install-config.env" ] || [ -f "$root/state/install-root" ] ||
      die "Non-empty unmanaged directory: $root. Choose an empty --install-root."
    if [ -f "$root/state/install-root" ]; then
      [ "$(cat "$root/state/install-root")" = "$root" ] || die "Install root marker does not match this directory"
    fi
  fi
}

prepare_install_root() {
  local root="$1"
  check_install_root "$root"
  mkdir -p "$root/bin" "$root/logs" "$root/state"
  exec {INSTALL_LOCK_FD}>"$root/state/installer.lock"
  flock -n "$INSTALL_LOCK_FD" || die "Another installer/update is using $root"
  printf '%s\n' "$root" >"$root/state/install-root"
}
