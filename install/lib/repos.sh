#!/usr/bin/env bash
# =============================================================================
# HORUS Installer - Repository & Manifest Management
# =============================================================================

MANIFEST_SDK_URL=""
MANIFEST_SDK_REF=""
MANIFEST_ROS2_URL=""
MANIFEST_ROS2_REF=""
MANIFEST_NOTES=""

parse_manifest_channel() {
  local manifest_path="$1"
  local channel="$2"
  local result

  [ -f "$manifest_path" ] || die "Missing manifest file: $manifest_path"

  if ! result="$(python3 - "$manifest_path" "$channel" <<'PY'
import json
import sys

manifest_path = sys.argv[1]
channel = sys.argv[2]
with open(manifest_path, encoding="utf-8") as fh:
    data = json.load(fh)

repos = data.get("repos", {})
channels = data.get("channels", {})
selected = channels.get(channel)
if selected is None:
    raise SystemExit(2)

sdk_url = repos.get("horus_sdk", "https://github.com/RICE-unige/horus_sdk.git")
ros2_url = repos.get("horus_ros2", "https://github.com/RICE-unige/horus_ros2.git")

print(sdk_url)
print(selected["sdk_ref"])
print(ros2_url)
print(selected["ros2_ref"])
print(selected.get("notes", ""))
PY
)"; then
    die "Failed parsing release manifest channel '$channel'"
  fi

  mapfile -t lines <<<"$result"
  MANIFEST_SDK_URL="${lines[0]}"
  MANIFEST_SDK_REF="${lines[1]}"
  MANIFEST_ROS2_URL="${lines[2]}"
  MANIFEST_ROS2_REF="${lines[3]}"
  MANIFEST_NOTES="${lines[4]}"

  [ -n "$MANIFEST_SDK_URL" ] || die "Manifest sdk URL is empty"
  [ -n "$MANIFEST_SDK_REF" ] || die "Manifest sdk ref is empty"
  [ -n "$MANIFEST_ROS2_URL" ] || die "Manifest ros2 URL is empty"
  [ -n "$MANIFEST_ROS2_REF" ] || die "Manifest ros2 ref is empty"
}

ensure_git_repo_clean() {
  local repo_path="$1"
  local repo_label="$2"

  if [ ! -d "$repo_path/.git" ]; then
    die "$repo_label path exists but is not a git repository: $repo_path"
  fi

  local status
  status="$(git -C "$repo_path" status --porcelain)"
  if [ -n "$status" ]; then
    local short_status
    short_status="$(git -C "$repo_path" status --short)"
    die "$repo_label repository is dirty: $repo_path
Uncommitted changes:
$short_status
Use one of:
  - stash changes: git -C \"$repo_path\" stash push -u -m \"horus-installer-backup\"
  - commit changes in that repo
  - use a different install root with --install-root <path>"
  fi
}

checkout_ref() {
  local repo_path="$1"
  local ref="$2"

  run_silent "Fetching remote refs" git -C "$repo_path" fetch --tags --prune origin

  if [[ "$ref" == refs/heads/* ]]; then
    local branch
    branch="${ref#refs/heads/}"

    if git -C "$repo_path" show-ref --verify --quiet "refs/heads/$branch"; then
      run_silent "Checking out branch $branch" git -C "$repo_path" checkout "$branch"
    else
      run_silent "Creating tracking branch $branch" git -C "$repo_path" checkout -b "$branch" "origin/$branch"
    fi

    run_silent "Resetting to origin/$branch" git -C "$repo_path" reset --hard "origin/$branch"
    return 0
  fi

  if [[ "$ref" == refs/tags/* ]]; then
    local tag
    tag="${ref#refs/tags/}"
    run_silent "Checking out tag $tag" git -C "$repo_path" checkout --detach "tags/$tag"
    return 0
  fi

  local short_ref="${ref:0:12}"
  run_silent "Checking out ref $short_ref" git -C "$repo_path" checkout --detach "$ref"
}

sync_repo_to_ref() {
  local repo_url="$1"
  local ref="$2"
  local repo_path="$3"
  local repo_label="$4"

  if [ -e "$repo_path" ]; then
    ensure_git_repo_clean "$repo_path" "$repo_label"
    run_silent "Updating $repo_label remote URL" git -C "$repo_path" remote set-url origin "$repo_url"
  else
    run_silent "Cloning $repo_label" git clone "$repo_url" "$repo_path"
  fi

  checkout_ref "$repo_path" "$ref"
  _log_to_file "$repo_label synced at ref $ref"
}

prepare_install_root() {
  local target_root="$1"
  local assume_yes="$2"

  if [ -e "$target_root" ] && [ ! -d "$target_root" ]; then
    die "Install root exists and is not a directory: $target_root"
  fi

  mkdir -p "$target_root" "$target_root/bin" "$target_root/logs" "$target_root/state"

  local marker="$target_root/state/install-config.env"
  if [ -f "$marker" ]; then
    return 0
  fi

  if [ -n "$(ls -A "$target_root" 2>/dev/null)" ]; then
    local non_managed
    non_managed="$(find "$target_root" -mindepth 1 -maxdepth 1 \
      ! -name bin ! -name logs ! -name state -print -quit)"

    if [ -n "$non_managed" ]; then
      if [ "$assume_yes" -eq 1 ]; then
        die "Install root is non-empty and not managed by HORUS: $target_root"
      fi

      log_warn "Install root contains existing files not managed by HORUS: $target_root"
      die "Choose an empty path with --install-root <path>"
    fi
  fi
}
