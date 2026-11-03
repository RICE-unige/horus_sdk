#!/usr/bin/env bash
# Reproduce only the isolated hospital dependency, never replace system ROS.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PREFIX="${HORUS_HOSPITAL_SLAM_PREFIX:-/media/omotoye/New Volume/HORUS/tools/hospital-slam-toolbox/install}"
BASE="$(dirname -- "$PREFIX")"
REVISION=02afdde003313a10b8d21461a92d9e5f4f0bc5f2
SOURCE="$BASE/source"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    printf '%s\n' 'ROS 2 Jazzy is required; this script does not install system packages.' >&2
    exit 1
fi
if [[ ! -d "$(dirname -- "$BASE")" ]]; then
    printf 'Data-drive parent is unavailable: %s\n' "$(dirname -- "$BASE")" >&2
    exit 1
fi
if [[ ! -d "$SOURCE" ]]; then
    mkdir -p -- "$BASE"
    git clone --filter=blob:none --no-checkout https://github.com/SteveMacenski/slam_toolbox.git "$SOURCE"
    git -C "$SOURCE" checkout --detach "$REVISION"
fi
if [[ "$(git -C "$SOURCE" rev-parse HEAD)" != "$REVISION" ]]; then
    printf '%s\n' 'Existing SLAM source has another revision; it was not changed.' >&2
    exit 1
fi
if [[ -z "$(git -C "$SOURCE" status --porcelain)" ]]; then
    git -C "$SOURCE" apply --check "$HERE/cooperative-slam-jazzy.patch"
    git -C "$SOURCE" apply "$HERE/cooperative-slam-jazzy.patch"
fi
(
    cd -- "$SOURCE"
    sha256sum --check "$HERE/cooperative-slam.sha256"
)
# Existing modified files are not reset or overwritten when these checks fail.
source /opt/ros/jazzy/setup.bash
MAKEFLAGS=-j6 /usr/bin/colcon --log-base "$BASE/log" build \
    --base-paths "$SOURCE" --build-base "$BASE/build" --install-base "$PREFIX" \
    --packages-select slam_toolbox --allow-overriding slam_toolbox \
    --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
printf 'Hospital cooperative SLAM is available in %s\n' "$PREFIX"
