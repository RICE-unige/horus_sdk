#!/usr/bin/env bash
# Source in a child shell, never change the user's interactive ROS settings.
HOSPITAL_SDK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/horus_ws/install/local_setup.bash" ]]; then
    source "$HOME/horus_ws/install/local_setup.bash"
fi
HOSPITAL_SLAM_PREFIX="${HORUS_HOSPITAL_SLAM_PREFIX:-/media/omotoye/New Volume/HORUS/tools/hospital-slam-toolbox/install}"
if [[ -f "$HOSPITAL_SLAM_PREFIX/local_setup.bash" ]]; then
    source "$HOSPITAL_SLAM_PREFIX/local_setup.bash"
fi
export ROS_DOMAIN_ID="${HORUS_HOSPITAL_DOMAIN:-87}"
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI ROS_DISCOVERY_SERVER ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOSPITAL_SDK_ROOT/simulation/hospital/fastdds-local.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export PYTHONPATH="$HOSPITAL_SDK_ROOT/python:$HOSPITAL_SDK_ROOT/simulation/hospital${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1
export HORUS_SDK_BRIDGE_AUTOSTART_MODE=ros2
