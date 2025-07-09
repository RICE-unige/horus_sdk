#!/bin/bash

# HORUS SDK Live Robot Integration Runner
# This script properly sources the ROS2 workspace and runs the integration

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="$(dirname "$SCRIPT_DIR")"

echo "🤖 HORUS SDK Live Robot Integration Runner"
echo "============================================================"

# Source ROS2 workspace
echo "📦 Sourcing HORUS ROS2 workspace..."
source "$SDK_ROOT/horus_ros2_ws/install/setup.bash"

if [ $? -eq 0 ]; then
    echo "   ✓ ROS2 workspace sourced successfully"
else
    echo "   ❌ Failed to source ROS2 workspace"
    exit 1
fi

# Check if HORUS backend package is available
echo "🔍 Checking HORUS backend availability..."
ros2 pkg list | grep horus_backend > /dev/null

if [ $? -eq 0 ]; then
    echo "   ✓ HORUS backend package found"
else
    echo "   ❌ HORUS backend package not found"
    echo "   Building workspace..."
    cd "$SDK_ROOT/horus_ros2_ws"
    colcon build
    source install/setup.bash
    cd "$SDK_ROOT"
fi

# Run the integration script
echo "🚀 Starting live robot integration..."
python3 "$SCRIPT_DIR/live_robot_integration.py"