#!/bin/bash

# HORUS SDK Carter Robot Registration Test
# Tests the complete robot registration flow with real backend

echo "🤖 HORUS SDK - Carter Robot Registration Test"
echo "============================================================"

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="$(dirname "$SCRIPT_DIR")"

# Source ROS2 workspace
echo "📦 Sourcing HORUS ROS2 workspace..."
source "$SDK_ROOT/horus_ros2_ws/install/setup.bash"

if [ $? -eq 0 ]; then
    echo "   ✓ ROS2 workspace sourced successfully"
else
    echo "   ❌ Failed to source ROS2 workspace"
    exit 1
fi

# Check if interfaces are built
echo "🔍 Checking HORUS interfaces..."
ros2 interface list | grep horus_interfaces > /dev/null

if [ $? -eq 0 ]; then
    echo "   ✓ HORUS interfaces available"
else
    echo "   ❌ HORUS interfaces not found, rebuilding..."
    cd "$SDK_ROOT/horus_ros2_ws"
    colcon build --packages-select horus_interfaces
    source install/setup.bash
    cd "$SDK_ROOT"
fi

# Build backend if needed
echo "🔧 Ensuring backend is built..."
cd "$SDK_ROOT/horus_ros2_ws"
colcon build --packages-select horus_backend
source install/setup.bash
cd "$SDK_ROOT"

# Run Carter setup with registration
echo "🚀 Starting Carter robot setup with registration..."
echo "   This will:"
echo "   1. Initialize HORUS SDK"
echo "   2. Create Carter robot model"
echo "   3. Register Carter with HORUS backend"
echo "   4. Monitor Carter's topics"
echo "   5. Connect to MR app"
echo ""

python3 "$SCRIPT_DIR/carter_robot_setup.py"
