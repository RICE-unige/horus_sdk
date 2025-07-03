#!/bin/bash
# Helper script to run HORUS SDK examples
# This script handles the ROS2 workspace sourcing automatically

set -e

# Get the SDK root directory (parent of examples/)
SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_SETUP="$SDK_ROOT/horus_ros2_ws/install/setup.bash"

echo "🚀 HORUS SDK Example Runner"
echo "SDK Root: $SDK_ROOT"

# Check if workspace is built
if [ ! -f "$WORKSPACE_SETUP" ]; then
    echo "❌ ROS2 workspace not built. Please run:"
    echo "   cd $SDK_ROOT/horus_ros2_ws"
    echo "   colcon build"
    exit 1
fi

# Source the workspace
echo "📦 Sourcing ROS2 workspace..."
source "$WORKSPACE_SETUP"

# Determine which example to run
EXAMPLE="${1:-quick_test.py}"

echo "🏃 Running: $EXAMPLE"
echo "=" * 50

cd "$SDK_ROOT"

case "$EXAMPLE" in
    "quick" | "quick_test" | "quick_test.py")
        python3 examples/quick_test.py
        ;;
    "basic" | "basic_initialization" | "basic_initialization.py")
        echo "⚠️  This requires SDK installation. Use 'quick' for no-install testing."
        python3 examples/basic_initialization.py
        ;;
    *)
        echo "Available examples:"
        echo "  quick_test.py (recommended - no installation needed)"
        echo "  basic_initialization.py (requires SDK installation)"
        echo ""
        echo "Usage: $0 [quick|basic]"
        exit 1
        ;;
esac