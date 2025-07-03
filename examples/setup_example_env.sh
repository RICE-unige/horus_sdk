#!/bin/bash
# Setup script for HORUS SDK examples
# This script creates a virtual environment and installs the SDK

set -e

echo "🚀 Setting up HORUS SDK Example Environment"
echo "=" * 50

# Get the SDK root directory
SDK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "SDK Root: $SDK_ROOT"

# Check if python3-venv is available
if ! python3 -m venv --help >/dev/null 2>&1; then
    echo "⚠️  python3-venv not available. Install it with:"
    echo "   sudo apt install python3.10-venv"
    echo "   (or python3-venv on other systems)"
    echo ""
    echo "Alternative: Install SDK directly without virtual environment:"
    echo "   pip install -e \"$SDK_ROOT/python\""
    exit 1
fi

# Create virtual environment
echo "Creating virtual environment..."
python3 -m venv horus_example_env

# Activate virtual environment
echo "Activating virtual environment..."
source horus_example_env/bin/activate

# Install the SDK
echo "Installing HORUS SDK..."
pip install -e "$SDK_ROOT/python"

# Verify installation
echo "Verifying installation..."
python -c "import horus; print(f'✓ HORUS SDK {horus.__version__} installed successfully')"

echo ""
echo "✅ Setup complete!"
echo ""
echo "To use the examples:"
echo "  1. Activate the environment: source horus_example_env/bin/activate"
echo "  2. Build ROS2 workspace: cd horus_ros2_ws && colcon build && source install/setup.bash"
echo "  3. Run examples: python examples/basic_initialization.py"
echo ""
echo "To deactivate: deactivate"