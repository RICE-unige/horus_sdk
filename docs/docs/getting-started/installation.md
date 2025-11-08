# Installation Guide

This guide covers different installation methods for the HORUS SDK, from quick testing to production deployment.

## Prerequisites

### System Requirements

- **Operating System**: Ubuntu 22.04 LTS (recommended) or similar Linux distribution
- **Python**: 3.10 or higher
- **ROS2**: Humble Hawksbill (required for robot integration)
- **Meta Quest 3**: Required for full Mixed Reality experience

### Hardware Requirements

- **Development Machine**: Modern Linux workstation with network connectivity
- **Meta Quest 3**: With HORUS MR app installed
- **ROS2 Robots**: Any ROS2 Humble compatible robot system

### Network Requirements

- **Local Network**: All devices (development machine, Quest 3, robots) on same network
- **Open Ports**: 8080 (SDK backend), 10000 (Unity TCP bridge)

## Installation Methods

### Method 1: Quick Test (Recommended for First-Time Users)

This method requires no installation and is perfect for testing:

```bash
# Clone the repository with submodules
git clone --recursive https://github.com/RICE-unige/horus_sdk.git
cd horus_sdk

# Build the ROS2 workspace
cd horus_ros2_ws
colcon build
source install/setup.bash
cd ..

# Test the SDK
python3 examples/quick_test.py
```

**Expected Output:**
```
🚀 HORUS SDK Backend Initialization
=====================================

✓ ROS2 Installation: ROS2 humble detected
✓ HORUS Backend Package: Found and ready
✓ Network Port 8080: Available for SDK communication
✓ Backend startup: Ready on port 8080
✓ Quest 3 Connection: Ready for MR app on port 10000
```

### Method 2: Development Installation

For active development and custom robot integration:

```bash
# Clone and build
git clone --recursive https://github.com/RICE-unige/horus_sdk.git
cd horus_sdk

# Install Python SDK in development mode
pip install -e python/

# Build ROS2 workspace
cd horus_ros2_ws
colcon build
source install/setup.bash
cd ..

# Verify installation
python3 -c "from horus import Client; print('✓ HORUS SDK installed')"
```

### Method 3: Production Installation

For deployment in production environments:

```bash
# Install from PyPI (when available)
pip install horus-sdk

# Clone backend and examples
git clone --recursive https://github.com/RICE-unige/horus_sdk.git
cd horus_sdk/horus_ros2_ws
colcon build
source install/setup.bash
```

## Verification

### Test SDK Backend Connection

```bash
# Source ROS2 environment
source horus_ros2_ws/install/setup.bash

# Run backend test
python3 examples/quick_test.py
```

Look for these success indicators:
- ✓ ROS2 installation detected
- ✓ Backend package found
- ✓ Ports 8080 and 10000 available
- ✓ Backend server running

### Test Robot Registration

```bash
# Run robot registration example
python3 examples/carter_robot_setup.py
```

Expected output includes:
- Robot creation and sensor configuration
- Backend connection establishment
- Successful robot registration with assigned color
- Quest 3 connection monitoring

## Quest 3 Setup

### Install HORUS MR App

1. **Download HORUS MR App** from Meta Quest Store (when available)
2. **Enable Developer Mode** on Quest 3 if using development builds
3. **Connect to Same Network** as your development machine

### Configure Connection

1. **Note IP Address** from SDK backend output:
   ```
   🎮 HORUS Mixed Reality Connection Ready
   ═══════════════════════════════════════
   IP Address: 192.168.1.100
   Port: 10000
   ```

2. **Enter Connection Details** in Quest 3 HORUS app
3. **Verify Connection** - you should see:
   ```
   ✓ Quest 3 Connected: Device detected from 192.168.1.50
   ```

## Troubleshooting

### Common Issues

!!! warning "ROS2 Not Found"
    **Error**: `ROS2 Installation: Not detected`
    
    **Solution**:
    ```bash
    # Install ROS2 Humble
    sudo apt update
    sudo apt install ros-humble-desktop
    
    # Source ROS2
    source /opt/ros/humble/setup.bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```

!!! warning "Port Already in Use"
    **Error**: `Port 8080 is in use`
    
    **Solution**:
    ```bash
    # Kill existing processes
    pkill -f horus_backend_node
    pkill -f default_server_endpoint
    
    # Or use different port
    export HORUS_SDK_PORT=8081
    ```

!!! warning "Quest 3 Not Connecting"
    **Error**: No Quest 3 connection detected
    
    **Solution**:
    - Verify Quest 3 and development machine are on same network
    - Check firewall settings (ports 8080, 10000)
    - Restart HORUS MR app on Quest 3
    - Verify IP address configuration

### Build Issues

!!! warning "Colcon Build Failed"
    **Error**: Build failures in ROS2 workspace
    
    **Solution**:
    ```bash
    # Install dependencies
    sudo apt update
    sudo apt install python3-colcon-common-extensions
    
    # Clean and rebuild
    cd horus_ros2_ws
    rm -rf build install log
    colcon build
    ```

### Python Dependencies

!!! warning "Missing Python Modules"
    **Error**: `ModuleNotFoundError`
    
    **Solution**:
    ```bash
    # Install Python dependencies
    pip install -r python/requirements.txt
    
    # Or install specific modules
    pip install pyyaml rclpy
    ```

## Next Steps

After successful installation:

1. **[Quick Start](quickstart.md)** - Create your first robot connection
2. **[First Robot](first-robot.md)** - Detailed robot integration tutorial
3. **[Examples](../examples/index.md)** - Explore comprehensive robot demos

## Advanced Configuration

### Environment Variables

```bash
# Custom port configuration
export HORUS_SDK_PORT=8081
export HORUS_UNITY_PORT=10001

# Debug mode
export HORUS_DEBUG=true

# Custom ROS2 domain
export ROS_DOMAIN_ID=42
```

### Custom Robot Configurations

Create custom robot types in `~/.horus/robots/`:

```python
# ~/.horus/robots/custom_robot.py
from horus import Robot, RobotType
from horus.sensors import Camera, Lidar3D

def create_custom_robot():
    robot = Robot("custom_robot", RobotType.CUSTOM)
    # Add custom sensors and configuration
    return robot
```

For advanced configuration options, see the [Developer Guide](../developer-guide/index.md).
