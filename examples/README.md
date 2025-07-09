# HORUS SDK Examples

This directory contains examples demonstrating how to use the HORUS SDK for robot communication and Mixed Reality integration.

## Prerequisites

### 1. Choose Your Approach

**Option 1: Quick Test (Recommended - No Installation)**
```bash
# Just source the workspace and run
source horus_ros2_ws/install/setup.bash
python3 examples/quick_test.py
```

**Option 2: Install with Sudo (System-wide)**
```bash
sudo pip install -e python/
python3 examples/basic_initialization.py
```

**Option 3: Virtual Environment (If venv available)**
```bash
# Install python3-venv if needed
sudo apt install python3.10-venv  # Ubuntu/Debian

# Create and activate virtual environment
python3 -m venv horus_env
source horus_env/bin/activate
pip install -e python/
```

**Option 4: Direct from Python directory**
```bash
cd python/
python3 -c "from horus import Client; Client(backend='ros2')"
```

### 2. Build and Source ROS2 Workspace (Required for all options)
```bash
cd horus_ros2_ws
colcon build
source install/setup.bash
cd ..
```

**Note:** You must source the workspace in every terminal session before running examples.

## Available Examples

### 1. Basic Initialization (`basic_initialization.py`)

The simplest example showing how to initialize the SDK and start all backend services.
Requires the SDK to be installed via pip.

### 2. Quick Test (`quick_test.py`)

A development-friendly test that works without installing the SDK system-wide.
Automatically finds the SDK in the current directory structure.

### 3. Carter Robot Setup (`carter_robot_setup.py`)

Complete example showing how to set up a real robot (NVIDIA Carter) with HORUS:
- Creates robot model with sensors (camera, LiDAR)
- Configures data visualization for MR
- Registers robot with HORUS backend
- Demonstrates robot-to-MR integration flow

### 4. Live Robot Integration (`live_robot_integration.py`)

Advanced example for integrating live robots with automatic topic discovery:
- Auto-discovers robot capabilities from ROS2 topics
- Creates comprehensive robot model based on detected sensors
- Sets up complete MR visualization pipeline
- Provides real-time monitoring and status updates

### 5. Robot Creation Demo (`robot_creation.py`)

Simple example showing basic robot creation and sensor configuration.

### 6. Sensor Import Test (`sensor_import_test.py`)

Test example demonstrating sensor type imports and basic usage.

### 7. Data Visualization Demo (`robot_sensors_dataviz.py`)

Example showing advanced data visualization features for robot sensors.

### 8. Color Assignment Demo (`color_assignment_path_planning.py`)

Demonstrates automatic color assignment for multi-robot scenarios.

### 9. Shell Scripts
- `run_carter_with_registration.sh`: Automated Carter robot setup
- `run_live_robot_integration.sh`: Live robot integration launcher

**Run basic_initialization.py (requires installation):**
```bash
# Option A: System install (requires sudo)
sudo pip install -e python/
python3 examples/basic_initialization.py

# Option B: Virtual environment
python3 -m venv horus_env
source horus_env/bin/activate
pip install -e python/
python3 examples/basic_initialization.py
```

**Run quick_test.py (recommended - no installation):**
```bash
# Just source workspace and run
source horus_ros2_ws/install/setup.bash
python3 examples/quick_test.py
```

**Expected output:**
```
🚀 HORUS SDK Basic Initialization Example
==================================================

1. Importing HORUS SDK...
   ✓ SDK imported successfully

2. Initializing SDK with ROS2 backend...
[ASCII Art Display]
Initializing ROS2 backend
  ✓ ROS2 Installation: ROS2 humble detected
  ✓ HORUS Backend Package: HORUS backend package found
  ✓ Network Port 8080: Port 8080 is available
  ✓ Backend startup: Ready on port 8080
  ✓ Backend connection: Connected on port 8080

SDK initialized successfully

3. SDK initialized successfully!
   ✓ Backend services running
   ✓ Ready for robot communication

4. Keeping connection alive for 10 seconds...
   (Backend will continue running in background)
   10... 9... 8... 7... 6... 5... 4... 3... 2... 1... 

✅ Example completed successfully!
   Backend services remain active for other applications
```

## Architecture Overview

When you run these examples, the following services are automatically started:

```
Your Python Script
       ↓
   HORUS SDK
       ↓ (TCP port 8080)
   HORUS Backend (C++)
       ↓ (ROS2 topics/services)
   ROS-TCP-Endpoint
       ↓ (TCP port 10000)
   Unity MR Application
```

## Troubleshooting

### Import Error "No module named 'horus'"

**Solution 1: Use quick_test.py (recommended)**
```bash
source horus_ros2_ws/install/setup.bash
python3 examples/quick_test.py
```

**Solution 2: Install with sudo**
```bash
sudo pip install -e python/
```

**Solution 3: Use virtual environment**
```bash
python3 -m venv horus_env
source horus_env/bin/activate
pip install -e python/
```

### User Site Packages Disabled
If you see "WARNING: The user site-packages directory is disabled", use sudo or virtual environment instead of `pip install --user`.

### Backend Connection Failed
```bash
# Ensure ROS2 workspace is built and sourced
cd horus_ros2_ws
colcon build
source install/setup.bash

# Check if backend is running
ps aux | grep horus_backend
```

### Port in Use
```bash
# Kill existing backend processes
pkill -f horus_backend_node

# Check port availability
netstat -tlnp | grep 8080
```

## Example Usage Patterns

### Quick Development Testing
```bash
# Test SDK functionality without installation
source horus_ros2_ws/install/setup.bash
python3 examples/quick_test.py
```

### Robot Integration Workflow
```bash
# 1. Basic robot setup
python3 examples/carter_robot_setup.py

# 2. Live robot integration with auto-discovery
python3 examples/live_robot_integration.py

# 3. Advanced visualization demos
python3 examples/robot_sensors_dataviz.py
```

### Automated Execution
```bash
# Use shell scripts for automated testing
./examples/run_carter_with_registration.sh
./examples/run_live_robot_integration.sh
```

## Next Steps

After running these examples successfully, you can:

1. **Explore the reorganized SDK structure** in `python/horus/`
   - `robot/` - Robot management and control
   - `sensors/` - Sensor modeling and management
   - `dataviz/` - Data visualization for MR
   - `color/` - Color management for multi-robot scenarios
2. **Check the backend implementation** in `horus_ros2_ws/src/horus_backend/`
3. **Review the robot registration system** with ROS2 services
4. **Build your own robot integration** using the comprehensive examples

For more advanced usage, see the documentation in each SDK subdirectory.