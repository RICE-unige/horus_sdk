# Quick Start Guide

Get up and running with HORUS SDK in less than 5 minutes!

## Prerequisites

Before you begin, ensure you have:

- **Python 3.10+** installed
- **ROS2 Humble** installed and sourced
- **Git** for cloning the repository
- **Meta Quest 3** (optional, for full MR experience)

!!! tip "Quick Check"
    Verify your setup:
    ```bash
    python3 --version  # Should be 3.10+
    ros2 --version     # Should show ROS2 Humble
    ```

## 1. Clone and Build

```bash
# Clone the repository with submodules
git clone --recursive https://github.com/RICE-unige/horus_sdk.git
cd horus_sdk

# Build the ROS2 workspace
cd horus_ros2_ws
colcon build
source install/setup.bash
cd ..
```

## 2. Test Your Installation

Run the quick test to verify everything works:

```bash
python3 examples/quick_test.py
```

**Expected Output:**
```
🚀 HORUS SDK Basic Initialization Example
==================================================

1. Importing HORUS SDK...
   ✓ SDK imported successfully

2. Initializing SDK with ROS2 backend...
Initializing ROS2 backend
  ✓ ROS2 Installation: ROS2 humble detected
  ✓ HORUS Backend Package: HORUS backend package found  
  ✓ Network Port 8080: Port 8080 is available
  ✓ Backend startup: Ready on port 8080
  ✓ Backend connection: Connected on port 8080

SDK initialized successfully
```

## 3. Create Your First Robot

Create a new Python file `my_first_robot.py`:

```python title="my_first_robot.py"
#!/usr/bin/env python3

from horus import Client, Robot, RobotType
from horus.sensors import Camera, LaserScan
from horus.dataviz import DataViz

def main():
    # Initialize HORUS SDK
    print("🚀 Initializing HORUS SDK...")
    client = Client(backend='ros2')
    
    # Create a robot
    print("🤖 Creating robot...")
    robot = Robot(
        name="my_first_robot",
        robot_type=RobotType.WHEELED
    )
    
    # Add sensors
    print("📷 Adding sensors...")
    camera = Camera(
        name="front_camera",
        topic="/camera/image_raw",
        frame_id="camera_link"
    )
    
    lidar = LaserScan(
        name="lidar",
        topic="/scan",
        frame_id="base_scan"
    )
    
    robot.add_sensor(camera)
    robot.add_sensor(lidar)
    
    # Create data visualization
    print("🎨 Setting up visualization...")
    dataviz = robot.create_dataviz()
    
    # Add path planning visualization
    robot.add_path_planning_to_dataviz(
        dataviz,
        global_path_topic="/path",
        local_path_topic="/local_path"
    )
    
    # Register with HORUS
    print("📡 Registering robot with HORUS...")
    success, result = robot.register_with_horus(dataviz)
    
    if success:
        print(f"✅ Robot registered successfully!")
        print(f"   Robot ID: {result.get('robot_id')}")
        print(f"   Assigned Color: {result.get('assigned_color')}")
        print(f"   Total Sensors: {robot.get_sensor_count()}")
        
        # Keep the connection alive
        print("\n🔄 Robot is now active. Press Ctrl+C to stop.")
        try:
            import time
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n🛑 Shutting down...")
    else:
        print(f"❌ Registration failed: {result.get('error')}")

if __name__ == "__main__":
    main()
```

Run your robot:

```bash
# Make sure ROS2 workspace is sourced
source horus_ros2_ws/install/setup.bash

# Run your robot
python3 my_first_robot.py
```

## 4. Connect Quest 3 (Optional)

If you have a Meta Quest 3, you can see your robot in Mixed Reality:

1. **Launch HORUS backend** (if not already running):
   ```bash
   source horus_ros2_ws/install/setup.bash
   python3 examples/quick_test.py
   ```

2. **Note the connection details** displayed:
   ```
   🎮 HORUS Mixed Reality Connection
   ═══════════════════════════════════════════════
     📡 Connection Details:
        IP Address: 192.168.1.100
        Port:       10000
        Protocol:   TCP
   ```

3. **In your Quest 3 HORUS app**, enter the IP and port shown

4. **Watch real-time connection status**:
   ```
   ✓ HORUS MR connection: Connected from 192.168.1.50
     → Mixed Reality interface active
   ```

## What's Next?

🎉 **Congratulations!** You've successfully:

- ✅ Set up HORUS SDK
- ✅ Created your first robot
- ✅ Connected to the MR system
- ✅ Registered sensors and visualization

### Next Steps

=== "Explore Examples"
    
    Check out more comprehensive examples:
    
    ```bash
    # Carter robot with full setup
    python3 examples/carter_robot_setup.py
    
    # Multi-robot management
    python3 examples/live_robot_integration.py
    
    # Color management demo
    python3 examples/color_assignment_path_planning.py
    ```

=== "Learn the API"
    
    Dive into the API documentation:
    
    - [Robot Management](../user-guide/robots.md) - Learn about robot types and configuration
    - [Sensors](../user-guide/sensors.md) - Understand sensor integration
    - [Data Visualization](../user-guide/dataviz.md) - Master MR visualization
    - [API Reference](../api/index.md) - Complete API documentation

=== "Build Real Robots"
    
    Connect HORUS to real robot systems:
    
    - Set up ROS2 topics for your robot
    - Configure sensors and transforms
    - Implement robot-specific plugins
    - Deploy to production environments

## Troubleshooting

### Common Issues

!!! warning "Port Already in Use"
    
    **Error**: `Port 8080 is in use`
    
    **Solution**:
    ```bash
    # Kill existing backend processes
    pkill -f horus_backend_node
    pkill -f default_server_endpoint
    ```

!!! warning "ROS2 Not Found"
    
    **Error**: `ROS2 Installation: Not detected`
    
    **Solution**:
    ```bash
    # Source ROS2 Humble
    source /opt/ros/humble/setup.bash
    
    # Source the workspace
    source horus_ros2_ws/install/setup.bash
    ```

!!! warning "Import Error"
    
    **Error**: `No module named 'horus'`
    
    **Solution**: Use the no-installation method or install the SDK:
    ```bash
    # Option 1: No installation (recommended)
    python3 examples/quick_test.py
    
    # Option 2: Install SDK
    pip install -e python/
    ```

### Getting Help

- 📖 **Documentation**: [User Guide](../user-guide/index.md)
- 🐛 **Issues**: [GitHub Issues](https://github.com/RICE-unige/horus_sdk/issues)
- 💬 **Discussions**: [GitHub Discussions](https://github.com/RICE-unige/horus_sdk/discussions)

---

<div align="center">
  **Ready for more advanced features?**
  
  [User Guide](../user-guide/index.md){ .md-button .md-button--primary }
  [API Reference](../api/index.md){ .md-button }
</div>
