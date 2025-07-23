# Your First Robot in Mixed Reality

This tutorial will guide you through connecting your first robot to the HORUS Mixed Reality system running on Quest 3.

## Overview

By the end of this tutorial, you'll have:

- ✅ Connected a robot to the HORUS MR system
- ✅ Configured sensors for 3D visualization
- ✅ Seen your robot appear in Quest 3 with live data
- ✅ Understood the complete robot-to-MR workflow

## Prerequisites

Before starting, ensure you have:

- **HORUS SDK installed** (see [Installation Guide](installation.md))
- **ROS2 Humble** environment sourced
- **Quest 3** with HORUS MR app ready
- **Network connectivity** between all devices

## Step 1: Initialize the SDK

Create a new file called `my_first_mr_robot.py`:

```python title="my_first_mr_robot.py"
#!/usr/bin/env python3
"""
My First HORUS Mixed Reality Robot
==================================
This example shows how to connect a robot to Quest 3 MR visualization
"""

from horus import Client, Robot, RobotType
from horus.sensors import Camera, LaserScan, Lidar3D
from horus.dataviz import DataViz
import time

def main():
    print("🚀 HORUS Mixed Reality Robot Setup")
    print("===================================")
    
    # Step 1: Initialize SDK backend connection
    print("\n1. Connecting to HORUS backend...")
    client = Client(backend='ros2')
    print("   ✓ Connected to MR backend system")
```

## Step 2: Create Your Robot

Add robot creation to your script:

```python
    # Step 2: Create robot for MR visualization
    print("\n2. Creating robot for Mixed Reality...")
    robot = Robot(
        name="my_first_mr_robot",
        robot_type=RobotType.WHEELED,
        description="My first robot in HORUS MR"
    )
    print(f"   ✓ Robot created: {robot.name}")
```

## Step 3: Configure Sensors for MR

Add sensors that will appear in Quest 3:

```python
    # Step 3: Add sensors for MR visualization
    print("\n3. Configuring sensors for Quest 3...")
    
    # Front camera - will show live feed in MR
    front_camera = Camera(
        name="front_camera",
        topic="/camera/image_raw",
        frame_id="camera_link",
        description="Front-facing camera for MR overlay"
    )
    
    # LiDAR - will show 3D point cloud in MR
    lidar = LaserScan(
        name="front_lidar",
        topic="/scan",
        frame_id="base_scan",
        description="Front LiDAR for 3D environment mapping"
    )
    
    # Optional: 3D LiDAR for richer point clouds
    # lidar_3d = Lidar3D(
    #     name="3d_lidar",
    #     topic="/lidar_points",
    #     frame_id="lidar_link"
    # )
    
    robot.add_sensor(front_camera)
    robot.add_sensor(lidar)
    
    print(f"   ✓ Added {robot.get_sensor_count()} sensors for MR")
```

## Step 4: Create MR Visualization

Configure how your robot appears in Quest 3:

```python
    # Step 4: Create Mixed Reality visualization
    print("\n4. Setting up MR visualization...")
    
    # Create 3D visualization configuration
    dataviz = robot.create_dataviz()
    
    # Add path planning visualization (if your robot has navigation)
    robot.add_path_planning_to_dataviz(
        dataviz,
        global_path_topic="/global_path",
        local_path_topic="/local_path"
    )
    
    print("   ✓ MR visualization configured")
```

## Step 5: Register with HORUS MR System

Connect your robot to Quest 3:

```python
    # Step 5: Register robot with HORUS MR system
    print("\n5. Registering robot with Quest 3...")
    
    success, result = robot.register_with_horus(dataviz)
    
    if success:
        print("   ✅ SUCCESS! Robot registered with HORUS MR")
        print(f"   📱 Robot ID: {result.get('robot_id')}")
        print(f"   🎨 MR Color: {result.get('assigned_color')}")
        print(f"   📡 Sensors: {robot.get_sensor_count()} active")
        
        # Display connection info for Quest 3
        print("\n🎮 Quest 3 Connection Info:")
        print("   ===========================")
        print(f"   IP Address: {result.get('backend_ip', 'localhost')}")
        print(f"   Port: 10000")
        print("   Status: Ready for MR connection")
        
        return True
    else:
        print(f"   ❌ Registration failed: {result.get('error')}")
        return False
```

## Step 6: Keep Robot Active

Add the monitoring loop:

```python
    # Step 6: Keep robot active for MR
    print("\n6. Robot active - Ready for Quest 3!")
    print("   ====================================")
    print("   🔄 Robot is now streaming to MR system")
    print("   📱 Open HORUS app on Quest 3")
    print("   🎯 Look for your robot in 3D space")
    print("\n   Press Ctrl+C to stop robot...")
    
    try:
        while True:
            # Robot stays active for MR visualization
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down robot...")
        print("   ✓ Robot disconnected from MR system")

if __name__ == "__main__":
    main()
```

## Complete Script

Here's the complete `my_first_mr_robot.py`:

```python title="my_first_mr_robot.py"
#!/usr/bin/env python3
"""
My First HORUS Mixed Reality Robot
==================================
This example shows how to connect a robot to Quest 3 MR visualization
"""

from horus import Client, Robot, RobotType
from horus.sensors import Camera, LaserScan
from horus.dataviz import DataViz
import time

def main():
    print("🚀 HORUS Mixed Reality Robot Setup")
    print("===================================")
    
    # Step 1: Initialize SDK backend connection
    print("\n1. Connecting to HORUS backend...")
    client = Client(backend='ros2')
    print("   ✓ Connected to MR backend system")
    
    # Step 2: Create robot for MR visualization
    print("\n2. Creating robot for Mixed Reality...")
    robot = Robot(
        name="my_first_mr_robot",
        robot_type=RobotType.WHEELED,
        description="My first robot in HORUS MR"
    )
    print(f"   ✓ Robot created: {robot.name}")
    
    # Step 3: Add sensors for MR visualization
    print("\n3. Configuring sensors for Quest 3...")
    
    front_camera = Camera(
        name="front_camera",
        topic="/camera/image_raw",
        frame_id="camera_link",
        description="Front-facing camera for MR overlay"
    )
    
    lidar = LaserScan(
        name="front_lidar",
        topic="/scan",
        frame_id="base_scan",
        description="Front LiDAR for 3D environment mapping"
    )
    
    robot.add_sensor(front_camera)
    robot.add_sensor(lidar)
    
    print(f"   ✓ Added {robot.get_sensor_count()} sensors for MR")
    
    # Step 4: Create Mixed Reality visualization
    print("\n4. Setting up MR visualization...")
    
    dataviz = robot.create_dataviz()
    
    robot.add_path_planning_to_dataviz(
        dataviz,
        global_path_topic="/global_path",
        local_path_topic="/local_path"
    )
    
    print("   ✓ MR visualization configured")
    
    # Step 5: Register robot with HORUS MR system
    print("\n5. Registering robot with Quest 3...")
    
    success, result = robot.register_with_horus(dataviz)
    
    if success:
        print("   ✅ SUCCESS! Robot registered with HORUS MR")
        print(f"   📱 Robot ID: {result.get('robot_id')}")
        print(f"   🎨 MR Color: {result.get('assigned_color')}")
        print(f"   📡 Sensors: {robot.get_sensor_count()} active")
        
        print("\n🎮 Quest 3 Connection Info:")
        print("   ===========================")
        print(f"   IP Address: {result.get('backend_ip', 'localhost')}")
        print(f"   Port: 10000")
        print("   Status: Ready for MR connection")
        
        # Step 6: Keep robot active for MR
        print("\n6. Robot active - Ready for Quest 3!")
        print("   ====================================")
        print("   🔄 Robot is now streaming to MR system")
        print("   📱 Open HORUS app on Quest 3")
        print("   🎯 Look for your robot in 3D space")
        print("\n   Press Ctrl+C to stop robot...")
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n🛑 Shutting down robot...")
            print("   ✓ Robot disconnected from MR system")
            
    else:
        print(f"   ❌ Registration failed: {result.get('error')}")

if __name__ == "__main__":
    main()
```

## Run Your Robot

Execute your script:

```bash
# Make sure ROS2 is sourced
source horus_ros2_ws/install/setup.bash

# Run your MR robot
python3 my_first_mr_robot.py
```

## Expected Output

You should see:

```
🚀 HORUS Mixed Reality Robot Setup
===================================

1. Connecting to HORUS backend...
   ✓ Connected to MR backend system

2. Creating robot for Mixed Reality...
   ✓ Robot created: my_first_mr_robot

3. Configuring sensors for Quest 3...
   ✓ Added 2 sensors for MR

4. Setting up MR visualization...
   ✓ MR visualization configured

5. Registering robot with Quest 3...
   ✅ SUCCESS! Robot registered with HORUS MR
   📱 Robot ID: robot_001
   🎨 MR Color: #FF6B35
   📡 Sensors: 2 active

🎮 Quest 3 Connection Info:
   ===========================
   IP Address: 192.168.1.100
   Port: 10000
   Status: Ready for MR connection

6. Robot active - Ready for Quest 3!
   ====================================
   🔄 Robot is now streaming to MR system
   📱 Open HORUS app on Quest 3
   🎯 Look for your robot in 3D space

   Press Ctrl+C to stop robot...
```

## Quest 3 Experience

Once your robot is registered:

1. **Put on Quest 3 headset**
2. **Open HORUS MR app**
3. **Connect to IP address shown** (192.168.1.100:10000)
4. **Look around** - you should see your robot in 3D space
5. **Robot appears with unique color** (e.g., orange #FF6B35)
6. **Live sensor data** overlays in MR (camera feed, LiDAR points)

## Troubleshooting

### Robot Not Appearing in Quest 3

!!! warning "No Robot Visible"
    **Problem**: Robot registered but not visible in Quest 3
    
    **Solutions**:
    - Verify Quest 3 connected to same network
    - Check IP address and port 10000
    - Restart HORUS MR app
    - Confirm robot script is still running

### Connection Issues

!!! warning "Registration Failed"
    **Problem**: Robot registration fails
    
    **Solutions**:
    ```bash
    # Check backend status
    python3 examples/quick_test.py
    
    # Verify ROS2 environment
    source horus_ros2_ws/install/setup.bash
    ros2 node list
    ```

## Next Steps

Congratulations! You've successfully connected your first robot to HORUS Mixed Reality. Next:

1. **[User Guide](../user-guide/index.md)** - Learn about advanced MR features
2. **[Examples](../examples/index.md)** - Explore multi-robot scenarios
3. **[API Reference](../api/index.md)** - Dive deeper into SDK capabilities

## Advanced Features

### Multi-Robot Fleet

```python
# Create multiple robots for fleet management
robot_1 = Robot("scout_01", RobotType.WHEELED)
robot_2 = Robot("scout_02", RobotType.WHEELED)

# Each gets unique color automatically
# Quest 3 shows both robots simultaneously
```

### Custom Robot Types

```python
# Create custom robot configurations
custom_robot = Robot("delivery_bot", RobotType.CUSTOM)
custom_robot.add_custom_property("payload_capacity", "50kg")
custom_robot.add_custom_property("battery_type", "LiFePO4")
```

### Advanced Sensor Configuration

```python
# High-resolution 3D LiDAR
lidar_3d = Lidar3D(
    name="velodyne_128",
    topic="/velodyne_points",
    frame_id="velodyne_link",
    range_max=200.0,
    point_cloud_color="#00FF00"
)
```

For more advanced features, see the [User Guide](../user-guide/index.md) and [Developer Guide](../developer-guide/index.md).
