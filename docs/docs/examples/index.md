# Examples

Comprehensive examples demonstrating HORUS Mixed Reality robot fleet management capabilities.

## Overview

This section contains real-world examples showcasing how to connect various robot types to the HORUS MR system running on Quest 3.

## Available Examples

### Getting Started Examples

<div class="grid cards" markdown>

-   :material-clock-fast:{ .lg .middle } **Basic Robot Connection**

    ---

    Simple robot registration and MR visualization setup.

    [:octicons-arrow-right-24: Basic Example](basic.md)

-   :material-robot:{ .lg .middle } **Carter Robot Integration**

    ---

    Complete Carter robot setup with sensors and navigation.

    [:octicons-arrow-right-24: Carter Example](carter.md)

-   :material-account-group:{ .lg .middle } **Multi-Robot Fleet**

    ---

    Managing multiple robots simultaneously in MR.

    [:octicons-arrow-right-24: Multi-Robot Example](multi-robot.md)

-   :material-puzzle:{ .lg .middle } **Custom Robot Plugins**

    ---

    Creating custom robot types and sensor configurations.

    [:octicons-arrow-right-24: Plugin Example](plugins.md)

</div>

## Example Categories

### Basic Usage
- **Quick Test**: Basic SDK initialization and backend connection
- **First Robot**: Step-by-step robot registration tutorial
- **Sensor Integration**: Adding cameras and LiDAR to robots

### Advanced Features
- **Multi-Robot Management**: Fleet coordination and color management
- **Live Integration**: Real-time robot data streaming to Quest 3
- **Path Planning**: Navigation visualization in Mixed Reality

### Real Robot Examples
- **Carter Robot**: NVIDIA Isaac Sim robot integration
- **Custom Robots**: Generic ROS2 robot setup
- **Sensor Configurations**: Various sensor combinations

## Running Examples

### Prerequisites

Before running examples, ensure you have:

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build and source HORUS workspace
cd horus_ros2_ws
colcon build
source install/setup.bash
cd ..
```

### Basic Example Structure

All examples follow this pattern:

```python
#!/usr/bin/env python3
"""
Example: [Description]
Purpose: [What this example demonstrates]
"""

from horus import Client, Robot, RobotType
from horus.sensors import Camera, LaserScan
from horus.dataviz import DataViz

def main():
    # 1. Initialize SDK
    client = Client(backend='ros2')
    
    # 2. Create robot
    robot = Robot("example_robot", RobotType.WHEELED)
    
    # 3. Configure sensors
    camera = Camera("camera", "/camera/image_raw", "camera_link")
    robot.add_sensor(camera)
    
    # 4. Create MR visualization
    dataviz = robot.create_dataviz()
    
    # 5. Register with HORUS MR system
    success, result = robot.register_with_horus(dataviz)
    
    if success:
        print(f"✓ Robot registered with Quest 3: {result['robot_id']}")
        # Keep robot active for MR
        input("Press Enter to stop...")
    else:
        print(f"✗ Registration failed: {result['error']}")

if __name__ == "__main__":
    main()
```

## Available Example Files

The following example files are available in the repository:

| File | Description | Complexity |
|------|-------------|------------|
| `quick_test.py` | Basic SDK initialization and backend test | Beginner |
| `carter_robot_setup.py` | Complete Carter robot with sensors | Intermediate |
| `live_robot_integration.py` | Multi-robot live integration | Advanced |
| `color_assignment_path_planning.py` | Color management and path visualization | Intermediate |
| `basic_initialization.py` | Simple robot registration | Beginner |

## Example Usage Patterns

### Single Robot Pattern

```python
# Create and register a single robot
robot = Robot("scout_01", RobotType.WHEELED)
robot.add_sensor(Camera("front_cam", "/camera/image_raw", "camera_link"))
dataviz = robot.create_dataviz()
success, result = robot.register_with_horus(dataviz)
```

### Multi-Robot Pattern

```python
# Create multiple robots with automatic color assignment
robots = []
for i in range(3):
    robot = Robot(f"robot_{i:02d}", RobotType.WHEELED)
    robot.add_sensor(Camera(f"cam_{i}", f"/robot_{i}/camera/image_raw", "camera_link"))
    dataviz = robot.create_dataviz()
    success, result = robot.register_with_horus(dataviz)
    if success:
        robots.append(robot)
```

### Sensor Configuration Pattern

```python
# Comprehensive sensor setup
def setup_sensors(robot: Robot):
    # Multiple cameras
    front_cam = Camera("front_cam", "/front_camera/image_raw", "front_camera_link")
    rear_cam = Camera("rear_cam", "/rear_camera/image_raw", "rear_camera_link")
    
    # LiDAR sensors
    lidar_2d = LaserScan("lidar_2d", "/scan", "base_scan")
    lidar_3d = Lidar3D("lidar_3d", "/velodyne_points", "velodyne_link")
    
    # Add all sensors
    for sensor in [front_cam, rear_cam, lidar_2d, lidar_3d]:
        robot.add_sensor(sensor)
```

## Testing Examples

### Running Individual Examples

```bash
# Run basic robot connection test
python3 examples/quick_test.py

# Run Carter robot example
python3 examples/carter_robot_setup.py

# Run multi-robot example
python3 examples/live_robot_integration.py
```

### Quest 3 Verification

After running examples:

1. **Note connection details** from terminal output
2. **Open HORUS MR app** on Quest 3
3. **Enter IP and port** shown in terminal
4. **Look for robots** in 3D space with assigned colors

## Troubleshooting Examples

### Common Issues

!!! warning "Robot Not Visible in Quest 3"
    **Problem**: Example runs but robot not visible in MR
    
    **Solution**:
    - Verify Quest 3 connected to same network
    - Check IP address and port configuration
    - Ensure example script is still running
    - Restart HORUS MR app

!!! warning "Registration Failed"
    **Problem**: Robot registration fails in example
    
    **Solution**:
    ```bash
    # Check backend status
    python3 examples/quick_test.py
    
    # Verify ROS2 environment
    source horus_ros2_ws/install/setup.bash
    ```

### Debug Mode

Enable debug output in examples:

```python
import logging
logging.basicConfig(level=logging.DEBUG)

# Run example with debug output
client = Client(backend='ros2', debug=True)
```

## Contributing Examples

To add new examples:

1. **Create example file** in `examples/` directory
2. **Follow naming convention**: `descriptive_name.py`
3. **Include comprehensive docstring** with purpose and usage
4. **Add error handling** and user-friendly output
5. **Test with Quest 3** to verify MR functionality

### Example Template

```python
#!/usr/bin/env python3
"""
HORUS Example: [Your Example Name]
==================================

Purpose: [What this example demonstrates]
Requirements: [Any special requirements]
Usage: python3 examples/your_example.py

This example shows how to [describe functionality]
"""

from horus import Client, Robot, RobotType
import time

def main():
    """Main example function."""
    print("🚀 HORUS Example: [Your Example Name]")
    print("=" * 50)
    
    try:
        # Your example code here
        client = Client(backend='ros2')
        # ... rest of example
        
        print("✓ Example completed successfully!")
        
    except Exception as e:
        print(f"✗ Example failed: {e}")
        return False
    
    return True

if __name__ == "__main__":
    main()
```

## Next Steps

After exploring examples:

1. **[User Guide](../user-guide/index.md)** - Comprehensive feature documentation
2. **[API Reference](../api/index.md)** - Complete API documentation
3. **[Developer Guide](../developer-guide/index.md)** - Advanced development topics

---

Choose an example above to start exploring HORUS Mixed Reality robot fleet management capabilities!
