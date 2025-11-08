# Sensors API

# Sensors API

## Overview

The Sensors API provides classes for integrating various sensor types with robots for Mixed Reality visualization.

## Usage

```python
from horus.sensors import Camera, LaserScan, Lidar3D

# Create sensors
camera = Camera("front_cam", "/camera/image_raw", "camera_link")
lidar_2d = LaserScan("lidar", "/scan", "base_scan")
lidar_3d = Lidar3D("velodyne", "/velodyne_points", "velodyne_link")

# Add to robot
robot.add_sensor(camera)
robot.add_sensor(lidar_2d)
robot.add_sensor(lidar_3d)
```

## Coming Soon

Complete API documentation will be automatically generated from the Python source code docstrings.
