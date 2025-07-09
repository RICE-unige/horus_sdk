#!/usr/bin/env python3
"""
HORUS SDK Robot Sensors and DataViz Example

This example demonstrates:
1. Creating robots with different sensors
2. Setting up data visualization for sensor and non-sensor data
3. Managing robot-specific and global visualizations
4. Working with different data source types
"""

import sys
import os

# Add the SDK to path (for development without installation)
script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(script_dir, 'python'))

from horus import Robot, RobotType, SensorType, Camera, LaserScan, Lidar3D, DataViz, VisualizationType

def main():
    """Demonstrate robot sensors and data visualization"""
    print("🤖 HORUS SDK Robot Sensors and DataViz Example")
    print("=" * 55)
    
    # ================================================
    # Example 1: Create robots with different sensors
    # ================================================
    print("\n1. Creating robots with sensors...")
    
    # Wheeled robot with camera and laser scanner
    wheeled_robot = Robot("rosbot_01", RobotType.WHEELED)
    
    # Add camera sensor
    front_camera = Camera(
        name="front_camera",
        frame_id="rosbot_01/camera_link",
        topic="/rosbot_01/camera/image_raw",
        is_stereo=False,
        resolution=(1920, 1080),
        fps=30,
        fov=60.0
    )
    wheeled_robot.add_sensor(front_camera)
    
    # Add laser scanner
    laser_scan = LaserScan(
        name="front_laser",
        frame_id="rosbot_01/laser_link",
        topic="/rosbot_01/scan",
        min_angle=-3.14159,
        max_angle=3.14159,
        max_range=30.0
    )
    wheeled_robot.add_sensor(laser_scan)
    
    print(f"   Created: {wheeled_robot}")
    print(f"   Sensors: {wheeled_robot.get_sensor_count()}")
    for sensor in wheeled_robot.sensors:
        print(f"     - {sensor}")
    
    # Aerial robot with 3D LiDAR and stereo camera
    print("\n   Creating aerial robot with advanced sensors...")
    aerial_robot = Robot("drone_alpha", RobotType.AERIAL)
    
    # Add 3D LiDAR
    lidar_3d = Lidar3D(
        name="velodyne_lidar",
        frame_id="drone_alpha/lidar_link",
        topic="/drone_alpha/velodyne_points",
        vertical_fov=40.0,
        horizontal_fov=360.0,
        num_layers=64,
        max_range=100.0
    )
    aerial_robot.add_sensor(lidar_3d)
    
    # Add stereo camera
    stereo_camera = Camera(
        name="stereo_camera",
        frame_id="drone_alpha/camera_link",
        topic="/drone_alpha/stereo/left/image_raw",
        is_stereo=True,
        resolution=(1280, 720),
        fps=60
    )
    aerial_robot.add_sensor(stereo_camera)
    
    print(f"   Created: {aerial_robot}")
    print(f"   Sensors: {aerial_robot.get_sensor_count()}")
    for sensor in aerial_robot.sensors:
        print(f"     - {sensor}")
    
    # ================================================
    # Example 2: Robot-specific DataViz
    # ================================================
    print("\n2. Creating robot-specific data visualization...")
    
    # Create DataViz for wheeled robot using the built-in method
    wheeled_dataviz = wheeled_robot.create_dataviz()
    
    # Add robot path visualization
    wheeled_dataviz.add_robot_path(
        robot_name="rosbot_01",
        topic="/rosbot_01/path",
        render_options={"color": "blue", "line_width": 2}
    )
    
    print(f"   {wheeled_dataviz}")
    print(f"   Robot visualizations: {len(wheeled_dataviz.get_robot_visualizations('rosbot_01'))}")
    
    # Create DataViz for aerial robot
    aerial_dataviz = aerial_robot.create_dataviz("drone_viz")
    
    # Add custom trajectory visualization
    aerial_dataviz.add_robot_path(
        robot_name="drone_alpha",
        topic="/drone_alpha/trajectory",
        render_options={"color": "red", "line_width": 3}
    )
    
    print(f"   {aerial_dataviz}")
    print(f"   Robot visualizations: {len(aerial_dataviz.get_robot_visualizations('drone_alpha'))}")
    
    # ================================================
    # Example 3: Global/Environmental DataViz
    # ================================================
    print("\n3. Creating global environmental data visualization...")
    
    # Create a global DataViz for environmental data
    global_dataviz = DataViz("global_environment")
    
    # Add occupancy grid (robot-independent)
    global_dataviz.add_occupancy_grid(
        topic="/map",
        render_options={"opacity": 0.7, "color_scheme": "grayscale"}
    )
    
    # Add 3D map
    global_dataviz.add_3d_map(
        topic="/octomap_points",
        render_options={"point_size": 0.1, "color_by": "height"}
    )
    
    # Add global navigation path
    global_dataviz.add_navigation_path(
        topic="/global_path",
        render_options={"color": "green", "line_width": 4}
    )
    
    # Add TF tree visualization
    global_dataviz.add_tf_tree(
        render_options={"show_labels": True, "frame_scale": 0.3}
    )
    
    print(f"   {global_dataviz}")
    print(f"   Global visualizations: {len(global_dataviz.get_global_visualizations())}")
    
    # ================================================
    # Example 4: Combined multi-robot DataViz
    # ================================================
    print("\n4. Creating combined multi-robot visualization...")
    
    # Create a master DataViz that combines everything
    master_dataviz = DataViz("master_view")
    
    # Add all robot sensors
    for sensor in wheeled_robot.sensors:
        master_dataviz.add_sensor_visualization(sensor, "rosbot_01")
    
    for sensor in aerial_robot.sensors:
        master_dataviz.add_sensor_visualization(sensor, "drone_alpha")
    
    # Add robot transforms
    master_dataviz.add_robot_transform("rosbot_01", "/rosbot_01/tf")
    master_dataviz.add_robot_transform("drone_alpha", "/drone_alpha/tf")
    
    # Add robot paths
    master_dataviz.add_robot_path("rosbot_01", "/rosbot_01/path")
    master_dataviz.add_robot_path("drone_alpha", "/drone_alpha/trajectory")
    
    # Add environmental data
    master_dataviz.add_occupancy_grid("/map")
    master_dataviz.add_navigation_path("/global_path")
    master_dataviz.add_tf_tree()
    
    print(f"   {master_dataviz}")
    print(f"   Total visualizations: {len(master_dataviz.visualizations)}")
    
    # ================================================
    # Example 5: DataViz management and queries
    # ================================================
    print("\n5. DataViz management and queries...")
    
    # Get summary of master DataViz
    summary = master_dataviz.get_summary()
    print(f"   Master DataViz Summary:")
    print(f"     Total: {summary['total_visualizations']}")
    print(f"     Robot-specific: {summary['robot_specific']}")
    print(f"     Global: {summary['global']}")
    print(f"     By robot:")
    for robot, count in summary['by_robot'].items():
        print(f"       {robot}: {count} visualizations")
    print(f"     By type:")
    for viz_type, count in summary['by_type'].items():
        print(f"       {viz_type}: {count}")
    
    # Query specific visualizations
    print(f"\n   Camera feeds: {len(master_dataviz.get_visualizations_by_type(VisualizationType.CAMERA_FEED))}")
    print(f"   Point clouds: {len(master_dataviz.get_visualizations_by_type(VisualizationType.POINT_CLOUD))}")
    print(f"   Paths: {len(master_dataviz.get_visualizations_by_type(VisualizationType.PATH))}")
    
    # Enable/disable robot visualizations
    print(f"\n   Testing robot visualization control...")
    initial_enabled = len(master_dataviz.get_enabled_visualizations())
    print(f"     Initially enabled: {initial_enabled}")
    
    # Disable all rosbot_01 visualizations
    master_dataviz.disable_robot_visualizations("rosbot_01")
    after_disable = len(master_dataviz.get_enabled_visualizations())
    print(f"     After disabling rosbot_01: {after_disable}")
    
    # Re-enable rosbot_01 visualizations
    master_dataviz.enable_robot_visualizations("rosbot_01")
    after_enable = len(master_dataviz.get_enabled_visualizations())
    print(f"     After re-enabling rosbot_01: {after_enable}")
    
    # ================================================
    # Example 6: Sensor management
    # ================================================
    print("\n6. Sensor management examples...")
    
    # Query sensors by type
    wheeled_cameras = wheeled_robot.get_sensors_by_type(SensorType.CAMERA)
    wheeled_lasers = wheeled_robot.get_sensors_by_type(SensorType.LASER_SCAN)
    
    print(f"   Wheeled robot cameras: {len(wheeled_cameras)}")
    print(f"   Wheeled robot laser scanners: {len(wheeled_lasers)}")
    
    aerial_lidars = aerial_robot.get_sensors_by_type(SensorType.LIDAR_3D)
    aerial_cameras = aerial_robot.get_sensors_by_type(SensorType.CAMERA)
    
    print(f"   Aerial robot 3D LiDARs: {len(aerial_lidars)}")
    print(f"   Aerial robot cameras: {len(aerial_cameras)}")
    for camera in aerial_cameras:
        print(f"     - {camera.name}: {camera.get_camera_type()} ({camera.get_resolution_str()})")
    
    # Sensor properties
    if aerial_lidars:
        lidar = aerial_lidars[0]
        print(f"   3D LiDAR details:")
        print(f"     Type: {lidar.get_lidar_type()}")
        print(f"     Point cloud size: ~{lidar.get_point_cloud_size():,} points")
        print(f"     Range: {lidar.min_range}m - {lidar.max_range}m")
    
    print("\n✅ Robot sensors and DataViz examples completed!")
    print("   System ready for robot data visualization in Mixed Reality")
    
    return 0

if __name__ == "__main__":
    exit(main())