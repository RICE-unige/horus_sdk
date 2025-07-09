#!/usr/bin/env python3
"""
HORUS SDK Color Assignment and Path Planning Example

This example demonstrates:
1. Automatic unique color assignment for robots
2. Laser scan color assignment
3. Global and local path planning visualizations
4. Multi-robot scenarios with color coordination
5. Color management and customization
"""

import sys
import os

# Add the SDK to path (for development without installation)
script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(script_dir, 'python'))

from horus.core import (
    Robot, RobotType, SensorType, Camera, LaserScan, Lidar3D, 
    DataViz, ColorManager, ColorScheme, RGBColor
)

def main():
    """Demonstrate color assignment and path planning"""
    print("🎨 HORUS SDK Color Assignment and Path Planning Example")
    print("=" * 60)
    
    # ================================================
    # Example 1: Color Manager Basics
    # ================================================
    print("\n1. Color Manager Basics...")
    
    # Create color managers with different schemes
    bright_colors = ColorManager(ColorScheme.BRIGHT, seed=42)
    pastel_colors = ColorManager(ColorScheme.PASTEL, seed=42)
    
    # Test color assignment
    robots = ["rosbot_01", "spot_alpha", "drone_beta", "turtle_gamma"]
    
    print("   Bright Color Scheme:")
    for robot in robots:
        color = bright_colors.get_robot_color(robot)
        print(f"     {robot}: {color} (RGBA: {color.to_normalized_tuple()})")
    
    print("\n   Pastel Color Scheme:")
    for robot in robots:
        color = pastel_colors.get_robot_color(robot)
        print(f"     {robot}: {color} (RGBA: {color.to_normalized_tuple()})")
    
    # ================================================
    # Example 2: Multi-Robot Fleet with Automatic Colors
    # ================================================
    print("\n2. Creating multi-robot fleet with automatic color assignment...")
    
    # Create robots with sensors
    fleet = []
    
    # Wheeled robot 1
    rosbot1 = Robot("rosbot_01", RobotType.WHEELED)
    front_laser = LaserScan("front_laser", "rosbot_01/laser", "/rosbot_01/scan")
    rosbot1.add_sensor(front_laser)
    fleet.append(rosbot1)
    
    # Wheeled robot 2  
    rosbot2 = Robot("rosbot_02", RobotType.WHEELED)
    back_laser = LaserScan("back_laser", "rosbot_02/laser", "/rosbot_02/scan")
    rosbot2.add_sensor(back_laser)
    fleet.append(rosbot2)
    
    # Legged robot
    spot = Robot("spot_alpha", RobotType.LEGGED)
    spot_lidar = Lidar3D("spot_lidar", "spot_alpha/lidar", "/spot_alpha/lidar_points")
    spot_camera = Camera("spot_camera", "spot_alpha/camera", "/spot_alpha/camera/image")
    spot.add_sensor(spot_lidar)
    spot.add_sensor(spot_camera)
    fleet.append(spot)
    
    # Aerial robot
    drone = Robot("drone_beta", RobotType.AERIAL)
    drone_camera = Camera("drone_camera", "drone_beta/camera", "/drone_beta/camera/image", 
                         is_stereo=True, resolution=(1920, 1080))
    drone.add_sensor(drone_camera)
    fleet.append(drone)
    
    print(f"   Created fleet of {len(fleet)} robots:")
    for robot in fleet:
        print(f"     - {robot} ({robot.get_sensor_count()} sensors)")
    
    # ================================================
    # Example 3: DataViz with Automatic Color Assignment
    # ================================================
    print("\n3. Creating DataViz with automatic color assignment...")
    
    # Create master DataViz with custom color scheme
    master_dataviz = DataViz("fleet_visualization")
    master_dataviz.color_manager = ColorManager(ColorScheme.BRIGHT, seed=42)
    
    # Add all robot sensors (laser scans will get unique colors)
    for robot in fleet:
        for sensor in robot.sensors:
            master_dataviz.add_sensor_visualization(sensor, robot.name)
    
    # Add robot transforms (each robot gets unique color)
    for robot in fleet:
        master_dataviz.add_robot_transform(
            robot_name=robot.name,
            topic=f"/{robot.name}/tf"
        )
    
    print(f"   {master_dataviz}")
    print(f"   Color assignments:")
    color_summary = master_dataviz.color_manager.get_color_summary()
    for robot_name, color_hex in color_summary.items():
        print(f"     {robot_name}: {color_hex}")
    
    # ================================================
    # Example 4: Path Planning Visualizations
    # ================================================
    print("\n4. Adding path planning visualizations...")
    
    # Add global and local paths for each robot
    path_topics = {
        "rosbot_01": {
            "global": "/rosbot_01/global_path", 
            "local": "/rosbot_01/local_path",
            "trajectory": "/rosbot_01/trajectory"
        },
        "rosbot_02": {
            "global": "/rosbot_02/global_path",
            "local": "/rosbot_02/local_path", 
            "trajectory": "/rosbot_02/trajectory"
        },
        "spot_alpha": {
            "global": "/spot_alpha/global_path",
            "local": "/spot_alpha/local_path",
            "trajectory": "/spot_alpha/trajectory"
        },
        "drone_beta": {
            "global": "/drone_beta/global_path",
            "local": "/drone_beta/local_path",
            "trajectory": "/drone_beta/trajectory"
        }
    }
    
    for robot_name, topics in path_topics.items():
        # Global path (solid line, robot color)
        master_dataviz.add_robot_global_path(
            robot_name=robot_name,
            topic=topics["global"]
        )
        
        # Local path (dashed line, lighter color)
        master_dataviz.add_robot_local_path(
            robot_name=robot_name,
            topic=topics["local"]
        )
        
        # Trajectory (thin line, transparent)
        master_dataviz.add_robot_trajectory(
            robot_name=robot_name,
            topic=topics["trajectory"]
        )
    
    print(f"   Added path planning for {len(fleet)} robots")
    print(f"   Total visualizations: {len(master_dataviz.visualizations)}")
    
    # ================================================
    # Example 5: Using Robot Helper Methods
    # ================================================
    print("\n5. Using robot helper methods for full DataViz creation...")
    
    # Create individual robot DataViz with full path planning
    individual_dataviz = {}
    
    for robot in fleet:
        robot_topics = path_topics.get(robot.name, {})
        
        # Use robot helper method to create complete DataViz
        dataviz = robot.create_full_dataviz(
            dataviz_name=f"{robot.name}_complete",
            global_path_topic=robot_topics.get("global"),
            local_path_topic=robot_topics.get("local"),
            trajectory_topic=robot_topics.get("trajectory")
        )
        
        individual_dataviz[robot.name] = dataviz
        print(f"   {robot.name}: {len(dataviz.visualizations)} visualizations")
    
    # ================================================
    # Example 6: Color Customization and Analysis
    # ================================================
    print("\n6. Color customization and analysis...")
    
    # Show laser scan specific colors
    print("   Laser scan colors (with transparency):")
    for robot in fleet:
        if robot.get_sensors_by_type(SensorType.LASER_SCAN):
            laser_color = master_dataviz.color_manager.get_laser_scan_color(robot.name)
            print(f"     {robot.name}: {laser_color} (alpha: {laser_color.a})")
    
    # Show path colors
    print("\n   Path planning colors:")
    for robot_name in ["rosbot_01", "spot_alpha"]:
        global_color = master_dataviz.color_manager.get_path_color(robot_name, "global")
        local_color = master_dataviz.color_manager.get_path_color(robot_name, "local")
        print(f"     {robot_name} global: {global_color} (alpha: {global_color.a})")
        print(f"     {robot_name} local:  {local_color} (alpha: {local_color.a})")
    
    # ================================================
    # Example 7: Environmental Data
    # ================================================
    print("\n7. Adding environmental data...")
    
    # Add environmental visualizations (robot-independent)
    master_dataviz.add_occupancy_grid("/map")
    master_dataviz.add_3d_map("/octomap_points")
    master_dataviz.add_global_navigation_path("/global_navigation_path")
    master_dataviz.add_tf_tree()
    
    # ================================================
    # Example 8: Visualization Summary and Management
    # ================================================
    print("\n8. Visualization summary and management...")
    
    summary = master_dataviz.get_summary()
    print(f"   Master DataViz Summary:")
    print(f"     Total: {summary['total_visualizations']}")
    print(f"     Enabled: {summary['enabled_visualizations']}")
    print(f"     Robot-specific: {summary['robot_specific']}")
    print(f"     Global: {summary['global']}")
    
    print(f"   By visualization type:")
    for viz_type, count in summary['by_type'].items():
        print(f"     {viz_type}: {count}")
    
    print(f"   By data source type:")
    for source_type, count in summary['by_data_source'].items():
        print(f"     {source_type}: {count}")
    
    # Test layer ordering
    enabled_viz = master_dataviz.get_enabled_visualizations()
    print(f"\n   Rendering order (by layer priority):")
    for i, viz in enumerate(enabled_viz[:10]):  # Show first 10
        print(f"     {i+1}. {viz.display_name} (priority: {viz.layer_priority})")
    if len(enabled_viz) > 10:
        print(f"     ... and {len(enabled_viz) - 10} more")
    
    # ================================================
    # Example 9: Color Scheme Comparison
    # ================================================
    print("\n9. Color scheme comparison...")
    
    schemes = [ColorScheme.BRIGHT, ColorScheme.PASTEL, ColorScheme.DARK, ColorScheme.RAINBOW]
    test_robots = ["robot_a", "robot_b", "robot_c"]
    
    for scheme in schemes:
        print(f"   {scheme.value.title()} scheme:")
        color_mgr = ColorManager(scheme, seed=42)
        for robot in test_robots:
            color = color_mgr.get_robot_color(robot)
            print(f"     {robot}: {color}")
    
    print("\n✅ Color assignment and path planning examples completed!")
    print("   🎨 Unique colors assigned to all robots")
    print("   🛣️  Global and local path planning configured") 
    print("   📊 Visualization system ready for Mixed Reality")
    
    return 0

if __name__ == "__main__":
    exit(main())