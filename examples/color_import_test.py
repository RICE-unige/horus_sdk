#!/usr/bin/env python3
"""
Test color assignment and path planning import from main horus module
"""

import os
import sys

# Add the SDK to path
script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(script_dir, "python"))


def main():
    print("Testing color assignment and path planning imports...")

    try:
        # Import from main horus module
        from horus import (
            ColorManager,
            ColorScheme,
            DataViz,
            LaserScan,
            RGBColor,
            Robot,
            RobotType,
        )

        print("✓ All imports successful")

        # Create a robot with laser scanner
        robot = Robot("test_robot", RobotType.WHEELED)
        laser = LaserScan("laser", "laser_link", "/scan")
        robot.add_sensor(laser)

        # Create DataViz with custom color scheme
        dataviz = DataViz("test_viz")
        dataviz.color_manager = ColorManager(ColorScheme.PASTEL, seed=42)

        # Add sensor visualization (should auto-assign color)
        dataviz.add_sensor_visualization(laser, "test_robot")

        # Add path planning
        dataviz.add_robot_global_path("test_robot", "/global_path")
        dataviz.add_robot_local_path("test_robot", "/local_path")

        # Check color assignment
        robot_color = dataviz.color_manager.get_robot_color("test_robot")
        laser_color = dataviz.color_manager.get_laser_scan_color("test_robot")
        global_path_color = dataviz.color_manager.get_path_color("test_robot", "global")
        local_path_color = dataviz.color_manager.get_path_color("test_robot", "local")

        print(f"✓ Robot: {robot}")
        print(f"✓ DataViz: {dataviz}")
        print(f"✓ Robot color: {robot_color}")
        print(f"✓ Laser color: {laser_color}")
        print(f"✓ Global path color: {global_path_color}")
        print(f"✓ Local path color: {local_path_color}")
        print(f"✓ Visualizations: {len(dataviz.visualizations)}")

        print("\n✅ Color assignment and path planning test passed!")
        return 0

    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit(main())
