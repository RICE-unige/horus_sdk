#!/usr/bin/env python3
"""
Example of registering a custom robot with Horus using the Python SDK.
Usage: python3 sdk_registration_demo.py
"""

import time
import sys
import os

# Ensure we can import 'horus' package regardless of where script is run from
script_dir = os.path.dirname(os.path.abspath(__file__))
package_root = os.path.join(script_dir, "..")
if package_root not in sys.path:
    sys.path.insert(0, package_root)

try:
    from horus.robot.robot import Robot, RobotType
    from horus.sensors import LaserScan, Camera, SensorType
    from horus.utils import cli
except ImportError:
    # Fallback/Debug
    print(f"Failed to import horus from {package_root}")
    raise

    from horus.utils import cli

def main():
    # Remove explicit CLI imports if not needed, or just keep minimal info
    from horus.utils import cli

    cli.print_step("Defining Robot Configuration...")
    
    # 1. Define your robot
    my_robot = Robot(
        name="SdkBot_Professional",
        robot_type=RobotType.WHEELED
    )

    # 2. Add Sensors
    my_robot.add_sensor(LaserScan(
        name="Front Lidar",
        topic="/scan",
        frame_id="laser_frame",
        min_range=0.1,
        max_range=12.0,
        color="#00FFFF",
        point_size=0.1
    ))
    
    my_robot.add_sensor(Camera(
        name="Realsense RGB",
        topic="/camera/color/image_raw",
        frame_id="camera_link"
    ))

    # 3. Register with Horus
    # The SDK now handles Auto-Start of Bridge and UI Feedback internally
    my_robot.register_with_horus()

    # The script can now exit or loop.
    # For demo purposes, we might want to keep it alive if it were a real robot control script.
    # But for registration demo, we are done.

if __name__ == "__main__":
    main()
