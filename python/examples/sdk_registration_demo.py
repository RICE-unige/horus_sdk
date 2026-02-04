#!/usr/bin/env python3
"""
Example of registering a custom robot with Horus using the Python SDK.
This demo sends only the robot transform (TF) using the robot name as the TF prefix
and includes robot dimensions for sizing the interactor surface.
Usage: python3 sdk_registration_demo.py
"""

import sys
import os

# Ensure we can import 'horus' package regardless of where script is run from
script_dir = os.path.dirname(os.path.abspath(__file__))
package_root = os.path.join(script_dir, "..")
if package_root not in sys.path:
    sys.path.insert(0, package_root)

try:
    from horus.robot.robot import Robot, RobotDimensions, RobotType
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
        # This name should match the TF prefix (e.g. test_bot/base_link)
        name="test_bot",
        robot_type=RobotType.WHEELED,
        dimensions=RobotDimensions(length=0.8, width=0.6, height=0.4),
    )

    # 2. Register with Horus (TF-only for now)
    # The SDK now handles Auto-Start of Bridge and UI Feedback internally
    # register_with_horus() now blocks and maintains the connection dashboard
    my_robot.register_with_horus()

    # When dashboard exits (Ctrl+C), script ends
    print("\nDisconnecting...")


if __name__ == "__main__":
    main()
