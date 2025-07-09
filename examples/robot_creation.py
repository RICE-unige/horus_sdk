#!/usr/bin/env python3
"""
HORUS SDK Robot Creation Example

This example demonstrates how to create robot objects with different types
and manage their metadata.
"""

import sys
import os

# Add the SDK to path (for development without installation)
script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(script_dir, 'python'))

from horus.core import Robot, RobotType

def main():
    """Demonstrate robot object creation and usage"""
    print("🤖 HORUS SDK Robot Creation Example")
    print("=" * 40)
    
    # Example 1: Create a wheeled robot
    print("\n1. Creating a wheeled robot...")
    wheeled_robot = Robot(
        name="rosbot_01",
        robot_type=RobotType.WHEELED
    )
    print(f"   Created: {wheeled_robot}")
    print(f"   Type: {wheeled_robot.get_type_str()}")
    
    # Example 2: Create a legged robot with metadata
    print("\n2. Creating a legged robot with metadata...")
    legged_robot = Robot(
        name="spot_alpha",
        robot_type=RobotType.LEGGED,
        metadata={
            "manufacturer": "Boston Dynamics",
            "model": "Spot",
            "max_speed": 3.3  # m/s
        }
    )
    print(f"   Created: {legged_robot}")
    print(f"   Manufacturer: {legged_robot.get_metadata('manufacturer')}")
    print(f"   Max Speed: {legged_robot.get_metadata('max_speed')} m/s")
    
    # Example 3: Create an aerial robot and add metadata
    print("\n3. Creating an aerial robot...")
    aerial_robot = Robot(
        name="drone_beta",
        robot_type=RobotType.AERIAL
    )
    aerial_robot.add_metadata("flight_ceiling", 120)  # meters
    aerial_robot.add_metadata("battery_life", 25)     # minutes
    print(f"   Created: {aerial_robot}")
    print(f"   Flight Ceiling: {aerial_robot.get_metadata('flight_ceiling')} m")
    print(f"   Battery Life: {aerial_robot.get_metadata('battery_life')} min")
    
    # Example 4: Create a robot fleet
    print("\n4. Creating a robot fleet...")
    fleet = [
        Robot("rosbot_01", RobotType.WHEELED),
        Robot("rosbot_02", RobotType.WHEELED),
        Robot("spot_alpha", RobotType.LEGGED),
        Robot("drone_beta", RobotType.AERIAL),
        Robot("drone_gamma", RobotType.AERIAL)
    ]
    
    print(f"   Fleet size: {len(fleet)} robots")
    
    # Group by type
    type_counts = {}
    for robot in fleet:
        robot_type = robot.get_type_str()
        type_counts[robot_type] = type_counts.get(robot_type, 0) + 1
    
    print("   Fleet composition:")
    for robot_type, count in type_counts.items():
        print(f"     {robot_type}: {count} robot(s)")
    
    # Example 5: Error handling
    print("\n5. Error handling examples...")
    
    try:
        # Empty name should raise error
        Robot("", RobotType.WHEELED)
    except ValueError as e:
        print(f"   ✓ Caught expected error: {e}")
    
    try:
        # Invalid type should raise error
        Robot("test", "invalid_type")
    except TypeError as e:
        print(f"   ✓ Caught expected error: {e}")
    
    print("\n✅ Robot creation examples completed!")
    print("   All robot objects created successfully")
    
    return 0

if __name__ == "__main__":
    exit(main())