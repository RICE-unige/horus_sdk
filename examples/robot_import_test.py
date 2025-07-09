#!/usr/bin/env python3
"""
Test robot import from main horus module
"""

import sys
import os

# Add the SDK to path
script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(script_dir, 'python'))

def main():
    print("Testing robot import from main horus module...")
    
    try:
        # Import directly from horus module
        from horus import Robot, RobotType
        
        # Create a robot
        robot = Robot("test_robot", RobotType.WHEELED)
        print(f"✓ Successfully created: {robot}")
        
        # Test all robot types
        types = [RobotType.WHEELED, RobotType.LEGGED, RobotType.AERIAL]
        for robot_type in types:
            test_robot = Robot(f"test_{robot_type.value}", robot_type)
            print(f"✓ {robot_type.value} robot: {test_robot}")
        
        print("\n✅ Robot import test passed!")
        return 0
        
    except Exception as e:
        print(f"❌ Import test failed: {e}")
        return 1

if __name__ == "__main__":
    exit(main())