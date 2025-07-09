#!/usr/bin/env python3
"""
Test sensor and DataViz import from main horus module
"""

import sys
import os

# Add the SDK to path
script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(script_dir, 'python'))

def main():
    print("Testing sensor and DataViz import from main horus module...")
    
    try:
        # Import directly from horus module
        from horus import Robot, RobotType, SensorType, Camera, LaserScan, Lidar3D, DataViz
        
        print("✓ All imports successful")
        
        # Create a robot with sensors
        robot = Robot("test_robot", RobotType.WHEELED)
        
        # Add a camera
        camera = Camera("test_cam", "base_link", "/camera/image")
        robot.add_sensor(camera)
        
        # Create DataViz
        dataviz = robot.create_dataviz()
        
        print(f"✓ Robot: {robot}")
        print(f"✓ Sensors: {robot.get_sensor_count()}")
        print(f"✓ DataViz: {dataviz}")
        print(f"✓ Visualizations: {len(dataviz.visualizations)}")
        
        print("\n✅ Sensor and DataViz import test passed!")
        return 0
        
    except Exception as e:
        print(f"❌ Import test failed: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    exit(main())