#!/usr/bin/env python3
"""
SDK Demo: Unity Visualization
Connects to an existing ROS 2 system and visualizes standard topics in Horus Unity.
"""

import time
import signal
import sys
import argparse
from horus.robot.robot import Robot, RobotType
from horus.sensors.sensors import LaserScan

# Graceful shutdown handler
def signal_handler(sig, frame):
    print("\nReceived interrupt signal. Exiting...")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def main():
    parser = argparse.ArgumentParser(description="Horus SDK Unity Visualization Demo")
    parser.add_argument("--name", type=str, default="sdk_viz_robot", help="Robot registration name")
    parser.add_argument("--scan_topic", type=str, default="/scan", help="LaserScan topic")
    parser.add_argument("--map_topic", type=str, default="/map", help="OccupancyGrid topic")
    parser.add_argument("--tf_topic", type=str, default="/tf", help="TF topic")
    args = parser.parse_args()

    print(f"Initializing SDK Demo Robot: '{args.name}'")
    
    # 1. Create a Robot
    # We use 'WHEELED' as a generic type.
    robot = Robot(name=args.name, robot_type=RobotType.WHEELED)
    
    # 2. Add LaserScan Sensor
    # We configure a generic lidar sensor.
    # Note: frame_id should ideally match what is being published on /scan, 
    # but for visualization specific color/size, we set it here.
    laser = LaserScan(
        name="lidar_main", 
        topic=args.scan_topic, 
        frame_id="laser_frame", # This is metadata for the SDK to know the frame
        min_range=0.1, 
        max_range=20.0,
        color="#FF0000", # Red laser scan
        point_size=0.05
    )
    robot.add_sensor(laser)
    print(f"Added Sensor: LaserScan on '{args.scan_topic}'")

    # 3. Create Custom DataViz config
    # We perform this manually to inject the OccupancyGrid visualization, 
    # which is strictly speaking 'Environment' data, not 'Robot' data, 
    # but we want to register it as part of this session.
    dataviz = robot.create_dataviz()
    
    # Add Occupancy Grid (Map)
    dataviz.add_occupancy_grid(topic=args.map_topic, frame_id="map")
    print(f"Added Visualization: OccupancyGrid on '{args.map_topic}'")
    
    # Add Full TF Tree (Useful for debug)
    dataviz.add_tf_tree(topic=args.tf_topic)
    print(f"Added Visualization: TF Tree on '{args.tf_topic}'")

    # 4. Register with Horus
    print(f"Registering with Horus Bridge (waiting for connection)...")
    try:
        success, result = robot.register_with_horus(dataviz)
        
        if success:
            print(f"✅ Registration Successful!")
            print(f"   Robot ID: {result.get('robot_id')}")
            print(f"   Color: {result.get('assigned_color')}")
            print("\nDemo running. Press Ctrl+C to stop.")
            
            # Keep alive
            while True:
                time.sleep(1)
        else:
            print(f"❌ Registration Failed.")
            print(f"   Error: {result.get('error')}")

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if robot.is_registered_with_horus():
            print("Unregistering...")
            robot.unregister_from_horus()
            print("Done.")

if __name__ == "__main__":
    main()
