#!/usr/bin/env python3
"""
SDK Demo: True Stereo Multi-Robot Visualization
Connects to an existing ROS 2 system, configures 4 robots (2 with true stereo Side-By-Side cameras),
and visualizes them in Horus Unity.
"""

import time
import signal
import sys
from horus.robot.robot import Robot, RobotType, register_robots
from horus.sensors.sensors import Camera, LaserScan, Lidar3D

# Graceful shutdown handler
def signal_handler(sig, frame):
    print("\nReceived interrupt signal. Exiting...")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def main():
    print(f"Initializing SDK Demo for 4 Robots (2 Stereo, 2 Mono)")
    
    robots = []
    
    # We create 4 robots
    robot_names = ["stereo_bot_1", "stereo_bot_2", "stereo_bot_3", "stereo_bot_4"]
    
    for i, name in enumerate(robot_names):
        # 1. Create a Robot
        robot = Robot(name=name, robot_type=RobotType.WHEELED)
        
        # Determine if this robot should have a stereo camera (first 2 robots)
        is_stereo_camera = (i < 2)
        
        # 2. Add Camera Sensor
        if is_stereo_camera:
            cam_resolution = (640, 180) # 320x180 Side-By-Side (Total 640x180)
            print(f"[{name}] Configuring TRUE STEREO Camera")
        else:
            cam_resolution = (320, 180) # Standard single camera
            print(f"[{name}] Configuring MONO Camera")
            
        camera = Camera(
            name="main_camera",
            topic=f"/{name}/camera/image_raw/compressed",
            frame_id=f"{name}/camera_link",
            is_stereo=is_stereo_camera,
            resolution=cam_resolution,
            fps=15,
            streaming_type="ros",
            minimap_streaming_type="ros",
            teleop_streaming_type="webrtc"
        )
        robot.add_sensor(camera)
        
        # Add basic LaserScan for mapping logic if needed
        laser = LaserScan(
            name="lidar", 
            topic="/scan", # Shared map
            frame_id=f"{name}/lidar_link"
        )
        robot.add_sensor(laser)
        
        robots.append(robot)

    # 3. Create Dataviz and Register
    print(f"Registering 4 robots with Horus Bridge...")
    try:
        # Register all robots natively via the SDK batch function
        datavizs = []
        for robot in robots:
            dv = robot.create_dataviz()
            # Optionally add global map
            dv.add_occupancy_grid(topic="/map", frame_id="map")
            datavizs.append(dv)
            
        success, results = register_robots(robots, datavizs=datavizs, keep_alive=True)
        
        if success:
            print(f"✅ Registration Successful!")
            print("\nStereo Demo running. Press Ctrl+C to stop.")
            while True:
                time.sleep(1)
        else:
            print(f"❌ Registration Failed: {results}")

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("Unregistering all robots...")
        for robot in robots:
            if robot.is_registered_with_horus():
                robot.unregister_from_horus()
        print("Done.")

if __name__ == "__main__":
    main()
