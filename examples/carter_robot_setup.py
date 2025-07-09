#!/usr/bin/env python3
"""
HORUS SDK - Carter Robot Setup

Simple example of setting up a known robot (NVIDIA Carter) for HORUS MR management.
This is what a real user would do - they know their robot, they just want to connect it.
"""

import sys
import os

# Add SDK to path
script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(script_dir, 'python'))

from horus import Client, Robot, RobotType, Camera, Lidar3D, DataViz

def setup_carter_robot():
    """Set up Carter robot for HORUS MR management"""
    
    print("🤖 Setting up Carter robot for HORUS MR management...")
    
    # 1. Create Carter robot model
    carter = Robot("carter", RobotType.WHEELED)
    
    # 2. Add Carter's known sensors based on its standard configuration
    
    # Carter has a front-facing stereo camera
    front_camera = Camera(
        name="front_camera",
        frame_id="carter/front_stereo_camera",
        topic="/front_stereo_camera/left/image_raw",
        is_stereo=True,
        resolution=(1920, 1080),
        fps=30
    )
    carter.add_sensor(front_camera)
    
    # Carter has a 3D LiDAR 
    front_lidar = Lidar3D(
        name="front_lidar",
        frame_id="carter/velodyne",
        topic="/front_3d_lidar/lidar_points",
        vertical_fov=30.0,
        horizontal_fov=360.0,
        max_range=100.0,
        num_layers=32
    )
    carter.add_sensor(front_lidar)
    
    print(f"   ✓ Created {carter.name} with {carter.get_sensor_count()} sensors")
    
    # 3. Create visualization setup for HORUS MR
    carter_viz = carter.create_full_dataviz(
        global_path_topic="/carter/global_path",
        local_path_topic="/carter/local_path"
    )
    
    print(f"   ✓ Configured {len(carter_viz.visualizations)} visualizations")
    
    # 4. Initialize HORUS connection
    print("🚀 Connecting to HORUS MR system...")
    horus_client = Client(backend='ros2')
    
    print("📝 Registering Carter with HORUS backend...")
    success, result = carter.register_with_horus(carter_viz)
    
    if success:
        print("✅ Carter robot registered with HORUS MR system!")
        print(f"   🤖 Robot: {carter.name}")
        print(f"   🆔 HORUS ID: {result['robot_id']}")
        print(f"   🎨 Assigned Color: {result['assigned_color']}")
        print(f"   📊 Visualizations: {len(carter_viz.visualizations)}")
        print("   🎮 Connect your Quest 3 to the HORUS app to manage Carter")
        print("   🔄 Backend is monitoring Carter's topics for health status")
    else:
        print(f"❌ Carter registration failed: {result.get('error', 'Unknown error')}")
        if 'validation_errors' in result:
            for error in result['validation_errors']:
                print(f"   - {error}")
        return None, None, horus_client
    
    return carter, carter_viz, horus_client

if __name__ == "__main__":
    try:
        carter, viz, client = setup_carter_robot()
        
        # Keep running for MR connection
        print("\n⏳ Carter connected - HORUS MR app ready")
        print("   Press Ctrl+C to disconnect")
        
        while True:
            import time
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n🛑 Disconnecting Carter from HORUS...")
        if 'carter' in locals() and carter and carter.is_registered_with_horus():
            carter.unregister_from_horus()
            print("   ✓ Carter unregistered from HORUS")
        
        if 'client' in locals():
            client.shutdown()
        print("   ✓ Carter disconnected")
    except Exception as e:
        print(f"❌ Setup failed: {e}")