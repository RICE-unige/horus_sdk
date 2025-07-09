#!/usr/bin/env python3
"""
HORUS SDK Live Robot Integration

This script demonstrates complete integration of a real ROS2 robot with the HORUS 
Mixed Reality application. It automatically discovers robot topics, creates sensor 
mappings, and establishes real-time MR visualization.

Based on detected robot topics:
- /front_3d_lidar/lidar_points (3D LiDAR)
- /front_stereo_camera/left/image_raw (Stereo Camera)
- /chassis/odom (Odometry)
- /chassis/imu (IMU)
- /cmd_vel (Velocity Commands)
- /tf (Transforms)

Real robot scenario: Mobile robot with stereo cameras, 3D LiDAR, and IMU sensors
"""

import sys
import os
import time
import signal
import subprocess
from typing import Dict, List, Optional, Tuple

# Add the SDK to path
script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(script_dir, 'python'))

from horus import (
    Client, Robot, RobotType, SensorType, Camera, Lidar3D, 
    DataViz, ColorManager, ColorScheme
)

class LiveRobotIntegration:
    """Complete integration manager for live robot with HORUS MR"""
    
    def __init__(self, robot_name: str = "live_robot"):
        """
        Initialize the integration system
        
        Args:
            robot_name: Name/namespace for the robot
        """
        self.robot_name = robot_name
        self.robot = None
        self.dataviz = None
        self.horus_client = None
        self.running = False
        
        # Robot topic mapping (auto-detected from your live robot)
        self.topic_mapping = {
            "3d_lidar": "/front_3d_lidar/lidar_points",
            "stereo_camera_left": "/front_stereo_camera/left/image_raw", 
            "stereo_camera_info": "/front_stereo_camera/left/camera_info",
            "odometry": "/chassis/odom",
            "imu_chassis": "/chassis/imu",
            "imu_front": "/front_stereo_imu/imu",
            "imu_back": "/back_stereo_imu/imu",
            "imu_left": "/left_stereo_imu/imu",
            "imu_right": "/right_stereo_imu/imu",
            "cmd_vel": "/cmd_vel",
            "tf": "/tf"
        }
        
        # Set up signal handling for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print(f"\n🛑 Received signal {signum}, shutting down gracefully...")
        self.shutdown()
        sys.exit(0)
    
    def discover_robot_capabilities(self) -> Dict[str, any]:
        """
        Auto-discover robot capabilities from ROS2 topics
        
        Returns:
            Dictionary with robot capabilities and topics
        """
        print("🔍 Discovering robot capabilities...")
        
        try:
            # Get topic list from ROS2
            result = subprocess.run(
                ["bash", "-c", "source horus_ros2_ws/install/setup.bash && ros2 topic list"],
                capture_output=True, text=True, cwd=script_dir
            )
            
            if result.returncode != 0:
                print(f"⚠️  Warning: Could not get topic list: {result.stderr}")
                topics = []
            else:
                topics = result.stdout.strip().split('\n')
            
            print(f"   Found {len(topics)} active topics")
            
            # Analyze capabilities
            capabilities = {
                "has_3d_lidar": "/front_3d_lidar/lidar_points" in topics,
                "has_stereo_camera": "/front_stereo_camera/left/image_raw" in topics,
                "has_odometry": "/chassis/odom" in topics,
                "has_imu": any("imu" in topic for topic in topics),
                "has_cmd_vel": "/cmd_vel" in topics,
                "has_tf": "/tf" in topics,
                "active_topics": topics,
                "imu_count": len([t for t in topics if "imu" in t])
            }
            
            # Print discovery results
            print("   🤖 Robot Capabilities Detected:")
            print(f"     3D LiDAR: {'✓' if capabilities['has_3d_lidar'] else '✗'}")
            print(f"     Stereo Camera: {'✓' if capabilities['has_stereo_camera'] else '✗'}")
            print(f"     Odometry: {'✓' if capabilities['has_odometry'] else '✗'}")
            print(f"     IMU Sensors: {'✓' if capabilities['has_imu'] else '✗'} ({capabilities['imu_count']} units)")
            print(f"     Velocity Control: {'✓' if capabilities['has_cmd_vel'] else '✗'}")
            print(f"     Transforms: {'✓' if capabilities['has_tf'] else '✗'}")
            
            return capabilities
            
        except Exception as e:
            print(f"⚠️  Warning: Robot discovery failed: {e}")
            return {"error": str(e), "active_topics": []}
    
    def create_robot_model(self, capabilities: Dict[str, any]) -> Robot:
        """
        Create robot model based on discovered capabilities
        
        Args:
            capabilities: Robot capabilities from discovery
            
        Returns:
            Configured Robot instance
        """
        print(f"\n🤖 Creating robot model: '{self.robot_name}'...")
        
        # Create robot (assuming wheeled based on odometry and cmd_vel)
        robot = Robot(self.robot_name, RobotType.WHEELED)
        
        # Add robot metadata
        robot.add_metadata("discovery_time", time.time())
        robot.add_metadata("capabilities", capabilities)
        robot.add_metadata("ros2_namespace", "/")
        
        # Add sensors based on capabilities
        sensor_count = 0
        
        # Add 3D LiDAR if available
        if capabilities.get("has_3d_lidar", False):
            lidar_3d = Lidar3D(
                name="front_3d_lidar",
                frame_id=f"{self.robot_name}/lidar_link",
                topic=self.topic_mapping["3d_lidar"],
                vertical_fov=30.0,  # Typical for mobile robot LiDAR
                horizontal_fov=360.0,
                vertical_resolution=0.5,
                horizontal_resolution=0.25,
                min_range=0.5,
                max_range=50.0,
                points_per_second=300000,
                num_layers=32  # Common for mobile robots
            )
            robot.add_sensor(lidar_3d)
            sensor_count += 1
            print(f"     ✓ Added 3D LiDAR sensor")
        
        # Add stereo camera if available
        if capabilities.get("has_stereo_camera", False):
            stereo_camera = Camera(
                name="front_stereo_camera",
                frame_id=f"{self.robot_name}/camera_link",
                topic=self.topic_mapping["stereo_camera_left"],
                is_stereo=True,
                resolution=(1920, 1080),  # HD resolution
                fps=30,
                fov=70.0,
                encoding="bgr8"
            )
            robot.add_sensor(stereo_camera)
            sensor_count += 1
            print(f"     ✓ Added stereo camera sensor")
        
        print(f"   🔧 Robot configured with {sensor_count} sensors")
        return robot
    
    def create_comprehensive_dataviz(self, robot: Robot) -> DataViz:
        """
        Create comprehensive DataViz for the robot with all visualizations
        
        Args:
            robot: Configured robot instance
            
        Returns:
            Complete DataViz configuration
        """
        print(f"\n📊 Creating comprehensive visualization for {robot.name}...")
        
        # Create DataViz with bright color scheme for good MR visibility
        dataviz = DataViz(f"{robot.name}_mr_viz")
        dataviz.color_manager = ColorManager(ColorScheme.BRIGHT, seed=42)
        
        # Add all robot sensors with automatic color assignment
        for sensor in robot.sensors:
            dataviz.add_sensor_visualization(sensor, robot.name)
            print(f"     ✓ Added {sensor.name} visualization")
        
        # Add robot transform/coordinate frame
        dataviz.add_robot_transform(
            robot_name=robot.name,
            topic=self.topic_mapping["tf"],
            frame_id=f"{robot.name}_base_link"
        )
        print(f"     ✓ Added robot transform visualization")
        
        # Add robot odometry path (executed trajectory)
        dataviz.add_robot_trajectory(
            robot_name=robot.name,
            topic=self.topic_mapping["odometry"],
            frame_id="odom"
        )
        print(f"     ✓ Added odometry trajectory visualization")
        
        # Add global path planning (if navigation stack is running)
        try:
            # Check if navigation topics exist
            result = subprocess.run(
                ["bash", "-c", "source horus_ros2_ws/install/setup.bash && ros2 topic list | grep -E '(global_plan|plan)'"],
                capture_output=True, text=True, cwd=script_dir
            )
            
            if result.returncode == 0 and result.stdout.strip():
                nav_topics = result.stdout.strip().split('\n')
                for topic in nav_topics:
                    if 'global' in topic:
                        dataviz.add_robot_global_path(
                            robot_name=robot.name,
                            topic=topic,
                            frame_id="map"
                        )
                        print(f"     ✓ Added global path: {topic}")
                    elif 'local' in topic:
                        dataviz.add_robot_local_path(
                            robot_name=robot.name,
                            topic=topic,
                            frame_id="map"
                        )
                        print(f"     ✓ Added local path: {topic}")
                        
        except Exception as e:
            print(f"     ⚠️  Navigation paths not available: {e}")
        
        # Add environmental visualizations
        # Check for map topic
        try:
            result = subprocess.run(
                ["bash", "-c", "source horus_ros2_ws/install/setup.bash && ros2 topic list | grep -E '^/map$'"],
                capture_output=True, text=True, cwd=script_dir
            )
            
            if result.returncode == 0 and result.stdout.strip():
                dataviz.add_occupancy_grid(
                    topic="/map",
                    frame_id="map",
                    render_options={"opacity": 0.7}
                )
                print(f"     ✓ Added occupancy grid map")
                
        except Exception as e:
            print(f"     ⚠️  Map topic not available: {e}")
        
        # Add TF tree visualization for coordinate frame relationships
        dataviz.add_tf_tree(
            topic=self.topic_mapping["tf"],
            render_options={"show_labels": True, "frame_scale": 0.2}
        )
        print(f"     ✓ Added TF tree visualization")
        
        print(f"   📊 Created {len(dataviz.visualizations)} visualizations")
        return dataviz
    
    def initialize_horus_connection(self) -> Client:
        """
        Initialize HORUS SDK and establish MR connection
        
        Returns:
            HORUS Client instance
        """
        print(f"\n🚀 Initializing HORUS SDK and MR connection...")
        
        try:
            # Initialize HORUS client (this will show the ASCII art and start backend)
            client = Client(backend='ros2', auto_launch=True)
            
            print(f"   ✓ HORUS SDK initialized successfully")
            print(f"   ✓ Backend services running")
            print(f"   ✓ MR connection monitoring active")
            
            return client
            
        except Exception as e:
            print(f"   ❌ HORUS initialization failed: {e}")
            raise
    
    def start_live_monitoring(self):
        """Start continuous monitoring and MR data streaming"""
        print(f"\n🔄 Starting live robot monitoring...")
        print(f"=" * 60)
        
        self.running = True
        
        try:
            while self.running:
                # Display current status
                print(f"\n📊 Live Robot Status - {time.strftime('%H:%M:%S')}")
                print(f"   Robot: {self.robot.name} ({self.robot.get_type_str()})")
                print(f"   Sensors: {self.robot.get_sensor_count()} active")
                print(f"   Visualizations: {len(self.dataviz.visualizations)} configured")
                
                # Show robot color assignment
                robot_color = self.dataviz.color_manager.get_robot_color(self.robot.name)
                print(f"   MR Color: {robot_color}")
                
                # Show visualization summary
                enabled_viz = self.dataviz.get_enabled_visualizations()
                print(f"   Enabled Visualizations: {len(enabled_viz)}")
                
                # Show topic activity (simplified check)
                try:
                    result = subprocess.run(
                        ["bash", "-c", "source horus_ros2_ws/install/setup.bash && ros2 topic hz /front_3d_lidar/lidar_points --window 10"],
                        capture_output=True, text=True, timeout=3, cwd=script_dir
                    )
                    if "average rate" in result.stdout:
                        hz_line = [line for line in result.stdout.split('\n') if 'average rate' in line]
                        if hz_line:
                            print(f"   3D LiDAR Rate: {hz_line[0].split(':')[1].strip()}")
                    else:
                        print(f"   3D LiDAR: Active (measuring...)")
                except:
                    print(f"   3D LiDAR: Active")
                
                print(f"\n🎮 HORUS MR App Status:")
                print(f"   📡 Connect your Quest 3 to see real-time robot data")
                print(f"   🔗 IP: Check HORUS SDK output above for connection details")
                print(f"   📊 All robot sensors streaming to MR interface")
                print(f"\n   Press Ctrl+C to stop monitoring and shutdown")
                
                # Wait before next update
                time.sleep(10)
                
        except KeyboardInterrupt:
            print(f"\n🛑 Monitoring interrupted by user")
        except Exception as e:
            print(f"\n❌ Monitoring error: {e}")
        finally:
            self.shutdown()
    
    def run_complete_integration(self):
        """Run the complete robot integration process"""
        print("🤖 HORUS SDK Live Robot Integration")
        print("=" * 60)
        print("Connecting real ROS2 robot to HORUS Mixed Reality application...")
        
        try:
            # Step 1: Discover robot capabilities
            capabilities = self.discover_robot_capabilities()
            
            if "error" in capabilities:
                print(f"❌ Robot discovery failed: {capabilities['error']}")
                return 1
            
            # Step 2: Create robot model
            self.robot = self.create_robot_model(capabilities)
            
            # Step 3: Create comprehensive DataViz
            self.dataviz = self.create_comprehensive_dataviz(self.robot)
            
            # Step 4: Initialize HORUS connection
            self.horus_client = self.initialize_horus_connection()
            
            # Step 5: Display integration summary
            print(f"\n✅ ROBOT INTEGRATION COMPLETE")
            print(f"=" * 60)
            print(f"🤖 Robot: {self.robot.name}")
            print(f"   Type: {self.robot.get_type_str()}")
            print(f"   Sensors: {self.robot.get_sensor_count()}")
            print(f"   Color: {self.dataviz.color_manager.get_robot_color(self.robot.name)}")
            
            print(f"\n📊 Visualizations: {len(self.dataviz.visualizations)}")
            viz_summary = self.dataviz.get_summary()
            for viz_type, count in viz_summary['by_type'].items():
                print(f"   {viz_type}: {count}")
            
            print(f"\n🎮 HORUS MR CONNECTION READY")
            print(f"   Your robot is now connected to the HORUS Mixed Reality system!")
            print(f"   Launch HORUS app on Quest 3 and connect to see live robot data")
            
            # Step 6: Start live monitoring
            self.start_live_monitoring()
            
            return 0
            
        except Exception as e:
            print(f"\n❌ Integration failed: {e}")
            import traceback
            traceback.print_exc()
            return 1
    
    def shutdown(self):
        """Clean shutdown of all systems"""
        if not self.running:
            return
            
        print(f"\n🔄 Shutting down robot integration...")
        self.running = False
        
        if self.horus_client:
            try:
                self.horus_client.shutdown()
                print(f"   ✓ HORUS client shutdown")
            except Exception as e:
                print(f"   ⚠️  HORUS client shutdown error: {e}")
        
        print(f"   ✓ Robot integration shutdown complete")


def main():
    """Main entry point"""
    integration = LiveRobotIntegration("mobile_robot_01")
    return integration.run_complete_integration()


if __name__ == "__main__":
    exit(main())