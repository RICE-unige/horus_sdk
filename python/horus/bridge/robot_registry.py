"""
Robot registry client for HORUS SDK

Handles robot registration with the HORUS backend system
"""

import time
from typing import Dict, Tuple

try:
    import rclpy
    from horus_interfaces.srv import RegisterRobot, UnregisterRobot
    from horus_interfaces.msg import RobotConfig, SensorConfig, VisualizationConfig

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class RobotRegistryClient:
    """Client for registering robots with HORUS backend"""

    def __init__(self):
        """Initialize robot registry client"""
        self.node = None
        self.register_client = None
        self.unregister_client = None
        self.ros_initialized = False

        if ROS2_AVAILABLE:
            self._initialize_ros2()

    def _initialize_ros2(self):
        """Initialize ROS2 components"""
        try:
            if not rclpy.ok():
                rclpy.init()

            self.node = rclpy.create_node("horus_robot_registry_client")

            # Create service clients
            self.register_client = self.node.create_client(
                RegisterRobot, "horus/register_robot"
            )

            self.unregister_client = self.node.create_client(
                UnregisterRobot, "horus/unregister_robot"
            )

            self.ros_initialized = True

        except Exception as e:
            print(f"Warning: ROS2 initialization failed: {e}")
            self.ros_initialized = False

    def register_robot(
        self, robot, dataviz, timeout_sec: float = 10.0
    ) -> Tuple[bool, Dict]:
        """
        Register robot with HORUS backend

        Args:
            robot: Robot instance from SDK
            dataviz: DataViz instance with visualizations
            timeout_sec: Service call timeout

        Returns:
            Tuple of (success, result_data)
        """
        if not self.ros_initialized:
            return False, {"error": "ROS2 not available"}

        # Wait for service to be available
        if not self.register_client.wait_for_service(timeout_sec=5.0):
            return False, {"error": "Registration service not available"}

        # Build robot config message
        config_msg = self._build_robot_config(robot, dataviz)

        # Create service request
        request = RegisterRobot.Request()
        request.robot_config = config_msg

        # Call service
        try:
            future = self.register_client.call_async(request)

            # Wait for response with timeout
            start_time = time.time()
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if time.time() - start_time > timeout_sec:
                    return False, {"error": "Service call timeout"}

            if future.done():
                response = future.result()

                if response.success:
                    return True, {
                        "robot_id": response.robot_id,
                        "assigned_color": response.assigned_color,
                        "message": "Robot registered successfully",
                    }
                else:
                    return False, {
                        "error": response.error_message,
                        "validation_errors": response.validation_errors,
                    }

        except Exception as e:
            return False, {"error": f"Service call failed: {str(e)}"}

        return False, {"error": "Unknown registration failure"}

    def unregister_robot(
        self, robot_id: str, timeout_sec: float = 10.0
    ) -> Tuple[bool, Dict]:
        """
        Unregister robot from HORUS backend

        Args:
            robot_id: Robot ID to unregister
            timeout_sec: Service call timeout

        Returns:
            Tuple of (success, result_data)
        """
        if not self.ros_initialized:
            return False, {"error": "ROS2 not available"}

        # Wait for service to be available
        if not self.unregister_client.wait_for_service(timeout_sec=5.0):
            return False, {"error": "Unregistration service not available"}

        # Create service request
        request = UnregisterRobot.Request()
        request.robot_id = robot_id

        # Call service
        try:
            future = self.unregister_client.call_async(request)

            # Wait for response with timeout
            start_time = time.time()
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if time.time() - start_time > timeout_sec:
                    return False, {"error": "Service call timeout"}

            if future.done():
                response = future.result()

                if response.success:
                    return True, {"message": "Robot unregistered successfully"}
                else:
                    return False, {"error": response.error_message}

        except Exception as e:
            return False, {"error": f"Service call failed: {str(e)}"}

        return False, {"error": "Unknown unregistration failure"}

    def _build_robot_config(self, robot, dataviz) -> RobotConfig:
        """Build ROS message from robot and dataviz objects"""
        config = RobotConfig()

        # Basic robot info
        config.name = robot.name
        config.robot_type = robot.get_type_str()

        # Convert sensors
        for sensor in robot.sensors:
            sensor_config = SensorConfig()
            sensor_config.name = sensor.name
            sensor_config.sensor_type = sensor.sensor_type.value
            sensor_config.frame_id = sensor.frame_id
            sensor_config.topic = sensor.topic
            sensor_config.enabled = sensor.enabled

            # Add sensor metadata
            metadata_keys = []
            metadata_values = []

            if hasattr(sensor, "metadata") and sensor.metadata:
                for key, value in sensor.metadata.items():
                    metadata_keys.append(str(key))
                    metadata_values.append(str(value))

            # Add sensor-specific properties as metadata
            if hasattr(sensor, "resolution"):
                metadata_keys.append("resolution")
                metadata_values.append(f"{sensor.resolution[0]}x{sensor.resolution[1]}")

            if hasattr(sensor, "max_range"):
                metadata_keys.append("max_range")
                metadata_values.append(str(sensor.max_range))

            if hasattr(sensor, "fov"):
                metadata_keys.append("fov")
                metadata_values.append(str(sensor.fov))

            sensor_config.metadata_keys = metadata_keys
            sensor_config.metadata_values = metadata_values

            config.sensors.append(sensor_config)

        # Convert visualizations
        for viz in dataviz.visualizations:
            if (
                viz.data_source.robot_name == robot.name
            ):  # Only robot-specific visualizations
                viz_config = VisualizationConfig()
                viz_config.viz_type = viz.viz_type.value
                viz_config.data_source_name = viz.data_source.name
                viz_config.topic = viz.data_source.topic
                viz_config.frame_id = viz.data_source.frame_id
                viz_config.layer_priority = viz.layer_priority

                # Convert render options
                render_keys = []
                render_values = []
                for key, value in viz.render_options.items():
                    render_keys.append(str(key))
                    render_values.append(str(value))

                viz_config.render_option_keys = render_keys
                viz_config.render_option_values = render_values

                config.visualizations.append(viz_config)

        # Add control and status topics (these would be robot-specific)
        config.control_topics = [f"/{robot.name}/cmd_vel"]
        config.status_topics = [f"/{robot.name}/odom", f"/{robot.name}/tf"]

        # Add robot metadata
        metadata_keys = []
        metadata_values = []

        if hasattr(robot, "metadata") and robot.metadata:
            for key, value in robot.metadata.items():
                metadata_keys.append(str(key))
                metadata_values.append(str(value))

        config.metadata_keys = metadata_keys
        config.metadata_values = metadata_values

        return config

    def check_backend_availability(self) -> bool:
        """Check if HORUS backend is available"""
        if not self.ros_initialized:
            return False

        return (
            self.register_client.service_is_ready()
            and self.unregister_client.service_is_ready()
        )

    def __del__(self):
        """Cleanup ROS2 resources"""
        if self.ros_initialized and self.node:
            try:
                self.node.destroy_node()
            except Exception:
                pass
