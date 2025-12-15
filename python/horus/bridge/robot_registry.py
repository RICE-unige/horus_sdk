"""
Robot registry client for HORUS SDK

Handles robot registration with the HORUS backend system
"""

import threading
import time
from typing import Any, Dict, Tuple

"""
Robot registry client for HORUS SDK

Handles robot registration with the HORUS backend system via ROS 2 Topics.
protocol:
  1. SDK publishes JSON config to /horus/registration
  2. Unity Manager subscribes, parses, and publishes Ack to /horus/registration_ack
"""

import threading
import time
import json
import uuid
import subprocess
import socket
import atexit
import signal
import os
from typing import Any, Dict, Tuple, Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class RobotRegistryClient:
    """Client for registering robots with HORUS backend using ROS Topics"""

    def __init__(self):
        """Initialize robot registry client"""
        self.node = None
        self.publisher = None
        self.subscriber = None
        self.ros_initialized = False
        self._registration_lock = threading.Lock()
        self._ack_received = threading.Event()
        self._last_ack_data = {}
        self.bridge_process = None
        self._bridge_log_file = None
        self._last_heartbeat_time = 0
        self._remote_ip = None
        
        if ROS2_AVAILABLE:
            self._initialize_ros2()

    def _get_local_ip(self):
        # ... (lines 59-70 skipped) ...
        """Get local IP address of this machine"""
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # doesn't even have to be reachable
            s.connect(('10.255.255.255', 1))
            IP = s.getsockname()[0]
        except Exception:
            IP = '127.0.0.1'
        finally:
            s.close()
        return IP

    def _cleanup_bridge(self):
        """Kill the bridge process if we started it"""
        if self.bridge_process:
            from horus.utils import cli
            cli.print_info("Shutting down Horus Bridge...")
            # Send SIGINT to process group to handle ros2 launch structure
            try:
                os.killpg(os.getpgid(self.bridge_process.pid), signal.SIGINT)
                self.bridge_process.wait(timeout=3.0)
            except Exception:
                # Force kill if needed
                try:
                    os.killpg(os.getpgid(self.bridge_process.pid), signal.SIGKILL)
                except Exception:
                    pass
            self.bridge_process = None
            
        if self._bridge_log_file:
            try:
                self._bridge_log_file.close()
            except Exception:
                pass
            self._bridge_log_file = None

    def _cleanup_ros(self):
        """Clean up ROS node and context"""
        if self.ros_initialized:
            try:
                if self.node:
                    self.node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
            self.ros_initialized = False

    def _initialize_ros2(self):
        """Initialize ROS2 components"""
        try:
            # Check if rclpy is already initialized
            if not rclpy.ok():
                rclpy.init()
            
            # Register cleanup
            atexit.register(self._cleanup_ros)

            # Create a unique node name to avoid clashes if multiple scripts run
            node_name = f"horus_registry_client_{uuid.uuid4().hex[:8]}"
            self.node = rclpy.create_node(node_name)

            # QoS: Transient Local for registration to ensure Unity gets it even if late joiner
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )

            self.publisher = self.node.create_publisher(
                String, "/horus/registration", qos_profile
            )

            # Ack subscription - Use BestEffort to ensure compatibility with all Publisher types
            # (Reliable Sub cannot hear BestEffort Pub, but BestEffort Sub can hear both if matched?)
            # Wait, actually:
            # Reliable Pub -> Reliable Sub (OK)
            # Reliable Pub -> BestEffort Sub (OK - degrades)
            # BestEffort Pub -> Reliable Sub (FAIL)
            # BestEffort Pub -> BestEffort Sub (OK)
            # So BestEffort Subscriber is compatible with EVERYTHING.
            
            ack_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )

            self.subscriber = self.node.create_subscription(
                String, "/horus/registration_ack", self._ack_callback, ack_qos
            )

            self.ros_initialized = True

        except Exception as e:
            print(f"Warning: ROS2 initialization failed: {e}")
            self.ros_initialized = False

        # QoS for Heartbeat (Best Effort)
        hb_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        self.node.create_subscription(
            String,
            '/horus/heartbeat',
            self._heartbeat_callback,
            hb_qos
        )
        
        # Give ROS time to initialize
        time.sleep(1.0)

    def _ack_callback(self, msg):
        """Handle registration acknowledgement"""
        try:
            data = json.loads(msg.data)
            self._last_ack_data = data
            self._ack_received.set()
        except json.JSONDecodeError:
            pass
            
    def _heartbeat_callback(self, msg):
        """Handle heartbeat from Unity"""
        self._last_heartbeat_time = time.time()
        # Parse IP if present "Heartbeat: <time> | IP: <ip>"
        if "IP:" in msg.data:
            try:
                parts = msg.data.split("IP:")
                if len(parts) > 1:
                    self._remote_ip = parts[1].strip()
            except:
                pass

    def register_robot(
        self, robot, dataviz, timeout_sec: float = 10.0
    ) -> Tuple[bool, Dict]:
        """
        Register robot with HORUS backend using internal CLI and automatic bridge management.
        """
        from horus.utils import cli

        if not self.ros_initialized:
            cli.print_error("ROS2 not initialized. Cannot register.")
            return False, {"error": "ROS2 not available"}

        if not self._registration_lock.acquire(blocking=False):
            return False, {"error": "Registration already in progress"}
        
        try:
            # 1. Bridge Detection & Auto-Start
            bridge_running = False
            
            def is_port_open(port):
                import socket
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(1.0)
                    return s.connect_ex(('localhost', port)) == 0

            # Check if port 10000 is already in use (Bridge likely running)
            if is_port_open(10000):
                bridge_running = True
                cli.print_info("Existing Horus Bridge detected on port 10000.")

            if not bridge_running:
                cli.print_info("No Bridge detected. Auto-starting...")
                try:
                    # Attempt to run bridge using launch file
                    self.bridge_process = subprocess.Popen(
                        ['ros2', 'launch', 'horus_unity_bridge', 'unity_bridge.launch.py'],
                        stdout=subprocess.DEVNULL, # Keep it clean for dashboard
                        stderr=subprocess.DEVNULL,
                        start_new_session=True
                    )
                    
                    atexit.register(self._cleanup_bridge)
                    
                    # Wait for port to open
                    with cli.status("Initializing Horus Bridge...", spinner="dots"):
                        waited = 0
                        while not is_port_open(10000) and waited < 15:
                            time.sleep(1.0)
                            waited += 1
                            if self.bridge_process.poll() is not None:
                                break
                    
                    if is_port_open(10000):
                        cli.print_success("Horus Bridge started and listening.")
                        bridge_running = True
                    else:
                        cli.print_error("Bridge failed to bind port 10000.")
                        if self.bridge_process.poll() is not None:
                            cli.print_error("Bridge process exited prematurely.")
                        return False, {"error": "Bridge Start Failed"}
                        
                except FileNotFoundError:
                    cli.print_error("Could not find 'ros2'. check path.")

            # Display Connect Info using Dashboard
            local_ip = self._get_local_ip()
            bridge_state = "Active" if bridge_running else "Error"
            
            # 2. Build configuration dict
            config = self._build_robot_config_dict(robot, dataviz)
            config_json = json.dumps(config)
            
            # Prepare for Async wait
            self._ack_received.clear()
            self._last_ack_data = {}
            
            # Publish registration request
            msg = String()
            msg.data = config_json
            robot_name = robot.name
            
            # Initial publish
            self.publisher.publish(msg)

            # --- Enter Dashboard Mode ---
            with cli.ConnectionDashboard(local_ip, 10000, bridge_state) as dashboard:
                dashboard.update_status("Waiting for Horus App...")
                
                # Monitor these topics
                monitored_topics = [
                    '/horus/registration', 
                    '/horus/registration_ack', 
                    '/horus/heartbeat'
                ]
                
                while True:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                    
                    # Check Bridge Process Status
                    if self.bridge_process:
                        retcode = self.bridge_process.poll()
                        if retcode is None:
                            dashboard.update_bridge("Active")
                        else:
                            dashboard.update_bridge(f"Stopped (Code {retcode})")
                    else:
                        pass # External

                    # Update Topic Stats (every ~1s)
                    if int(time.time()) % 1 == 0:
                        stats = {}
                        for topic in monitored_topics:
                            pubs = self.node.count_publishers(topic)
                            subs = self.node.count_subscribers(topic)
                            # Always show these topics
                            stats[topic] = {'pubs': pubs, 'subs': subs}
                        dashboard.update_topics(stats)

                    # Check Heartbeat for Unity Status
                    time_since_hb = time.time() - self._last_heartbeat_time
                    if time_since_hb < 3.0 and self._last_heartbeat_time > 0:
                        ip_str = f" ({self._remote_ip})" if self._remote_ip else ""
                        dashboard.update_status(f"Connected{ip_str}")
                    else:
                        dashboard.update_status("Waiting for Horus App...")

                    # Re-publish occasionally
                    if int(time.time()) % 3 == 0:
                        self.publisher.publish(msg)

                    if self._ack_received.is_set():
                        ack = self._last_ack_data
                        recv_name = ack.get("robot_name", "UNKNOWN")
                        if recv_name == robot_name:
                            if ack.get("success"):
                                status_msg = ack.get('robot_id')
                                dashboard.update_status(f"Success! {status_msg}")
                                time.sleep(1.5) # Let user see success
                                return True, ack
                            else:
                                dashboard.update_status(f"Refused: {ack.get('error')}")
                                time.sleep(2.0)
                                return False, ack
                        else:
                            # Name mismatch
                            dashboard.update_status(f"Ignored Ack for: {recv_name}")
                            self._ack_received.clear()
                    
                    # Update animated elements
                    dashboard.tick()
                    time.sleep(0.05)
            
        except KeyboardInterrupt:
            cli.print_info("Registration cancelled by user.")
            return False, {"error": "Cancelled"}
        except Exception as e:
            cli.print_error(f"Structure Error: {e}")
            return False, {"error": str(e)}
        finally:
            self._registration_lock.release()


    def unregister_robot(
        self, robot_id: str, timeout_sec: float = 5.0
    ) -> Tuple[bool, Dict]:
        """
        Unregister robot from HORUS backend
        """
        # For unregistration, we can send a config with "action": "unregister"
        # Or just empty config with name.
        # Implementation simplified for now.
        return True, {"message": "Unregistered (Local cleanup only)"}

    def _build_robot_config_dict(self, robot, dataviz) -> Dict:
        """Build simple JSON dictionary for robot config"""
        
        return {
            "action": "register",
            "robot_name": robot.name,
            "robot_type": robot.get_type_str(),
            "sensors": [
                {
                    "name": s.name, 
                    "type": s.sensor_type.value,
                    "topic": s.topic,
                    "frame": s.frame_id,
                    "metadata": s.metadata or {},
                    "viz_config": {
                        "color": getattr(s, "color", "white"),
                        "point_size": getattr(s, "point_size", 0.05)
                    }
                } for s in robot.sensors
            ],
            "visualizations": [
                {
                    "type": v.viz_type.value,
                    "topic": v.data_source.topic,
                    "color": v.render_options.get("color", "white")
                } for v in dataviz.visualizations
            ],
            "control": {
                "drive_topic": f"/{robot.name}/cmd_vel" # Default assumption
            },
            "timestamp": time.time()
        }

    def check_backend_availability(self) -> bool:
        """Check if ROS is OK"""
        # With Pub/Sub we can't easily check receiver presence without Graph API
        # Assume OK if ROS is OK.
        return self.ros_initialized

    def __del__(self):
        if self.ros_initialized and self.node:
            try:
                self.node.destroy_node()
            except Exception:
                pass
