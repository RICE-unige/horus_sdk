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
        self._ack_by_robot = {}
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
            from horus.utils import cli
            if not rclpy.ok():
                rclpy.init()
            cli.print_success("ROS2 Context Initialized")
            
            self.ros_initialized = True
            
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

            # Ack subscription - Use BestEffort or Reliable
            ack_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )

            self.subscriber = self.node.create_subscription(
                String, "/horus/registration_ack", self._ack_callback, ack_qos
            )

            # QoS for Heartbeat (Best Effort)
            hb_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=1
            )
            
            self.node.create_subscription(
                String, 
                "/horus/heartbeat", 
                self._heartbeat_callback, 
                hb_qos
            )

        except Exception as e:
            cli.print_error(f"Failed to init ROS context: {e}")
            import traceback
            traceback.print_exc()
            self.ros_initialized = False

        
        # Give ROS time to initialize
        time.sleep(1.0)

    def _ack_callback(self, msg):
        """Handle registration acknowledgement"""
        try:
            data = json.loads(msg.data)
            self._last_ack_data = data
            robot_name = data.get("robot_name") if isinstance(data, dict) else None
            if robot_name:
                self._ack_by_robot[robot_name] = data
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

    def _await_ack(self, robot_name: str, timeout_sec: float):
        start_time = time.time()
        while (time.time() - start_time) < timeout_sec:
            if robot_name in self._ack_by_robot:
                return self._ack_by_robot.pop(robot_name)
            self._ack_received.wait(0.1)
            self._ack_received.clear()
        return None

    def _collect_topics(self, dataviz):
        topics = []
        try:
            for viz in dataviz.get_enabled_visualizations():
                topic = viz.data_source.topic
                if topic and topic not in topics:
                    topics.append(topic)
        except Exception:
            pass
        return topics

    def _apply_registration_metadata(self, robot, dataviz, ack: Dict):
        try:
            robot.add_metadata("horus_robot_id", ack.get("robot_id"))
            robot.add_metadata("horus_color", ack.get("assigned_color") or ack.get("color"))
            robot.add_metadata("horus_registered", True)
        except Exception:
            return
        topics = self._collect_topics(dataviz)
        robot.add_metadata("horus_topics", topics)
        if topics:
            try:
                from horus.utils.topic_monitor import get_topic_monitor
                monitor = get_topic_monitor()
                monitor.watch_topics(topics)
                monitor.start()
            except Exception:
                pass

    def _get_topic_counts(self, topic: str) -> Tuple[int, int]:
        pubs_info = subs_info = None
        pubs_count = subs_count = None
        try:
            pubs_info = len(self.node.get_publishers_info_by_topic(topic))
        except Exception:
            pubs_info = None
        try:
            subs_info = len(self.node.get_subscriptions_info_by_topic(topic))
        except Exception:
            subs_info = None
        try:
            pubs_count = self.node.count_publishers(topic)
        except Exception:
            pubs_count = None
        try:
            subs_count = self.node.count_subscribers(topic)
        except Exception:
            subs_count = None

        def _choose(*values: Optional[int]) -> int:
            available = [v for v in values if v is not None]
            return max(available) if available else 0

        return _choose(pubs_info, pubs_count), _choose(subs_info, subs_count)

    def _is_backend_node_name(self, node_name: Optional[str]) -> bool:
        if not node_name:
            return False
        try:
            if self.node and node_name == self.node.get_name():
                return False
        except Exception:
            pass
        if node_name in ("horus_backend_node", "horus_backend", "horus_unity_bridge"):
            return True
        if node_name.startswith("horus_unity_bridge"):
            return True
        if "horus_backend" in node_name:
            return True
        if node_name.endswith("_RosSubscriber") or node_name.endswith("_RosPublisher"):
            return True
        return False

    def _has_backend_subscriber(self, topic: str) -> bool:
        try:
            infos = self.node.get_subscriptions_info_by_topic(topic)
        except Exception:
            return False
        return any(self._is_backend_node_name(info.node_name) for info in infos)

    def _has_backend_publisher(self, topic: str) -> bool:
        try:
            infos = self.node.get_publishers_info_by_topic(topic)
        except Exception:
            return False
        return any(self._is_backend_node_name(info.node_name) for info in infos)

    def register_robot(
        self,
        robot,
        dataviz,
        timeout_sec: float = 10.0,
        keep_alive: bool = False,
        show_dashboard: bool = True,
    ) -> Tuple[bool, Dict]:
        """
        Register robot with HORUS backend using internal CLI and automatic bridge management.
        
        Args:
            robot: Robot instance
            dataviz: DataViz instance
            timeout_sec: Timeout for initial registration
            keep_alive: If True, blocks and maintains the dashboard after registration, 
                       handling monitoring and re-registration automatically.
            show_dashboard: If False, perform a quiet registration without the UI dashboard.
        """
        from horus.utils import cli

        if not self.ros_initialized:
            cli.print_error("ROS2 not initialized. Cannot register.")
            return False, {"error": "ROS2 not available"}

        if not self._registration_lock.acquire(blocking=False):
            return False, {"error": "Registration already in progress"}
        
        from horus.utils.topic_status import get_topic_status_board
        board = get_topic_status_board()
        board.set_silent(show_dashboard)

        try:
            # 1. Bridge Detection (Infrastructure must be external)
            bridge_running = False
            
            def is_port_open(port):
                import socket
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(0.5)
                    return s.connect_ex(('localhost', port)) == 0

            # ... (Bridge Auto-start logic omitted for brevity, keeping existing flow) ...
            if is_port_open(10000):
                bridge_running = True
                cli.print_info("Horus Bridge detected on port 10000.")
            else:
                cli.print_info("No Bridge on port 10000. Attempting auto-start...")
                # Try to launch via ROS 2
                try:
                    check_pkg = subprocess.run(['ros2', 'pkg', 'prefix', 'horus_unity_bridge'], capture_output=True)
                    if check_pkg.returncode == 0:
                        self.bridge_process = subprocess.Popen(
                            ['ros2', 'launch', 'horus_unity_bridge', 'unity_bridge.launch.py'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True
                        )
                        atexit.register(self._cleanup_bridge)
                        with cli.status("Launching Horus Bridge...", spinner="dots"):
                            for _ in range(15):
                                if is_port_open(10000):
                                    bridge_running = True
                                    cli.print_success("Horus Bridge launched successfully.")
                                    break
                                time.sleep(1.0)
                    
                    if not bridge_running:
                        cli.print_error("Failed to auto-launch bridge.")
                        return False, {"error": "Bridge Start Failed"}
                except FileNotFoundError:
                     cli.print_error("'ros2' command not found.")
                     return False, {"error": "ROS2 Not Found"}

            # Display Connect Info using Dashboard
            local_ip = self._get_local_ip()
            bridge_state = "Active" if bridge_running else "Error"
            
            # 2. Build configuration dict
            config = self._build_robot_config_dict(robot, dataviz)
            config_json = json.dumps(config)
            
            robot_topics = self._collect_topics(dataviz)
            monitored_topics = ['/horus/registration', '/horus/registration_ack', '/horus/heartbeat'] + robot_topics
            backend_publishes = {"/horus/registration_ack", "/horus/heartbeat"}

            try:
                from horus.utils.topic_monitor import get_topic_monitor
                monitor = get_topic_monitor()
                monitor.watch_topics(monitored_topics)
                monitor.start()
            except Exception:
                pass

            # Prepare for Async wait
            self._ack_received.clear()
            self._last_ack_data = {}
            
            # Publish registration request
            msg = String()
            msg.data = config_json
            robot_name = robot.name
            self._ack_by_robot.pop(robot_name, None)
            
            # Initial publish
            self.publisher.publish(msg)
            
            registration_success = False
            final_ack = {}

            if not show_dashboard:
                start_time = time.time()
                while time.time() - start_time < timeout_sec:
                    try:
                        rclpy.spin_once(self.node, timeout_sec=0.1)
                    except Exception:
                        pass

                    # Re-publish occasionally
                    if int(time.time()) % 2 == 0:
                        self.publisher.publish(msg)

                    if self._ack_received.is_set():
                        ack = self._ack_by_robot.pop(robot_name, None) or self._last_ack_data
                        recv_name = ack.get("robot_name", "UNKNOWN")
                        if recv_name == robot_name:
                            if ack.get("success"):
                                return True, ack
                            return False, ack
                        self._ack_received.clear()

                return False, {"error": "Registration timeout"}

            # --- Enter Dashboard Mode ---
            with cli.ConnectionDashboard(local_ip, 10000, bridge_state) as dashboard:
                dashboard.update_status("Waiting for Horus App...")
                last_stats_update = 0.0
                
                # Connection Monitoring State (for keep_alive)
                last_connection_state = True # Assume true initially to avoid "Re-established" log on first connect
                
                while True:
                    # spin to process callbacks (Ack, Heartbeat)
                    try:
                        rclpy.spin_once(self.node, timeout_sec=0.1)
                    except Exception:
                        pass
                    
                    # Update Topic Stats (every ~1s)
                    if (time.time() - last_stats_update) >= 1.0:
                        stats = {}
                        for topic in monitored_topics:
                            pubs, subs = self._get_topic_counts(topic)
                            stats[topic] = {'pubs': pubs, 'subs': subs}

                        topic_states = board.snapshot()
                        states = {}
                        for topic, counts in stats.items():
                            state = topic_states.get(topic, "")
                            if not state:
                                if topic in backend_publishes:
                                    if self._has_backend_publisher(topic):
                                        state = "SUBSCRIBED"
                                    elif counts.get("subs", 0) > 0:
                                        state = "UNSUBSCRIBED"
                                else:
                                    if self._has_backend_subscriber(topic) and counts.get("pubs", 0) > 0:
                                        state = "SUBSCRIBED"
                                    elif counts.get("pubs", 0) > 0:
                                        state = "UNSUBSCRIBED"
                            states[topic] = state

                        dashboard.update_topics(stats, states)
                        last_stats_update = time.time()

                    # --- Registration Phase ---
                    if not registration_success:
                        # Re-publish occasionally
                        if int(time.time()) % 2 == 0:
                            self.publisher.publish(msg)

                        if self._ack_received.is_set():
                            ack = self._ack_by_robot.pop(robot_name, None) or self._last_ack_data
                            recv_name = ack.get("robot_name", "UNKNOWN")
                            if recv_name == robot_name:
                                if ack.get("success"):
                                    status_msg = ack.get('robot_id')
                                    dashboard.update_status(f"Success! {status_msg}")
                                    registration_success = True
                                    final_ack = ack
                                    
                                    # If NOT keep_alive, we are done
                                    if not keep_alive:
                                        time.sleep(1.5)
                                        return True, ack
                                    
                                    # If keep_alive, we transition to monitoring
                                    # Reset connection state to 'Connected' explicitly
                                    last_connection_state = True 
                                    self._last_heartbeat_time = time.time() # Fake a fresh heartbeat to prevent immediate timeout
                                else:
                                    dashboard.update_status(f"Refused: {ack.get('error')}")
                                    time.sleep(2.0)
                                    return False, ack
                            else:
                                self._ack_received.clear()
                    
                    # --- Monitoring Phase (keep_alive=True) ---
                    else:
                        # Check Heartbeat with debounce to avoid false disconnects
                        heartbeat_timeout_s = 3.0
                        time_since_hb = time.time() - self._last_heartbeat_time
                        is_connected = time_since_hb < heartbeat_timeout_s and self._last_heartbeat_time > 0
                        
                        if is_connected:
                            ip_str = f" ({self._remote_ip})" if self._remote_ip else ""
                            dashboard.update_status(f"Connected{ip_str}")
                            
                            if not last_connection_state:
                                # Start Re-registration
                                dashboard.update_status(f"Re-connecting{ip_str}...")
                                # Re-publish fresh config
                                config = self._build_robot_config_dict(robot, dataviz)
                                msg = String()
                                msg.data = json.dumps(config)
                                self.publisher.publish(msg)
                                dashboard.update_status(f"Re-registering...")
                                time.sleep(0.5) # Give it a moment
                        else:
                            dashboard.update_status("Disconnected (Waiting for App...)")
                        
                        last_connection_state = is_connected

                    # Update animated elements
                    dashboard.tick()
                    
                    # Simple loop throttle
                    # Note: rclpy.spin_once acts as the sleep/throttle
            
        except KeyboardInterrupt:
            cli.print_info("Registration/Monitoring cancelled by user.")
            return False, {"error": "Cancelled"}
        except Exception as e:
            cli.print_error(f"Structure Error: {e}")
            import traceback
            traceback.print_exc()
            return False, {"error": str(e)}
        finally:
            self._registration_lock.release()



    def register_robots(
        self,
        robots,
        datavizs=None,
        timeout_sec: float = 10.0,
        keep_alive: bool = True,
        show_dashboard: bool = True,
    ) -> Tuple[bool, Dict]:
        """Register multiple robots with a single session."""
        from horus.utils import cli
        from horus.utils.topic_status import get_topic_status_board

        if not self.ros_initialized:
            cli.print_error("ROS2 not initialized. Cannot register.")
            return False, {"error": "ROS2 not available"}

        if not self._registration_lock.acquire(blocking=False):
            return False, {"error": "Registration already in progress"}

        board = get_topic_status_board()
        board.set_silent(show_dashboard)

        try:
            if datavizs is None:
                datavizs = [robot.create_dataviz() for robot in robots]

            if len(datavizs) != len(robots):
                return False, {"error": "Robot/dataviz length mismatch"}

            bridge_running = False

            def is_port_open(port):
                import socket
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(0.5)
                    return s.connect_ex(('localhost', port)) == 0

            if is_port_open(10000):
                bridge_running = True
                cli.print_info("Horus Bridge detected on port 10000.")
            else:
                cli.print_info("No Bridge on port 10000. Attempting auto-start...")
                try:
                    check_pkg = subprocess.run(['ros2', 'pkg', 'prefix', 'horus_unity_bridge'], capture_output=True)
                    if check_pkg.returncode == 0:
                        self.bridge_process = subprocess.Popen(
                            ['ros2', 'launch', 'horus_unity_bridge', 'unity_bridge.launch.py'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True
                        )
                        atexit.register(self._cleanup_bridge)
                        with cli.status("Launching Horus Bridge...", spinner="dots"):
                            for _ in range(15):
                                if is_port_open(10000):
                                    bridge_running = True
                                    cli.print_success("Horus Bridge launched successfully.")
                                    break
                                time.sleep(1.0)

                    if not bridge_running:
                        cli.print_error("Failed to auto-launch bridge.")
                        return False, {"error": "Bridge Start Failed"}
                except FileNotFoundError:
                    cli.print_error("'ros2' command not found.")
                    return False, {"error": "ROS2 Not Found"}

            local_ip = self._get_local_ip()
            bridge_state = "Active" if bridge_running else "Error"

            entries = []
            monitored_topics = set(["/horus/registration", "/horus/registration_ack", "/horus/heartbeat"])
            backend_publishes = {"/horus/registration_ack", "/horus/heartbeat"}
            for robot, dataviz in zip(robots, datavizs):
                config = self._build_robot_config_dict(robot, dataviz)
                msg = String()
                msg.data = json.dumps(config)
                entries.append((robot, dataviz, msg))
                for topic in self._collect_topics(dataviz):
                    monitored_topics.add(topic)
            monitored_topics = sorted(monitored_topics)

            try:
                from horus.utils.topic_monitor import get_topic_monitor
                monitor = get_topic_monitor()
                monitor.watch_topics(monitored_topics)
                monitor.start()
            except Exception:
                pass

            def publish_and_wait(robot, dataviz, msg):
                robot_name = robot.name
                self._ack_received.clear()
                self._last_ack_data = {}
                self._ack_by_robot.pop(robot_name, None)
                start_time = time.time()
                last_publish = 0.0
                while (time.time() - start_time) < timeout_sec:
                    now = time.time()
                    if now - last_publish >= 2.0:
                        self.publisher.publish(msg)
                        last_publish = now
                    try:
                        rclpy.spin_once(self.node, timeout_sec=0.1)
                    except Exception:
                        pass
                    ack = self._ack_by_robot.pop(robot_name, None)
                    if ack is None and self._ack_received.is_set():
                        self._ack_received.clear()
                        if self._last_ack_data.get("robot_name") == robot_name:
                            ack = self._last_ack_data
                    if ack:
                        if ack.get("success"):
                            self._apply_registration_metadata(robot, dataviz, ack)
                            return True, ack
                        return False, ack
                return False, {"error": "Registration timeout"}

            def register_all(dashboard=None):
                for robot, dataviz, msg in entries:
                    if dashboard is not None:
                        dashboard.update_status(f"Registering {robot.name}...")
                    ok, ack = publish_and_wait(robot, dataviz, msg)
                    if not ok:
                        return False, ack
                return True, {"success": True}

            def update_dashboard_topics(dashboard):
                if dashboard is None:
                    return
                stats = {}
                for topic in monitored_topics:
                    pubs, subs = self._get_topic_counts(topic)
                    stats[topic] = {'pubs': pubs, 'subs': subs}

                topic_states = board.snapshot()
                states = {}
                for topic, counts in stats.items():
                    state = topic_states.get(topic, "")
                    if not state:
                        if topic in backend_publishes:
                            if self._has_backend_publisher(topic):
                                state = "SUBSCRIBED"
                            elif counts.get("subs", 0) > 0:
                                state = "UNSUBSCRIBED"
                        else:
                            if self._has_backend_subscriber(topic) and counts.get("pubs", 0) > 0:
                                state = "SUBSCRIBED"
                            elif counts.get("pubs", 0) > 0:
                                state = "UNSUBSCRIBED"
                    states[topic] = state

                dashboard.update_topics(stats, states)

            if not show_dashboard:
                ok, result = register_all(None)
                if not ok:
                    return False, result
                if not keep_alive:
                    return True, result
                last_connection_state = True
                self._last_heartbeat_time = time.time()
                while True:
                    try:
                        rclpy.spin_once(self.node, timeout_sec=0.1)
                    except Exception:
                        pass
                    heartbeat_timeout_s = 3.0
                    time_since_hb = time.time() - self._last_heartbeat_time
                    is_connected = time_since_hb < heartbeat_timeout_s and self._last_heartbeat_time > 0
                    if is_connected and not last_connection_state:
                        for robot, dataviz, msg in entries:
                            publish_and_wait(robot, dataviz, msg)
                    last_connection_state = is_connected
                    time.sleep(0.2)

            with cli.ConnectionDashboard(local_ip, 10000, bridge_state) as dashboard:
                dashboard.update_status("Waiting for Horus App...")
                update_dashboard_topics(dashboard)

                registration_done = False
                last_topics_update = 0.0
                last_register_attempt = 0.0
                last_connection_state = True

                while True:
                    try:
                        rclpy.spin_once(self.node, timeout_sec=0.1)
                    except Exception:
                        pass

                    if (time.time() - last_topics_update) >= 1.0:
                        update_dashboard_topics(dashboard)
                        last_topics_update = time.time()

                    heartbeat_timeout_s = 3.0
                    time_since_hb = time.time() - self._last_heartbeat_time
                    is_connected = time_since_hb < heartbeat_timeout_s and self._last_heartbeat_time > 0

                    if not registration_done:
                        if time.time() - last_register_attempt < 1.0:
                            dashboard.tick()
                            continue
                        last_register_attempt = time.time()

                        dashboard.update_status("Registering robots...")
                        ok, result = register_all(dashboard)
                        if not ok:
                            if keep_alive:
                                dashboard.update_status("Waiting for Horus App...")
                                dashboard.tick()
                                continue
                            return False, result

                        registration_done = True
                        if not keep_alive:
                            time.sleep(1.0)
                            return True, result

                        last_connection_state = False
                        continue

                    if is_connected:
                        ip_str = f" ({self._remote_ip})" if self._remote_ip else ""
                        dashboard.update_status(f"Connected{ip_str}")
                        if not last_connection_state:
                            dashboard.update_status(f"Re-registering{ip_str}...")
                            register_all(dashboard)
                    else:
                        if last_connection_state:
                            dashboard.update_status("Disconnected (Waiting for App...)")
                        else:
                            dashboard.update_status("Registered (Waiting for App...)")

                    last_connection_state = is_connected
                    dashboard.tick()

        except KeyboardInterrupt:
            cli.print_info("Registration/Monitoring cancelled by user.")
            return False, {"error": "Cancelled"}
        except Exception as e:
            cli.print_error(f"Structure Error: {e}")
            import traceback
            traceback.print_exc()
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
        dimensions = None
        if getattr(robot, "dimensions", None):
            dimensions = {
                "length": float(robot.dimensions.length),
                "width": float(robot.dimensions.width),
                "height": float(robot.dimensions.height),
            }

        config = {
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

        if dimensions is not None:
            config["dimensions"] = dimensions

        return config



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


_singleton_registry: Optional[RobotRegistryClient] = None


def get_robot_registry_client() -> RobotRegistryClient:
    global _singleton_registry
    if _singleton_registry is None:
        _singleton_registry = RobotRegistryClient()
    return _singleton_registry
