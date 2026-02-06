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
        self._app_heartbeats = {}
        self._app_uptime_by_id = {}
        self._app_restart_seq = 0
        self._last_consumed_restart_seq = 0
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
        now = time.time()
        self._last_heartbeat_time = now
        app_id = "unknown"
        heartbeat_uptime = None

        if "Heartbeat:" in msg.data:
            try:
                heartbeat_part = msg.data.split("Heartbeat:", 1)[1]
                heartbeat_value = heartbeat_part.split("|", 1)[0].strip()
                heartbeat_uptime = float(heartbeat_value)
            except Exception:
                heartbeat_uptime = None

        # Parse IP if present "Heartbeat: <time> | IP: <ip>"
        if "IP:" in msg.data:
            try:
                parts = msg.data.split("IP:")
                if len(parts) > 1:
                    ip = parts[1].strip()
                    self._remote_ip = ip
                    if ip:
                        app_id = ip
                        self._app_heartbeats[ip] = now
            except:
                pass
        else:
            self._app_heartbeats["unknown"] = now

        if heartbeat_uptime is not None:
            previous_uptime = self._app_uptime_by_id.get(app_id)
            # Unity Time.time resets on app restart; detect this to force re-registration.
            if previous_uptime is not None and heartbeat_uptime < (previous_uptime - 0.75):
                self._app_restart_seq += 1
            self._app_uptime_by_id[app_id] = heartbeat_uptime

    def _get_active_app_count(self, timeout_s: float) -> int:
        now = time.time()
        stale = [ip for ip, ts in self._app_heartbeats.items() if (now - ts) > timeout_s]
        for ip in stale:
            self._app_heartbeats.pop(ip, None)
            self._app_uptime_by_id.pop(ip, None)
        return len(self._app_heartbeats)

    def _consume_app_restart_signal(self) -> bool:
        if self._app_restart_seq != self._last_consumed_restart_seq:
            self._last_consumed_restart_seq = self._app_restart_seq
            return True
        return False

    def _await_ack(self, robot_name: str, timeout_sec: float):
        start_time = time.time()
        while (time.time() - start_time) < timeout_sec:
            if robot_name in self._ack_by_robot:
                return self._ack_by_robot.pop(robot_name)
            self._ack_received.wait(0.1)
            self._ack_received.clear()
        return None

    def _queued_reason_from_ack(self, ack: Optional[Dict]) -> Optional[str]:
        if not isinstance(ack, dict):
            return None
        status_msg = ack.get("robot_id") or ""
        if isinstance(status_msg, str) and status_msg.startswith("Queued"):
            detail = status_msg[len("Queued"):].strip()
            if detail.startswith("(") and detail.endswith(")"):
                detail = detail[1:-1].strip()
            if detail.startswith(":") or detail.startswith("-"):
                detail = detail[1:].strip()
            return detail or "Waiting for Workspace"
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

    def _is_local_node_name(self, node_name: Optional[str]) -> bool:
        if not node_name:
            return False
        try:
            return self.node is not None and node_name == self.node.get_name()
        except Exception:
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

    def _has_local_publisher(self, topic: str) -> bool:
        try:
            infos = self.node.get_publishers_info_by_topic(topic)
        except Exception:
            return False
        return any(self._is_local_node_name(info.node_name) for info in infos)

    def _has_local_subscriber(self, topic: str) -> bool:
        try:
            infos = self.node.get_subscriptions_info_by_topic(topic)
        except Exception:
            return False
        return any(self._is_local_node_name(info.node_name) for info in infos)

    def _build_topic_roles(self, robot_topics):
        roles = {
            "/horus/registration": "sdk_pub",
            "/horus/registration_ack": "sdk_sub",
            "/horus/heartbeat": "sdk_sub",
        }
        for topic in robot_topics:
            if topic not in roles:
                roles[topic] = "backend_sub"
        return roles

    def _topic_group(self, topic: str, robot_names):
        if topic in ("/horus/registration", "/horus/registration_ack", "/horus/heartbeat"):
            return "sdk"
        if topic == "/tf":
            return "shared"
        for name in robot_names:
            prefix = f"/{name}/"
            if topic.startswith(prefix):
                return name
        return "shared"

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
            topic_roles = self._build_topic_roles(robot_topics)
            core_topics = ["/horus/registration", "/horus/registration_ack", "/horus/heartbeat"]
            monitored_topics = core_topics + [t for t in robot_topics if t not in core_topics]

            try:
                from horus.utils.topic_monitor import get_topic_monitor
                monitor = get_topic_monitor()
                monitor.watch_topics(monitored_topics, topic_roles)
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
            show_counts = bool(os.getenv("HORUS_DASHBOARD_SHOW_COUNTS"))
            with cli.ConnectionDashboard(local_ip, 10000, bridge_state, show_counts=show_counts) as dashboard:
                dashboard.update_app_link("Waiting for Horus App...")
                dashboard.update_registration("Idle")
                dashboard.update_status("")
                last_stats_update = 0.0
                
                # Connection Monitoring State (for keep_alive)
                last_connection_state = True # Assume true initially to avoid "Re-established" log on first connect
                robot_names = [robot.name]
                seen_heartbeat = False
                queued_ack = False
                queued_reason = ""
                last_register_publish = 0.0
                had_connection_drop = False
                
                while True:
                    # spin to process callbacks (Ack, Heartbeat)
                    try:
                        rclpy.spin_once(self.node, timeout_sec=0.1)
                    except Exception:
                        pass

                    if self._last_heartbeat_time > 0:
                        seen_heartbeat = True
                    heartbeat_timeout_s = 3.0
                    is_connected = self._get_active_app_count(heartbeat_timeout_s) > 0
                    app_restarted = self._consume_app_restart_signal()
                    if app_restarted:
                        had_connection_drop = True

                    if is_connected:
                        dashboard.update_app_link("Connected")
                    elif seen_heartbeat:
                        dashboard.update_app_link("Disconnected (Waiting for Horus App...)")
                    else:
                        dashboard.update_app_link("Waiting for Horus App...")
                    
                    # Update Topic Stats (every ~1s)
                    if (time.time() - last_stats_update) >= 1.0:
                        rows = []
                        for topic in monitored_topics:
                            role = topic_roles.get(topic, "backend_sub")
                            if role == "sdk_pub":
                                role_label = "sdk->bridge"
                                link_ok = self._has_backend_subscriber(topic)
                                pubs = 1 if self._has_local_publisher(topic) else 0
                                subs = 1 if link_ok else 0
                                link = "OK" if link_ok else "NO"
                                data = "ACTIVE" if pubs > 0 and link_ok else ("IDLE" if pubs > 0 else "")
                            elif role == "sdk_sub":
                                role_label = "bridge->sdk"
                                link_ok = self._has_backend_publisher(topic)
                                pubs = 1 if link_ok else 0
                                subs = 1 if self._has_local_subscriber(topic) else 0
                                link = "OK" if link_ok else "NO"
                                data = "ACTIVE" if link_ok else ("IDLE" if subs > 0 else "")
                            else:
                                role_label = "data->bridge"
                                link_ok = self._has_backend_subscriber(topic)
                                pubs, _ = self._get_topic_counts(topic)
                                subs = 1 if link_ok else 0
                                link = "OK" if link_ok else "NO"
                                if link_ok and pubs > 0:
                                    data = "ACTIVE"
                                elif link_ok and pubs == 0:
                                    data = "STALE"
                                else:
                                    data = "IDLE"

                            if not is_connected:
                                link = "NO"
                                data = "IDLE"

                            rows.append({
                                "robot": self._topic_group(topic, robot_names),
                                "topic": topic,
                                "role": role_label,
                                "link": link,
                                "data": data,
                                "pubs": pubs,
                                "subs": subs,
                            })

                        rows.sort(key=lambda r: (r.get("robot") or "", r.get("topic") or ""))
                        dashboard.update_topics(rows)
                        last_stats_update = time.time()

                    # --- Registration Phase ---
                    if not registration_success:
                        if not is_connected:
                            if seen_heartbeat:
                                dashboard.update_registration("Waiting for App")
                                dashboard.update_status("")
                            else:
                                dashboard.update_registration("Idle")
                                dashboard.update_status("")
                        elif is_connected:
                            if queued_ack:
                                dashboard.update_registration("Queued")
                                dashboard.update_status(queued_reason or "Waiting for Workspace")
                            else:
                                dashboard.update_registration("Registering")
                                dashboard.update_status(f"Registering {robot_name}...")
                                # Re-publish occasionally (only before queued)
                                now = time.time()
                                if (now - last_register_publish) >= 2.0:
                                    self.publisher.publish(msg)
                                    last_register_publish = now

                        if self._ack_received.is_set():
                            ack = self._ack_by_robot.pop(robot_name, None) or self._last_ack_data
                            recv_name = ack.get("robot_name", "UNKNOWN")
                            if recv_name == robot_name:
                                if ack.get("success"):
                                    queued_msg = self._queued_reason_from_ack(ack)
                                    if queued_msg:
                                        queued_ack = True
                                        queued_reason = queued_msg
                                        dashboard.update_registration("Queued")
                                        dashboard.update_status(queued_msg)
                                    else:
                                        queued_ack = False
                                        queued_reason = ""
                                        dashboard.update_registration("Registered")
                                        status_msg = ack.get("robot_id") or ""
                                        dashboard.update_status(f"Registered {status_msg}")
                                        registration_success = True
                                        final_ack = ack
                                        # If NOT keep_alive, we are done
                                        if not keep_alive:
                                            time.sleep(1.5)
                                            return True, ack
                                    
                                    # If keep_alive, we transition to monitoring
                                    # Reset connection state to 'Connected' explicitly
                                    last_connection_state = True
                                    had_connection_drop = False
                                    self._last_heartbeat_time = time.time() # Fake a fresh heartbeat to prevent immediate timeout
                                else:
                                    dashboard.update_registration("Failed")
                                    dashboard.update_status(f"Refused: {ack.get('error')}")
                                    time.sleep(2.0)
                                    return False, ack
                            else:
                                self._ack_received.clear()
                    
                    # --- Monitoring Phase (keep_alive=True) ---
                    else:
                        if app_restarted:
                            had_connection_drop = True
                        if self._ack_received.is_set():
                            ack = self._ack_by_robot.pop(robot_name, None) or self._last_ack_data
                            recv_name = ack.get("robot_name", "UNKNOWN")
                            if recv_name == robot_name:
                                if ack.get("success"):
                                    queued_msg = self._queued_reason_from_ack(ack)
                                    if queued_msg:
                                        queued_ack = True
                                        queued_reason = queued_msg
                                        dashboard.update_registration("Queued")
                                        dashboard.update_status(queued_msg)
                                    else:
                                        queued_ack = False
                                        queued_reason = ""
                                        dashboard.update_registration("Registered")
                                        status_msg = ack.get("robot_id") or ""
                                        dashboard.update_status(f"Registered {status_msg}")
                                else:
                                    dashboard.update_registration("Failed")
                                    dashboard.update_status(f"Refused: {ack.get('error')}")
                        if is_connected:
                            ip_str = f" ({self._remote_ip})" if self._remote_ip else ""
                            if had_connection_drop and (not last_connection_state or app_restarted):
                                # Start Re-registration
                                dashboard.update_registration("Re-registering")
                                dashboard.update_status(f"Re-registering{ip_str}...")
                                # Re-publish fresh config
                                config = self._build_robot_config_dict(robot, dataviz)
                                msg = String()
                                msg.data = json.dumps(config)
                                self.publisher.publish(msg)
                                dashboard.update_status("Re-registering...")
                                time.sleep(0.5) # Give it a moment
                                had_connection_drop = False
                            else:
                                dashboard.update_registration("Registered")
                                dashboard.update_status("")
                        else:
                            if seen_heartbeat:
                                dashboard.update_registration("Waiting for App")
                            else:
                                dashboard.update_registration("Idle")
                            dashboard.update_status("")
                            if last_connection_state:
                                had_connection_drop = True
                        
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
            entry_by_name = {}
            core_topics = ["/horus/registration", "/horus/registration_ack", "/horus/heartbeat"]
            robot_topics = []
            for robot, dataviz in zip(robots, datavizs):
                config = self._build_robot_config_dict(robot, dataviz)
                msg = String()
                msg.data = json.dumps(config)
                entries.append((robot, dataviz, msg))
                entry_by_name[robot.name] = (robot, dataviz, msg)
                for topic in self._collect_topics(dataviz):
                    if topic and topic not in robot_topics:
                        robot_topics.append(topic)

            topic_roles = self._build_topic_roles(robot_topics)
            monitored_topics = core_topics + [t for t in robot_topics if t not in core_topics]

            try:
                from horus.utils.topic_monitor import get_topic_monitor
                monitor = get_topic_monitor()
                monitor.watch_topics(monitored_topics, topic_roles)
                monitor.start()
            except Exception:
                pass

            robot_states = {robot.name: "pending" for robot in robots}
            queued_reasons = {}
            last_failure = None

            def handle_ack(ack):
                nonlocal last_failure
                if not isinstance(ack, dict):
                    return
                robot_name = ack.get("robot_name")
                if not robot_name:
                    return
                if ack.get("success"):
                    queued_msg = self._queued_reason_from_ack(ack)
                    if queued_msg:
                        robot_states[robot_name] = "queued"
                        queued_reasons[robot_name] = queued_msg
                        return
                    robot_states[robot_name] = "registered"
                    queued_reasons.pop(robot_name, None)
                    entry = entry_by_name.get(robot_name)
                    if entry is not None:
                        self._apply_registration_metadata(entry[0], entry[1], ack)
                else:
                    robot_states[robot_name] = "failed"
                    last_failure = ack.get("error") or "Registration failed"

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
                            return True, ack
                        return False, ack
                return False, {"error": "Registration timeout"}

            def register_all(dashboard=None, force: bool = False):
                for robot, dataviz, msg in entries:
                    state = robot_states.get(robot.name, "pending")
                    if not force and state in ("registered", "queued"):
                        continue
                    if dashboard is not None:
                        dashboard.update_status(f"Registering {robot.name}...")
                    ok, ack = publish_and_wait(robot, dataviz, msg)
                    if not ok:
                        return False, ack
                    handle_ack(ack)
                return True, {"success": True}

            def update_dashboard_topics(dashboard, is_connected: bool, registration_done: bool):
                if dashboard is None:
                    return
                rows = []
                for topic in monitored_topics:
                    role = topic_roles.get(topic, "backend_sub")
                    if role == "sdk_pub":
                        role_label = "sdk->bridge"
                        link_ok = self._has_backend_subscriber(topic)
                        pubs = 1 if self._has_local_publisher(topic) else 0
                        subs = 1 if link_ok else 0
                        link = "OK" if link_ok else "NO"
                        data = "ACTIVE" if pubs > 0 and link_ok else ("IDLE" if pubs > 0 else "")
                    elif role == "sdk_sub":
                        role_label = "bridge->sdk"
                        link_ok = self._has_backend_publisher(topic)
                        pubs = 1 if link_ok else 0
                        subs = 1 if self._has_local_subscriber(topic) else 0
                        link = "OK" if link_ok else "NO"
                        data = "ACTIVE" if link_ok else ("IDLE" if subs > 0 else "")
                    else:
                        role_label = "data->bridge"
                        link_ok = self._has_backend_subscriber(topic)
                        pubs, _ = self._get_topic_counts(topic)
                        subs = 1 if link_ok else 0
                        link = "OK" if link_ok else "NO"
                        if link_ok and pubs > 0:
                            data = "ACTIVE"
                        elif link_ok and pubs == 0:
                            data = "STALE"
                        else:
                            data = "IDLE"

                    if not is_connected:
                        link = "NO"
                        data = "IDLE"

                    rows.append({
                        "robot": self._topic_group(topic, robot_names),
                        "topic": topic,
                        "role": role_label,
                        "link": link,
                        "data": data,
                        "pubs": pubs,
                        "subs": subs,
                    })

                rows.sort(key=lambda r: (r.get("robot") or "", r.get("topic") or ""))
                dashboard.update_topics(rows)

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
                    is_connected = self._get_active_app_count(heartbeat_timeout_s) > 0
                    app_restarted = self._consume_app_restart_signal()
                    if is_connected and (not last_connection_state or app_restarted):
                        for robot, dataviz, msg in entries:
                            publish_and_wait(robot, dataviz, msg)
                    last_connection_state = is_connected
                    time.sleep(0.2)

            show_counts = bool(os.getenv("HORUS_DASHBOARD_SHOW_COUNTS"))
            with cli.ConnectionDashboard(local_ip, 10000, bridge_state, show_counts=show_counts) as dashboard:
                dashboard.update_app_link("Waiting for Horus App...")
                dashboard.update_registration("Idle")
                dashboard.update_status("")

                registration_done = False
                last_topics_update = 0.0
                last_register_attempt = 0.0
                last_connection_state = True
                robot_names = [r.name for r in robots]
                seen_heartbeat = False
                had_connection_drop = False

                update_dashboard_topics(dashboard, False, False)

                while True:
                    try:
                        rclpy.spin_once(self.node, timeout_sec=0.1)
                    except Exception:
                        pass

                    if self._last_heartbeat_time > 0:
                        seen_heartbeat = True
                    heartbeat_timeout_s = 3.0
                    is_connected = self._get_active_app_count(heartbeat_timeout_s) > 0
                    app_restarted = self._consume_app_restart_signal()
                    if app_restarted:
                        had_connection_drop = True

                    if is_connected:
                        dashboard.update_app_link("Connected")
                    elif seen_heartbeat:
                        dashboard.update_app_link("Disconnected (Waiting for Horus App...)")
                    else:
                        dashboard.update_app_link("Waiting for Horus App...")

                    if not is_connected:
                        if last_connection_state:
                            had_connection_drop = True
                        if seen_heartbeat:
                            dashboard.update_registration("Waiting for App")
                        else:
                            dashboard.update_registration("Idle")
                        dashboard.update_status("")
                        if (time.time() - last_topics_update) >= 1.0:
                            update_dashboard_topics(dashboard, is_connected, registration_done)
                            last_topics_update = time.time()
                        last_connection_state = is_connected
                        dashboard.tick()
                        continue

                    if self._ack_received.is_set() or self._ack_by_robot:
                        self._ack_received.clear()
                        pending = list(self._ack_by_robot.items())
                        self._ack_by_robot.clear()
                        if not pending and isinstance(self._last_ack_data, dict):
                            last_robot = self._last_ack_data.get("robot_name")
                            if last_robot:
                                pending = [(last_robot, self._last_ack_data)]
                        for _, ack in pending:
                            handle_ack(ack)

                    states = list(robot_states.values())
                    any_failed = any(state == "failed" for state in states)
                    any_queued = any(state == "queued" for state in states)
                    any_pending = any(state == "pending" for state in states)
                    all_registered = bool(states) and all(state == "registered" for state in states)
                    registration_done = all_registered

                    if (time.time() - last_topics_update) >= 1.0:
                        update_dashboard_topics(dashboard, is_connected, registration_done)
                        last_topics_update = time.time()

                    if any_failed:
                        dashboard.update_registration("Failed")
                        dashboard.update_status(last_failure or "Registration failed")
                        dashboard.tick()
                        continue

                    queued_status = ""
                    if any_queued:
                        reasons = list(queued_reasons.values())
                        if reasons and all(reason == reasons[0] for reason in reasons):
                            queued_status = reasons[0]
                        else:
                            queued_status = "Waiting for Workspace"

                    if not registration_done:
                        if not is_connected:
                            if any_queued:
                                dashboard.update_registration("Queued")
                                dashboard.update_status(queued_status)
                            else:
                                dashboard.update_registration("Idle")
                                dashboard.update_status("")
                            dashboard.tick()
                            continue
                        if any_queued and not any_pending:
                            dashboard.update_registration("Queued")
                            dashboard.update_status(queued_status)
                            dashboard.tick()
                            continue
                        if time.time() - last_register_attempt < 1.0:
                            dashboard.tick()
                            continue
                        last_register_attempt = time.time()

                        dashboard.update_registration("Registering")
                        dashboard.update_status("Registering robots...")
                        ok, result = register_all(dashboard)
                        if not ok:
                            if keep_alive:
                                dashboard.update_registration("Idle")
                                dashboard.update_status("")
                                dashboard.tick()
                                continue
                            return False, result

                        states = list(robot_states.values())
                        any_queued = any(state == "queued" for state in states)
                        all_registered = bool(states) and all(state == "registered" for state in states)
                        registration_done = all_registered
                        if any_queued:
                            reasons = list(queued_reasons.values())
                            if reasons and all(reason == reasons[0] for reason in reasons):
                                queued_status = reasons[0]
                            else:
                                queued_status = "Waiting for Workspace"

                        if all_registered:
                            if not keep_alive:
                                time.sleep(1.0)
                                return True, result
                            dashboard.update_registration("Registered")
                            dashboard.update_status("")
                            last_connection_state = is_connected
                            continue

                        if any_queued:
                            dashboard.update_registration("Queued")
                            dashboard.update_status(queued_status or "Waiting for Workspace")
                            dashboard.tick()
                            continue

                        dashboard.update_registration("Idle")
                        dashboard.update_status("")
                        dashboard.tick()
                        continue

                    if is_connected:
                        ip_str = f" ({self._remote_ip})" if self._remote_ip else ""
                        if had_connection_drop and (not last_connection_state or app_restarted):
                            dashboard.update_registration("Re-registering")
                            dashboard.update_status(f"Re-registering{ip_str}...")
                            ok, result = register_all(dashboard, force=True)
                            had_connection_drop = False

                            states = list(robot_states.values())
                            any_queued = any(state == "queued" for state in states)
                            all_registered = bool(states) and all(state == "registered" for state in states)
                            registration_done = all_registered
                            if any_queued:
                                reasons = list(queued_reasons.values())
                                if reasons and all(reason == reasons[0] for reason in reasons):
                                    queued_status = reasons[0]
                                else:
                                    queued_status = "Waiting for Workspace"

                            if not ok:
                                dashboard.update_registration("Failed")
                                dashboard.update_status((result or {}).get("error") or "Registration failed")
                            elif all_registered:
                                dashboard.update_registration("Registered")
                                dashboard.update_status("")
                            elif any_queued:
                                dashboard.update_registration("Queued")
                                dashboard.update_status(queued_status or "Waiting for Workspace")
                        else:
                            dashboard.update_registration("Registered")
                            dashboard.update_status("")
                    else:
                        dashboard.update_registration("Registered")
                        dashboard.update_status("")

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

        def _coerce_bool(value, default):
            if isinstance(value, bool):
                return value
            if isinstance(value, str):
                normalized = value.strip().lower()
                if normalized in ("1", "true", "yes", "on"):
                    return True
                if normalized in ("0", "false", "no", "off"):
                    return False
            if isinstance(value, (int, float)):
                return bool(value)
            return default

        def _coerce_float(value, default):
            if value is None:
                return float(default)
            try:
                return float(value)
            except (TypeError, ValueError):
                return float(default)

        def _coerce_vec3(value, default_xyz):
            x_default, y_default, z_default = default_xyz
            if isinstance(value, dict):
                x = _coerce_float(value.get("x"), x_default)
                y = _coerce_float(value.get("y"), y_default)
                z = _coerce_float(value.get("z"), z_default)
                return {"x": x, "y": y, "z": z}
            if isinstance(value, (list, tuple)) and len(value) == 3:
                x = _coerce_float(value[0], x_default)
                y = _coerce_float(value[1], y_default)
                z = _coerce_float(value[2], z_default)
                return {"x": x, "y": y, "z": z}
            return {
                "x": float(x_default),
                "y": float(y_default),
                "z": float(z_default),
            }

        def _build_camera_config(sensor):
            metadata = sensor.metadata or {}
            return {
                "image_type": str(metadata.get("image_type", "raw")).lower(),
                "display_mode": str(metadata.get("display_mode", "projected")).lower(),
                "use_tf": _coerce_bool(metadata.get("use_tf"), True),
                "projection_target_frame": str(metadata.get("projection_target_frame", "")),
                "image_scale": _coerce_float(metadata.get("image_scale"), 1.0),
                "focal_length_scale": _coerce_float(metadata.get("focal_length_scale"), 0.5),
                "view_position_offset": _coerce_vec3(
                    metadata.get("view_position_offset"), (0.0, 0.0, 0.0)
                ),
                "view_rotation_offset": _coerce_vec3(
                    metadata.get("view_rotation_offset"), (0.0, 0.0, 0.0)
                ),
                "show_frustum": _coerce_bool(metadata.get("show_frustum"), True),
                "frustum_color": str(metadata.get("frustum_color", "#FFFF00")),
                "overhead_size": _coerce_float(metadata.get("overhead_size"), 1.0),
                "overhead_position_offset": _coerce_vec3(
                    metadata.get("overhead_position_offset"), (0.0, 2.0, 0.0)
                ),
                "overhead_face_camera": _coerce_bool(
                    metadata.get("overhead_face_camera"), True
                ),
                "overhead_rotation_offset": _coerce_vec3(
                    metadata.get("overhead_rotation_offset"), (90.0, 0.0, 0.0)
                ),
                "resolution": {
                    "width": int(getattr(sensor, "resolution", (640, 480))[0]),
                    "height": int(getattr(sensor, "resolution", (640, 480))[1]),
                },
                "fps": int(getattr(sensor, "fps", 30)),
                "fov": _coerce_float(getattr(sensor, "fov", 60.0), 60.0),
                "encoding": str(getattr(sensor, "encoding", "bgr8")),
            }

        sensor_payloads = []
        for sensor in robot.sensors:
            payload = {
                "name": sensor.name,
                "type": sensor.sensor_type.value,
                "topic": sensor.topic,
                "frame": sensor.frame_id,
                "metadata": sensor.metadata or {},
                "viz_config": {
                    "color": getattr(sensor, "color", "white"),
                    "point_size": getattr(sensor, "point_size", 0.05),
                },
            }
            if sensor.sensor_type.value == "camera":
                payload["camera_config"] = _build_camera_config(sensor)
            sensor_payloads.append(payload)

        config = {
            "action": "register",
            "robot_name": robot.name,
            "robot_type": robot.get_type_str(),
            "sensors": sensor_payloads,
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
