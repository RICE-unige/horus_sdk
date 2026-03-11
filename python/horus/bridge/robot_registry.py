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
import math
import shutil
import shlex
from typing import Any, Dict, Tuple, Optional, List
from horus.description import RobotDescriptionResolver

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
        self.sdk_replay_begin_publisher = None
        self.sdk_replay_item_publisher = None
        self.sdk_replay_end_publisher = None
        self.robot_description_chunk_begin_publisher = None
        self.robot_description_chunk_item_publisher = None
        self.robot_description_chunk_end_publisher = None
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
        self._app_presence_by_id: Dict[str, Dict[str, Any]] = {}
        self._app_restart_seq = 0
        self._last_consumed_restart_seq = 0
        self._remote_ip = None
        self._teleop_runtime_by_topic: Dict[str, Dict[str, Any]] = {}
        self._teleop_runtime_state_ttl_s = 3.5
        self._multi_operator_presence_ttl_s = 5.0
        self._registration_replay_request_seq = 0
        self._last_consumed_registration_replay_request_seq = 0
        self._last_registration_replay_request: Dict[str, Any] = {}
        self._sdk_replay_burst_attempts = 3
        self._sdk_replay_burst_initial_delay_s = 0.15
        self._sdk_replay_burst_inter_attempt_delay_s = 0.25
        self._robot_description_chunk_begin_delay_s = 0.03
        self._robot_description_chunk_item_delay_s = 0.006
        self._robot_description_chunk_batch_size = 6
        self._robot_description_chunk_batch_pause_s = 0.02
        self._robot_description_chunk_end_delay_s = 0.05
        self._robot_description_resolver = RobotDescriptionResolver()
        self._robot_description_by_robot: Dict[str, Dict[str, Any]] = {}
        self._robot_description_by_id: Dict[str, Dict[str, Any]] = {}
        
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
                depth=256
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

            sdk_replay_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=256,
            )

            self.sdk_replay_begin_publisher = self.node.create_publisher(
                String, "/horus/multi_operator/sdk_registry_replay_begin", sdk_replay_qos
            )
            self.sdk_replay_item_publisher = self.node.create_publisher(
                String, "/horus/multi_operator/sdk_registry_replay_item", sdk_replay_qos
            )
            self.sdk_replay_end_publisher = self.node.create_publisher(
                String, "/horus/multi_operator/sdk_registry_replay_end", sdk_replay_qos
            )
            self.robot_description_chunk_begin_publisher = self.node.create_publisher(
                String, "/horus/robot_description/chunk_begin", sdk_replay_qos
            )
            self.robot_description_chunk_item_publisher = self.node.create_publisher(
                String, "/horus/robot_description/chunk_item", sdk_replay_qos
            )
            self.robot_description_chunk_end_publisher = self.node.create_publisher(
                String, "/horus/robot_description/chunk_end", sdk_replay_qos
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

            self.node.create_subscription(
                String,
                "/horus/teleop/runtime_state",
                self._teleop_runtime_state_callback,
                ack_qos,
            )

            self.node.create_subscription(
                String,
                "/horus/multi_operator_presence",
                self._multi_operator_presence_callback,
                ack_qos,
            )

            self.node.create_subscription(
                String,
                "/horus/multi_operator/sdk_registration_replay_request",
                self._multi_operator_registration_replay_request_callback,
                ack_qos,
            )

            self.node.create_subscription(
                String,
                "/horus/robot_description/request",
                self._robot_description_request_callback,
                ack_qos,
            )

        except Exception as e:
            cli.print_error(f"Failed to init ROS context: {e}")
            import traceback
            traceback.print_exc()
            self.ros_initialized = False

    def _is_port_open(self, port: int) -> bool:
        """Check whether localhost:port is accepting TCP connections."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            return s.connect_ex(("localhost", port)) == 0

    def _get_bridge_autostart_mode(self) -> str:
        """Resolve bridge auto-start mode from environment."""
        from horus.utils import cli

        raw_mode = str(os.environ.get("HORUS_SDK_BRIDGE_AUTOSTART_MODE", "auto") or "auto").strip().lower()
        valid_modes = {"auto", "ros2", "helper", "off"}
        if raw_mode in valid_modes:
            return raw_mode

        cli.print_info(
            f"Unknown HORUS_SDK_BRIDGE_AUTOSTART_MODE='{raw_mode}'. Using 'auto'."
        )
        return "auto"

    def _get_bridge_startup_settle_delay_s(self) -> float:
        """Optional delay after bridge port opens to allow internal startup to settle."""
        raw_value = str(os.environ.get("HORUS_SDK_BRIDGE_STARTUP_SETTLE_MS", "750") or "750").strip()
        try:
            delay_ms = float(raw_value)
        except Exception:
            delay_ms = 750.0

        if not math.isfinite(delay_ms) or delay_ms < 0:
            delay_ms = 750.0

        return delay_ms / 1000.0

    def _sleep_after_bridge_startup_settle(self) -> None:
        """Sleep briefly after bridge startup if configured."""
        from horus.utils import cli

        delay_s = self._get_bridge_startup_settle_delay_s()
        if delay_s <= 0:
            return

        cli.print_info(f"Waiting {delay_s:.2f}s for bridge startup settle...")
        time.sleep(delay_s)

    def _get_horus_start_helper_candidates(self) -> List[str]:
        """Collect possible horus-start helper paths, preserving priority order."""
        candidate_paths: List[str] = []
        env_horus_home = os.environ.get("HORUS_HOME")
        if env_horus_home:
            candidate_paths.append(os.path.join(env_horus_home, "bin", "horus-start"))

        candidate_paths.append(os.path.expanduser("~/horus/bin/horus-start"))

        horus_start_in_path = shutil.which("horus-start")
        if horus_start_in_path:
            candidate_paths.append(horus_start_in_path)

        deduped_paths: List[str] = []
        seen = set()
        for path in candidate_paths:
            if path not in seen:
                deduped_paths.append(path)
                seen.add(path)
        return deduped_paths

    def _get_horus_helper_env_path(self, helper_path: str) -> Optional[str]:
        """Resolve sibling horus-env for an installer helper path if present."""
        helper_dir = os.path.dirname(os.path.abspath(helper_path))
        env_path = os.path.join(helper_dir, "horus-env")
        if os.path.isfile(env_path):
            return env_path
        return None

    def _get_ros2_pkg_prefix_current_shell(self, package_name: str) -> Optional[str]:
        """Resolve a ROS package prefix using the current shell environment."""
        if shutil.which("ros2") is None:
            return None

        try:
            check_pkg = subprocess.run(
                ["ros2", "pkg", "prefix", package_name],
                capture_output=True,
                text=True,
                timeout=5.0,
            )
        except Exception:
            return None

        if check_pkg.returncode != 0:
            return None

        prefix = str(check_pkg.stdout or "").strip()
        return prefix or None

    def _get_ros2_pkg_prefix_from_helper_env(self, helper_path: str, package_name: str) -> Optional[str]:
        """Resolve a ROS package prefix by sourcing the helper's horus-env script."""
        env_script = self._get_horus_helper_env_path(helper_path)
        if not env_script:
            return None

        bash_path = shutil.which("bash")
        if not bash_path and os.path.isfile("/bin/bash"):
            bash_path = "/bin/bash"
        if not bash_path:
            return None

        command = (
            f"source {shlex.quote(env_script)} >/dev/null 2>&1 && "
            f"ros2 pkg prefix {shlex.quote(package_name)}"
        )
        try:
            check_pkg = subprocess.run(
                [bash_path, "-lc", command],
                capture_output=True,
                text=True,
                timeout=8.0,
            )
        except Exception:
            return None

        if check_pkg.returncode != 0:
            return None

        prefix = str(check_pkg.stdout or "").strip()
        return prefix or None

    def _get_first_helper_bridge_prefix(self) -> Optional[str]:
        """Inspect the first available helper environment and return bridge package prefix."""
        for helper_path in self._get_horus_start_helper_candidates():
            if not (os.path.isfile(helper_path) and os.access(helper_path, os.X_OK)):
                continue
            prefix = self._get_ros2_pkg_prefix_from_helper_env(helper_path, "horus_unity_bridge")
            if prefix:
                return prefix
        return None

    def _auto_start_bridge_with_horus_helper(self) -> bool:
        """Try bridge startup using installer-generated horus-start helper."""
        from horus.utils import cli

        for helper_path in self._get_horus_start_helper_candidates():
            if not (os.path.isfile(helper_path) and os.access(helper_path, os.X_OK)):
                continue

            helper_bridge_prefix = self._get_ros2_pkg_prefix_from_helper_env(helper_path, "horus_unity_bridge")
            if helper_bridge_prefix:
                cli.print_info(f"Helper env horus_unity_bridge prefix: {helper_bridge_prefix}")

            cli.print_info(f"Trying helper auto-start: {helper_path}")
            try:
                self.bridge_process = subprocess.Popen(
                    [helper_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
            except Exception:
                continue

            atexit.register(self._cleanup_bridge)
            with cli.status("Launching Horus Bridge...", spinner="dots"):
                for _ in range(20):
                    if self._is_port_open(10000):
                        self._sleep_after_bridge_startup_settle()
                        cli.print_success("Horus Bridge launched successfully.")
                        return True
                    if self.bridge_process and self.bridge_process.poll() is not None:
                        break
                    time.sleep(1.0)

            self._cleanup_bridge()

        return False

    def _auto_start_bridge_with_ros2_launch(self) -> bool:
        """Fallback bridge startup using direct ros2 launch command."""
        from horus.utils import cli

        if shutil.which("ros2") is None:
            cli.print_error("'ros2' command not found.")
            return False

        try:
            bridge_prefix = self._get_ros2_pkg_prefix_current_shell("horus_unity_bridge")
            if not bridge_prefix:
                return False

            cli.print_info(f"Using horus_unity_bridge from current shell: {bridge_prefix}")

            self.bridge_process = subprocess.Popen(
                ["ros2", "launch", "horus_unity_bridge", "unity_bridge.launch.py"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            atexit.register(self._cleanup_bridge)

            with cli.status("Launching Horus Bridge...", spinner="dots"):
                for _ in range(15):
                    if self._is_port_open(10000):
                        self._sleep_after_bridge_startup_settle()
                        cli.print_success("Horus Bridge launched successfully.")
                        return True
                    if self.bridge_process and self.bridge_process.poll() is not None:
                        break
                    time.sleep(1.0)
        except Exception:
            return False

        self._cleanup_bridge()
        return False

    def _ensure_bridge_running(self) -> bool:
        """Ensure bridge port is available, auto-starting bridge if needed."""
        from horus.utils import cli

        if self._is_port_open(10000):
            cli.print_info("Horus Bridge detected on port 10000.")
            return True

        mode = self._get_bridge_autostart_mode()
        cli.print_info("No Bridge on port 10000. Attempting auto-start...")
        cli.print_info(f"Bridge auto-start mode: {mode}")

        if mode == "off":
            cli.print_error("Bridge auto-start disabled (HORUS_SDK_BRIDGE_AUTOSTART_MODE=off).")
            return False

        if mode == "auto":
            current_shell_prefix = self._get_ros2_pkg_prefix_current_shell("horus_unity_bridge")
            helper_prefix = self._get_first_helper_bridge_prefix()
            if current_shell_prefix and helper_prefix and current_shell_prefix != helper_prefix:
                cli.print_info(
                    "Warning: Current shell ROS workspace differs from helper workspace for horus_unity_bridge."
                )
                cli.print_info(f"Current shell prefix: {current_shell_prefix}")
                cli.print_info(f"Helper workspace prefix: {helper_prefix}")
                cli.print_info("Auto mode will try current-shell ros2 launch first.")

            if self._auto_start_bridge_with_ros2_launch():
                return True

            if self._auto_start_bridge_with_horus_helper():
                return True
        elif mode == "ros2":
            if self._auto_start_bridge_with_ros2_launch():
                return True
        elif mode == "helper":
            if self._auto_start_bridge_with_horus_helper():
                return True

        cli.print_error("Failed to auto-launch bridge.")
        return False

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

    def _multi_operator_presence_callback(self, msg):
        payload_raw = str(getattr(msg, "data", "") or "").strip()
        if not payload_raw:
            return

        try:
            payload = json.loads(payload_raw)
        except Exception:
            return

        if not isinstance(payload, dict):
            return

        now = time.time()
        app_id = str(payload.get("app_id") or payload.get("ip") or "unknown").strip() or "unknown"
        role = str(payload.get("role") or "").strip().lower()
        state = str(payload.get("state") or "").strip().lower()
        ip = str(payload.get("ip") or "").strip()
        session_id = str(payload.get("session_id") or "").strip()

        self._app_presence_by_id[app_id] = {
            "ts": now,
            "role": role,
            "state": state,
            "ip": ip,
            "session_id": session_id,
            "workspace_active": bool(payload.get("workspace_active", False)),
        }

    def _multi_operator_registration_replay_request_callback(self, msg):
        payload_raw = str(getattr(msg, "data", "") or "").strip()
        payload: Dict[str, Any] = {}
        if payload_raw:
            try:
                parsed = json.loads(payload_raw)
                if isinstance(parsed, dict):
                    payload = parsed
            except Exception:
                payload = {}

        self._last_registration_replay_request = payload
        self._registration_replay_request_seq += 1

    def _consume_registration_replay_request(self) -> Optional[Dict[str, Any]]:
        if self._registration_replay_request_seq == self._last_consumed_registration_replay_request_seq:
            return None
        self._last_consumed_registration_replay_request_seq = self._registration_replay_request_seq
        return dict(self._last_registration_replay_request or {})

    def _robot_description_request_callback(self, msg):
        if (
            self.robot_description_chunk_begin_publisher is None
            or self.robot_description_chunk_item_publisher is None
            or self.robot_description_chunk_end_publisher is None
        ):
            return

        payload_raw = str(getattr(msg, "data", "") or "").strip()
        if not payload_raw:
            return

        try:
            payload = json.loads(payload_raw)
        except Exception:
            return

        if not isinstance(payload, dict):
            return

        request_id = str(payload.get("request_id") or uuid.uuid4().hex).strip() or uuid.uuid4().hex
        robot_name = str(payload.get("robot_name") or "").strip()
        description_id = str(payload.get("description_id") or "").strip()
        session_id = str(payload.get("session_id") or "").strip()
        app_id = str(payload.get("app_id") or "").strip()

        artifact_container: Optional[Dict[str, Any]] = None
        if description_id:
            artifact_container = self._robot_description_by_id.get(description_id)
        if artifact_container is None and robot_name:
            artifact_container = self._robot_description_by_robot.get(robot_name)

        if artifact_container is None:
            return

        artifact = artifact_container.get("artifact")
        if artifact is None:
            return

        resolved_description_id = str(artifact.manifest.description_id)
        if description_id and description_id != resolved_description_id:
            return

        chunks = list(artifact.chunks or [])
        expected_chunks = len(chunks)
        now_ms = int(time.time() * 1000)

        begin = String()
        begin.data = json.dumps(
            {
                "request_id": request_id,
                "robot_name": robot_name,
                "description_id": resolved_description_id,
                "session_id": session_id,
                "app_id": app_id,
                "expected_chunks": expected_chunks,
                "encoding": artifact.manifest.encoding,
                "ts_unix_ms": now_ms,
            }
        )
        self.robot_description_chunk_begin_publisher.publish(begin)
        begin_delay = max(0.0, float(getattr(self, "_robot_description_chunk_begin_delay_s", 0.03)))
        if begin_delay > 0.0:
            time.sleep(begin_delay)

        item_delay = max(0.0, float(getattr(self, "_robot_description_chunk_item_delay_s", 0.0)))
        batch_size = max(1, int(getattr(self, "_robot_description_chunk_batch_size", 1)))
        batch_pause = max(0.0, float(getattr(self, "_robot_description_chunk_batch_pause_s", 0.0)))
        for index, chunk_data in enumerate(chunks):
            item = String()
            item.data = json.dumps(
                {
                    "request_id": request_id,
                    "robot_name": robot_name,
                    "description_id": resolved_description_id,
                    "chunk_index": index,
                    "total_chunks": expected_chunks,
                    "chunk_data": chunk_data,
                }
            )
            self.robot_description_chunk_item_publisher.publish(item)
            if item_delay > 0.0:
                time.sleep(item_delay)
            if batch_pause > 0.0 and (index + 1) < expected_chunks and ((index + 1) % batch_size) == 0:
                time.sleep(batch_pause)

        end_delay = max(0.0, float(getattr(self, "_robot_description_chunk_end_delay_s", 0.03)))
        if end_delay > 0.0:
            time.sleep(end_delay)
        end = String()
        end.data = json.dumps(
            {
                "request_id": request_id,
                "robot_name": robot_name,
                "description_id": resolved_description_id,
                "received_count": expected_chunks,
                "ts_unix_ms": int(time.time() * 1000),
            }
        )
        self.robot_description_chunk_end_publisher.publish(end)

    def _publish_sdk_registry_replay_once(self, entries, replay_request: Optional[Dict[str, Any]] = None) -> Tuple[bool, Dict[str, Any]]:
        if not self.ros_initialized or self.node is None:
            return False, {"error": "ROS2 not initialized"}
        if self.sdk_replay_begin_publisher is None or self.sdk_replay_item_publisher is None or self.sdk_replay_end_publisher is None:
            return False, {"error": "SDK replay publishers unavailable"}

        replay_request = dict(replay_request or {})
        request_id = str(replay_request.get("request_id") or uuid.uuid4().hex).strip() or uuid.uuid4().hex
        join_attempt_id = str(replay_request.get("join_attempt_id") or "").strip()
        requester_app_id = str(replay_request.get("requester_app_id") or replay_request.get("app_id") or "unknown").strip() or "unknown"
        now_ms = int(time.time() * 1000)

        payloads = []
        for _, _, msg in (entries or []):
            json_payload = str(getattr(msg, "data", "") or "")
            if json_payload:
                payloads.append(json_payload)

        expected_count = len(payloads)

        begin_msg = String()
        begin_msg.data = json.dumps(
            {
                "request_id": request_id,
                "join_attempt_id": join_attempt_id,
                "expected_count": expected_count,
                "ts_unix_ms": now_ms,
            }
        )
        self.sdk_replay_begin_publisher.publish(begin_msg)

        published_count = 0
        for idx, registration_json in enumerate(payloads):
            item_msg = String()
            item_msg.data = json.dumps(
                {
                    "request_id": request_id,
                    "join_attempt_id": join_attempt_id,
                    "index": idx,
                    "registration_json": registration_json,
                }
            )
            self.sdk_replay_item_publisher.publish(item_msg)
            published_count += 1

        end_msg = String()
        end_msg.data = json.dumps(
            {
                "request_id": request_id,
                "join_attempt_id": join_attempt_id,
                "expected_count": expected_count,
                "published_count": published_count,
                "ts_unix_ms": int(time.time() * 1000),
            }
        )
        self.sdk_replay_end_publisher.publish(end_msg)

        return True, {
            "request_id": request_id,
            "join_attempt_id": join_attempt_id,
            "requester_app_id": requester_app_id,
            "expected_count": expected_count,
            "published_count": published_count,
        }

    def _publish_sdk_registry_replay(self, entries, replay_request: Optional[Dict[str, Any]] = None) -> Tuple[bool, Dict[str, Any]]:
        if not self.ros_initialized or self.node is None:
            return False, {"error": "ROS2 not initialized"}
        if self.sdk_replay_begin_publisher is None or self.sdk_replay_item_publisher is None or self.sdk_replay_end_publisher is None:
            return False, {"error": "SDK replay publishers unavailable"}

        base_request = dict(replay_request or {})
        request_id = str(base_request.get("request_id") or uuid.uuid4().hex).strip() or uuid.uuid4().hex
        base_request["request_id"] = request_id

        # Replay settings are configurable via internal attributes. The Unity
        # coordinator also retries requests, so keep this path simple/synchronous.
        attempt_count = max(1, int(getattr(self, "_sdk_replay_burst_attempts", 1) or 1))
        initial_delay_s = max(0.0, float(getattr(self, "_sdk_replay_burst_initial_delay_s", 0.0) or 0.0))
        inter_attempt_delay_s = max(0.0, float(getattr(self, "_sdk_replay_burst_inter_attempt_delay_s", 0.0) or 0.0))

        if initial_delay_s > 0.0:
            time.sleep(initial_delay_s)

        per_attempt_published_count: List[int] = []
        last_result: Dict[str, Any] = {}

        for attempt_idx in range(attempt_count):
            ok, result = self._publish_sdk_registry_replay_once(entries, base_request)
            if not ok:
                return False, result
            last_result = dict(result or {})
            per_attempt_published_count.append(int(last_result.get("published_count") or 0))
            if attempt_idx < (attempt_count - 1) and inter_attempt_delay_s > 0.0:
                time.sleep(inter_attempt_delay_s)

        last_result["attempt_count"] = attempt_count
        last_result["per_attempt_published_count"] = per_attempt_published_count
        last_result["total_published_count"] = int(sum(per_attempt_published_count))
        return True, last_result

    def _prune_stale_app_presence(self, timeout_s: float):
        now = time.time()
        timeout = max(1.0, float(timeout_s))
        stale = [
            app_id
            for app_id, data in self._app_presence_by_id.items()
            if (now - float(data.get("ts", 0.0) or 0.0)) > timeout
        ]
        for app_id in stale:
            self._app_presence_by_id.pop(app_id, None)

    def _get_app_presence_summary(self, timeout_s: float = 5.0) -> Dict[str, Any]:
        self._prune_stale_app_presence(timeout_s)
        summary = {
            "total": 0,
            "host_count": 0,
            "joiner_count": 0,
            "private_count": 0,
            "single_count": 0,
            "host_created_count": 0,
            "joiner_joined_count": 0,
            "private_ready_count": 0,
            "alignment_pending_count": 0,
            "alignment_ready_count": 0,
            "workspace_ready_count": 0,
            "registry_synced_count": 0,
        }

        for _, data in self._app_presence_by_id.items():
            role = str(data.get("role") or "").strip().lower()
            state = str(data.get("state") or "").strip().lower()
            summary["total"] += 1
            if role == "host":
                summary["host_count"] += 1
                if state in {
                    "host_created",
                    "alignment_pending",
                    "alignment_ready",
                    "colocation_ready",
                    "workspace_ready_host",
                    "workspace_synced",
                    "registry_synced",
                    "workspace_ready",
                }:
                    summary["host_created_count"] += 1
            elif role == "joiner":
                summary["joiner_count"] += 1
                if state in {
                    "joiner_discovered_host",
                    "joiner_joined",
                    "alignment_pending",
                    "alignment_ready",
                    "colocation_ready",
                    "workspace_ready_joiner",
                    "workspace_synced",
                    "registry_synced",
                    "workspace_ready",
                }:
                    summary["joiner_joined_count"] += 1
            elif role == "private":
                summary["private_count"] += 1
                if state in {
                    "private_selected",
                    "private_workspace_ready",
                    "workspace_ready",
                    "workspace_synced",
                    "registry_synced",
                }:
                    summary["private_ready_count"] += 1
            elif role == "single":
                summary["single_count"] += 1

            if state in {"alignment_pending", "joiner_discovered_host"}:
                summary["alignment_pending_count"] += 1

            if state in {"alignment_ready", "colocation_ready", "workspace_ready_host", "workspace_ready_joiner", "workspace_synced", "registry_synced"}:
                summary["alignment_ready_count"] += 1

            if state.startswith("workspace_ready") or state == "workspace_synced":
                summary["workspace_ready_count"] += 1

            if state == "registry_synced":
                summary["registry_synced_count"] += 1

        return summary

    def _format_multi_operator_summary(self) -> str:
        summary = self._get_app_presence_summary(self._multi_operator_presence_ttl_s)

        if summary["total"] <= 0:
            return "No operator presence"

        host_count = int(summary.get("host_count", 0) or 0)
        joiner_count = int(summary.get("joiner_count", 0) or 0)
        private_count = int(summary.get("private_count", 0) or 0)
        alignment_pending = int(summary.get("alignment_pending_count", 0) or 0)
        alignment_ready = int(summary.get("alignment_ready_count", 0) or 0)
        workspace_ready = int(summary.get("workspace_ready_count", 0) or 0)
        registry_synced = int(summary.get("registry_synced_count", 0) or 0)

        if host_count <= 0 and joiner_count > 0:
            base = f"Joiners only ({joiner_count})"
            if private_count > 0:
                base += f" + {private_count} Private" + ("" if private_count == 1 else "s")
        elif host_count > 0:
            base = "Host Created" if int(summary.get("host_created_count", 0) or 0) > 0 else "Host Detected"
            if joiner_count > 0:
                base += f" + {joiner_count} Joiner" + ("" if joiner_count == 1 else "s")
            if private_count > 0:
                base += f" + {private_count} Private" + ("" if private_count == 1 else "s")
        elif private_count > 0:
            base = f"Private Workspace: {private_count}"
        else:
            base = f"Operators: {summary['total']}"

        if joiner_count > 0 and alignment_pending > 0 and alignment_ready <= 0:
            base += " (Aligning)"
        elif alignment_ready > 0:
            base += f" | Alignment Ready: {alignment_ready}"

        if workspace_ready > 0:
            base += f" | Workspace Ready: {workspace_ready}"

        if registry_synced > 0:
            base += f" | Registry Synced: {registry_synced}"

        return base

    def _format_app_link_status(self, is_connected: bool, seen_heartbeat: bool) -> str:
        summary = self._get_app_presence_summary(self._multi_operator_presence_ttl_s)

        if is_connected:
            details = []
            if summary["host_count"] > 0:
                if summary["host_created_count"] > 0:
                    details.append("Host Created")
                else:
                    details.append("Host Detected")
            if summary["joiner_count"] > 0:
                if summary["joiner_joined_count"] > 0:
                    details.append(f"Joiners Joined: {summary['joiner_joined_count']}")
                else:
                    details.append(f"Joiners: {summary['joiner_count']}")
            if summary["private_count"] > 0:
                if summary.get("private_ready_count", 0) > 0:
                    details.append(f"Private Ready: {summary['private_ready_count']}")
                else:
                    details.append(f"Private: {summary['private_count']}")
            if summary.get("alignment_ready_count", 0) > 0:
                details.append(f"Alignment Ready: {summary['alignment_ready_count']}")
            elif summary.get("alignment_pending_count", 0) > 0:
                details.append(f"Aligning: {summary['alignment_pending_count']}")
            if summary["workspace_ready_count"] > 0:
                details.append(f"Workspace Ready: {summary['workspace_ready_count']}")

            if details:
                return "Connected (" + " | ".join(details) + ")"
            return "Connected"

        if seen_heartbeat:
            if summary["joiner_count"] > 0:
                return "Disconnected (Waiting for Host App...)"
            return "Disconnected (Waiting for Horus App...)"
        return "Waiting for Horus App..."

    @staticmethod
    def _normalize_transport(value: Any) -> Optional[str]:
        normalized = str(value or "").strip().lower()
        if normalized in ("ros", "webrtc"):
            return normalized
        return None

    def _teleop_runtime_state_callback(self, msg):
        payload_raw = str(getattr(msg, "data", "") or "").strip()
        if not payload_raw:
            return

        try:
            payload = json.loads(payload_raw)
        except Exception:
            return

        camera_topic = str(payload.get("camera_topic", "") or "").strip()
        if not camera_topic:
            return

        self._teleop_runtime_by_topic[camera_topic] = {
            "active": bool(payload.get("active", False)),
            "mode": str(payload.get("mode", "") or "").strip().lower(),
            "transport": self._normalize_transport(payload.get("transport")),
            "ts": time.time(),
        }

    def _get_runtime_transport_override(self, camera_topic: str) -> Optional[str]:
        topic = str(camera_topic or "").strip()
        if not topic:
            return None

        state = self._teleop_runtime_by_topic.get(topic)
        if not state:
            return None

        now = time.time()
        ts = float(state.get("ts", 0.0) or 0.0)
        if (now - ts) > max(0.5, float(self._teleop_runtime_state_ttl_s)):
            self._teleop_runtime_by_topic.pop(topic, None)
            return None

        if not bool(state.get("active", False)):
            return None

        return self._normalize_transport(state.get("transport"))

    def _get_active_app_count(self, timeout_s: float) -> int:
        now = time.time()
        stale = [ip for ip, ts in self._app_heartbeats.items() if (now - ts) > timeout_s]
        for ip in stale:
            self._app_heartbeats.pop(ip, None)
            self._app_uptime_by_id.pop(ip, None)
            self._app_presence_by_id.pop(ip, None)
        self._prune_stale_app_presence(self._multi_operator_presence_ttl_s)
        stale_topics = [
            topic
            for topic, state in self._teleop_runtime_by_topic.items()
            if (now - float(state.get("ts", 0.0) or 0.0)) > max(0.5, float(self._teleop_runtime_state_ttl_s))
        ]
        for topic in stale_topics:
            self._teleop_runtime_by_topic.pop(topic, None)
        active_count = len(self._app_heartbeats)
        if active_count <= 0:
            self._teleop_runtime_by_topic.clear()
        return active_count

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

    def _resolve_dashboard_queued_status(self, queued_reasons: Dict[str, str]) -> str:
        if not isinstance(queued_reasons, dict) or not queued_reasons:
            return ""

        reasons = [str(reason or "").strip() for reason in queued_reasons.values() if str(reason or "").strip()]
        if reasons and all(reason == reasons[0] for reason in reasons):
            status = reasons[0]
        else:
            status = "Waiting for Workspace"

        if status == "Waiting for Workspace":
            try:
                if self.node.count_publishers("/tf") <= 0:
                    return "Waiting for TF"
            except Exception:
                pass

        return status

    def _collect_topics(self, dataviz):
        topics = []
        if dataviz is None:
            return topics
        try:
            for viz in dataviz.get_enabled_visualizations():
                topic = viz.data_source.topic
                if topic and topic not in topics:
                    topics.append(topic)
        except Exception:
            pass
        return topics

    def _collect_control_topics(self, config: Dict[str, Any]) -> list:
        topics = []
        if not isinstance(config, dict):
            return topics

        control = config.get("control")
        if not isinstance(control, dict):
            return topics

        def _append_topic(value: Any):
            topic = str(value or "").strip()
            if topic and topic not in topics:
                topics.append(topic)

        _append_topic(control.get("drive_topic"))

        teleop = control.get("teleop")
        if isinstance(teleop, dict):
            _append_topic(teleop.get("command_topic"))
            _append_topic(teleop.get("raw_input_topic"))
            _append_topic(teleop.get("head_pose_topic"))

        return topics

    def _collect_camera_topics(self, config: Dict[str, Any]) -> list:
        topics = []
        if not isinstance(config, dict):
            return topics

        sensors = config.get("sensors")
        if not isinstance(sensors, list):
            return topics

        def _append_topic(value: Any):
            topic = str(value or "").strip()
            if topic and topic not in topics:
                topics.append(topic)

        for sensor_cfg in sensors:
            if not isinstance(sensor_cfg, dict):
                continue
            if str(sensor_cfg.get("type", "")).strip().lower() != "camera":
                continue

            _append_topic(sensor_cfg.get("topic"))
            cam_cfg = sensor_cfg.get("camera_config") or {}
            if not isinstance(cam_cfg, dict):
                cam_cfg = {}
            _append_topic(cam_cfg.get("right_topic"))
            _append_topic(cam_cfg.get("minimap_topic"))
            _append_topic(cam_cfg.get("teleop_topic"))
            _append_topic(cam_cfg.get("teleop_right_topic"))

        return topics

    @staticmethod
    def _merge_topics(*topic_lists: list) -> list:
        merged = []
        for topic_list in topic_lists:
            for topic in topic_list or []:
                normalized = str(topic or "").strip()
                if normalized and normalized not in merged:
                    merged.append(normalized)
        return merged

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
        normalized_name = str(node_name).strip().lstrip("/")
        if not normalized_name:
            return False
        try:
            if self.node and normalized_name == str(self.node.get_name()).strip().lstrip("/"):
                return False
        except Exception:
            pass
        if normalized_name in ("horus_backend_node", "horus_backend", "horus_unity_bridge"):
            return True
        if normalized_name.startswith("horus_unity_bridge"):
            return True
        if "horus_backend" in normalized_name:
            return True
        if normalized_name.endswith("_RosSubscriber") or normalized_name.endswith("_RosPublisher"):
            return True
        return False

    def _is_local_node_name(self, node_name: Optional[str]) -> bool:
        if not node_name:
            return False
        try:
            return (
                self.node is not None
                and str(node_name).strip().lstrip("/")
                == str(self.node.get_name()).strip().lstrip("/")
            )
        except Exception:
            return False

    def _get_monitored_subscription_state(self, topic: str) -> Optional[bool]:
        try:
            from horus.utils.topic_status import get_topic_status_board

            snapshot = get_topic_status_board().snapshot()
            state = snapshot.get(topic)
            if state == "SUBSCRIBED":
                return True
            if state == "UNSUBSCRIBED":
                return False
        except Exception:
            pass
        return None

    def _resolve_data_topic_link(self, topic: str) -> bool:
        return self._has_backend_subscriber(topic)

    def _has_backend_subscriber(self, topic: str) -> bool:
        try:
            infos = self.node.get_subscriptions_info_by_topic(topic)
        except Exception:
            return False
        if any(self._is_backend_node_name(info.node_name) for info in infos):
            return True
        # Fallback: any non-local subscriber means the bridge side has a consumer.
        return any(not self._is_local_node_name(info.node_name) for info in infos)

    def _has_backend_publisher(self, topic: str) -> bool:
        try:
            infos = self.node.get_publishers_info_by_topic(topic)
        except Exception:
            return False
        if any(self._is_backend_node_name(info.node_name) for info in infos):
            return True
        # Fallback: any non-local publisher means topic source is present beyond SDK process.
        return any(not self._is_local_node_name(info.node_name) for info in infos)

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

    def _build_topic_roles(self, robot_topics, control_topics: Optional[list] = None):
        roles = {
            "/horus/registration": "sdk_pub",
            "/horus/registration_ack": "sdk_sub",
            "/horus/heartbeat": "sdk_sub",
            "/horus/multi_operator_presence": "sdk_sub",
            "/horus/multi_operator/sdk_registration_replay_request": "sdk_sub",
            "/horus/multi_operator/sdk_registry_replay_begin": "sdk_pub",
            "/horus/multi_operator/sdk_registry_replay_item": "sdk_pub",
            "/horus/multi_operator/sdk_registry_replay_end": "sdk_pub",
            "/horus/robot_description/request": "sdk_sub",
            "/horus/robot_description/chunk_begin": "sdk_pub",
            "/horus/robot_description/chunk_item": "sdk_pub",
            "/horus/robot_description/chunk_end": "sdk_pub",
        }
        control_topic_set = set(control_topics or [])
        for topic in robot_topics:
            if topic not in roles:
                roles[topic] = "backend_pub" if topic in control_topic_set else "backend_sub"
        return roles

    def _topic_group(self, topic: str, robot_names, topic_group_overrides: Optional[Dict[str, str]] = None):
        topic_group_overrides = topic_group_overrides or {}
        override = str(topic_group_overrides.get(topic, "") or "").strip()
        if override:
            return override
        if topic in (
            "/horus/registration",
            "/horus/registration_ack",
            "/horus/heartbeat",
            "/horus/multi_operator_presence",
            "/horus/multi_operator/sdk_registration_replay_request",
            "/horus/multi_operator/sdk_registry_replay_begin",
            "/horus/multi_operator/sdk_registry_replay_item",
            "/horus/multi_operator/sdk_registry_replay_end",
            "/horus/robot_description/request",
            "/horus/robot_description/chunk_begin",
            "/horus/robot_description/chunk_item",
            "/horus/robot_description/chunk_end",
        ):
            return "sdk"
        if topic == "/tf":
            return "shared"
        for name in robot_names:
            prefix = f"/{name}/"
            if topic.startswith(prefix):
                return name
        return "shared"

    def _build_topic_group_overrides(self, robot_name: str, topics: list) -> Dict[str, str]:
        overrides: Dict[str, str] = {}
        normalized_robot_name = str(robot_name or "").strip()
        if not normalized_robot_name:
            return overrides

        canonical_prefix = f"/{normalized_robot_name}/"
        for topic in topics or []:
            normalized_topic = str(topic or "").strip()
            if not normalized_topic:
                continue
            if normalized_topic.startswith(canonical_prefix):
                continue
            overrides[normalized_topic] = normalized_robot_name
        return overrides

    def _extract_camera_topic_profiles(self, config: Dict) -> Dict[str, Dict[str, str]]:
        profiles: Dict[str, Dict[str, str]] = {}

        def _normalize_transport(value: Any, fallback: str) -> str:
            normalized = str(value or "").strip().lower()
            return normalized if normalized in ("ros", "webrtc") else fallback

        def _normalize_mode(value: Any, fallback: str) -> str:
            normalized = str(value or "").strip().lower()
            return normalized if normalized in ("minimap", "teleop", "both") else fallback

        def _merge_profile(topic_name: str, source_mode: str, minimap_streaming: str, teleop_streaming: str, startup_mode: str):
            if not topic_name:
                return
            existing = profiles.get(topic_name)
            if existing is None:
                profiles[topic_name] = {
                    "minimap": minimap_streaming,
                    "teleop": teleop_streaming,
                    "startup": startup_mode,
                    "source_mode": source_mode,
                }
                return

            existing["minimap"] = _normalize_transport(existing.get("minimap"), minimap_streaming)
            existing["teleop"] = _normalize_transport(existing.get("teleop"), teleop_streaming)
            existing["startup"] = _normalize_mode(existing.get("startup"), startup_mode)
            merged_mode = existing.get("source_mode", source_mode)
            if merged_mode != source_mode:
                merged_mode = "both"
            existing["source_mode"] = _normalize_mode(merged_mode, "both")

        for sensor_cfg in config.get("sensors", []):
            if str(sensor_cfg.get("type", "")).strip().lower() != "camera":
                continue
            base_topic = str(sensor_cfg.get("topic", "")).strip()
            if not base_topic:
                continue

            cam_cfg = sensor_cfg.get("camera_config") or {}
            minimap_streaming = str(
                cam_cfg.get("minimap_streaming_type")
                or cam_cfg.get("streaming_type")
                or "ros"
            ).strip().lower()
            teleop_streaming = str(
                cam_cfg.get("teleop_streaming_type")
                or cam_cfg.get("streaming_type")
                or "webrtc"
            ).strip().lower()
            startup_mode = str(cam_cfg.get("startup_mode") or "minimap").strip().lower()

            if minimap_streaming not in ("ros", "webrtc"):
                minimap_streaming = "ros"
            if teleop_streaming not in ("ros", "webrtc"):
                teleop_streaming = "webrtc"
            if startup_mode not in ("minimap", "teleop"):
                startup_mode = "minimap"

            minimap_topic = str(cam_cfg.get("minimap_topic") or "").strip() or base_topic
            teleop_topic = str(cam_cfg.get("teleop_topic") or "").strip() or base_topic

            _merge_profile(minimap_topic, "minimap", minimap_streaming, teleop_streaming, startup_mode)
            _merge_profile(teleop_topic, "teleop", minimap_streaming, teleop_streaming, startup_mode)

        return profiles

    def _payload_coerce_bool(self, value: Any, default: bool) -> bool:
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

    def _payload_coerce_float(self, value: Any, default: float) -> float:
        if value is None:
            return float(default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def _payload_coerce_vec3(self, value: Any, default_xyz: Tuple[float, float, float]) -> Dict[str, float]:
        x_default, y_default, z_default = default_xyz
        if isinstance(value, dict):
            x = self._payload_coerce_float(value.get("x"), x_default)
            y = self._payload_coerce_float(value.get("y"), y_default)
            z = self._payload_coerce_float(value.get("z"), z_default)
            return {"x": x, "y": y, "z": z}
        if isinstance(value, (list, tuple)) and len(value) == 3:
            x = self._payload_coerce_float(value[0], x_default)
            y = self._payload_coerce_float(value[1], y_default)
            z = self._payload_coerce_float(value[2], z_default)
            return {"x": x, "y": y, "z": z}
        return {
            "x": float(x_default),
            "y": float(y_default),
            "z": float(z_default),
        }

    def _serialize_visualization_payload(
        self,
        visualization: Any,
        scope_override: Optional[str] = None,
    ) -> Optional[Dict[str, Any]]:
        if visualization is None:
            return None

        data_source = getattr(visualization, "data_source", None)
        if data_source is None:
            return None

        topic = str(getattr(data_source, "topic", "") or "").strip()
        viz_type = getattr(visualization, "viz_type", None)
        if not topic or viz_type is None:
            return None

        viz_type_value = str(getattr(viz_type, "value", viz_type)).strip()
        if not viz_type_value:
            return None

        frame_id = str(getattr(data_source, "frame_id", "") or "").strip()
        is_robot_specific = bool(getattr(visualization, "is_robot_specific", lambda: False)())
        scope = scope_override or ("robot" if is_robot_specific else "global")

        payload: Dict[str, Any] = {
            "type": viz_type_value,
            "topic": topic,
            "scope": scope,
            "enabled": bool(getattr(visualization, "enabled", True)),
        }
        if frame_id:
            payload["frame"] = frame_id

        render_options = getattr(visualization, "render_options", {}) or {}
        color = render_options.get("color")
        if color not in (None, ""):
            payload["color"] = str(color)

        if viz_type_value == "path":
            path_payload: Dict[str, Any] = {}
            source_type = getattr(data_source, "source_type", None)
            source_type_value = str(getattr(source_type, "value", source_type) or "").strip()
            role = None
            if source_type_value == "robot_global_path":
                role = "global"
            elif source_type_value == "robot_local_path":
                role = "local"
            elif source_type_value == "robot_trajectory":
                role = "trajectory"

            if role:
                path_payload["role"] = role

            if "line_width" in render_options:
                path_payload["line_width"] = self._payload_coerce_float(
                    render_options.get("line_width"),
                    1.0,
                )

            line_style = render_options.get("line_style")
            if line_style not in (None, ""):
                path_payload["line_style"] = str(line_style)

            if path_payload:
                payload["path"] = path_payload

        if viz_type_value == "velocity_data":
            velocity_payload: Dict[str, Any] = {}
            units = render_options.get("units")
            if units not in (None, ""):
                velocity_payload["units"] = str(units)

            if "text_back_offset_m" in render_options:
                velocity_payload["text_back_offset_m"] = self._payload_coerce_float(
                    render_options.get("text_back_offset_m"),
                    0.36,
                )
            if "floor_offset_m" in render_options:
                velocity_payload["floor_offset_m"] = self._payload_coerce_float(
                    render_options.get("floor_offset_m"),
                    0.01,
                )
            if "update_hz" in render_options:
                velocity_payload["update_hz"] = self._payload_coerce_float(
                    render_options.get("update_hz"),
                    10.0,
                )

            if velocity_payload:
                payload["velocity"] = velocity_payload

        if viz_type_value == "odometry_trail":
            trail_payload: Dict[str, Any] = {}
            if "max_points" in render_options:
                trail_payload["max_points"] = int(
                    round(
                        self._payload_coerce_float(
                            render_options.get("max_points"),
                            48.0,
                        )
                    )
                )
            if "history_seconds" in render_options:
                trail_payload["history_seconds"] = self._payload_coerce_float(
                    render_options.get("history_seconds"),
                    3.2,
                )
            if "min_spacing_m" in render_options:
                trail_payload["min_spacing_m"] = self._payload_coerce_float(
                    render_options.get("min_spacing_m"),
                    0.07,
                )
            if "line_width_m" in render_options:
                trail_payload["line_width_m"] = self._payload_coerce_float(
                    render_options.get("line_width_m"),
                    0.0048,
                )
            if "trail_back_offset_m" in render_options:
                trail_payload["trail_back_offset_m"] = self._payload_coerce_float(
                    render_options.get("trail_back_offset_m"),
                    0.44,
                )

            if trail_payload:
                payload["trail"] = trail_payload

        if viz_type_value == "collision_risk":
            collision_payload: Dict[str, Any] = {}
            if "threshold_m" in render_options:
                collision_payload["threshold_m"] = self._payload_coerce_float(
                    render_options.get("threshold_m"),
                    1.2,
                )
            if "radius_m" in render_options:
                collision_payload["radius_m"] = self._payload_coerce_float(
                    render_options.get("radius_m"),
                    1.2,
                )
            source = render_options.get("source")
            if source not in (None, ""):
                collision_payload["source"] = str(source)
            if "alpha_min" in render_options:
                collision_payload["alpha_min"] = self._payload_coerce_float(
                    render_options.get("alpha_min"),
                    0.0,
                )
            if "alpha_max" in render_options:
                collision_payload["alpha_max"] = self._payload_coerce_float(
                    render_options.get("alpha_max"),
                    0.55,
                )

            if collision_payload:
                payload["collision"] = collision_payload

        if viz_type_value == "occupancy_grid":
            occupancy_payload: Dict[str, Any] = {}
            if "show_unknown_space" in render_options:
                occupancy_payload["show_unknown_space"] = self._payload_coerce_bool(
                    render_options.get("show_unknown_space"),
                    True,
                )
            if "position_scale" in render_options:
                occupancy_payload["position_scale"] = self._payload_coerce_float(
                    render_options.get("position_scale"),
                    1.0,
                )
            if "position_offset" in render_options:
                occupancy_payload["position_offset"] = self._payload_coerce_vec3(
                    render_options.get("position_offset"),
                    (0.0, 0.0, 0.0),
                )
            if "rotation_offset_euler" in render_options:
                occupancy_payload["rotation_offset_euler"] = self._payload_coerce_vec3(
                    render_options.get("rotation_offset_euler"),
                    (0.0, 0.0, 0.0),
                )
            if occupancy_payload:
                payload["occupancy"] = occupancy_payload

        if viz_type_value == "point_cloud":
            point_cloud_payload: Dict[str, Any] = {}
            point_cloud_payload["point_size"] = max(
                0.001,
                self._payload_coerce_float(render_options.get("point_size"), 0.05),
            )

            max_points_raw = int(
                self._payload_coerce_float(render_options.get("max_points_per_frame"), 0.0)
            )
            point_cloud_payload["max_points_per_frame"] = 0 if max_points_raw <= 0 else max(1, max_points_raw)

            point_cloud_payload["base_sample_stride"] = max(
                1,
                int(self._payload_coerce_float(render_options.get("base_sample_stride"), 1.0)),
            )
            point_cloud_payload["min_update_interval"] = max(
                0.0,
                self._payload_coerce_float(render_options.get("min_update_interval"), 0.0),
            )
            point_cloud_payload["enable_adaptive_quality"] = self._payload_coerce_bool(
                render_options.get("enable_adaptive_quality"),
                False,
            )
            point_cloud_payload["target_framerate"] = max(
                30.0,
                self._payload_coerce_float(render_options.get("target_framerate"), 72.0),
            )
            point_cloud_payload["min_quality_multiplier"] = min(
                1.0,
                max(
                    0.25,
                    self._payload_coerce_float(render_options.get("min_quality_multiplier"), 0.6),
                ),
            )
            point_cloud_payload["min_distance"] = max(
                0.0,
                self._payload_coerce_float(render_options.get("min_distance"), 0.0),
            )
            point_cloud_payload["max_distance"] = max(
                0.0,
                self._payload_coerce_float(render_options.get("max_distance"), 0.0),
            )
            point_cloud_payload["replace_latest"] = self._payload_coerce_bool(
                render_options.get("replace_latest"),
                True,
            )
            point_cloud_payload["render_all_points"] = self._payload_coerce_bool(
                render_options.get("render_all_points"),
                True,
            )
            point_cloud_payload["auto_point_size_by_workspace_scale"] = self._payload_coerce_bool(
                render_options.get("auto_point_size_by_workspace_scale"),
                True,
            )
            min_point_size = max(
                0.0001,
                self._payload_coerce_float(render_options.get("min_point_size"), 0.002),
            )
            max_point_size = max(
                min_point_size,
                self._payload_coerce_float(render_options.get("max_point_size"), 0.04),
            )
            point_cloud_payload["min_point_size"] = min_point_size
            point_cloud_payload["max_point_size"] = max_point_size

            render_mode = str(render_options.get("render_mode", "opaque_fast")).strip().lower()
            if render_mode not in {"opaque_fast", "transparent_hq"}:
                render_mode = "opaque_fast"
            point_cloud_payload["render_mode"] = render_mode
            point_cloud_payload["enable_view_frustum_culling"] = self._payload_coerce_bool(
                render_options.get("enable_view_frustum_culling"),
                True,
            )
            point_cloud_payload["frustum_padding"] = min(
                0.5,
                max(
                    0.0,
                    self._payload_coerce_float(render_options.get("frustum_padding"), 0.03),
                ),
            )
            point_cloud_payload["enable_subpixel_culling"] = self._payload_coerce_bool(
                render_options.get("enable_subpixel_culling"),
                True,
            )
            point_cloud_payload["min_screen_radius_px"] = max(
                0.0,
                self._payload_coerce_float(render_options.get("min_screen_radius_px"), 0.8),
            )
            max_visible_points_budget = max(
                1000,
                int(self._payload_coerce_float(render_options.get("max_visible_points_budget"), 200000.0)),
            )
            visible_points_budget = max(
                1000,
                int(self._payload_coerce_float(render_options.get("visible_points_budget"), 120000.0)),
            )
            point_cloud_payload["max_visible_points_budget"] = max_visible_points_budget
            point_cloud_payload["visible_points_budget"] = min(visible_points_budget, max_visible_points_budget)
            point_cloud_payload["map_static_mode"] = self._payload_coerce_bool(
                render_options.get("map_static_mode"),
                True,
            )

            payload["point_cloud"] = point_cloud_payload

        if viz_type_value == "mesh":
            mesh_payload: Dict[str, Any] = {}
            mesh_payload["use_vertex_colors"] = self._payload_coerce_bool(
                render_options.get("use_vertex_colors"),
                True,
            )
            mesh_payload["alpha"] = min(
                1.0,
                max(
                    0.0,
                    self._payload_coerce_float(render_options.get("alpha"), 1.0),
                ),
            )
            mesh_payload["double_sided"] = self._payload_coerce_bool(
                render_options.get("double_sided"),
                True,
            )
            mesh_payload["max_triangles"] = max(
                1000,
                int(self._payload_coerce_float(render_options.get("max_triangles"), 200000.0)),
            )
            source_coordinate_space = str(
                render_options.get("source_coordinate_space", "enu")
            ).strip().lower()
            if source_coordinate_space not in {"enu", "optical"}:
                source_coordinate_space = "enu"
            mesh_payload["source_coordinate_space"] = source_coordinate_space

            payload["mesh"] = mesh_payload

        if viz_type_value == "octomap":
            octomap_payload: Dict[str, Any] = {}
            render_mode = str(
                render_options.get("render_mode", "surface_mesh")
            ).strip().lower()
            if render_mode not in {"surface_mesh", "voxel_cubes"}:
                render_mode = "surface_mesh"
            octomap_payload["render_mode"] = render_mode
            octomap_payload["use_vertex_colors"] = self._payload_coerce_bool(
                render_options.get("use_vertex_colors"),
                True,
            )
            octomap_payload["alpha"] = min(
                1.0,
                max(
                    0.0,
                    self._payload_coerce_float(render_options.get("alpha"), 1.0),
                ),
            )
            octomap_payload["double_sided"] = self._payload_coerce_bool(
                render_options.get("double_sided"),
                False,
            )
            octomap_payload["max_triangles"] = max(
                1000,
                int(self._payload_coerce_float(render_options.get("max_triangles"), 60000.0)),
            )
            source_coordinate_space = str(
                render_options.get("source_coordinate_space", "enu")
            ).strip().lower()
            if source_coordinate_space not in {"enu", "optical"}:
                source_coordinate_space = "enu"
            octomap_payload["source_coordinate_space"] = source_coordinate_space
            native_topic = str(
                render_options.get("native_topic", "/map_3d_octomap")
            ).strip()
            if native_topic:
                octomap_payload["native_topic"] = native_topic
            native_frame = str(
                render_options.get("native_frame", frame_id or "map")
            ).strip()
            if native_frame:
                octomap_payload["native_frame"] = native_frame
            octomap_payload["native_binary_only"] = self._payload_coerce_bool(
                render_options.get("native_binary_only"),
                True,
            )
            payload["octomap"] = octomap_payload

        return payload

    def _build_global_visualizations_payload(self, datavizs: list) -> list:
        deduped: Dict[Tuple[str, str, str], Dict[str, Any]] = {}
        ordered_keys = []

        for dataviz in datavizs:
            if dataviz is None:
                continue
            try:
                visualizations = dataviz.get_enabled_visualizations()
            except Exception:
                visualizations = []

            for visualization in visualizations:
                if visualization is None:
                    continue
                try:
                    if visualization.is_robot_specific():
                        continue
                except Exception:
                    continue

                payload = self._serialize_visualization_payload(
                    visualization,
                    scope_override="global",
                )
                if not payload:
                    continue

                key = (
                    str(payload.get("type", "")),
                    str(payload.get("topic", "")),
                    str(payload.get("frame", "")),
                )
                if key in deduped:
                    continue

                deduped[key] = payload
                ordered_keys.append(key)

        return [deduped[key] for key in ordered_keys]

    def _build_dashboard_topic_rows(
        self,
        monitored_topics,
        topic_roles,
        robot_names,
        is_connected: bool,
        registration_done: bool,
        camera_topic_profiles: Optional[Dict[str, Dict[str, str]]] = None,
        topic_group_overrides: Optional[Dict[str, str]] = None,
    ) -> list:
        camera_topic_profiles = camera_topic_profiles or {}
        topic_group_overrides = topic_group_overrides or {}
        core_topic_set = {
            "/horus/registration",
            "/horus/registration_ack",
            "/horus/heartbeat",
            "/horus/multi_operator_presence",
            "/horus/multi_operator/sdk_registration_replay_request",
            "/horus/multi_operator/sdk_registry_replay_begin",
            "/horus/multi_operator/sdk_registry_replay_item",
            "/horus/multi_operator/sdk_registry_replay_end",
            "/horus/robot_description/request",
            "/horus/robot_description/chunk_begin",
            "/horus/robot_description/chunk_item",
            "/horus/robot_description/chunk_end",
        }
        rows = []

        for topic in monitored_topics:
            role = topic_roles.get(topic, "backend_sub")
            topic_kind = "core" if topic in core_topic_set else "data"
            camera_profile = dict(camera_topic_profiles.get(topic, {}) or {})
            runtime_transport = self._get_runtime_transport_override(topic)
            if runtime_transport:
                camera_profile["active_transport"] = runtime_transport

            camera_active_transport = self._normalize_transport(camera_profile.get("active_transport"))
            if not camera_active_transport and camera_profile:
                startup_mode = str(camera_profile.get("startup", "minimap")).strip().lower()
                minimap_transport = self._normalize_transport(camera_profile.get("minimap")) or "ros"
                teleop_transport = self._normalize_transport(camera_profile.get("teleop")) or "webrtc"
                camera_active_transport = teleop_transport if startup_mode == "teleop" else minimap_transport

            if role == "sdk_pub":
                role_label = "sdk->bridge"
                backend_sub = self._has_backend_subscriber(topic)
                local_pub = self._has_local_publisher(topic)
                pubs = 1 if local_pub else 0
                subs = 1 if backend_sub else 0
                if not is_connected:
                    link = "NO"
                    data = "IDLE"
                else:
                    link = "OK" if backend_sub else "NO"
                    data = "ACTIVE" if local_pub and backend_sub else "IDLE"
            elif role == "sdk_sub":
                role_label = "bridge->sdk"
                backend_pub = self._has_backend_publisher(topic)
                local_sub = self._has_local_subscriber(topic)
                pubs = 1 if backend_pub else 0
                subs = 1 if local_sub else 0
                if not is_connected:
                    link = "NO"
                    data = "IDLE"
                else:
                    link = "OK" if backend_pub else "NO"
                    data = "ACTIVE" if backend_pub else "IDLE"
            elif role == "backend_pub":
                backend_pub = self._has_backend_publisher(topic)
                _, subs = self._get_topic_counts(topic)
                pubs = 1 if backend_pub else 0
                role_label = "bridge->robot" if topic.endswith("/cmd_vel") else "bridge->topic"
                if not is_connected or not registration_done:
                    link = "NO"
                    data = "IDLE"
                else:
                    link = "OK" if backend_pub else "NO"
                    if not backend_pub:
                        data = "IDLE"
                    elif subs > 0:
                        data = "ACTIVE"
                    else:
                        data = "STALE"
            else:
                role_label = "data->bridge"
                pubs, _ = self._get_topic_counts(topic)
                if not is_connected or not registration_done:
                    link = "NO"
                    subs = 0
                    data = "IDLE"
                else:
                    if camera_active_transport == "webrtc":
                        source_available = pubs > 0
                        link_ok = source_available and is_connected
                        link = "OK" if link_ok else "NO"
                        subs = 1 if link_ok else 0
                        if not source_available:
                            data = "STALE"
                        elif link_ok:
                            data = "ACTIVE"
                        else:
                            data = "IDLE"
                    else:
                        link_ok = self._resolve_data_topic_link(topic)
                        link = "OK" if link_ok else "NO"
                        subs = 1 if link_ok else 0
                        if not link_ok:
                            data = "IDLE"
                        elif pubs > 0:
                            data = "ACTIVE"
                        else:
                            data = "STALE"

            rows.append(
                {
                    "robot": self._topic_group(topic, robot_names, topic_group_overrides=topic_group_overrides),
                    "topic": topic,
                    "role": role_label,
                    "topic_kind": topic_kind,
                    "link": link,
                    "data": data,
                    "camera_profile": camera_profile,
                    "pubs": pubs,
                    "subs": subs,
                }
            )

        rows.sort(
            key=lambda row: (
                0 if row.get("topic_kind") == "core" else 1,
                row.get("robot") or "",
                row.get("topic") or "",
            )
        )
        return rows

    def register_robot(
        self,
        robot,
        dataviz,
        timeout_sec: float = 10.0,
        keep_alive: bool = False,
        show_dashboard: bool = True,
        workspace_scale: Optional[float] = None,
        wait_for_app_before_register: bool = True,
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
            wait_for_app_before_register: When False, seed registration before the app connects.
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
            cli.print_info("Building robot registration payload...")
            global_visualizations = self._build_global_visualizations_payload([dataviz])
            config = self._build_robot_config_dict(
                robot,
                dataviz,
                global_visualizations=global_visualizations,
                workspace_scale=workspace_scale,
            )
            cli.print_info(f"Registration payload ready for '{robot.name}'.")
            config_json = json.dumps(config)
            camera_topic_profiles = self._extract_camera_topic_profiles(config)

            # 1. Bridge Detection (Infrastructure must be external)
            bridge_running = self._ensure_bridge_running()
            if not bridge_running:
                return False, {"error": "Bridge Start Failed"}
            cli.print_info("Bridge ready. Entering registration phase.")

            # Display Connect Info using Dashboard
            local_ip = self._get_local_ip()
            bridge_state = "Active" if bridge_running else "Error"
            
            data_topics = self._collect_topics(dataviz)
            control_topics = self._collect_control_topics(config)
            camera_topics = self._collect_camera_topics(config)
            robot_topics = self._merge_topics(data_topics, control_topics, camera_topics)
            topic_roles = self._build_topic_roles(robot_topics, control_topics=control_topics)
            topic_group_overrides = self._build_topic_group_overrides(robot.name, robot_topics)
            core_topics = [
                "/horus/registration",
                "/horus/registration_ack",
                "/horus/heartbeat",
                "/horus/multi_operator_presence",
                "/horus/multi_operator/sdk_registration_replay_request",
                "/horus/multi_operator/sdk_registry_replay_begin",
                "/horus/multi_operator/sdk_registry_replay_item",
                "/horus/multi_operator/sdk_registry_replay_end",
                "/horus/robot_description/request",
                "/horus/robot_description/chunk_begin",
                "/horus/robot_description/chunk_item",
                "/horus/robot_description/chunk_end",
            ]
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
            seeded_before_app = False

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
                dashboard.update_multi_operator(self._format_multi_operator_summary())
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

                    dashboard.update_app_link(self._format_app_link_status(is_connected, seen_heartbeat))
                    dashboard.update_multi_operator(self._format_multi_operator_summary())
                    
                    # Update Topic Stats (every ~1s)
                    if (time.time() - last_stats_update) >= 1.0:
                        rows = self._build_dashboard_topic_rows(
                            monitored_topics=monitored_topics,
                            topic_roles=topic_roles,
                            robot_names=robot_names,
                            is_connected=is_connected,
                            registration_done=registration_success,
                            camera_topic_profiles=camera_topic_profiles,
                            topic_group_overrides=topic_group_overrides,
                        )
                        dashboard.update_topics(rows)
                        last_stats_update = time.time()

                    # --- Registration Phase ---
                    if not registration_success:
                        if not is_connected:
                            if not wait_for_app_before_register and not seeded_before_app:
                                self.publisher.publish(msg)
                                last_register_publish = time.time()
                                seeded_before_app = True
                                dashboard.update_registration("Published")
                                dashboard.update_status("Registration seeded; waiting for Horus App...")
                                dashboard.tick()
                                continue
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
                                config = self._build_robot_config_dict(
                                    robot,
                                    dataviz,
                                    workspace_scale=workspace_scale,
                                )
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
        workspace_scale: Optional[float] = None,
        wait_for_app_before_register: bool = True,
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

            cli.print_info("Building robot registration payloads...")
            entries = []
            entry_by_name = {}
            core_topics = [
                "/horus/registration",
                "/horus/registration_ack",
                "/horus/heartbeat",
                "/horus/multi_operator_presence",
                "/horus/multi_operator/sdk_registration_replay_request",
                "/horus/multi_operator/sdk_registry_replay_begin",
                "/horus/multi_operator/sdk_registry_replay_item",
                "/horus/multi_operator/sdk_registry_replay_end",
                "/horus/robot_description/request",
                "/horus/robot_description/chunk_begin",
                "/horus/robot_description/chunk_item",
                "/horus/robot_description/chunk_end",
            ]
            robot_topics = []
            control_topics = []
            camera_topic_profiles = {}
            topic_group_overrides = {}
            global_visualizations = self._build_global_visualizations_payload(datavizs)
            for robot, dataviz in zip(robots, datavizs):
                config = self._build_robot_config_dict(
                    robot,
                    dataviz,
                    global_visualizations=global_visualizations,
                    workspace_scale=workspace_scale,
                )
                msg = String()
                msg.data = json.dumps(config)
                entries.append((robot, dataviz, msg))
                entry_by_name[robot.name] = (robot, dataviz, msg)
                camera_topic_profiles.update(self._extract_camera_topic_profiles(config))
                entry_data_topics = self._collect_topics(dataviz)
                entry_control_topics = self._collect_control_topics(config)
                entry_camera_topics = self._collect_camera_topics(config)
                entry_topics = self._merge_topics(
                    entry_data_topics,
                    entry_control_topics,
                    entry_camera_topics,
                )
                for topic, group in self._build_topic_group_overrides(robot.name, entry_topics).items():
                    topic_group_overrides[topic] = group
                robot_topics = self._merge_topics(robot_topics, entry_data_topics)
                robot_topics = self._merge_topics(robot_topics, entry_camera_topics)
                control_topics = self._merge_topics(control_topics, entry_control_topics)
            cli.print_info(f"Built {len(entries)} robot registration payload(s).")

            bridge_running = self._ensure_bridge_running()
            if not bridge_running:
                return False, {"error": "Bridge Start Failed"}
            cli.print_info("Bridge ready. Entering registration phase.")

            local_ip = self._get_local_ip()
            bridge_state = "Active" if bridge_running else "Error"

            robot_topics = self._merge_topics(robot_topics, control_topics)
            topic_roles = self._build_topic_roles(robot_topics, control_topics=control_topics)
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
                if dashboard is not None:
                    dashboard.update_status("Seeding registrations...")
                for robot, dataviz, msg in entries:
                    state = robot_states.get(robot.name, "pending")
                    if not force and state == "registered":
                        continue
                    if dashboard is not None:
                        dashboard.update_status(f"Registering {robot.name}...")
                    ok, ack = publish_and_wait(robot, dataviz, msg)
                    if not ok:
                        return False, ack
                    handle_ack(ack)
                return True, {"success": True}

            def seed_all_registrations() -> None:
                cli.print_info(f"Seeding registrations for {len(entries)} robot(s)...")
                for _, _, msg in entries:
                    self.publisher.publish(msg)

            def update_dashboard_topics(dashboard, is_connected: bool, registration_done: bool):
                if dashboard is None:
                    return
                rows = self._build_dashboard_topic_rows(
                    monitored_topics=monitored_topics,
                    topic_roles=topic_roles,
                    robot_names=robot_names,
                    is_connected=is_connected,
                    registration_done=registration_done,
                    camera_topic_profiles=camera_topic_profiles,
                    topic_group_overrides=topic_group_overrides,
                )
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
                    replay_request = self._consume_registration_replay_request()
                    if is_connected and (not last_connection_state or app_restarted):
                        for robot, dataviz, msg in entries:
                            publish_and_wait(robot, dataviz, msg)
                    elif is_connected and replay_request:
                        # Re-publish registrations to /horus/registration directly.
                        # The joiner now also processes this topic, so this uses the
                        # same proven channel as host registration instead of relying
                        # on the separate replay topic assembly mechanism.
                        for _, _, msg in entries:
                            self.publisher.publish(msg)
                        self._publish_sdk_registry_replay(entries, replay_request)
                    last_connection_state = is_connected
                    time.sleep(0.2)

            show_counts = bool(os.getenv("HORUS_DASHBOARD_SHOW_COUNTS"))
            with cli.ConnectionDashboard(local_ip, 10000, bridge_state, show_counts=show_counts) as dashboard:
                dashboard.update_app_link("Waiting for Horus App...")
                dashboard.update_multi_operator(self._format_multi_operator_summary())
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
                seeded_before_app = False

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
                    replay_request = self._consume_registration_replay_request()
                    if app_restarted:
                        had_connection_drop = True

                    dashboard.update_app_link(self._format_app_link_status(is_connected, seen_heartbeat))
                    dashboard.update_multi_operator(self._format_multi_operator_summary())

                    if replay_request and is_connected:
                        dashboard.update_registration("Re-registering")
                        dashboard.update_status("Replaying robot registrations for joiner...")
                        # Re-publish to /horus/registration directly so the joiner
                        # receives registrations through the same channel as the host.
                        for _, _, reg_msg in entries:
                            self.publisher.publish(reg_msg)
                        ok, result = self._publish_sdk_registry_replay(entries, replay_request)
                        if not ok:
                            dashboard.update_registration("Failed")
                            dashboard.update_status((result or {}).get("error") or "Registration replay failed")
                        else:
                            dashboard.update_registration("Registered")
                            published = int((result or {}).get("published_count") or 0)
                            attempts = int((result or {}).get("attempt_count") or 1)
                            dashboard.update_status(f"Replay complete ({published} robots, {attempts} attempts)")

                    if not is_connected:
                        if not wait_for_app_before_register and not seeded_before_app:
                            seed_all_registrations()
                            seeded_before_app = True
                            last_register_attempt = time.time()
                            dashboard.update_registration("Published")
                            dashboard.update_status("Registrations seeded; waiting for Horus App...")
                            if (time.time() - last_topics_update) >= 1.0:
                                update_dashboard_topics(dashboard, is_connected, registration_done)
                                last_topics_update = time.time()
                            last_connection_state = is_connected
                            dashboard.tick()
                            continue
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
                        queued_status = self._resolve_dashboard_queued_status(queued_reasons)

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
                        if any_queued:
                            dashboard.update_registration("Queued")
                            dashboard.update_status(queued_status or "Waiting for Workspace")
                            retry_interval = 2.0
                        else:
                            retry_interval = 1.0

                        if time.time() - last_register_attempt < retry_interval:
                            dashboard.tick()
                            continue
                        last_register_attempt = time.time()

                        if not any_queued:
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
                            queued_status = self._resolve_dashboard_queued_status(queued_reasons)

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
                                queued_status = self._resolve_dashboard_queued_status(queued_reasons)

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
        self._remove_robot_description_cache(str(robot_id or ""))
        # For unregistration, we can send a config with "action": "unregister"
        # Or just empty config with name.
        # Implementation simplified for now.
        return True, {"message": "Unregistered (Local cleanup only)"}

    def _resolve_robot_description_artifact(self, robot) -> Optional[Dict[str, Any]]:
        if robot is None:
            return None

        from horus.utils import cli

        metadata = getattr(robot, "metadata", None)
        description_config = metadata.get("robot_description_config") if isinstance(metadata, dict) else None
        has_description_config = isinstance(description_config, dict) and bool(description_config.get("enabled", True))

        if not hasattr(self, "_robot_description_resolver") or self._robot_description_resolver is None:
            self._robot_description_resolver = RobotDescriptionResolver()
        if not hasattr(self, "_robot_description_by_robot") or self._robot_description_by_robot is None:
            self._robot_description_by_robot = {}
        if not hasattr(self, "_robot_description_by_id") or self._robot_description_by_id is None:
            self._robot_description_by_id = {}

        robot_name = str(getattr(robot, "name", "") or "").strip()
        if robot_name and robot_name in self._robot_description_by_robot:
            cached_container = self._robot_description_by_robot.get(robot_name)
            if isinstance(cached_container, dict) and cached_container.get("artifact") is not None:
                cached_artifact = cached_container.get("artifact")
                cli.print_info(
                    f"Using cached robot-description artifact for '{robot_name}' "
                    f"(groups={int(getattr(cached_artifact.manifest, 'mesh_asset_count', 0) or 0)}, "
                    f"bytes={int(getattr(cached_artifact.manifest, 'mesh_asset_encoded_bytes', 0) or 0)}, "
                    f"chunks={int(len(getattr(cached_artifact, 'chunks', []) or []))})."
                )
                return cached_container

        artifact = self._robot_description_resolver.resolve_for_robot(robot)
        if artifact is None:
            reason = ""
            try:
                reason = str(self._robot_description_resolver.last_error or "").strip()
            except Exception:
                reason = ""
            if not has_description_config and not reason:
                return None
            robot_name = str(getattr(robot, "name", "") or "").strip() or "unknown"
            if reason:
                cli.print_info(
                    f"Robot description unavailable for '{robot_name}': {reason}"
                )
            else:
                cli.print_info(
                    f"Robot description unavailable for '{robot_name}': unresolved source."
                )
            return None

        cache_status = str(getattr(self._robot_description_resolver, "last_resolution_cache_status", "miss") or "miss")
        if robot_name:
            artifact_groups = int(getattr(artifact.manifest, "mesh_asset_count", 0) or 0)
            artifact_bytes = int(getattr(artifact.manifest, "mesh_asset_encoded_bytes", 0) or 0)
            artifact_chunks = int(len(getattr(artifact, "chunks", []) or []))
            if cache_status == "hit":
                cli.print_info(
                    f"Using cached robot-description artifact for '{robot_name}' "
                    f"(groups={artifact_groups}, bytes={artifact_bytes}, chunks={artifact_chunks})."
                )
            else:
                cli.print_info(
                    f"Built robot-description artifact for '{robot_name}' "
                    f"(groups={artifact_groups}, bytes={artifact_bytes}, chunks={artifact_chunks})."
                )

        container = {
            "robot_name": robot_name,
            "artifact": artifact,
        }
        if robot_name:
            self._robot_description_by_robot[robot_name] = container
        self._robot_description_by_id[artifact.manifest.description_id] = container
        return container

    def _remove_robot_description_cache(self, robot_name: str) -> None:
        normalized = str(robot_name or "").strip()
        if not normalized:
            return
        cache_by_robot = getattr(self, "_robot_description_by_robot", None)
        cache_by_id = getattr(self, "_robot_description_by_id", None)
        if not isinstance(cache_by_robot, dict) or not isinstance(cache_by_id, dict):
            return

        container = cache_by_robot.pop(normalized, None)
        if container is None:
            return
        artifact = container.get("artifact")
        if artifact is None:
            return
        description_id = str(artifact.manifest.description_id or "").strip()
        if description_id:
            existing = cache_by_id.get(description_id)
            if existing is container:
                cache_by_id.pop(description_id, None)

    def _build_robot_config_dict(
        self,
        robot,
        dataviz,
        global_visualizations: Optional[list] = None,
        workspace_scale: Optional[float] = None,
    ) -> Dict:
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

        def _coerce_int(value, default):
            if value is None:
                return int(default)
            try:
                return int(value)
            except (TypeError, ValueError):
                return int(default)

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

        def _coerce_text(value, default):
            if value is None:
                return str(default)
            text = str(value).strip()
            return text if text else str(default)

        def _build_camera_config(sensor):
            metadata = sensor.metadata or {}
            def _normalize_transport(value, fallback):
                normalized = str(value).strip().lower() if value is not None else ""
                if normalized in ("ros", "webrtc"):
                    return normalized
                return fallback

            def _normalize_image_type(value, fallback):
                normalized = str(value).strip().lower() if value is not None else ""
                if normalized in ("raw", "compressed"):
                    return normalized
                return fallback

            def _normalize_layout(value, fallback):
                normalized = str(value).strip().lower() if value is not None else ""
                if normalized in ("sbs", "side_by_side", "side-by-side"):
                    return "side_by_side"
                if normalized in ("dual_topic", "dual", "two_topics", "two_topic"):
                    return "dual_topic"
                if normalized in ("mono", ""):
                    return "mono" if normalized == "mono" else fallback
                return fallback

            legacy_streaming_type = _normalize_transport(
                metadata.get("streaming_type"),
                "",
            )
            if not legacy_streaming_type:
                legacy_streaming_type = _normalize_transport(
                    getattr(sensor, "streaming_type", ""),
                    "ros",
                )
            if not legacy_streaming_type:
                legacy_streaming_type = "ros"

            minimap_streaming_type = _normalize_transport(
                metadata.get("minimap_streaming_type"),
                "",
            )
            if not minimap_streaming_type:
                minimap_streaming_type = _normalize_transport(
                    getattr(sensor, "minimap_streaming_type", ""),
                    "",
                )
            if not minimap_streaming_type:
                minimap_streaming_type = _normalize_transport(legacy_streaming_type, "ros")

            teleop_streaming_type = _normalize_transport(
                metadata.get("teleop_streaming_type"),
                "",
            )
            if not teleop_streaming_type:
                teleop_streaming_type = _normalize_transport(
                    getattr(sensor, "teleop_streaming_type", ""),
                    "",
                )
            if not teleop_streaming_type:
                teleop_streaming_type = _normalize_transport(legacy_streaming_type, "webrtc")

            startup_mode = str(
                metadata.get("startup_mode", getattr(sensor, "startup_mode", "minimap"))
            ).strip().lower()
            if startup_mode not in ("minimap", "teleop"):
                startup_mode = "minimap"

            is_stereo = bool(getattr(sensor, "is_stereo", False))
            raw_stereo_layout = str(
                metadata.get("stereo_layout", getattr(sensor, "stereo_layout", ""))
            ).strip().lower()
            if is_stereo:
                if raw_stereo_layout in ("dual_topic", "dual", "two_topics", "two_topic"):
                    stereo_layout = "dual_topic"
                else:
                    stereo_layout = "side_by_side"
            else:
                stereo_layout = "mono"

            right_topic = str(
                metadata.get("right_topic", getattr(sensor, "right_topic", ""))
            ).strip()

            minimap_topic = str(
                metadata.get("minimap_topic", getattr(sensor, "minimap_topic", ""))
            ).strip()
            if not minimap_topic:
                minimap_topic = str(getattr(sensor, "topic", "")).strip()

            teleop_topic = str(
                metadata.get("teleop_topic", getattr(sensor, "teleop_topic", ""))
            ).strip()
            if not teleop_topic:
                teleop_topic = minimap_topic

            image_type = _normalize_image_type(metadata.get("image_type"), "raw")
            minimap_image_type = _normalize_image_type(
                metadata.get("minimap_image_type", getattr(sensor, "minimap_image_type", "")),
                image_type,
            )
            teleop_image_type = _normalize_image_type(
                metadata.get("teleop_image_type", getattr(sensor, "teleop_image_type", "")),
                image_type,
            )

            minimap_max_fps = _coerce_int(
                metadata.get("minimap_max_fps", getattr(sensor, "minimap_max_fps", 30)),
                30,
            )
            minimap_max_fps = max(1, min(30, minimap_max_fps))

            teleop_stereo_layout = _normalize_layout(
                metadata.get("teleop_stereo_layout", getattr(sensor, "teleop_stereo_layout", "")),
                stereo_layout,
            )
            teleop_right_topic = str(
                metadata.get("teleop_right_topic", getattr(sensor, "teleop_right_topic", ""))
            ).strip()
            if not teleop_right_topic:
                teleop_right_topic = right_topic
            if not is_stereo:
                teleop_stereo_layout = "mono"
                teleop_right_topic = ""

            return {
                "streaming_type": legacy_streaming_type,
                "minimap_streaming_type": minimap_streaming_type,
                "teleop_streaming_type": teleop_streaming_type,
                "minimap_topic": minimap_topic,
                "teleop_topic": teleop_topic,
                "minimap_image_type": minimap_image_type,
                "teleop_image_type": teleop_image_type,
                "minimap_max_fps": minimap_max_fps,
                "teleop_stereo_layout": teleop_stereo_layout,
                "teleop_right_topic": teleop_right_topic,
                "startup_mode": startup_mode,
                "is_stereo": is_stereo,
                "stereo_layout": stereo_layout,
                "right_topic": right_topic,
                "image_type": image_type,
                "display_mode": str(metadata.get("display_mode", "projected")).lower(),
                "use_tf": _coerce_bool(metadata.get("use_tf"), True),
                "projection_target_frame": str(metadata.get("projection_target_frame", "")),
                "webrtc_client_signal_topic": str(
                    metadata.get("webrtc_client_signal_topic", "/horus/webrtc/client_signal")
                ),
                "webrtc_server_signal_topic": str(
                    metadata.get("webrtc_server_signal_topic", "/horus/webrtc/server_signal")
                ),
                "webrtc_bitrate_kbps": _coerce_int(
                    metadata.get("webrtc_bitrate_kbps"),
                    2000,
                ),
                "webrtc_framerate": _coerce_int(
                    metadata.get("webrtc_framerate"),
                    max(30, min(90, _coerce_int(getattr(sensor, "fps", None), 30))),
                ),
                "webrtc_stun_server_url": str(
                    metadata.get("webrtc_stun_server_url", "stun:stun.l.google.com:19302")
                ),
                "webrtc_turn_server_url": str(
                    metadata.get("webrtc_turn_server_url", "")
                ),
                "webrtc_turn_username": str(
                    metadata.get("webrtc_turn_username", "")
                ),
                "webrtc_turn_credential": str(
                    metadata.get("webrtc_turn_credential", "")
                ),
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

        def _build_sensor_viz_override_index():
            overrides = {}
            robot_name_key = str(getattr(robot, "name", "") or "").strip()

            for visualization in getattr(dataviz, "visualizations", []) or []:
                data_source = getattr(visualization, "data_source", None)
                if data_source is None:
                    continue

                render_options = getattr(visualization, "render_options", {}) or {}
                if not isinstance(render_options, dict) or not render_options:
                    continue

                source_robot_name = str(
                    getattr(data_source, "robot_name", "") or robot_name_key
                ).strip()
                if source_robot_name != robot_name_key:
                    continue

                override = {}
                color_override = render_options.get("color")
                if color_override not in (None, ""):
                    override["color"] = str(color_override)

                point_size_override = render_options.get("point_size")
                if point_size_override not in (None, ""):
                    override["point_size"] = _coerce_float(point_size_override, 0.05)

                if not override:
                    continue

                topic_key = str(getattr(data_source, "topic", "") or "").strip()
                name_key = str(getattr(data_source, "name", "") or "").strip()
                if topic_key:
                    overrides[(source_robot_name, "topic", topic_key)] = override
                if name_key:
                    overrides.setdefault((source_robot_name, "name", name_key), override)

            return overrides

        sensor_viz_override_index = _build_sensor_viz_override_index()

        def _build_sensor_viz_config(sensor):
            viz_config = {
                "color": getattr(sensor, "color", "white"),
                "point_size": getattr(sensor, "point_size", 0.05),
            }

            robot_name_key = str(getattr(robot, "name", "") or "").strip()
            topic_key = str(getattr(sensor, "topic", "") or "").strip()
            name_key = str(getattr(sensor, "name", "") or "").strip()

            override = None
            if topic_key:
                override = sensor_viz_override_index.get((robot_name_key, "topic", topic_key))
            if override is None and name_key:
                override = sensor_viz_override_index.get((robot_name_key, "name", name_key))

            if isinstance(override, dict):
                color_override = override.get("color")
                if color_override not in (None, ""):
                    viz_config["color"] = str(color_override)

                if "point_size" in override:
                    viz_config["point_size"] = _coerce_float(
                        override.get("point_size"),
                        viz_config["point_size"],
                    )

            return viz_config

        sensor_payloads = []
        for sensor in robot.sensors:
            payload = {
                "name": sensor.name,
                "type": sensor.sensor_type.value,
                "topic": sensor.topic,
                "frame": sensor.frame_id,
                "metadata": sensor.metadata or {},
                "viz_config": _build_sensor_viz_config(sensor),
            }
            if sensor.sensor_type.value == "camera":
                payload["camera_config"] = _build_camera_config(sensor)
            sensor_payloads.append(payload)

        def _build_robot_manager_config():
            robot_metadata = getattr(robot, "metadata", {}) or {}
            manager_metadata = robot_metadata.get("robot_manager_config")
            if not isinstance(manager_metadata, dict):
                manager_metadata = {}

            sections_metadata = manager_metadata.get("sections")
            if not isinstance(sections_metadata, dict):
                sections_metadata = {}

            return {
                "enabled": _coerce_bool(manager_metadata.get("enabled"), True),
                # Editor fallback path keeps setup friction low while the runtime path
                # can be provided explicitly when using Resources/Addressables.
                "prefab_asset_path": str(
                    manager_metadata.get("prefab_asset_path", "Assets/Prefabs/UI/RobotManager.prefab")
                ),
                "prefab_resource_path": str(manager_metadata.get("prefab_resource_path", "")),
                "sections": {
                    "status": _coerce_bool(sections_metadata.get("status"), True),
                    "data_viz": _coerce_bool(sections_metadata.get("data_viz"), True),
                    "teleop": _coerce_bool(sections_metadata.get("teleop"), True),
                    "tasks": _coerce_bool(sections_metadata.get("tasks"), True),
                },
            }

        def _build_teleop_control():
            robot_metadata = getattr(robot, "metadata", {}) or {}
            teleop_metadata = robot_metadata.get("teleop_config")
            if not isinstance(teleop_metadata, dict):
                teleop_metadata = {}

            allowed_profiles = {"wheeled", "legged", "aerial", "custom"}
            default_profile = str(getattr(robot, "get_type_str", lambda: "wheeled")()).strip().lower()
            if default_profile not in allowed_profiles:
                default_profile = "wheeled"

            allowed_response_modes = {"analog", "discrete"}
            allowed_deadman_policies = {
                "either_index_trigger",
                "left_index_trigger",
                "right_index_trigger",
                "either_grip_trigger",
            }

            command_topic = _coerce_text(
                teleop_metadata.get("command_topic"),
                robot.resolve_topic("cmd_vel"),
            )
            raw_input_topic = _coerce_text(
                teleop_metadata.get("raw_input_topic"),
                f"/horus/teleop/{robot.name}/joy",
            )
            head_pose_topic = _coerce_text(
                teleop_metadata.get("head_pose_topic"),
                f"/horus/teleop/{robot.name}/head_pose",
            )

            response_mode = _coerce_text(teleop_metadata.get("response_mode"), "analog").lower()
            if response_mode not in allowed_response_modes:
                response_mode = "analog"

            robot_profile = _coerce_text(teleop_metadata.get("robot_profile"), default_profile).lower()
            if robot_profile not in allowed_profiles:
                robot_profile = default_profile

            publish_rate_hz = _coerce_float(teleop_metadata.get("publish_rate_hz"), 30.0)
            publish_rate_hz = max(5.0, min(120.0, publish_rate_hz))

            deadman_metadata = teleop_metadata.get("deadman")
            if not isinstance(deadman_metadata, dict):
                deadman_metadata = {}

            deadman_policy = _coerce_text(
                deadman_metadata.get("policy"),
                "either_grip_trigger",
            ).lower()
            if deadman_policy not in allowed_deadman_policies:
                deadman_policy = "either_grip_trigger"

            deadman_timeout_ms = _coerce_int(deadman_metadata.get("timeout_ms"), 200)
            deadman_timeout_ms = max(50, min(2000, deadman_timeout_ms))

            axes_metadata = teleop_metadata.get("axes")
            if not isinstance(axes_metadata, dict):
                axes_metadata = {}

            deadzone = _coerce_float(axes_metadata.get("deadzone"), 0.15)
            deadzone = max(0.0, min(0.5, deadzone))
            expo = _coerce_float(axes_metadata.get("expo"), 1.7)
            expo = max(1.0, min(3.0, expo))
            linear_xy_max_mps = _coerce_float(axes_metadata.get("linear_xy_max_mps"), 1.0)
            linear_xy_max_mps = max(0.0, min(5.0, linear_xy_max_mps))
            linear_z_max_mps = _coerce_float(axes_metadata.get("linear_z_max_mps"), 0.8)
            linear_z_max_mps = max(0.0, min(5.0, linear_z_max_mps))
            angular_z_max_rps = _coerce_float(axes_metadata.get("angular_z_max_rps"), 1.2)
            angular_z_max_rps = max(0.0, min(6.0, angular_z_max_rps))

            discrete_metadata = teleop_metadata.get("discrete")
            if not isinstance(discrete_metadata, dict):
                discrete_metadata = {}

            discrete_threshold = _coerce_float(discrete_metadata.get("threshold"), 0.6)
            discrete_threshold = max(0.1, min(1.0, discrete_threshold))
            linear_xy_step_mps = _coerce_float(discrete_metadata.get("linear_xy_step_mps"), 0.6)
            linear_xy_step_mps = max(0.0, min(5.0, linear_xy_step_mps))
            linear_z_step_mps = _coerce_float(discrete_metadata.get("linear_z_step_mps"), 0.4)
            linear_z_step_mps = max(0.0, min(5.0, linear_z_step_mps))
            angular_z_step_rps = _coerce_float(discrete_metadata.get("angular_z_step_rps"), 0.9)
            angular_z_step_rps = max(0.0, min(6.0, angular_z_step_rps))

            return {
                "enabled": _coerce_bool(teleop_metadata.get("enabled"), True),
                "command_topic": command_topic,
                "raw_input_topic": raw_input_topic,
                "head_pose_topic": head_pose_topic,
                "robot_profile": robot_profile,
                "response_mode": response_mode,
                "publish_rate_hz": publish_rate_hz,
                "custom_passthrough_only": _coerce_bool(
                    teleop_metadata.get("custom_passthrough_only"),
                    False,
                ),
                "deadman": {
                    "policy": deadman_policy,
                    "timeout_ms": deadman_timeout_ms,
                },
                "axes": {
                    "deadzone": deadzone,
                    "expo": expo,
                    "linear_xy_max_mps": linear_xy_max_mps,
                    "linear_z_max_mps": linear_z_max_mps,
                    "angular_z_max_rps": angular_z_max_rps,
                },
                "discrete": {
                    "threshold": discrete_threshold,
                    "linear_xy_step_mps": linear_xy_step_mps,
                    "linear_z_step_mps": linear_z_step_mps,
                    "angular_z_step_rps": angular_z_step_rps,
                },
            }

        def _build_task_control():
            robot_metadata = getattr(robot, "metadata", {}) or {}
            task_metadata = robot_metadata.get("task_config")
            if not isinstance(task_metadata, dict):
                task_metadata = {}

            go_to_point_metadata = task_metadata.get("go_to_point")
            if not isinstance(go_to_point_metadata, dict):
                go_to_point_metadata = {}

            goal_topic = _coerce_text(
                go_to_point_metadata.get("goal_topic"),
                robot.resolve_topic("goal_pose"),
            )
            cancel_topic = _coerce_text(
                go_to_point_metadata.get("cancel_topic"),
                robot.resolve_topic("goal_cancel"),
            )
            status_topic = _coerce_text(
                go_to_point_metadata.get("status_topic"),
                robot.resolve_topic("goal_status"),
            )
            frame_id = _coerce_text(go_to_point_metadata.get("frame_id"), "map")
            if not frame_id:
                frame_id = "map"

            position_tolerance_m = _coerce_float(go_to_point_metadata.get("position_tolerance_m"), 0.20)
            position_tolerance_m = max(0.01, min(10.0, position_tolerance_m))
            yaw_tolerance_deg = _coerce_float(go_to_point_metadata.get("yaw_tolerance_deg"), 12.0)
            yaw_tolerance_deg = max(0.1, min(180.0, yaw_tolerance_deg))
            min_altitude_m = _coerce_float(go_to_point_metadata.get("min_altitude_m"), 0.0)
            min_altitude_m = max(0.0, min(100.0, min_altitude_m))
            max_altitude_m = _coerce_float(go_to_point_metadata.get("max_altitude_m"), 10.0)
            max_altitude_m = max(min_altitude_m + 0.1, min(100.0, max_altitude_m))

            waypoint_metadata = task_metadata.get("waypoint")
            if not isinstance(waypoint_metadata, dict):
                waypoint_metadata = {}

            waypoint_path_topic = _coerce_text(
                waypoint_metadata.get("path_topic"),
                robot.resolve_topic("waypoint_path"),
            )
            waypoint_status_topic = _coerce_text(
                waypoint_metadata.get("status_topic"),
                robot.resolve_topic("waypoint_status"),
            )
            waypoint_frame_id = _coerce_text(waypoint_metadata.get("frame_id"), "map")
            if not waypoint_frame_id:
                waypoint_frame_id = "map"

            waypoint_position_tolerance_m = _coerce_float(waypoint_metadata.get("position_tolerance_m"), 0.20)
            waypoint_position_tolerance_m = max(0.01, min(10.0, waypoint_position_tolerance_m))
            waypoint_yaw_tolerance_deg = _coerce_float(waypoint_metadata.get("yaw_tolerance_deg"), 12.0)
            waypoint_yaw_tolerance_deg = max(0.1, min(180.0, waypoint_yaw_tolerance_deg))

            return {
                "go_to_point": {
                    "enabled": _coerce_bool(go_to_point_metadata.get("enabled"), True),
                    "goal_topic": goal_topic,
                    "cancel_topic": cancel_topic,
                    "status_topic": status_topic,
                    "frame_id": frame_id,
                    "position_tolerance_m": position_tolerance_m,
                    "yaw_tolerance_deg": yaw_tolerance_deg,
                    "min_altitude_m": min_altitude_m,
                    "max_altitude_m": max_altitude_m,
                },
                "waypoint": {
                    "enabled": _coerce_bool(waypoint_metadata.get("enabled"), True),
                    "path_topic": waypoint_path_topic,
                    "status_topic": waypoint_status_topic,
                    "frame_id": waypoint_frame_id,
                    "position_tolerance_m": waypoint_position_tolerance_m,
                    "yaw_tolerance_deg": waypoint_yaw_tolerance_deg,
                },
            }

        robot_visualizations = []
        fallback_global_visualizations = []
        for visualization in dataviz.visualizations:
            payload = self._serialize_visualization_payload(visualization)
            if not payload:
                continue

            if payload.get("scope") == "global":
                fallback_global_visualizations.append(payload)
                continue

            robot_visualizations.append(payload)

        if global_visualizations is None:
            global_visualizations = []
            seen_global = set()
            for payload in fallback_global_visualizations:
                key = (
                    str(payload.get("type", "")),
                    str(payload.get("topic", "")),
                    str(payload.get("frame", "")),
                )
                if key in seen_global:
                    continue
                seen_global.add(key)
                global_visualizations.append(payload)

        drive_topic = robot.resolve_topic("cmd_vel")
        teleop_control = _build_teleop_control()
        task_control = _build_task_control()
        if isinstance(teleop_control, dict):
            command_override = str(teleop_control.get("command_topic", "")).strip()
            if command_override:
                drive_topic = command_override

        description_container = self._resolve_robot_description_artifact(robot)

        config = {
            "action": "register",
            "robot_name": robot.name,
            "robot_type": robot.get_type_str(),
            "ros_binding": robot.get_ros_binding(),
            "sensors": sensor_payloads,
            "visualizations": robot_visualizations,
            "global_visualizations": global_visualizations,
            "control": {
                "drive_topic": drive_topic,
                "teleop": teleop_control,
                "tasks": task_control,
            },
            "robot_manager_config": _build_robot_manager_config(),
            "timestamp": time.time()
        }

        local_body_config = robot.get_metadata("local_body_model_config", {})
        if isinstance(local_body_config, dict):
            robot_model_id = str(local_body_config.get("robot_model_id", "") or "").strip().lower()
            local_body_enabled = bool(local_body_config.get("enabled", False))
            if local_body_enabled and robot_model_id:
                config["robot_model_id"] = robot_model_id
                config["has_visual_mesh_model"] = True

        if description_container is not None:
            artifact = description_container.get("artifact")
            if artifact is not None:
                config["robot_description_manifest"] = artifact.to_manifest_payload()
                if bool(getattr(artifact.manifest, "supports_visual_meshes", False)):
                    config["has_visual_mesh_model"] = True
                # Keep inline payloads only when they stay reasonably small.
                # Mesh-backed descriptions use the existing chunk transport path.
                if len(str(artifact.payload_json or "")) <= 250000:
                    config["robot_description_payload_json"] = artifact.payload_json

        if dimensions is not None:
            config["dimensions"] = dimensions

        normalized_workspace_scale = None
        if workspace_scale is not None:
            try:
                candidate = float(workspace_scale)
            except (TypeError, ValueError):
                candidate = None

            if candidate is not None and math.isfinite(candidate) and candidate > 0.0:
                normalized_workspace_scale = candidate

        if normalized_workspace_scale is not None:
            config["workspace_config"] = {
                "position_scale": normalized_workspace_scale,
            }

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
