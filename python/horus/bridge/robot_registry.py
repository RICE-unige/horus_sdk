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
import logging
import shutil
import shlex
import tempfile
from typing import Any, Dict, Tuple, Optional, List

from horus.utils.network import resolve_advertise_ip
from horus.utils.dashboard_status import is_bridge_node, topic_graph

logger = logging.getLogger(__name__)
MODEL_TRACE_TAG = "HORUS_MODEL_TRACE"

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
        self._robot_description_chunk_publishers: Dict[str, Any] = {}
        self._robot_description_chunk_qos = None
        self.subscriber = None
        self.ros_initialized = False
        self._registration_lock = threading.Lock()
        self._ack_received = threading.Event()
        self._last_ack_data = {}
        self._ack_by_robot = {}
        self._dashboard_receipts = {}
        self.bridge_process = None
        self._bridge_log_file = None
        self._bridge_log_path = None
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
        self._last_spin_once_error_log_time = 0.0
        self._sdk_replay_burst_attempts = 3
        self._sdk_replay_burst_initial_delay_s = 0.15
        self._sdk_replay_burst_inter_attempt_delay_s = 0.25
        self._robot_description_chunk_begin_delay_s = 0.03
        self._robot_description_chunk_item_delay_s = 0.006
        self._robot_description_chunk_batch_size = 6
        self._robot_description_chunk_batch_pause_s = 0.02
        self._robot_description_chunk_end_delay_s = 0.05
        from horus.description.robot_description_resolver import RobotDescriptionResolver

        self._robot_description_resolver = RobotDescriptionResolver()
        self._robot_description_by_robot: Dict[str, Dict[str, Any]] = {}
        self._robot_description_by_id: Dict[str, Dict[str, Any]] = {}

        if ROS2_AVAILABLE:
            self._initialize_ros2()

    def _trace_robot_description(self, message: str) -> None:
        line = f"{MODEL_TRACE_TAG} {message}"
        try:
            from horus.utils import cli
            if cli.env_enabled("HORUS_SDK_DEBUG"):
                cli.print_info(line)
            else:
                logger.debug(line)
        except Exception:
            logger.debug(line)

    def _get_robot_description_chunk_publisher(self, topic: str):
        """Return a std_msgs/String publisher for a robot-description chunk reply topic."""
        resolved_topic = str(topic or "").strip()
        if not resolved_topic.startswith("/"):
            return None

        legacy_publishers = {
            "/horus/robot_description/chunk_begin": self.robot_description_chunk_begin_publisher,
            "/horus/robot_description/chunk_item": self.robot_description_chunk_item_publisher,
            "/horus/robot_description/chunk_end": self.robot_description_chunk_end_publisher,
        }
        publisher = legacy_publishers.get(resolved_topic)
        if publisher is not None:
            return publisher

        if self.node is None:
            return None

        publishers = getattr(self, "_robot_description_chunk_publishers", None)
        if publishers is None:
            publishers = {}
            self._robot_description_chunk_publishers = publishers

        publisher = publishers.get(resolved_topic)
        if publisher is not None:
            return publisher

        qos = self._robot_description_chunk_qos
        if qos is None:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=2048,
            )
            self._robot_description_chunk_qos = qos

        publisher = self.node.create_publisher(String, resolved_topic, qos)
        publishers[resolved_topic] = publisher
        self._trace_robot_description(f"[SDK chunks] dynamic_publisher_ready topic='{resolved_topic}'")
        return publisher

    def _get_local_ip(self):
        """Get local IP address of this machine"""
        return resolve_advertise_ip()

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
                logger.debug("Graceful Horus bridge shutdown failed; forcing process group kill", exc_info=True)
                # Force kill if needed
                try:
                    os.killpg(os.getpgid(self.bridge_process.pid), signal.SIGKILL)
                except Exception:
                    logger.debug("Forced Horus bridge process group kill failed", exc_info=True)
            self.bridge_process = None

        if self._bridge_log_file:
            try:
                self._bridge_log_file.close()
            except Exception:
                logger.debug("Failed to close Horus bridge log file", exc_info=True)
            self._bridge_log_file = None

    def _cleanup_ros(self):
        """Clean up ROS node and context"""
        if self.ros_initialized:
            try:
                if self.node:
                    self.node.destroy_node()
                rclpy.shutdown()
            except Exception:
                logger.debug("Failed to clean up RobotRegistryClient ROS context", exc_info=True)
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
            robot_description_chunk_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=2048,
            )
            self._robot_description_chunk_qos = robot_description_chunk_qos

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
                String, "/horus/robot_description/chunk_begin", robot_description_chunk_qos
            )
            self.robot_description_chunk_item_publisher = self.node.create_publisher(
                String, "/horus/robot_description/chunk_item", robot_description_chunk_qos
            )
            self.robot_description_chunk_end_publisher = self.node.create_publisher(
                String, "/horus/robot_description/chunk_end", robot_description_chunk_qos
            )
            self._robot_description_chunk_publishers = {
                "/horus/robot_description/chunk_begin": self.robot_description_chunk_begin_publisher,
                "/horus/robot_description/chunk_item": self.robot_description_chunk_item_publisher,
                "/horus/robot_description/chunk_end": self.robot_description_chunk_end_publisher,
            }

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
            self._trace_robot_description(
                "[SDK init] robot_description_transport_ready request_sub='/horus/robot_description/request' "
                "chunk_begin_pub='/horus/robot_description/chunk_begin' "
                "chunk_item_pub='/horus/robot_description/chunk_item' "
                "chunk_end_pub='/horus/robot_description/chunk_end'"
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

    def _is_likely_horus_bridge_listener(self, port: int) -> Optional[bool]:
        """Best-effort process identity check for localhost bridge listeners."""
        if shutil.which("lsof") is None:
            return None

        pids = self._listener_pids_for_port(port)
        if pids is None or not pids:
            return None

        inspected_any = False
        for pid in pids:
            cmdline = self._read_process_cmdline(pid)
            if not cmdline:
                continue
            inspected_any = True
            if self._cmdline_looks_like_horus_bridge(cmdline):
                return True

        return False if inspected_any else None

    def _listener_pids_for_port(self, port: int) -> Optional[List[int]]:
        """Return listener pids for a TCP port, or None when ownership cannot be inspected."""
        try:
            result = subprocess.run(
                ["lsof", "-t", f"-iTCP:{port}", "-sTCP:LISTEN"],
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except Exception:
            logger.debug("Failed to inspect listener pids for port %s", port, exc_info=True)
            return None

        if result.returncode != 0:
            return None

        pids: List[int] = []
        for line in (result.stdout or "").splitlines():
            value = line.strip()
            if value.isdigit():
                pids.append(int(value))
        return pids

    def _read_process_cmdline(self, pid: int) -> str:
        try:
            with open(f"/proc/{int(pid)}/cmdline", "rb") as handle:
                raw = handle.read()
            return raw.replace(b"\x00", b" ").decode("utf-8", errors="replace")
        except Exception:
            logger.debug("Failed to read process command line for pid %s", pid, exc_info=True)
            return ""

    @staticmethod
    def _cmdline_looks_like_horus_bridge(cmdline: str) -> bool:
        output = str(cmdline or "").lower()
        if not output.strip():
            return False

        bridge_markers = (
            "horus_unity_bridge",
            "unity_bridge",
            "horus_backend",
            "horus_complete_backend.launch.py",
            "horus_backend.launch.py",
            "horus-start",
        )
        return any(marker in output for marker in bridge_markers)

    def _is_horus_bridge_port_open(self, port: int = 10000) -> bool:
        """Check whether the local Unity bridge port is open and plausibly HORUS-owned."""
        if not self._is_port_open(port):
            return False

        identity = self._is_likely_horus_bridge_listener(port)
        if identity is False:
            from horus.utils import cli
            cli.print_error(
                f"Port {port} is open, but the listener does not look like Horus Bridge."
            )
            return False

        return True

    def _open_bridge_log_file(self):
        """Open a persistent log file for bridge auto-start diagnostics."""
        if self._bridge_log_file and not self._bridge_log_file.closed:
            return self._bridge_log_file

        log_file = tempfile.NamedTemporaryFile(
            mode="w+",
            prefix="horus-bridge-",
            suffix=".log",
            delete=False,
        )
        self._bridge_log_file = log_file
        self._bridge_log_path = log_file.name
        return log_file

    def _print_bridge_log_tail(self, max_lines: int = 40) -> None:
        """Print the tail of the bridge startup log after an auto-start failure."""
        from horus.utils import cli

        log_path = getattr(self, "_bridge_log_path", None)
        if not log_path or not os.path.isfile(log_path):
            return

        try:
            if self._bridge_log_file and not self._bridge_log_file.closed:
                self._bridge_log_file.flush()
            with open(log_path, "r", encoding="utf-8", errors="replace") as handle:
                lines = handle.readlines()
        except Exception:
            logger.debug("Failed to read bridge startup log tail from %s", log_path, exc_info=True)
            return

        tail = [line.rstrip() for line in lines[-max_lines:] if line.rstrip()]
        if not tail:
            return

        cli.print_info(f"Bridge startup log: {log_path}")
        for line in tail:
            cli.print_info(f"  {line}")

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
            logger.debug(
                "Invalid HORUS_SDK_BRIDGE_STARTUP_SETTLE_MS value: %r",
                raw_value,
                exc_info=True,
            )
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
            logger.debug("Failed to resolve ROS 2 package prefix for %s", package_name, exc_info=True)
            return None

        if check_pkg.returncode != 0:
            logger.debug(
                "ROS 2 package prefix lookup failed for %s: %s",
                package_name,
                str(check_pkg.stderr or "").strip(),
            )
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
            logger.debug(
                "Failed to resolve ROS 2 package prefix for %s via helper %s",
                package_name,
                helper_path,
                exc_info=True,
            )
            return None

        if check_pkg.returncode != 0:
            logger.debug(
                "Helper ROS 2 package prefix lookup failed for %s via %s: %s",
                package_name,
                helper_path,
                str(check_pkg.stderr or "").strip(),
            )
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

    def _bridge_priority_scheduling_launch_arg(self) -> Optional[str]:
        """Return a ros2 launch argument when bridge priority scheduling is explicitly configured."""
        raw_value = os.getenv("HORUS_BRIDGE_ENABLE_PRIORITY_SCHEDULING")
        if raw_value is None:
            return None

        value = raw_value.strip().lower()
        if value in ("1", "true", "yes", "on"):
            return "enable_priority_scheduling:=true"
        if value in ("0", "false", "no", "off"):
            return "enable_priority_scheduling:=false"
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
                bridge_log = self._open_bridge_log_file()
                self.bridge_process = subprocess.Popen(
                    [helper_path],
                    stdout=bridge_log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except Exception as exc:
                cli.print_warning(f"Helper auto-start failed for {helper_path}: {exc}")
                continue

            atexit.register(self._cleanup_bridge)
            with cli.status("Launching Horus Bridge...", spinner="dots"):
                for _ in range(20):
                    if self._is_horus_bridge_port_open(10000):
                        self._sleep_after_bridge_startup_settle()
                        cli.print_success("Horus Bridge launched successfully.")
                        return True
                    if self.bridge_process and self.bridge_process.poll() is not None:
                        break
                    time.sleep(1.0)

            self._print_bridge_log_tail()
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

            bridge_log = self._open_bridge_log_file()
            launch_command = ["ros2", "launch", "horus_unity_bridge", "unity_bridge.launch.py"]
            override = os.environ.get("HORUS_BRIDGE_CONFIG", "").strip()
            if override:
                config_path = os.path.abspath(os.path.expanduser(override))
                if not os.path.isfile(config_path):
                    cli.print_error(f"HORUS_BRIDGE_CONFIG does not exist: {config_path}")
                    return False
                launch_command.append(f"config_file:={config_path}")
            priority_scheduling_arg = self._bridge_priority_scheduling_launch_arg()
            if priority_scheduling_arg:
                launch_command.append(priority_scheduling_arg)
                cli.print_info(f"Bridge launch option: {priority_scheduling_arg}")

            self.bridge_process = subprocess.Popen(
                launch_command,
                stdout=bridge_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            atexit.register(self._cleanup_bridge)

            with cli.status("Launching Horus Bridge...", spinner="dots"):
                for _ in range(15):
                    if self._is_horus_bridge_port_open(10000):
                        self._sleep_after_bridge_startup_settle()
                        cli.print_success("Horus Bridge launched successfully.")
                        return True
                    if self.bridge_process and self.bridge_process.poll() is not None:
                        break
                    time.sleep(1.0)
        except Exception:
            logger.debug("Failed to auto-start Horus Bridge with ros2 launch", exc_info=True)
            return False

        self._print_bridge_log_tail()
        self._cleanup_bridge()
        return False

    def _ensure_bridge_running(self) -> bool:
        """Ensure bridge port is available, auto-starting bridge if needed."""
        from horus.utils import cli

        if self._is_horus_bridge_port_open(10000):
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
            if not isinstance(data, dict) or not isinstance(data.get("success"), bool):
                return
            if not isinstance(data.get("robot_name"), str) or not data["robot_name"]:
                return
            self._record_dashboard_receipt("/horus/registration_ack")
            self._last_ack_data = data
            robot_name = data.get("robot_name") if isinstance(data, dict) else None
            if robot_name:
                self._ack_by_robot[robot_name] = data
            self._ack_received.set()
        except json.JSONDecodeError:
            logger.debug("Ignored malformed registration ACK payload: %r", getattr(msg, "data", None))

    def _heartbeat_callback(self, msg):
        """Handle heartbeat from Unity"""
        now = time.monotonic()
        app_id = "unknown"
        heartbeat_uptime = None

        if "Heartbeat:" in msg.data:
            try:
                heartbeat_part = msg.data.split("Heartbeat:", 1)[1]
                heartbeat_value = heartbeat_part.split("|", 1)[0].strip()
                heartbeat_uptime = float(heartbeat_value)
            except Exception:
                logger.debug("Failed to parse heartbeat uptime from %r", msg.data, exc_info=True)
                heartbeat_uptime = None

        if heartbeat_uptime is None or not math.isfinite(heartbeat_uptime) or heartbeat_uptime < 0:
            return
        # Keep the legacy wall-time attribute for external diagnostic scripts only.
        self._last_heartbeat_time = time.time()
        self._record_dashboard_receipt("/horus/heartbeat")

        # Parse IP if present "Heartbeat: <time> | IP: <ip>"
        if "IP:" in msg.data:
            try:
                parts = msg.data.split("IP:")
                if len(parts) > 1:
                    ip = parts[1].split("|", 1)[0].strip()
                    self._remote_ip = ip
                    if ip:
                        app_id = ip
                        self._app_heartbeats[ip] = now
            except Exception:
                logger.debug("Failed to parse app heartbeat IP", exc_info=True)
        self._app_heartbeats[app_id] = now

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
            logger.debug("Ignored malformed multi-operator presence payload: %r", payload_raw, exc_info=True)
            return

        if not isinstance(payload, dict):
            return

        now = time.monotonic()
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
            "workspace_active": payload.get("workspace_active") is True,
        }
        self._record_dashboard_receipt("/horus/multi_operator_presence")

    def _multi_operator_registration_replay_request_callback(self, msg):
        payload_raw = str(getattr(msg, "data", "") or "").strip()
        payload: Dict[str, Any] = {}
        if payload_raw:
            try:
                parsed = json.loads(payload_raw)
                if isinstance(parsed, dict):
                    payload = parsed
            except Exception:
                self._trace_robot_description(
                    f"[SDK replay] ignored_malformed_request payload_chars={len(payload_raw)}"
                )
                logger.debug("Ignored malformed registration replay request: %r", payload_raw, exc_info=True)
                payload = {}

        self._record_dashboard_receipt("/horus/multi_operator/sdk_registration_replay_request")
        self._last_registration_replay_request = payload
        self._registration_replay_request_seq += 1
        self._trace_robot_description(
            f"[SDK replay] request_received seq={self._registration_replay_request_seq} "
            f"request_id='{str(payload.get('request_id') or '')}' "
            f"requester_app_id='{str(payload.get('requester_app_id') or payload.get('app_id') or '')}' "
            f"join_attempt_id='{str(payload.get('join_attempt_id') or '')}'"
        )

    def _consume_registration_replay_request(self) -> Optional[Dict[str, Any]]:
        if self._registration_replay_request_seq == self._last_consumed_registration_replay_request_seq:
            return None
        self._last_consumed_registration_replay_request_seq = self._registration_replay_request_seq
        payload = dict(self._last_registration_replay_request or {})
        self._trace_robot_description(
            f"[SDK replay] request_consumed seq={self._last_consumed_registration_replay_request_seq} "
            f"request_id='{str(payload.get('request_id') or '')}'"
        )
        return payload

    def _robot_description_request_callback(self, msg):
        if (
            self.robot_description_chunk_begin_publisher is None
            or self.robot_description_chunk_item_publisher is None
            or self.robot_description_chunk_end_publisher is None
        ):
            self._trace_robot_description("[SDK request] ignored_publishers_missing")
            return

        payload_raw = str(getattr(msg, "data", "") or "").strip()
        if not payload_raw:
            self._trace_robot_description("[SDK request] ignored_empty_payload")
            return

        try:
            payload = json.loads(payload_raw)
        except Exception:
            self._trace_robot_description(f"[SDK request] ignored_malformed payload_chars={len(payload_raw)}")
            logger.debug("Ignored malformed robot-description request payload: %r", payload_raw, exc_info=True)
            return

        if not isinstance(payload, dict):
            self._trace_robot_description("[SDK request] ignored_non_dict_payload")
            return

        request_id = str(payload.get("request_id") or uuid.uuid4().hex).strip() or uuid.uuid4().hex
        self._record_dashboard_receipt("/horus/robot_description/request")
        robot_name = str(payload.get("robot_name") or "").strip()
        description_id = str(payload.get("description_id") or "").strip()
        session_id = str(payload.get("session_id") or "").strip()
        app_id = str(payload.get("app_id") or "").strip()
        chunk_begin_topic = str(payload.get("chunk_begin_topic") or "/horus/robot_description/chunk_begin").strip()
        chunk_item_topic = str(payload.get("chunk_item_topic") or "/horus/robot_description/chunk_item").strip()
        chunk_end_topic = str(payload.get("chunk_end_topic") or "/horus/robot_description/chunk_end").strip()
        begin_publisher = self._get_robot_description_chunk_publisher(chunk_begin_topic)
        item_publisher = self._get_robot_description_chunk_publisher(chunk_item_topic)
        end_publisher = self._get_robot_description_chunk_publisher(chunk_end_topic)
        if begin_publisher is None or item_publisher is None or end_publisher is None:
            self._trace_robot_description(
                f"[SDK request] invalid_reply_topics request_id={request_id} robot='{robot_name}' "
                f"begin='{chunk_begin_topic}' item='{chunk_item_topic}' end='{chunk_end_topic}'"
            )
            return

        self._trace_robot_description(
            f"[SDK request] received request_id={request_id} robot='{robot_name}' "
            f"description_id='{description_id}' session='{session_id}' app='{app_id}' "
            f"reply_begin='{chunk_begin_topic}' reply_item='{chunk_item_topic}' reply_end='{chunk_end_topic}' "
            f"cache_robots={len(getattr(self, '_robot_description_by_robot', {}) or {})} "
            f"cache_ids={len(getattr(self, '_robot_description_by_id', {}) or {})}"
        )

        artifact_container: Optional[Dict[str, Any]] = None
        if description_id:
            artifact_container = self._robot_description_by_id.get(description_id)
        if artifact_container is None and robot_name:
            artifact_container = self._robot_description_by_robot.get(robot_name)

        if artifact_container is None:
            cached_robots = sorted(list((getattr(self, "_robot_description_by_robot", {}) or {}).keys()))[:8]
            cached_ids = sorted(list((getattr(self, "_robot_description_by_id", {}) or {}).keys()))[:8]
            self._trace_robot_description(
                f"[SDK request] cache_miss request_id={request_id} robot='{robot_name}' "
                f"description_id='{description_id}' cached_robots={cached_robots} cached_ids={cached_ids}"
            )
            return

        artifact = artifact_container.get("artifact")
        if artifact is None:
            self._trace_robot_description(
                f"[SDK request] cache_entry_without_artifact request_id={request_id} robot='{robot_name}' "
                f"description_id='{description_id}'"
            )
            return

        resolved_description_id = str(artifact.manifest.description_id)
        if description_id and description_id != resolved_description_id:
            self._trace_robot_description(
                f"[SDK request] description_mismatch request_id={request_id} robot='{robot_name}' "
                f"requested_description_id='{description_id}' resolved_description_id='{resolved_description_id}'"
            )
            return

        chunks = list(artifact.chunks or [])
        expected_chunks = len(chunks)
        now_ms = int(time.time() * 1000)
        self._trace_robot_description(
            f"[SDK chunks] begin_publish request_id={request_id} robot='{robot_name}' "
            f"description_id='{resolved_description_id}' chunks={expected_chunks} "
            f"encoded_bytes={int(getattr(artifact.manifest, 'mesh_asset_encoded_bytes', 0) or 0)}"
        )

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
        begin_publisher.publish(begin)
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
            item_publisher.publish(item)
            if index == 0 or index == expected_chunks - 1 or ((index + 1) % (50 if expected_chunks >= 200 else 10)) == 0:
                self._trace_robot_description(
                    f"[SDK chunks] item_publish request_id={request_id} robot='{robot_name}' "
                    f"description_id='{resolved_description_id}' chunk_index={index} sent={index + 1}/{expected_chunks}"
                )
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
        end_publisher.publish(end)
        self._trace_robot_description(
            f"[SDK chunks] end_publish request_id={request_id} robot='{robot_name}' "
            f"description_id='{resolved_description_id}' sent={expected_chunks}/{expected_chunks} "
            f"reply_begin='{chunk_begin_topic}'"
        )

    def _publish_sdk_registry_replay_once(self, entries, replay_request: Optional[Dict[str, Any]] = None) -> Tuple[bool, Dict[str, Any]]:
        from .multi_operator_replay import publish_sdk_registry_replay_once

        return publish_sdk_registry_replay_once(self, entries, replay_request)

    def _publish_sdk_registry_replay(self, entries, replay_request: Optional[Dict[str, Any]] = None) -> Tuple[bool, Dict[str, Any]]:
        from .multi_operator_replay import publish_sdk_registry_replay

        return publish_sdk_registry_replay(self, entries, replay_request)

    def _prune_stale_app_presence(self, timeout_s: float):
        now = time.monotonic()
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
                    "private_workspace_ready",
                    "workspace_ready",
                    "workspace_synced",
                    "registry_synced",
                } and data.get("workspace_active") is True:
                    summary["private_ready_count"] += 1
            elif role == "single":
                summary["single_count"] += 1

            if state in {"alignment_pending", "joiner_discovered_host"}:
                summary["alignment_pending_count"] += 1

            if state in {"alignment_ready", "colocation_ready", "workspace_ready_host", "workspace_ready_joiner", "workspace_synced", "registry_synced"}:
                summary["alignment_ready_count"] += 1

            if data.get("workspace_active") is True and (
                state.startswith("workspace_ready") or state in {"private_workspace_ready", "workspace_synced", "registry_synced"}
            ):
                summary["workspace_ready_count"] += 1

            if state == "registry_synced":
                summary["registry_synced_count"] += 1

        return summary

    def _format_multi_operator_summary(self) -> str:
        summary = self._get_app_presence_summary(self._multi_operator_presence_ttl_s)
        if summary["total"] <= 0:
            return "No fresh presence report"
        parts = [f"{summary['total']} reporting"]
        for key, label in (("host_count", "host"), ("joiner_count", "joiner"), ("private_count", "private")):
            if summary[key]:
                parts.append(f"{label}: {summary[key]}")
        if summary["alignment_pending_count"]:
            parts.append(f"aligning: {summary['alignment_pending_count']}")
        parts.append(f"workspace-ready: {summary['workspace_ready_count']}")
        if summary["registry_synced_count"]:
            parts.append(f"registry-synced: {summary['registry_synced_count']}")
        return " | ".join(parts)

    def _format_app_link_status(self, is_connected: bool, seen_heartbeat: bool) -> str:
        if is_connected:
            return "Connected"
        return "Disconnected (heartbeat expired)" if seen_heartbeat else "Waiting for Horus App"

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
            logger.debug("Ignoring malformed teleop runtime state payload: %s", payload_raw, exc_info=True)
            return

        if not isinstance(payload, dict):
            return
        camera_topic = str(payload.get("camera_topic", "") or "").strip()
        if not camera_topic:
            return

        self._teleop_runtime_by_topic[camera_topic] = {
            "active": payload.get("active") is True,
            "mode": str(payload.get("mode", "") or "").strip().lower(),
            "transport": self._normalize_transport(payload.get("transport")),
            "ts": time.monotonic(),
        }

    def _get_runtime_transport_override(self, camera_topic: str) -> Optional[str]:
        topic = str(camera_topic or "").strip()
        if not topic:
            return None

        state = self._teleop_runtime_by_topic.get(topic)
        if not state:
            return None

        now = time.monotonic()
        ts = float(state.get("ts", 0.0) or 0.0)
        if (now - ts) > max(0.5, float(self._teleop_runtime_state_ttl_s)):
            self._teleop_runtime_by_topic.pop(topic, None)
            return None

        if not bool(state.get("active", False)):
            return None

        return self._normalize_transport(state.get("transport"))

    def _get_active_app_count(self, timeout_s: float) -> int:
        now = time.monotonic()
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
        start_time = time.monotonic()
        while (time.monotonic() - start_time) < timeout_sec:
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

        return status

    def _collect_topics(self, dataviz):
        topics = []
        if dataviz is None:
            return topics
        try:
            for viz in dataviz.get_enabled_visualizations():
                viz_type = str(getattr(getattr(viz, "viz_type", None), "value", "") or "")
                if viz_type == "remote_render":
                    render_options = getattr(viz, "render_options", {}) or {}
                    transport = str(
                        render_options.get("transport", "ros_compressed")
                        or "ros_compressed"
                    ).strip().lower()
                    if transport == "ros_compressed":
                        ros_topic = str(render_options.get("ros_compressed_topic", "") or "").strip()
                        if ros_topic and ros_topic not in topics:
                            topics.append(ros_topic)
                    status_topic = str(
                        render_options.get("status_topic", "") or ""
                    ).strip()
                    if status_topic and status_topic not in topics:
                        topics.append(status_topic)
                    continue
                topic = viz.data_source.topic
                if topic and topic not in topics:
                    topics.append(topic)
        except Exception:
            logger.debug("Failed to collect enabled visualization topics", exc_info=True)
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

    def _apply_registration_metadata(self, robot, dataviz, ack: Dict, *, monitor_topics=True):
        try:
            robot.add_metadata("horus_robot_id", ack.get("robot_id"))
            robot.add_metadata("horus_color", ack.get("assigned_color") or ack.get("color"))
            robot.add_metadata("horus_registered", True)
        except Exception:
            logger.debug("Failed to apply registration metadata to robot %r", getattr(robot, "name", robot), exc_info=True)
            return
        topics = self._collect_topics(dataviz)
        robot.add_metadata("horus_topics", topics)
        if topics and monitor_topics:
            try:
                from horus.utils.topic_monitor import get_topic_monitor
                monitor = get_topic_monitor()
                monitor.watch_topics(topics)
                monitor.start()
            except Exception:
                logger.debug("Failed to start topic monitor after registration metadata update", exc_info=True)

    def _get_topic_counts(self, topic: str) -> Tuple[int, int]:
        pubs_info = subs_info = None
        pubs_count = subs_count = None
        try:
            pubs_info = len(self.node.get_publishers_info_by_topic(topic))
        except Exception:
            logger.debug("Failed to read publisher info for topic %s", topic, exc_info=True)
            pubs_info = None
        try:
            subs_info = len(self.node.get_subscriptions_info_by_topic(topic))
        except Exception:
            logger.debug("Failed to read subscription info for topic %s", topic, exc_info=True)
            subs_info = None
        try:
            pubs_count = self.node.count_publishers(topic)
        except Exception:
            logger.debug("Failed to count publishers for topic %s", topic, exc_info=True)
            pubs_count = None
        try:
            subs_count = self.node.count_subscribers(topic)
        except Exception:
            logger.debug("Failed to count subscribers for topic %s", topic, exc_info=True)
            subs_count = None

        def _choose(*values: Optional[int]) -> int:
            available = [v for v in values if v is not None]
            return max(available) if available else 0

        return _choose(pubs_info, pubs_count), _choose(subs_info, subs_count)

    def _is_backend_node_name(self, node_name: Optional[str]) -> bool:
        return not self._is_local_node_name(node_name) and is_bridge_node(node_name)

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
            logger.debug("Failed to compare local node name %s", node_name, exc_info=True)
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
            logger.debug("Failed to read monitored subscription state for %s", topic, exc_info=True)
        return None

    def _spin_once_registration(self, timeout_sec: float = 0.1) -> None:
        """Spin the registry node once and rate-limit diagnostics for ROS errors."""
        try:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)
        except Exception:
            now = time.time()
            if now - self._last_spin_once_error_log_time >= 5.0:
                logger.debug("ROS 2 spin_once failed while waiting for registration state", exc_info=True)
                self._last_spin_once_error_log_time = now

    def _resolve_data_topic_link(self, topic: str) -> bool:
        return self._has_backend_subscriber(topic)

    def _has_backend_subscriber(self, topic: str) -> bool:
        try:
            infos = self.node.get_subscriptions_info_by_topic(topic)
        except Exception:
            logger.debug("Failed to read backend subscription info for topic %s", topic, exc_info=True)
            return False
        if any(self._is_backend_node_name(info.node_name) for info in infos):
            return True
        return False

    def _has_backend_publisher(self, topic: str) -> bool:
        try:
            infos = self.node.get_publishers_info_by_topic(topic)
        except Exception:
            logger.debug("Failed to read backend publisher info for topic %s", topic, exc_info=True)
            return False
        if any(self._is_backend_node_name(info.node_name) for info in infos):
            return True
        return False

    def _has_local_publisher(self, topic: str) -> bool:
        try:
            infos = self.node.get_publishers_info_by_topic(topic)
        except Exception:
            logger.debug("Failed to read local publisher info for topic %s", topic, exc_info=True)
            return False
        return any(self._is_local_node_name(info.node_name) for info in infos)

    def _has_local_subscriber(self, topic: str) -> bool:
        try:
            infos = self.node.get_subscriptions_info_by_topic(topic)
        except Exception:
            logger.debug("Failed to read local subscription info for topic %s", topic, exc_info=True)
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

        # OPTIONAL declarative transport lane for 3D-map visualizations. When a map global viz
        # sets render_options["transport_lane"] to one of the recognized tokens, surface it so
        # Unity can forward it as the __subscribe "policy" key and the bridge honors it verbatim.
        # Default unset (or an unrecognized value) emits no field, so the bridge keeps using its
        # heuristic lane classifier -- fully backward-compatible, no behavior change.
        _MAP_LANE_VIZ_TYPES = {
            "point_cloud",
            "gaussian_splat",
            "mesh",
            "octomap",
            "occupancy_grid",
            "costmap",
        }
        _VALID_TRANSPORT_LANES = {
            "strict",
            "replaceable",
            "bulk_strict",
            "bulk_replaceable",
        }
        if viz_type_value in _MAP_LANE_VIZ_TYPES:
            transport_lane = render_options.get("transport_lane")
            if transport_lane not in (None, ""):
                normalized_lane = str(transport_lane).strip().lower()
                if normalized_lane in _VALID_TRANSPORT_LANES:
                    payload["transport_lane"] = normalized_lane

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

        if viz_type_value == "costmap":
            payload["costmap"] = {
                "alpha": self._payload_coerce_float(render_options.get("alpha"), 0.65),
                "show_unknown_space": self._payload_coerce_bool(render_options.get("show_unknown_space"), False),
                "height_offset": self._payload_coerce_float(render_options.get("height_offset"), 0.01),
                "stale_timeout": self._payload_coerce_float(render_options.get("stale_timeout"), 5.0),
                "tf_topic": str(render_options.get("tf_topic") or "/tf"),
            }

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

        if viz_type_value == "remote_render":
            protocol_version = str(
                render_options.get("protocol_version", "remote_stereo_rgbd_v2")
                or "remote_stereo_rgbd_v2"
            ).strip()
            if protocol_version not in {
                "remote_stereo_rgbd_v2",
                "horus.remote_render.v3",
            }:
                protocol_version = "remote_stereo_rgbd_v2"
            encoder = str(render_options.get("encoder", "auto") or "auto").strip().lower()
            if encoder not in {"auto", "nvenc", "x264"}:
                encoder = "auto"
            remote_transport = str(
                render_options.get("transport", "ros_compressed")
                or "ros_compressed"
            ).strip().lower()
            if remote_transport not in {"webrtc", "ros_compressed"}:
                remote_transport = "ros_compressed"
            remote_format = (
                (
                    "horus.remote_render.v3.webrtc"
                    if remote_transport == "webrtc"
                    else "horus.remote_render.v3.ros"
                )
                if protocol_version == "horus.remote_render.v3"
                else (
                    "remote_stereo_rgbd_ros_v2"
                    if remote_transport == "ros_compressed"
                    else "remote_stereo_rgbd_webrtc_v2"
                )
            )
            remote_render_payload = {
                "protocol_version": protocol_version,
                "format_version": remote_format,
                "viewer_pose_topic": str(
                    render_options.get(
                        "viewer_pose_topic", "/horus/remote_render/viewer_pose"
                    )
                    or "/horus/remote_render/viewer_pose"
                ).strip(),
                "viewer_pose_rate_hz": max(
                    1.0,
                    min(
                        120.0,
                        self._payload_coerce_float(
                            render_options.get("viewer_pose_rate_hz"), 60.0
                        ),
                    ),
                ),
                "transport": remote_transport,
                "stream_topic": topic,
                "frame_data_topic": str(
                    render_options.get(
                        "frame_data_topic",
                        "" if (
                            protocol_version == "horus.remote_render.v3"
                            and remote_transport == "webrtc"
                        ) else "/horus/remote_render/frame_data",
                    )
                    or ""
                ).strip(),
                "ros_compressed_topic": str(
                    render_options.get(
                        "ros_compressed_topic",
                        "/horus/remote_render/rgbd",
                    )
                    or "/horus/remote_render/rgbd"
                ).strip(),
                "client_signal_topic": str(
                    render_options.get("client_signal_topic", "/horus/webrtc/client_signal")
                ).strip(),
                "server_signal_topic": str(
                    render_options.get("server_signal_topic", "/horus/webrtc/server_signal")
                ).strip(),
                "status_topic": str(
                    render_options.get("status_topic", "/horus/remote_render/agent_status")
                ).strip(),
                "encoder": encoder,
                "image_type": (
                    "h264"
                    if str(render_options.get("image_type", "raw") or "raw")
                    .strip()
                    .lower() == "h264"
                    else "raw"
                ),
                "bitrate_kbps": max(
                    1000, int(self._payload_coerce_float(render_options.get("bitrate_kbps"), 12000.0))
                ),
                "framerate": max(
                    1, min(60, int(self._payload_coerce_float(render_options.get("framerate"), 60.0)))
                ),
                "flip_y": self._payload_coerce_bool(render_options.get("flip_y"), False),
            }
            if protocol_version == "horus.remote_render.v3":
                remote_render_payload.update(
                    {
                        "session_id": str(
                            render_options.get("session_id", "") or ""
                        ).strip(),
                        "reference_space_id": str(
                            render_options.get(
                                "reference_space_id", "workspace-local"
                            )
                            or "workspace-local"
                        ).strip(),
                        "workspace_revision": max(
                            0,
                            int(
                                self._payload_coerce_float(
                                    render_options.get("workspace_revision"), 0.0
                                )
                            ),
                        ),
                        "viewport_width": max(
                            1,
                            int(
                                self._payload_coerce_float(
                                    render_options.get("viewport_width"), 1024.0
                                )
                            ),
                        ),
                        "viewport_height": max(
                            1,
                            int(
                                self._payload_coerce_float(
                                    render_options.get("viewport_height"), 1024.0
                                )
                            ),
                        ),
                        "display_refresh_rate_hz": max(
                            1.0,
                            self._payload_coerce_float(
                                render_options.get("display_refresh_rate_hz"), 72.0
                            ),
                        ),
                        "maximum_frame_age_ms": max(
                            1,
                            int(
                                self._payload_coerce_float(
                                    render_options.get("maximum_frame_age_ms"), 250.0
                                )
                            ),
                        ),
                    }
                )
            if not remote_render_payload["viewer_pose_topic"].startswith("/"):
                remote_render_payload["viewer_pose_topic"] = "/horus/remote_render/viewer_pose"
            payload["remote_render"] = remote_render_payload

        if viz_type_value == "point_cloud":
            point_cloud_payload: Dict[str, Any] = {}
            color_mode = str(render_options.get("color_mode", "rgb")).strip().lower()
            point_cloud_payload["color_mode"] = (
                color_mode if color_mode in ("rgb", "flat", "height", "rgb_height") else "rgb"
            )
            height_min = self._payload_coerce_float(render_options.get("height_min_m"), -1.0)
            height_max = self._payload_coerce_float(render_options.get("height_max_m"), 2.0)
            if not math.isfinite(height_min):
                height_min = -1.0
            if not math.isfinite(height_max) or height_max <= height_min:
                height_max = height_min + 3.0
            point_cloud_payload["height_min_m"] = height_min
            point_cloud_payload["height_max_m"] = height_max
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
            point_shape = str(render_options.get("point_shape", "square")).strip().lower()
            if point_shape == "disc":
                point_shape = "circle"
            if point_shape not in {"square", "circle"}:
                point_shape = "square"
            point_cloud_payload["point_shape"] = point_shape
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

        if viz_type_value == "gaussian_splat":
            gaussian_payload: Dict[str, Any] = {}
            manifest_topic = str(render_options.get("manifest_topic", topic)).strip()
            gaussian_payload["manifest_topic"] = manifest_topic or topic
            chunk_begin_topic = str(
                render_options.get("chunk_begin_topic", "/horus/gaussian_splat/chunk_begin")
            ).strip()
            gaussian_payload["chunk_begin_topic"] = chunk_begin_topic or "/horus/gaussian_splat/chunk_begin"
            chunk_item_topic = str(
                render_options.get("chunk_item_topic", "/horus/gaussian_splat/chunk_item")
            ).strip()
            gaussian_payload["chunk_item_topic"] = chunk_item_topic or "/horus/gaussian_splat/chunk_item"
            chunk_end_topic = str(
                render_options.get("chunk_end_topic", "/horus/gaussian_splat/chunk_end")
            ).strip()
            gaussian_payload["chunk_end_topic"] = chunk_end_topic or "/horus/gaussian_splat/chunk_end"
            gaussian_payload["binary_chunks"] = self._payload_coerce_bool(
                render_options.get("binary_chunks"), False
            )

            asset_format = str(render_options.get("asset_format", "3dgs_ply")).strip().lower()
            if asset_format not in {"3dgs_ply"}:
                asset_format = "3dgs_ply"
            gaussian_payload["asset_format"] = asset_format

            source_coordinate_space = str(
                render_options.get("source_coordinate_space", "colmap")
            ).strip().lower()
            if source_coordinate_space not in {"colmap", "3dgs", "opencv", "unity"}:
                source_coordinate_space = "colmap"
            gaussian_payload["source_coordinate_space"] = source_coordinate_space

            render_mode = str(render_options.get("render_mode", "splats")).strip().lower()
            if render_mode not in {"splats", "debug_points", "mono_center_eye", "no_covariance"}:
                render_mode = "splats"
            gaussian_payload["render_mode"] = render_mode

            gaussian_payload["max_splats"] = max(
                1000,
                int(self._payload_coerce_float(render_options.get("max_splats"), 350000.0)),
            )
            gaussian_payload["render_scale"] = min(
                1.0,
                max(0.25, self._payload_coerce_float(render_options.get("render_scale"), 0.5)),
            )
            gaussian_payload["sh_order"] = min(
                3,
                max(0, int(self._payload_coerce_float(render_options.get("sh_order"), 2.0))),
            )
            gaussian_payload["half_precision_sh"] = self._payload_coerce_bool(
                render_options.get("half_precision_sh"),
                True,
            )
            gaussian_payload["adaptive_sort"] = self._payload_coerce_bool(
                render_options.get("adaptive_sort"),
                True,
            )
            gaussian_payload["sort_passes"] = min(
                4,
                max(2, int(self._payload_coerce_float(render_options.get("sort_passes"), 2.0))),
            )
            gaussian_payload["opacity_scale"] = min(
                20.0,
                max(0.05, self._payload_coerce_float(render_options.get("opacity_scale"), 1.0)),
            )
            gaussian_payload["splat_scale"] = min(
                4.0,
                max(0.05, self._payload_coerce_float(render_options.get("splat_scale"), 1.0)),
            )
            gaussian_payload["contribution_cull_threshold"] = min(
                1.0,
                max(
                    0.0,
                    self._payload_coerce_float(
                        render_options.get("contribution_cull_threshold"),
                        0.1,
                    ),
                ),
            )
            gaussian_payload["high_precision_rt"] = self._payload_coerce_bool(
                render_options.get("high_precision_rt"),
                False,
            )
            gaussian_payload["preserve_source_detail"] = self._payload_coerce_bool(
                render_options.get("preserve_source_detail"), False
            )
            gaussian_payload["pointcloud_fallback"] = self._payload_coerce_bool(
                render_options.get("pointcloud_fallback"),
                True,
            )

            fallback_topic = str(
                render_options.get("fallback_topic", "/map_gaussian_splat_preview")
            ).strip()
            gaussian_payload["fallback_topic"] = fallback_topic or "/map_gaussian_splat_preview"
            fallback_frame = str(render_options.get("fallback_frame", frame_id or "map")).strip()
            gaussian_payload["fallback_frame"] = fallback_frame or "map"

            fallback_point_cloud = render_options.get("fallback_point_cloud")
            if isinstance(fallback_point_cloud, dict):
                gaussian_payload["fallback_point_cloud"] = dict(fallback_point_cloud)

            payload["gaussian_splat"] = gaussian_payload

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
            mesh_payload["use_indexed_binary"] = self._payload_coerce_bool(
                render_options.get("use_indexed_binary"),
                True,
            )
            mesh_payload["static_mesh"] = self._payload_coerce_bool(
                render_options.get("static_mesh"),
                False,
            )
            mesh_payload["indexed_live"] = self._payload_coerce_bool(
                render_options.get("indexed_live"), default=False)
            mesh_payload["max_chunk_renderers"] = min(
                1024,
                max(
                    8,
                    int(self._payload_coerce_float(render_options.get("max_chunk_renderers"), 1024.0)),
                ),
            )
            mesh_payload["max_chunk_uploads_per_frame"] = min(
                32,
                max(
                    1,
                    int(self._payload_coerce_float(render_options.get("max_chunk_uploads_per_frame"), 2.0)),
                ),
            )
            mesh_payload["max_triangle_uploads_per_frame"] = max(
                2000,
                int(self._payload_coerce_float(render_options.get("max_triangle_uploads_per_frame"), 20000.0)),
            )
            mesh_payload["max_upload_milliseconds_per_frame"] = min(
                8.0,
                max(
                    0.25,
                    self._payload_coerce_float(
                        render_options.get("max_upload_milliseconds_per_frame"),
                        2.5,
                    ),
                ),
            )
            mesh_payload["static_finalize_delay_seconds"] = min(
                10.0,
                max(
                    0.1,
                    self._payload_coerce_float(render_options.get("static_finalize_delay_seconds"), 0.75),
                ),
            )
            mesh_payload["enable_perf_diagnostics"] = self._payload_coerce_bool(
                render_options.get("enable_perf_diagnostics"),
                False,
            )
            adaptive = render_options.get("adaptive")
            if isinstance(adaptive, dict) and adaptive.get("enabled"):
                # Keep the nested per-operator policy in the registration contract.
                mesh_payload["adaptive"] = json.loads(json.dumps(adaptive, allow_nan=False))

            if "display" in render_options:
                from horus.dataviz.mesh_display import mesh_display_options
                mesh_payload["display"] = mesh_display_options(render_options["display"])
            payload["mesh"] = mesh_payload

        if viz_type_value == "octomap":
            octomap_payload: Dict[str, Any] = {}
            render_mode = str(
                render_options.get("render_mode", "surface_mesh")
            ).strip().lower()
            if render_mode not in {"surface_mesh", "voxel_cubes"}:
                render_mode = "surface_mesh"
            octomap_payload["render_mode"] = render_mode
            octomap_payload["static_mesh"] = self._payload_coerce_bool(
                render_options.get("static_mesh"), False
            )
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

        if viz_type_value == "semantic_box":
            semantic_id = str(render_options.get("id", "") or "").strip()
            semantic_label = str(render_options.get("label", "") or "").strip()
            if not semantic_id or not semantic_label:
                return None

            payload["semantic_box"] = {
                "id": semantic_id,
                "label": semantic_label,
                "center": self._payload_coerce_vec3(
                    render_options.get("center"),
                    (0.0, 0.0, 0.0),
                ),
                "size": self._payload_coerce_vec3(
                    render_options.get("size"),
                    (0.25, 0.25, 0.25),
                ),
                "rotation_offset_euler": self._payload_coerce_vec3(
                    render_options.get("rotation_offset_euler"),
                    (0.0, 0.0, 0.0),
                ),
            }

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
                logger.debug("Failed to read global visualization list from %r", dataviz, exc_info=True)
                visualizations = []

            for visualization in visualizations:
                if visualization is None:
                    continue
                try:
                    if visualization.is_robot_specific():
                        continue
                except Exception:
                    logger.debug("Failed to classify visualization scope for %r", visualization, exc_info=True)
                    continue

                payload = self._serialize_visualization_payload(
                    visualization,
                    scope_override="global",
                )
                if not payload:
                    continue

                if str(payload.get("type", "")) == "semantic_box":
                    semantic_payload = payload.get("semantic_box") or {}
                    key = (
                        str(payload.get("type", "")),
                        str(semantic_payload.get("id", "")),
                        str(payload.get("frame", "")),
                    )
                else:
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

    def _record_dashboard_receipt(self, topic: str) -> None:
        if not hasattr(self, "_dashboard_receipts"):
            self._dashboard_receipts = {}
        self._dashboard_receipts[topic] = time.monotonic()

    def _get_dashboard_bridge_status(self, is_connected: bool) -> str:
        if is_connected:
            return "Responding (app heartbeat received)"
        try:
            if any(is_bridge_node(name) for name, _ in self.node.get_node_names_and_namespaces()):
                return "Discovered (ROS graph)"
        except Exception:
            return "Unknown (ROS graph unavailable)"
        return "Not discovered"

    def _take_registration_ack(self, robot_name: str):
        """Consume each acknowledgement once; never reuse the previous session's result."""
        ack = self._ack_by_robot.pop(robot_name, None)
        if not self._ack_by_robot:
            self._ack_received.clear()
        return ack

    def _build_dashboard_topic_rows(
        self, monitored_topics, topic_roles, robot_names, is_connected: bool,
        registration_done: bool, camera_topic_profiles=None, topic_group_overrides=None,
    ) -> list:
        camera_topic_profiles = camera_topic_profiles or {}
        topic_group_overrides = topic_group_overrides or {}
        receipts = getattr(self, "_dashboard_receipts", {})
        now = time.monotonic()
        rows = []
        for topic in dict.fromkeys(monitored_topics):
            role = topic_roles.get(topic, "backend_sub")
            core = role in {"sdk_pub", "sdk_sub"}
            profile = dict(camera_topic_profiles.get(topic, {}) or {})
            runtime_transport = self._get_runtime_transport_override(topic) if is_connected else None
            if runtime_transport:
                profile["active_transport"] = runtime_transport
            else:
                profile.pop("active_transport", None)
            graph = topic_graph(self.node, topic, role)
            mode = profile.get("source_mode")
            if mode not in {"minimap", "teleop"}:
                mode = profile.get("startup", "minimap")
            transport = runtime_transport or profile.get(mode)
            # Media does not require a ROS bridge image subscriber. Neither the
            # profile nor a source publisher proves a WebRTC connection/decoder.
            if transport == "webrtc" and graph["link"] != "UNKNOWN":
                graph["link"] = "MEDIA"
            data, stale = "Not measured", False
            if role == "sdk_sub":
                ts = receipts.get(topic)
                if ts is None:
                    data = "Waiting"
                else:
                    age = max(0.0, now - ts)
                    stale = topic == "/horus/heartbeat" and age > 3.0
                    data = f"{'Stale' if stale else 'Last'} {age:.1f}s ago"
            rows.append({
                "robot": self._topic_group(topic, robot_names, topic_group_overrides=topic_group_overrides),
                "topic": topic, "role": {
                    "sdk_pub": "SDK -> bridge", "sdk_sub": "bridge -> SDK",
                    "backend_pub": "bridge -> robot", "backend_sub": "robot -> bridge",
                }.get(role, role),
                "topic_kind": "core" if core else "data",
                **graph, "data": data, "stale": stale, "camera_profile": profile,
            })
        rows.sort(key=lambda r: (r["topic_kind"] != "core", r["robot"] or "", r["topic"]))
        return rows

    def register_robot(
        self,
        robot,
        dataviz,
        timeout_sec: float = 10.0,
        keep_alive: bool = False,
        show_dashboard: bool = True,
        workspace_scale: Optional[float] = None,
        compass_enabled: Optional[bool] = None,
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
            compass_enabled: Optional workspace-scoped Compass availability toggle.
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
        previous_silent = board.is_silent
        board.set_silent(show_dashboard)

        try:
            cli.print_info("Building robot registration payload...")
            global_visualizations = self._build_global_visualizations_payload([dataviz])
            config = self._build_robot_config_dict(
                robot,
                dataviz,
                global_visualizations=global_visualizations,
                workspace_scale=workspace_scale,
                compass_enabled=compass_enabled,
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
            bridge_state = "Starting; awaiting runtime evidence"

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
                if not show_dashboard:
                    monitor.watch_topics(monitored_topics, topic_roles)
                    monitor.start()
            except Exception:
                logger.debug("Failed to start topic monitor for registration dashboard", exc_info=True)

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
                start_time = time.monotonic()
                last_publish = start_time
                last_queued_reason = ""
                while time.monotonic() - start_time < timeout_sec:
                    self._spin_once_registration(timeout_sec=0.1)

                    # Re-publish occasionally
                    if time.monotonic() - last_publish >= 2.0:
                        self.publisher.publish(msg)
                        last_publish = time.monotonic()

                    if self._ack_received.is_set():
                        ack = self._take_registration_ack(robot_name) or {}
                        recv_name = ack.get("robot_name", "UNKNOWN")
                        if recv_name == robot_name:
                            if ack.get("success"):
                                queued_msg = self._queued_reason_from_ack(ack)
                                if queued_msg:
                                    last_queued_reason = queued_msg
                                    self._ack_received.clear()
                                    continue
                                return True, ack
                            return False, ack
                        self._ack_received.clear()

                if last_queued_reason:
                    return False, {
                        "error": "Registration queued",
                        "queued": True,
                        "reason": last_queued_reason,
                    }
                return False, {"error": "Registration timeout"}

            with cli.ConnectionDashboard(local_ip, 10000, bridge_state) as dashboard:
                last_stats_update = float("-inf")
                last_connection_state = False
                seen_heartbeat = False
                queued_reason = ""
                last_register_publish = time.monotonic()
                registration_deadline_started_at = None
                while True:
                    self._spin_once_registration(timeout_sec=0.1)
                    now = time.monotonic()
                    seen_heartbeat = seen_heartbeat or self._last_heartbeat_time > 0
                    is_connected = self._get_active_app_count(3.0) > 0
                    app_restarted = self._consume_app_restart_signal()
                    if app_restarted or (last_connection_state and not is_connected):
                        registration_success = False
                        queued_reason = ""
                        self._take_registration_ack(robot_name)
                        registration_deadline_started_at = None
                        last_register_publish = float("-inf")

                    if not registration_success and registration_deadline_started_at is None:
                        if is_connected or not wait_for_app_before_register:
                            registration_deadline_started_at = now
                    ack = self._take_registration_ack(robot_name)
                    if ack:
                        if not ack.get("success"):
                            dashboard.update_registration("Failed")
                            dashboard.update_status(ack.get("error") or "Registration rejected")
                            dashboard.tick(force=True)
                            return False, ack
                        queued_reason = self._queued_reason_from_ack(ack) or ""
                        registration_success = not queued_reason
                        if registration_success:
                            final_ack = ack
                            self._apply_registration_metadata(robot, dataviz, ack, monitor_topics=False)

                    dashboard.update_app_link(self._format_app_link_status(is_connected, seen_heartbeat))
                    dashboard.update_multi_operator(self._format_multi_operator_summary())
                    if not is_connected:
                        dashboard.update_registration("Waiting for App")
                        dashboard.update_status("Registration acknowledgement retained" if registration_success else "")
                    elif registration_success:
                        dashboard.update_registration("Registered")
                        dashboard.update_status(robot_name)
                    elif queued_reason:
                        dashboard.update_registration("Queued")
                        dashboard.update_status(queued_reason)
                    else:
                        timed_out = registration_deadline_started_at is not None and now - registration_deadline_started_at >= timeout_sec
                        dashboard.update_registration("Retrying" if timed_out else "Registering")
                        dashboard.update_status(f"Awaiting acknowledgement for {robot_name}")
                        if now - last_register_publish >= 2.0:
                            self.publisher.publish(msg)
                            last_register_publish = now

                    if now - last_stats_update >= 1.0:
                        dashboard.update_bridge(self._get_dashboard_bridge_status(is_connected))
                        dashboard.update_topics(self._build_dashboard_topic_rows(
                            monitored_topics, topic_roles, [robot.name], is_connected,
                            registration_success, camera_topic_profiles, topic_group_overrides,
                        ))
                        last_stats_update = now
                    dashboard.tick()
                    last_connection_state = is_connected
                    if registration_success and not keep_alive:
                        return True, final_ack
                    if not keep_alive and registration_deadline_started_at is not None and now - registration_deadline_started_at >= timeout_sec:
                        dashboard.update_registration("Failed")
                        dashboard.update_status("Registration timeout")
                        return False, {"error": "Registration timeout"}

        except KeyboardInterrupt:
            cli.print_info("Registration/Monitoring cancelled by user.")
            return False, {"error": "Cancelled"}
        except Exception as e:
            cli.print_error(f"Structure Error: {e}")
            import traceback
            traceback.print_exc()
            return False, {"error": str(e)}
        finally:
            board.set_silent(previous_silent)
            self._registration_lock.release()



    def register_robots(
        self,
        robots,
        datavizs=None,
        timeout_sec: float = 10.0,
        keep_alive: bool = True,
        show_dashboard: bool = True,
        workspace_scale: Optional[float] = None,
        compass_enabled: Optional[bool] = None,
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
        previous_silent = board.is_silent
        board.set_silent(show_dashboard)

        try:
            if not robots or len({robot.name for robot in robots}) != len(robots):
                return False, {"error": "Provide at least one robot with unique names"}
            if datavizs is None:
                datavizs = [robot.create_dataviz() for robot in robots]

            if len(datavizs) != len(robots):
                return False, {"error": "Robot/dataviz length mismatch"}

            cli.print_info("Building HORUS registration payloads...")
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
                    compass_enabled=compass_enabled,
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
                entity_kind = str(
                    getattr(robot, "get_entity_kind", lambda: "robot")() or "robot"
                ).strip().lower()
                dashboard_group = (
                    "Workspace Render"
                    if entity_kind == "workspace_visualization"
                    else robot.name
                )
                for topic, group in self._build_topic_group_overrides(dashboard_group, entry_topics).items():
                    topic_group_overrides[topic] = group
                robot_topics = self._merge_topics(robot_topics, entry_data_topics)
                robot_topics = self._merge_topics(robot_topics, entry_camera_topics)
                control_topics = self._merge_topics(control_topics, entry_control_topics)
            cli.print_info(f"Built {len(entries)} registration payload(s).")

            bridge_running = self._ensure_bridge_running()
            if not bridge_running:
                return False, {"error": "Bridge Start Failed"}
            cli.print_info("Bridge ready. Entering registration phase.")

            local_ip = self._get_local_ip()
            bridge_state = "Starting; awaiting runtime evidence"

            robot_topics = self._merge_topics(robot_topics, control_topics)
            topic_roles = self._build_topic_roles(robot_topics, control_topics=control_topics)
            monitored_topics = core_topics + [t for t in robot_topics if t not in core_topics]

            try:
                from horus.utils.topic_monitor import get_topic_monitor
                monitor = get_topic_monitor()
                if not show_dashboard:
                    monitor.watch_topics(monitored_topics, topic_roles)
                    monitor.start()
            except Exception:
                logger.debug("Failed to start topic monitor for multi-registration dashboard", exc_info=True)

            robot_states = {robot.name: "pending" for robot in robots}
            queued_reasons = {}
            last_failure = None

            def handle_ack(ack):
                nonlocal last_failure
                if not isinstance(ack, dict):
                    return
                robot_name = ack.get("robot_name")
                if robot_name not in entry_by_name:
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
                        self._apply_registration_metadata(entry[0], entry[1], ack, monitor_topics=not show_dashboard)
                else:
                    robot_states[robot_name] = "failed"
                    last_failure = ack.get("error") or "Registration failed"

            def publish_and_wait(robot, dataviz, msg):
                robot_name = robot.name
                robot_states[robot_name] = "pending"
                queued_reasons.pop(robot_name, None)
                self._ack_received.clear()
                self._last_ack_data = {}
                self._ack_by_robot.pop(robot_name, None)
                start_time = time.monotonic()
                last_publish = float("-inf")
                while (time.monotonic() - start_time) < timeout_sec:
                    now = time.monotonic()
                    if now - last_publish >= 2.0:
                        self.publisher.publish(msg)
                        last_publish = now
                    self._spin_once_registration(timeout_sec=0.1)
                    ack = self._take_registration_ack(robot_name)
                    if ack:
                        if ack.get("success"):
                            return True, ack
                        return False, ack
                return False, {"error": "Registration timeout"}

            def register_all(dashboard=None, force: bool = False, allow_queued: bool = True):
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
                    if not allow_queued and robot_states.get(robot.name) == "queued":
                        return False, {
                            "error": "Registration queued",
                            "queued": True,
                            "robot_name": robot.name,
                            "reason": queued_reasons.get(robot.name) or "Waiting for Workspace",
                        }
                if not allow_queued:
                    queued_names = [
                        name for name, state in robot_states.items()
                        if state == "queued"
                    ]
                    if queued_names:
                        return False, {
                            "error": "Registration queued",
                            "queued": True,
                            "robots": queued_names,
                            "reason": self._resolve_dashboard_queued_status(queued_reasons),
                        }
                return True, {"success": True}

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
                ok, result = register_all(None, allow_queued=False)
                if not ok:
                    return False, result
                if not keep_alive:
                    return True, result
                last_connection_state = True
                while True:
                    self._spin_once_registration(timeout_sec=0.1)
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

            with cli.ConnectionDashboard(local_ip, 10000, bridge_state) as dashboard:
                robot_names = [r.name for r in robots]
                # Do not block the dashboard on one robot's ACK. Pace publications
                # and process all callbacks between them, including app heartbeats.
                self._ack_by_robot.clear()
                self._ack_received.clear()
                last_publish_by_robot = {}
                last_publish = float("-inf")
                last_topics_update = float("-inf")
                last_connection_state = False
                seen_heartbeat = False
                deadline_started = None
                replay_note = ""
                while True:
                    self._spin_once_registration(timeout_sec=0.1)
                    now = time.monotonic()
                    seen_heartbeat = seen_heartbeat or self._last_heartbeat_time > 0
                    is_connected = self._get_active_app_count(3.0) > 0
                    app_restarted = self._consume_app_restart_signal()
                    if app_restarted or (last_connection_state and not is_connected):
                        robot_states = {name: "pending" for name in robot_names}
                        queued_reasons.clear()
                        self._ack_by_robot.clear()
                        self._ack_received.clear()
                        last_publish_by_robot.clear()
                        deadline_started = None
                        replay_note = ""
                        last_failure = None

                    dashboard.update_app_link(self._format_app_link_status(is_connected, seen_heartbeat))
                    dashboard.update_multi_operator(self._format_multi_operator_summary())
                    for name in robot_names:
                        ack = self._take_registration_ack(name)
                        if ack is not None:
                            handle_ack(ack)

                    replay_request = self._consume_registration_replay_request()
                    if replay_request and is_connected:
                        dashboard.update_status("Publishing registry replay")
                        dashboard.tick(force=True)
                        for _, _, reg_msg in entries:
                            self.publisher.publish(reg_msg)
                        ok, result = self._publish_sdk_registry_replay(entries, replay_request)
                        replay_note = (
                            "Replay published; joiner confirmation pending"
                            if ok else (result or {}).get("error") or "Replay publication failed"
                        )

                    registered = sum(state == "registered" for state in robot_states.values())
                    failed = [name for name, state in robot_states.items() if state == "failed"]
                    pending = [name for name, state in robot_states.items() if state != "registered"]
                    registration_done = registered == len(entries)
                    if not registration_done and deadline_started is None:
                        if is_connected or not wait_for_app_before_register:
                            deadline_started = now
                    if not is_connected:
                        dashboard.update_registration("Waiting for App")
                        dashboard.update_status("")
                    elif failed:
                        dashboard.update_registration(f"Failed ({registered}/{len(entries)} acknowledged)")
                        dashboard.update_status(f"{', '.join(failed)}: {last_failure}")
                    elif registration_done:
                        dashboard.update_registration(f"Registered ({registered}/{len(entries)})")
                        dashboard.update_status(replay_note)
                    elif queued_reasons:
                        dashboard.update_registration(f"Queued ({registered}/{len(entries)} acknowledged)")
                        dashboard.update_status(self._resolve_dashboard_queued_status(queued_reasons))
                    else:
                        timed_out = deadline_started is not None and now - deadline_started >= timeout_sec
                        dashboard.update_registration(f"{'Retrying' if timed_out else 'Registering'} ({registered}/{len(entries)} acknowledged)")
                        dashboard.update_status("Awaiting: " + ", ".join(pending))

                    if (is_connected or not wait_for_app_before_register) and now - last_publish >= 0.25:
                        eligible = [
                            entry for entry in entries
                            if robot_states[entry[0].name] not in {"registered", "failed"}
                            and now - last_publish_by_robot.get(entry[0].name, float("-inf")) >= 2.0
                        ]
                        if eligible:
                            robot, _, msg = min(
                                eligible, key=lambda entry: last_publish_by_robot.get(entry[0].name, float("-inf"))
                            )
                            self.publisher.publish(msg)
                            last_publish_by_robot[robot.name] = now
                            last_publish = now

                    if now - last_topics_update >= 1.0:
                        dashboard.update_bridge(self._get_dashboard_bridge_status(is_connected))
                        update_dashboard_topics(dashboard, is_connected, registration_done)
                        last_topics_update = now
                    dashboard.tick()
                    last_connection_state = is_connected
                    if not keep_alive:
                        if failed:
                            return False, {"error": last_failure or "Registration failed", "robots": failed}
                        if registration_done:
                            return True, {"success": True}
                        if deadline_started is not None and now - deadline_started >= timeout_sec:
                            dashboard.update_registration("Failed")
                            dashboard.update_status("Registration timeout")
                            return False, {"error": "Registration timeout", "robots": pending}

        except KeyboardInterrupt:
            cli.print_info("Registration/Monitoring cancelled by user.")
            return False, {"error": "Cancelled"}
        except Exception as e:
            cli.print_error(f"Structure Error: {e}")
            import traceback
            traceback.print_exc()
            return False, {"error": str(e)}
        finally:
            board.set_silent(previous_silent)
            self._registration_lock.release()

    def unregister_robot(
        self, robot_id: str, timeout_sec: float = 5.0
    ) -> Tuple[bool, Dict]:
        """
        Unregister robot from HORUS backend
        """
        from horus.utils import cli

        robot_name = str(robot_id or "").strip()
        if not robot_name:
            return False, {"error": "Robot id is required"}

        if not self.ros_initialized or self.publisher is None or self.node is None:
            cli.print_error("ROS2 not initialized. Cannot unregister.")
            return False, {"error": "ROS2 not available"}

        if not self._registration_lock.acquire(blocking=False):
            return False, {"error": "Registration already in progress"}

        try:
            if not self._ensure_bridge_running():
                return False, {"error": "Bridge Start Failed"}

            payload = {"action": "unregister", "robot_name": robot_name}
            msg = String()
            msg.data = json.dumps(payload)

            self._ack_received.clear()
            self._last_ack_data = {}
            self._ack_by_robot.pop(robot_name, None)

            start_time = time.time()
            last_publish = 0.0
            while time.time() - start_time < timeout_sec:
                now = time.time()
                if now - last_publish >= 1.0:
                    self.publisher.publish(msg)
                    last_publish = now

                try:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                except Exception:
                    logger.debug("Failed to spin ROS 2 while waiting for unregister ACK", exc_info=True)

                ack = self._ack_by_robot.pop(robot_name, None)
                if ack is None and self._ack_received.is_set():
                    self._ack_received.clear()
                    if self._last_ack_data.get("robot_name") == robot_name:
                        ack = self._last_ack_data

                if not ack:
                    continue

                if ack.get("success"):
                    self._remove_robot_description_cache(robot_name)
                    return True, ack

                return False, ack

            return False, {
                "error": "Unregistration timeout",
                "robot_id": robot_name,
            }
        finally:
            self._registration_lock.release()

    def _resolve_robot_description_artifact(self, robot) -> Optional[Dict[str, Any]]:
        if robot is None:
            return None

        from horus.utils import cli

        metadata = getattr(robot, "metadata", None)
        description_config = metadata.get("robot_description_config") if isinstance(metadata, dict) else None
        has_description_config = isinstance(description_config, dict) and bool(description_config.get("enabled", True))

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
                self._trace_robot_description(
                    f"[SDK cache] hit robot='{robot_name}' "
                    f"description_id='{str(getattr(cached_artifact.manifest, 'description_id', '') or '')}' "
                    f"chunks={int(len(getattr(cached_artifact, 'chunks', []) or []))}"
                )
                return cached_container

        if not has_description_config:
            return None

        if not hasattr(self, "_robot_description_resolver") or self._robot_description_resolver is None:
            from horus.description.robot_description_resolver import RobotDescriptionResolver

            self._robot_description_resolver = RobotDescriptionResolver()

        artifact = self._robot_description_resolver.resolve_for_robot(robot)
        if artifact is None:
            reason = ""
            try:
                reason = str(self._robot_description_resolver.last_error or "").strip()
            except Exception:
                logger.debug("Failed to read robot description resolver error", exc_info=True)
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
                self._trace_robot_description(
                    f"[SDK cache] resolver_hit robot='{robot_name}' "
                    f"description_id='{str(getattr(artifact.manifest, 'description_id', '') or '')}' "
                    f"groups={artifact_groups} bytes={artifact_bytes} chunks={artifact_chunks}"
                )
            else:
                cli.print_info(
                    f"Built robot-description artifact for '{robot_name}' "
                    f"(groups={artifact_groups}, bytes={artifact_bytes}, chunks={artifact_chunks})."
                )
                self._trace_robot_description(
                    f"[SDK cache] built robot='{robot_name}' "
                    f"description_id='{str(getattr(artifact.manifest, 'description_id', '') or '')}' "
                    f"groups={artifact_groups} bytes={artifact_bytes} chunks={artifact_chunks}"
                )

        container = {
            "robot_name": robot_name,
            "artifact": artifact,
        }
        if robot_name:
            self._robot_description_by_robot[robot_name] = container
        self._robot_description_by_id[artifact.manifest.description_id] = container
        self._trace_robot_description(
            f"[SDK cache] stored robot='{robot_name}' "
            f"description_id='{str(getattr(artifact.manifest, 'description_id', '') or '')}' "
            f"cache_robots={len(self._robot_description_by_robot)} cache_ids={len(self._robot_description_by_id)}"
        )
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
        compass_enabled: Optional[bool] = None,
    ) -> Dict:
        """Build the HORUS MR registration payload for one robot."""
        from .registration_payload import build_robot_config_dict

        return build_robot_config_dict(
            self,
            robot,
            dataviz,
            global_visualizations=global_visualizations,
            workspace_scale=workspace_scale,
            compass_enabled=compass_enabled,
        )


    def check_backend_availability(self) -> bool:
        """Check if ROS is OK"""
        # With Pub/Sub we can't easily check receiver presence without Graph API
        # Assume OK if ROS is OK.
        return self.ros_initialized

    def __del__(self):
        if getattr(self, "ros_initialized", False) and getattr(self, "node", None):
            try:
                self.node.destroy_node()
            except Exception:
                logger.debug("Failed to destroy RobotRegistryClient node in destructor", exc_info=True)


_singleton_registry: Optional[RobotRegistryClient] = None


def get_robot_registry_client() -> RobotRegistryClient:
    global _singleton_registry
    if _singleton_registry is None:
        _singleton_registry = RobotRegistryClient()
    return _singleton_registry
