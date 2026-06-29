import os
import logging
import re
import socket
import subprocess
import time

logger = logging.getLogger(__name__)


class BackendManager:
    def __init__(self, backend_type):
        self.backend_type = backend_type
        self.backend_process = None
        self._shutdown_called = False  # Prevent duplicate shutdowns
        self.backend_configs = {
            "ros2": {
                "package": "horus_backend",
                "launch_file": "horus_complete_backend.launch.py",
                "check_command": "ros2 pkg list | grep horus_backend",
                "launch_command": (
                    "ros2 launch horus_backend " "horus_complete_backend.launch.py"
                ),
                "tcp_port": 8080,
                "unity_port": 10000,
            },
        }

    def launch_backend(self):
        """Launch the appropriate backend"""
        from .spinner import Spinner

        if not self._is_backend_running():
            spinner = Spinner(f"Starting {self.backend_type.upper()} backend")
            spinner.start()

            self._start_backend_process()
            self._wait_for_backend_ready_silent()

            spinner.stop()
            config = self.backend_configs[self.backend_type]
            print(
                f"  \033[92m✓\033[0m Backend startup: "
                f"\033[90mReady on port {config['tcp_port']}\033[0m"
            )
        else:
            print("  \033[92m✓\033[0m Backend status: \033[90mAlready running\033[0m")

    def _is_backend_running(self):
        """Check if the HORUS backend is already running."""
        config = self.backend_configs[self.backend_type]
        port = int(config["tcp_port"])
        if self.backend_process is not None and self.backend_process.poll() is None:
            return self._is_port_open(port)

        if not self._is_port_open(port):
            return False

        return self._port_owner_looks_like_horus(port, config)

    def _is_port_open(self, port):
        try:
            with socket.create_connection(("localhost", int(port)), timeout=0.25):
                return True
        except Exception:
            logger.debug("Port probe failed for localhost:%s", port, exc_info=True)
            return False

    def _port_owner_looks_like_horus(self, port, config):
        pids = self._list_listening_pids(port)
        if not pids:
            return False

        markers = {
            str(config.get("package") or ""),
            str(config.get("launch_file") or ""),
            "horus_backend",
            "horus_unity_bridge",
        }
        markers = {marker for marker in markers if marker}

        for pid in pids:
            cmdline = self._read_process_cmdline(pid)
            if any(marker in cmdline for marker in markers):
                return True

        return False

    def _list_listening_pids(self, port):
        pids = set()
        try:
            result = subprocess.run(
                ["ss", "-H", "-ltnp", f"sport = :{int(port)}"],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=1.0,
            )
            pids.update(int(match) for match in re.findall(r"pid=(\d+)", result.stdout or ""))
        except Exception:
            logger.debug("ss listener lookup failed for port %s", port, exc_info=True)

        if not pids:
            try:
                result = subprocess.run(
                    ["lsof", "-t", f"-iTCP:{int(port)}", "-sTCP:LISTEN"],
                    check=False,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    timeout=1.0,
                )
                for line in (result.stdout or "").splitlines():
                    line = line.strip()
                    if line.isdigit():
                        pids.add(int(line))
            except Exception:
                logger.debug("lsof listener lookup failed for port %s", port, exc_info=True)

        return pids

    def _read_process_cmdline(self, pid):
        try:
            with open(f"/proc/{int(pid)}/cmdline", "rb") as handle:
                raw = handle.read()
            return raw.replace(b"\x00", b" ").decode("utf-8", errors="replace")
        except Exception:
            logger.debug("Failed to read process command line for pid %s", pid, exc_info=True)
            return ""

    def _is_unity_endpoint_running(self):
        """Check if Unity bridge is running"""
        config = self.backend_configs[self.backend_type]
        unity_port = config.get("unity_port", 10000)
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            result = sock.connect_ex(("localhost", unity_port))
            sock.close()
            return result == 0
        except Exception:
            logger.debug("Unity endpoint probe failed", exc_info=True)
            return False

    def _start_backend_process(self):
        """Start the backend process"""
        config = self.backend_configs[self.backend_type]

        # Launch in background
        launch_command = config["launch_command"]
        assert isinstance(launch_command, str), "launch_command must be a string"
        self.backend_process = subprocess.Popen(
            launch_command.split(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,  # Create new process group
        )

    def _wait_for_backend_ready(self):
        """Wait for backend to be ready"""
        config = self.backend_configs[self.backend_type]
        print("   Waiting for backend to be ready", end="")

        max_attempts = 30
        for attempt in range(max_attempts):
            if self._is_backend_running():
                print(" ✅")
                print(f"   Backend ready on port {config['tcp_port']}")
                return

            print(".", end="", flush=True)
            time.sleep(1)

        print(" ❌")
        raise RuntimeError(f"Backend failed to start after {max_attempts} seconds")

    def _wait_for_backend_ready_silent(self):
        """Wait for backend to be ready without output"""
        max_attempts = 30
        for attempt in range(max_attempts):
            if self._is_backend_running():
                return
            time.sleep(1)

        raise RuntimeError(f"Backend failed to start after {max_attempts} seconds")

    def test_connection(self):
        """Test connection to backend"""
        return self._is_backend_running()

    def test_unity_endpoint(self):
        """Test Unity bridge connection"""
        return self._is_unity_endpoint_running()

    def get_unity_port(self):
        """Get Unity bridge TCP port"""
        return self.backend_configs[self.backend_type].get("unity_port", 10000)

    def get_port(self):
        """Get backend TCP port"""
        return self.backend_configs[self.backend_type]["tcp_port"]

    def stop_backend(self):
        """Stop the backend process and all related ROS2 nodes"""
        # Prevent duplicate shutdown calls
        if self._shutdown_called:
            return
        self._shutdown_called = True

        print("\033[90mStopping HORUS backend processes...\033[0m")

        # First, try to stop our launched process gracefully
        if self.backend_process:
            try:
                # Send SIGTERM to the process group (this will stop both
                # backend and unity endpoint)
                os.killpg(os.getpgid(self.backend_process.pid), 15)
                self.backend_process.wait(timeout=8)
                print("\033[90m  ✓ Launch process stopped gracefully\033[0m")
            except (ProcessLookupError, subprocess.TimeoutExpired):
                # Force kill if graceful shutdown fails
                try:
                    os.killpg(os.getpgid(self.backend_process.pid), 9)
                    print("\033[90m  ✓ Launch process force-killed\033[0m")
                except ProcessLookupError:
                    pass
            finally:
                self.backend_process = None

        if self._force_external_cleanup_enabled():
            print(
                "\033[90m  • Ignored HORUS_SDK_FORCE_EXTERNAL_CLEANUP; "
                "cleanup is limited to SDK-owned child processes\033[0m"
            )

        print("\033[90m  ✓ All HORUS backend processes stopped\033[0m")

    def _force_external_cleanup_enabled(self):
        value = os.environ.get("HORUS_SDK_FORCE_EXTERNAL_CLEANUP", "")
        return value.strip().lower() in {"1", "true", "yes", "on"}
