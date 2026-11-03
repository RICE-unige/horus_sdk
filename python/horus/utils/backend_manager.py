import os
import logging
import re
import shlex
import signal
import socket
import subprocess
import threading
import time
from collections import deque

logger = logging.getLogger(__name__)


class BackendManager:
    def __init__(self, backend_type):
        self.backend_type = backend_type
        self.backend_process = None
        self._output_tail = deque(maxlen=128)
        self._output_lock = threading.Lock()
        self._output_reader = None
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
        if backend_type not in self.backend_configs:
            raise ValueError(f"Unsupported backend type: {backend_type}")

    def launch_backend(self):
        """Launch the appropriate backend"""
        from .spinner import Spinner
        from . import cli

        if not self._is_backend_running():
            spinner = Spinner(f"Starting {self.backend_type.upper()} backend")
            spinner.start()

            try:
                self._start_backend_process()
                self._wait_for_backend_ready_silent()
            except Exception as exc:
                self.stop_backend()
                tail = self._backend_output_tail()
                detail = f"\nBackend output:\n{tail}" if tail else ""
                raise RuntimeError(f"Backend startup failed: {exc}{detail}") from exc
            finally:
                spinner.stop()
            config = self.backend_configs[self.backend_type]
            cli.print_success(f"Backend listener ready on port {config['tcp_port']}")
        else:
            cli.print_info("Existing HORUS backend listener detected")

    def _is_backend_running(self):
        """Check if the HORUS backend is already running."""
        config = self.backend_configs[self.backend_type]
        port = int(config["tcp_port"])
        if self.backend_process is not None:
            if self.backend_process.poll() is not None or not self._is_port_open(port):
                return False
            for pid in self._list_listening_pids(port):
                try:
                    if os.getpgid(pid) == self.backend_process.pid:
                        return True
                except (ProcessLookupError, PermissionError):
                    continue
            return False

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
        return self._is_port_open(unity_port)

    def _start_backend_process(self):
        """Start the backend process"""
        config = self.backend_configs[self.backend_type]

        # Launch in background
        launch_command = config["launch_command"]
        assert isinstance(launch_command, str), "launch_command must be a string"
        self._shutdown_called = False
        with self._output_lock:
            self._output_tail.clear()
        self.backend_process = subprocess.Popen(
            shlex.split(launch_command),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        # Drain continuously; an unread pipe eventually blocks the ROS process.
        self._output_reader = threading.Thread(
            target=self._drain_backend_output,
            args=(self.backend_process.stdout,),
            name="horus-backend-output",
            daemon=True,
        )
        self._output_reader.start()

    def _drain_backend_output(self, stream):
        try:
            while True:
                chunk = stream.read1(4096)
                if not chunk:
                    break
                with self._output_lock:
                    self._output_tail.append(chunk)
        except (OSError, ValueError):
            logger.debug("Backend output reader stopped", exc_info=True)
        finally:
            stream.close()

    def _backend_output_tail(self):
        with self._output_lock:
            output = b"".join(self._output_tail)
        return output[-8192:].decode("utf-8", errors="replace").strip()

    def _wait_for_backend_ready(self):
        """Wait for backend to be ready"""
        from . import cli
        config = self.backend_configs[self.backend_type]
        with cli.status("Waiting for backend listener"):
            self._wait_for_backend_ready_silent()
        cli.print_success(f"Backend listener ready on port {config['tcp_port']}")

    def _wait_for_backend_ready_silent(self):
        """Wait for backend to be ready without output"""
        max_attempts = 30
        for attempt in range(max_attempts):
            if self._is_backend_running():
                return
            if self.backend_process is not None and self.backend_process.poll() is not None:
                raise RuntimeError(f"Backend exited with code {self.backend_process.returncode}")
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
        """Stop SDK-owned backend processes without touching external services."""
        from . import cli
        # Prevent duplicate shutdown calls
        if self._shutdown_called:
            return
        self._shutdown_called = True

        cli.print_info("Stopping SDK-owned backend processes")

        # First, try to stop our launched process gracefully
        if self.backend_process:
            try:
                # Send SIGTERM to the process group (this will stop both
                # backend and unity endpoint)
                os.killpg(self.backend_process.pid, signal.SIGTERM)
                self.backend_process.wait(timeout=8)
                cli.print_success("SDK-owned launch process stopped gracefully")
            except (ProcessLookupError, subprocess.TimeoutExpired):
                # Force kill if graceful shutdown fails
                try:
                    os.killpg(self.backend_process.pid, signal.SIGKILL)
                    cli.print_warning("Sent forced stop to SDK-owned process group")
                except ProcessLookupError:
                    pass
                self.backend_process.wait(timeout=3)
            finally:
                self.backend_process = None
                if self._output_reader is not None:
                    self._output_reader.join(timeout=2)
                    self._output_reader = None

        if self._force_external_cleanup_enabled():
            cli.print_warning(
                "Ignored HORUS_SDK_FORCE_EXTERNAL_CLEANUP; "
                "cleanup is limited to SDK-owned child processes"
            )

        cli.print_info("Backend cleanup finished; externally managed services were left running")

    def _force_external_cleanup_enabled(self):
        value = os.environ.get("HORUS_SDK_FORCE_EXTERNAL_CLEANUP", "")
        return value.strip().lower() in {"1", "true", "yes", "on"}
