import subprocess
import time
import os
import socket


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
            "ros1": {
                "package": "horus_backend_ros1",
                "launch_file": "horus_backend.launch",
                "check_command": "rospack find horus_backend_ros1",
                "launch_command": "roslaunch horus_backend_ros1 horus_backend.launch",
                "tcp_port": 8081,
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
        """Check if backend is already running"""
        config = self.backend_configs[self.backend_type]
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            result = sock.connect_ex(("localhost", config["tcp_port"]))
            sock.close()
            return result == 0
        except Exception:
            return False

    def _is_unity_endpoint_running(self):
        """Check if Unity TCP endpoint is running"""
        config = self.backend_configs[self.backend_type]
        unity_port = config.get("unity_port", 10000)
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            result = sock.connect_ex(("localhost", unity_port))
            sock.close()
            return result == 0
        except Exception:
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
        """Test Unity TCP endpoint connection"""
        return self._is_unity_endpoint_running()

    def get_unity_port(self):
        """Get Unity TCP endpoint port"""
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

        # Additionally, ensure all HORUS-related processes are stopped
        self._cleanup_horus_processes()

        print("\033[90m  ✓ All HORUS backend processes stopped\033[0m")

    def _cleanup_horus_processes(self):
        """Cleanup any remaining HORUS-related ROS2 processes"""
        processes_to_kill = [
            "horus_backend_node",
            "default_server_endpoint",
            "ros_tcp_endpoint",
        ]

        for process_name in processes_to_kill:
            try:
                # Use pkill to find and terminate processes by name
                result = subprocess.run(
                    ["pkill", "-f", process_name], capture_output=True, timeout=3
                )
                if result.returncode == 0:
                    print(f"\033[90m  ✓ Terminated {process_name} processes\033[0m")
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass

        # Give processes time to cleanup
        time.sleep(1)

        # Double-check by killing any processes using our ports
        self._cleanup_port_processes()

    def _cleanup_port_processes(self):
        """Kill any processes using HORUS ports"""
        ports_to_cleanup = [8080, 10000]  # Backend and Unity ports

        for port in ports_to_cleanup:
            try:
                # Find processes using the port
                result = subprocess.run(
                    ["lsof", "-t", f"-i:{port}"],
                    capture_output=True,
                    text=True,
                    timeout=3,
                )

                if result.returncode == 0 and result.stdout.strip():
                    pids = result.stdout.strip().split("\n")
                    for pid in pids:
                        try:
                            subprocess.run(["kill", "-TERM", pid], timeout=2)
                            print(
                                f"\033[90m  ✓ Terminated process {pid} "
                                f"on port {port}\033[0m"
                            )
                        except (subprocess.TimeoutExpired, FileNotFoundError):
                            pass
            except (subprocess.TimeoutExpired, FileNotFoundError):
                # lsof might not be available, that's okay
                pass
