import subprocess
import time
import os
import socket
from pathlib import Path

class BackendManager:
    def __init__(self, backend_type):
        self.backend_type = backend_type
        self.backend_process = None
        self.backend_configs = {
            'ros2': {
                'package': 'horus_backend',
                'launch_file': 'horus_complete_backend.launch.py',
                'check_command': 'ros2 pkg list | grep horus_backend',
                'launch_command': 'ros2 launch horus_backend horus_complete_backend.launch.py',
                'tcp_port': 8080
            },
            'ros1': {
                'package': 'horus_backend_ros1',
                'launch_file': 'horus_backend.launch',
                'check_command': 'rospack find horus_backend_ros1',
                'launch_command': 'roslaunch horus_backend_ros1 horus_backend.launch',
                'tcp_port': 8081
            }
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
            print(f"  \033[92m✓\033[0m Backend startup: \033[90mReady on port {config['tcp_port']}\033[0m")
        else:
            print(f"  \033[92m✓\033[0m Backend status: \033[90mAlready running\033[0m")
    
    def _is_backend_running(self):
        """Check if backend is already running"""
        config = self.backend_configs[self.backend_type]
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            result = sock.connect_ex(('localhost', config['tcp_port']))
            sock.close()
            return result == 0
        except:
            return False
    
    def _start_backend_process(self):
        """Start the backend process"""
        config = self.backend_configs[self.backend_type]
        
        # Launch in background
        self.backend_process = subprocess.Popen(
            config['launch_command'].split(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # Create new process group
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
    
    def get_port(self):
        """Get backend TCP port"""
        return self.backend_configs[self.backend_type]['tcp_port']
    
    def stop_backend(self):
        """Stop the backend process"""
        if self.backend_process:
            try:
                # Terminate the process group
                os.killpg(os.getpgid(self.backend_process.pid), 15)
                self.backend_process.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                # Force kill if graceful shutdown fails
                try:
                    os.killpg(os.getpgid(self.backend_process.pid), 9)
                except ProcessLookupError:
                    pass
            finally:
                self.backend_process = None
                print(f"\\033[90mBackend stopped\\033[0m")