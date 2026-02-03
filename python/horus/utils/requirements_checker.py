import os
import shutil
import socket
import subprocess


class RequirementsChecker:
    def __init__(self):
        self.backend_requirements = {
            "ros2": [
                ("ROS2 Installation", self._check_ros2_installation),
                ("HORUS Backend Package", self._check_horus_backend_package),
                ("Network Port 8080", self._check_port_availability),
                ("Unity Bridge (horus_unity_bridge)", self._check_unity_endpoint),
            ],
        }

    def get_backend_checks(self, backend_type):
        """Get the list of checks for a backend type"""
        if backend_type not in self.backend_requirements:
            return [
                (
                    "Invalid Backend",
                    lambda bt: (False, f"Unknown backend: {backend_type}"),
                )
            ]
        return self.backend_requirements[backend_type]

    def check_backend(self, backend_type):
        """Check all requirements for specified backend"""
        requirements = {}

        if backend_type not in self.backend_requirements:
            return {
                "Invalid Backend": {
                    "available": False,
                    "message": f"Unknown backend: {backend_type}",
                }
            }

        for req_name, check_func in self.backend_requirements[backend_type]:
            try:
                available, message = check_func(backend_type)
                requirements[req_name] = {"available": available, "message": message}
            except Exception as e:
                requirements[req_name] = {
                    "available": False,
                    "message": f"Check failed: {str(e)}",
                }

        return requirements

    def _check_ros2_installation(self, backend_type):
        """Check if ROS2 is installed"""
        if shutil.which("ros2") and "ROS_DISTRO" in os.environ:
            distro = os.environ.get("ROS_DISTRO", "unknown")
            return True, f"ROS2 {distro} detected"
        return (
            False,
            "ROS2 not found - install with: sudo apt install ros-humble-desktop",
        )



    def _check_horus_backend_package(self, backend_type):
        """Check if HORUS ROS2 backend package is installed"""
        try:
            result = subprocess.run(
                ["ros2", "pkg", "list"], capture_output=True, text=True, timeout=5
            )
            if "horus_backend" in result.stdout:
                return True, "HORUS backend package found"
            return False, "Install with: sudo apt install ros-humble-horus-backend"
        except Exception:
            return False, "Unable to check package list"



    def _check_port_availability(self, backend_type):
        """Check if required port is available"""
        port = 8080
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            result = sock.connect_ex(("localhost", port))
            sock.close()

            if result == 0:
                return True, f"Port {port} is in use (backend may be running)"
            return True, f"Port {port} is available"
        except Exception:
            return False, f"Unable to check port {port}"

    def _check_unity_endpoint(self, backend_type):
        """Check if Unity bridge is running on port 10000"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)  # 1 second timeout
            result = sock.connect_ex(("localhost", 10000))
            sock.close()

            if result == 0:
            return True, "Unity bridge is running on port 10000"
        return (
            True,
            "Unity bridge not detected (will be started automatically)",
        )
        except Exception:
            return False, "Unable to check Unity bridge"
