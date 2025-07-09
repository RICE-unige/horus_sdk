"""Unity MR Connection Monitor"""

import threading
import time
import subprocess
import re
import os
from typing import Optional, Callable


class UnityConnectionMonitor:
    """Monitor Unity MR connections through ROS TCP Endpoint logs"""

    def __init__(self, unity_port: int = 10000):
        self.unity_port = unity_port
        self.is_monitoring = False
        self.monitor_thread = None
        self.connection_callback: Optional[Callable[[str, bool], None]] = None
        self.connected_clients = set()
        self.log_process = None

    def set_connection_callback(self, callback: Callable[[str, bool], None]):
        """Set callback function for connection events

        Args:
            callback: Function called with (ip_address, is_connected)
        """
        self.connection_callback = callback

    def start_monitoring(self):
        """Start monitoring Unity connections"""
        if self.is_monitoring:
            return

        self.is_monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()

    def stop_monitoring(self):
        """Stop monitoring Unity connections"""
        self.is_monitoring = False
        if self.log_process:
            self.log_process.terminate()
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2)

    def _monitor_loop(self):
        """Main monitoring loop that monitors network connections"""
        while self.is_monitoring:
            try:
                # Monitor network connections directly
                self._monitor_network_connections()
            except Exception as e:
                if (
                    self.is_monitoring
                ):  # Only log if we're still supposed to be monitoring
                    pass  # Silent failure to avoid spam

            if self.is_monitoring:
                time.sleep(2)  # Check every 2 seconds

    def _monitor_network_connections(self):
        """Monitor network connections for Unity TCP Endpoint"""
        try:
            # Use ss command to get current connections on Unity port
            result = subprocess.run(
                ["ss", "-tn"], capture_output=True, text=True, timeout=5
            )

            if result.returncode == 0:
                current_connections = set()
                for line in result.stdout.split("\n"):
                    if "ESTAB" in line and f":{self.unity_port}" in line:
                        # Parse ss output: State Recv-Q Send-Q Local_Address:Port Peer_Address:Port
                        parts = line.split()
                        if len(parts) >= 5:
                            local_addr = parts[3]  # Local address:port
                            peer_addr = parts[4]  # Peer address:port

                            # Check if this is our Unity port (local side)
                            if f":{self.unity_port}" in local_addr and ":" in peer_addr:
                                ip = peer_addr.split(":")[0]
                                # Filter out localhost connections (likely from backend)
                                if ip not in ["127.0.0.1", "::1", "[::1]"]:
                                    current_connections.add(ip)

                # Check for new connections
                new_connections = current_connections - self.connected_clients
                for ip in new_connections:
                    self.connected_clients.add(ip)
                    if self.connection_callback:
                        self.connection_callback(ip, True)

                # Check for disconnections
                disconnections = self.connected_clients - current_connections
                for ip in disconnections:
                    self.connected_clients.discard(ip)
                    if self.connection_callback:
                        self.connection_callback(ip, False)

        except Exception:
            # Fallback to netstat if ss fails
            self._monitor_with_netstat()

    def _monitor_with_netstat(self):
        """Alternative monitoring using netstat command"""
        try:
            # Use netstat command as fallback
            result = subprocess.run(
                ["netstat", "-tn"], capture_output=True, text=True, timeout=5
            )

            if result.returncode == 0:
                current_connections = set()
                for line in result.stdout.split("\n"):
                    if f":{self.unity_port}" in line and "ESTABLISHED" in line:
                        parts = line.split()
                        if len(parts) >= 5:
                            foreign_addr = parts[4]
                            if ":" in foreign_addr:
                                ip = foreign_addr.split(":")[0]
                                # Filter out localhost connections
                                if ip not in ["127.0.0.1", "::1"]:
                                    current_connections.add(ip)

                # Update connections
                new_connections = current_connections - self.connected_clients
                for ip in new_connections:
                    self.connected_clients.add(ip)
                    if self.connection_callback:
                        self.connection_callback(ip, True)

                disconnections = self.connected_clients - current_connections
                for ip in disconnections:
                    self.connected_clients.discard(ip)
                    if self.connection_callback:
                        self.connection_callback(ip, False)

        except Exception:
            pass

    def get_current_connections_debug(self):
        """Get current connections using ss - for debugging"""
        try:
            result = subprocess.run(
                ["ss", "-tn"], capture_output=True, text=True, timeout=5
            )

            connections = []
            if result.returncode == 0:
                for line in result.stdout.split("\n"):
                    if line.strip() and ("ESTAB" in line or "State" in line):
                        connections.append(line.strip())
            return connections
        except Exception:
            return []

    def debug_connections(self):
        """Debug method to show current connections"""
        connections = self.get_current_connections_debug()
        print(
            f"\n\033[93mDEBUG: Current ss connections on port {self.unity_port}:\033[0m"
        )
        for conn in connections:
            print(f"  {conn}")
        print(f"\033[93mDEBUG: Tracked clients: {self.connected_clients}\033[0m\n")

    def get_connected_clients(self) -> set:
        """Get currently connected client IPs"""
        return self.connected_clients.copy()

    def is_unity_connected(self) -> bool:
        """Check if any Unity clients are connected"""
        return len(self.connected_clients) > 0
