"""Observe TCP peers. This does not establish HORUS app or workspace readiness."""

import ipaddress
import logging
import subprocess
import threading
import time

logger = logging.getLogger(__name__)


def _endpoint(value):
    host, separator, port = value.rpartition(":")
    if not separator or not port.isdigit():
        return None
    host = host.strip("[]")
    try:
        host = str(ipaddress.ip_address(host))
    except ValueError:
        return None
    return host, int(port)


def _peers(output, port, *, netstat=False):
    peers = set()
    for line in output.splitlines():
        fields = line.split()
        if netstat:
            if len(fields) < 6 or fields[5] != "ESTABLISHED":
                continue
        elif len(fields) < 5 or fields[0] != "ESTAB":
            continue
        local, remote = _endpoint(fields[3]), _endpoint(fields[4])
        if local and remote and local[1] == port:
            peers.add(remote[0])
    return peers


class UnityConnectionMonitor:
    """Legacy API name retained; connected means an observed server-side TCP peer."""

    def __init__(self, unity_port=10000, disconnect_grace_s=3.0):
        self.unity_port = unity_port
        self.disconnect_grace_s = max(0.0, disconnect_grace_s)
        self.is_monitoring = False
        self.monitor_thread = None
        self.connection_callback = None
        self.connected_clients = set()
        self._last_seen = {}
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self.observation_available = False
        self._last_probe = None
        self.log_process = None

    def set_connection_callback(self, callback):
        self.connection_callback = callback

    def start_monitoring(self):
        if self.monitor_thread and self.monitor_thread.is_alive():
            return
        self._stop.clear()
        self.is_monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()

    def stop_monitoring(self):
        self._stop.set()
        self.is_monitoring = False
        if self.monitor_thread and self.monitor_thread is not threading.current_thread():
            self.monitor_thread.join(timeout=3.0)
        with self._lock:
            self.observation_available = False
            self.connected_clients.clear()
            self._last_seen.clear()

    def _monitor_loop(self):
        try:
            while not self._stop.is_set():
                self._monitor_network_connections()
                self._stop.wait(1.0)
        finally:
            self.is_monitoring = False

    def _probe(self, command, netstat=False):
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=1)
            if result.returncode == 0:
                return _peers(result.stdout, self.unity_port, netstat=netstat)
        except (OSError, subprocess.TimeoutExpired):
            logger.debug("TCP observation unavailable for %s", command[0], exc_info=True)
        return None

    def _monitor_network_connections(self):
        peers = self._probe(["ss", "-tn"])
        if peers is None:
            peers = self._probe(["netstat", "-tn"], netstat=True)
        self._update_observation(peers)

    def _monitor_with_netstat(self):
        self._update_observation(self._probe(["netstat", "-tn"], netstat=True))

    def _update_observation(self, peers):
        if self._stop.is_set():
            return
        now = time.monotonic()
        with self._lock:
            previous = self.connected_clients.copy()
            self.observation_available = peers is not None
            self._last_probe = now
            if peers is None:
                self._last_seen.clear()
                self.connected_clients.clear()
            else:
                self._last_seen.update((peer, now) for peer in peers)
                self._last_seen = {
                    peer: ts for peer, ts in self._last_seen.items()
                    if now - ts <= self.disconnect_grace_s
                }
                self.connected_clients = set(self._last_seen)
            changes = [(peer, True) for peer in self.connected_clients - previous]
            changes += [(peer, False) for peer in previous - self.connected_clients]
        for peer, connected in changes:
            if self.connection_callback:
                try:
                    self.connection_callback(peer, connected)
                except Exception:
                    logger.debug("TCP observation callback failed", exc_info=True)

    def get_connected_clients(self):
        with self._lock:
            if not self.observation_available or self._last_probe is None:
                return set()
            if time.monotonic() - self._last_probe > max(5.0, self.disconnect_grace_s):
                return set()
            return self.connected_clients.copy()

    def is_unity_connected(self):
        return bool(self.get_connected_clients())

    def get_current_connections_debug(self):
        try:
            result = subprocess.run(["ss", "-tn"], capture_output=True, text=True, timeout=1)
            return [line for line in result.stdout.splitlines() if _peers(line, self.unity_port)]
        except (OSError, subprocess.TimeoutExpired):
            return []

    def debug_connections(self):
        from .cli import print_info
        print_info(f"TCP peers on local port {self.unity_port}: {sorted(self.get_connected_clients())}")
