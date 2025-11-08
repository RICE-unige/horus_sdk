"""Monitor /rosout for UnityEndpoint log messages to track topic subscription status."""

import re
import threading
import time
from typing import Dict, Optional, Set

try:
    import rclpy
    from rcl_interfaces.msg import Log
    from rclpy.node import Node

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

from .topic_status import get_topic_status_board


class RosoutSubscriptionMonitor:
    """
    Monitor /rosout for UnityEndpoint log messages to detect topic subscription/unsubscription.

    Subscribes: "RegisterSubscriber(<topic>, ...) OK" from UnityEndpoint logger
    Unsubscribes: "Disconnected from <ip>" from UnityEndpoint logger (marks all topics for that IP as unsubscribed)
    """

    def __init__(self):
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._node: Optional[Node] = None
        self._lock = threading.RLock()

        # Track sessions: ip -> set of topics
        self._sessions: Dict[str, Set[str]] = {}
        # Track topic states
        self._topic_states: Dict[str, bool] = {}  # topic -> is_subscribed

        # Regex patterns for parsing UnityEndpoint logs
        self._connection_pattern = re.compile(r"Connection from (.+)")
        self._register_pattern = re.compile(
            r"RegisterSubscriber\(([^,]+),\s*([^)]+)\)\s*OK"
        )
        self._disconnect_pattern = re.compile(r"Disconnected from (.+)")

    def start(self):
        """Start monitoring /rosout."""
        if not ROS2_AVAILABLE:
            return
        if self._running:
            return

        # Check for existing monitor node (singleton enforcement)
        try:
            if rclpy.ok():
                temp_node = rclpy.create_node("_rosout_monitor_checker")
                existing_nodes = temp_node.get_node_names()
                temp_node.destroy_node()
                if "horus_rosout_monitor" in existing_nodes:
                    # Monitor already running, don't start another
                    return
        except Exception:
            pass

        self._running = True

        # Initialize ROS2 if needed
        if not rclpy.ok():
            try:
                rclpy.init()
            except Exception:
                self._running = False
                return

        # Create node and subscription
        try:
            self._node = rclpy.create_node("horus_rosout_monitor")
            self._rosout_sub = self._node.create_subscription(
                Log, "/rosout", self._on_rosout_message, 10
            )
        except Exception:
            self._running = False
            if self._node:
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
                self._node = None
            return

        # Start spinning in background thread
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop monitoring."""
        self._running = False

        # Mark all tracked topics as unsubscribed
        with self._lock:
            for topic in list(self._topic_states.keys()):
                if self._topic_states.get(topic, False):
                    self._topic_states[topic] = False
                    try:
                        get_topic_status_board().on_unsubscribe(topic)
                    except Exception:
                        pass

        if self._thread:
            self._thread.join(timeout=0.5)

        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None

    def _spin_loop(self):
        """Background thread to spin the node."""
        while self._running and self._node is not None:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception:
                pass

    def _on_rosout_message(self, msg: "Log"):
        """Handle incoming rosout messages."""
        # Only process messages from UnityEndpoint logger
        if msg.name != "UnityEndpoint":
            return

        message = msg.msg

        with self._lock:
            # Check for connection
            connection_match = self._connection_pattern.match(message)
            if connection_match:
                ip = connection_match.group(1)
                if ip not in self._sessions:
                    self._sessions[ip] = set()
                return

            # Check for RegisterSubscriber
            register_match = self._register_pattern.match(message)
            if register_match:
                topic = register_match.group(1)
                # Find the last connected IP (simple heuristic)
                # In production, might need more sophisticated session tracking
                if self._sessions:
                    # Get most recent session (last in dict)
                    last_ip = list(self._sessions.keys())[-1]
                    self._sessions[last_ip].add(topic)

                    # Update topic state
                    if not self._topic_states.get(topic, False):
                        self._topic_states[topic] = True
                        try:
                            get_topic_status_board().on_subscribe(topic)
                        except Exception:
                            pass
                return

            # Check for disconnection
            disconnect_match = self._disconnect_pattern.match(message)
            if disconnect_match:
                ip = disconnect_match.group(1)
                if ip in self._sessions:
                    # Mark all topics from this session as unsubscribed
                    for topic in self._sessions[ip]:
                        if self._topic_states.get(topic, False):
                            self._topic_states[topic] = False
                            try:
                                get_topic_status_board().on_unsubscribe(topic)
                            except Exception:
                                pass
                    # Clear the session
                    del self._sessions[ip]
                return


_singleton_monitor: Optional[RosoutSubscriptionMonitor] = None


def get_rosout_monitor() -> RosoutSubscriptionMonitor:
    """Get singleton rosout monitor instance."""
    global _singleton_monitor
    if _singleton_monitor is None:
        _singleton_monitor = RosoutSubscriptionMonitor()
    return _singleton_monitor
