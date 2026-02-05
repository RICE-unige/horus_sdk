import threading
import time
from typing import Dict, List, Optional, Set

try:
    import rclpy
    from rclpy.node import Node

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

from .topic_status import get_topic_status_board


class TopicSubscriptionMonitor:
    """
    Monitor ROS2 graph to detect when the Unity bridge has a live
    subscriber for a topic AND that topic has >=1 publishers.

    SUBSCRIBED: backend subscriber present AND publishers > 0
    UNSUBSCRIBED: subscriber destroyed OR publishers==0 for >= debounce_s OR unwatch/teardown
    """

    def __init__(self, debounce_s: float = 2.0, poll_hz: float = 2.0):
        self._debounce_s = debounce_s
        self._poll_interval = 1.0 / max(poll_hz, 0.1)
        self._topics: Set[str] = set()
        self._topic_modes: Dict[str, str] = {}
        self._lock = threading.RLock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._node: Optional[Node] = None
        # per-topic state
        self._is_subscribed: Dict[str, bool] = {}
        self._zero_since: Dict[str, Optional[float]] = {}

    def start(self):
        if not ROS2_AVAILABLE:
            return
        if self._running:
            return
        self._running = True
        if not (rclpy.ok() if ROS2_AVAILABLE else False):
            try:
                rclpy.init()
            except Exception:
                self._running = False
                return
        if self._node is None:
            try:
                self._node = rclpy.create_node("horus_topic_subscription_monitor")
            except Exception:
                self._running = False
                return
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=0.5)
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None

    def watch_topics(self, topics: List[str], modes: Optional[Dict[str, str]] = None):
        with self._lock:
            for t in topics:
                if not isinstance(t, str) or not t:
                    continue
                if t not in self._topics:
                    self._topics.add(t)
                    self._is_subscribed.setdefault(t, False)
                    self._zero_since.setdefault(t, None)
                if modes and t in modes:
                    mode = modes[t]
                    if mode == "sdk_sub":
                        self._topic_modes[t] = "backend_pub"
                    else:
                        self._topic_modes[t] = "backend_sub"
                else:
                    self._topic_modes.setdefault(t, "backend_sub")

    def unwatch_topics(self, topics: List[str], emit_unsubscribed: bool = False):
        with self._lock:
            for t in topics:
                if t in self._topics:
                    self._topics.remove(t)
                if emit_unsubscribed and self._is_subscribed.get(t, False):
                    # Emit UNSUBSCRIBED immediately on unwatch
                    try:
                        get_topic_status_board().on_unsubscribe(t)
                    except Exception:
                        pass
                self._is_subscribed.pop(t, None)
                self._zero_since.pop(t, None)
                self._topic_modes.pop(t, None)

    def _loop(self):
        while self._running and self._node is not None:
            topics_snapshot: List[str]
            with self._lock:
                topics_snapshot = list(self._topics)
            now = time.time()
            for topic in topics_snapshot:
                try:
                    self._check_topic(topic, now)
                except Exception:
                    # Avoid noisy logs
                    pass
            time.sleep(self._poll_interval)

    def _check_topic(self, topic: str, now: float):
        assert self._node is not None
        mode = self._topic_modes.get(topic, "backend_sub")

        def _is_backend_node(node_name: Optional[str]) -> bool:
            if not node_name:
                return False
            if node_name in ("horus_backend_node", "horus_backend", "horus_unity_bridge"):
                return True
            if node_name.startswith("horus_unity_bridge"):
                return True
            if "horus_backend" in node_name:
                return True
            if node_name.endswith("_RosSubscriber"):
                return True
            return False

        if mode == "backend_pub":
            pub_infos = self._node.get_publishers_info_by_topic(topic)
            has_backend_pub = any(_is_backend_node(info.node_name) for info in pub_infos)
            pub_count = len(pub_infos)

            currently_subscribed = self._is_subscribed.get(topic, False)
            if has_backend_pub:
                if not currently_subscribed:
                    self._is_subscribed[topic] = True
                    self._zero_since[topic] = None
                    try:
                        get_topic_status_board().on_subscribe(topic)
                    except Exception:
                        pass
                else:
                    self._zero_since[topic] = None
                return

            # No backend publisher
            if currently_subscribed:
                zs = self._zero_since.get(topic)
                if zs is None:
                    self._zero_since[topic] = now
                elif (now - zs) >= self._debounce_s:
                    self._is_subscribed[topic] = False
                    self._zero_since[topic] = None
                    try:
                        get_topic_status_board().on_unsubscribe(topic)
                    except Exception:
                        pass
            else:
                self._zero_since[topic] = None
            return

        # backend_sub mode: check backend subscriber presence and publisher count
        sub_infos = self._node.get_subscriptions_info_by_topic(topic)
        has_backend_sub = any(_is_backend_node(info.node_name) for info in sub_infos)
        pub_infos = self._node.get_publishers_info_by_topic(topic)
        pub_count = len(pub_infos)

        currently_subscribed = self._is_subscribed.get(topic, False)

        if has_backend_sub and pub_count > 0:
            # Transition to SUBSCRIBED
            if not currently_subscribed:
                self._is_subscribed[topic] = True
                self._zero_since[topic] = None
                try:
                    get_topic_status_board().on_subscribe(topic)
                except Exception:
                    pass
            else:
                self._zero_since[topic] = None
            return

        # Not meeting SUBSCRIBED criteria
        if currently_subscribed:
            # If backend subscriber disappeared -> immediate UNSUBSCRIBED
            if not has_backend_sub:
                self._is_subscribed[topic] = False
                self._zero_since[topic] = None
                try:
                    get_topic_status_board().on_unsubscribe(topic)
                except Exception:
                    pass
                return
            # Backend subscriber still present but publishers == 0 -> debounce
            if pub_count == 0:
                zs = self._zero_since.get(topic)
                if zs is None:
                    self._zero_since[topic] = now
                elif (now - zs) >= self._debounce_s:
                    self._is_subscribed[topic] = False
                    self._zero_since[topic] = None
                    try:
                        get_topic_status_board().on_unsubscribe(topic)
                    except Exception:
                        pass
        else:
            # Not subscribed, reset timer
            self._zero_since[topic] = None


_singleton_monitor: Optional[TopicSubscriptionMonitor] = None


def get_topic_monitor() -> TopicSubscriptionMonitor:
    global _singleton_monitor
    if _singleton_monitor is None:
        _singleton_monitor = TopicSubscriptionMonitor()
    return _singleton_monitor
