"""Low-rate ROS graph monitoring. Endpoint presence is not message activity."""

import logging
import threading
import time

try:
    import rclpy
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

from .dashboard_status import topic_graph
from .topic_status import get_topic_status_board

logger = logging.getLogger(__name__)


class TopicSubscriptionMonitor:
    def __init__(self, debounce_s=2.0, poll_hz=2.0):
        self._debounce_s = max(0.0, debounce_s)
        self._poll_interval = 1.0 / max(poll_hz, 0.1)
        self._topics = set()
        self._topic_modes = {}
        self._lock = threading.RLock()
        self._running = False
        self._thread = None
        self._node = None
        self._stop = threading.Event()
        self._is_subscribed = {}
        self._zero_since = {}

    def start(self):
        if not ROS2_AVAILABLE:
            return
        with self._lock:
            if self._running or (self._thread and self._thread.is_alive()):
                return
            try:
                if not rclpy.ok():
                    rclpy.init()
                self._node = rclpy.create_node("horus_topic_subscription_monitor")
            except Exception:
                logger.debug("Failed to start topic monitor", exc_info=True)
                return
            self._stop.clear()
            self._running = True
            get_topic_status_board().start()
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._stop.set()
        self._running = False
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
        # The worker owns node teardown, even if a graph query outlives this wait.

    def watch_topics(self, topics, modes=None):
        with self._lock:
            for topic in topics:
                if not isinstance(topic, str) or not topic:
                    continue
                if topic not in self._topics:
                    self._topics.add(topic)
                    self._is_subscribed[topic] = False
                    self._zero_since[topic] = None
                    get_topic_status_board().on_unknown(topic)
                if modes and topic in modes:
                    self._topic_modes[topic] = (
                        "backend_pub" if modes[topic] in {"sdk_sub", "backend_pub"} else "backend_sub"
                    )
                else:
                    self._topic_modes.setdefault(topic, "backend_sub")

    def unwatch_topics(self, topics, emit_unsubscribed=False):
        with self._lock:
            for topic in topics:
                self._topics.discard(topic)
                self._topic_modes.pop(topic, None)
                self._is_subscribed.pop(topic, None)
                self._zero_since.pop(topic, None)
                if emit_unsubscribed:
                    get_topic_status_board().on_unsubscribe(topic)

    def _loop(self):
        try:
            while not self._stop.is_set():
                with self._lock:
                    topics = list(self._topics)
                for topic in topics:
                    if self._stop.is_set():
                        break
                    self._check_topic(topic, time.monotonic())
                self._stop.wait(self._poll_interval)
        finally:
            with self._lock:
                for topic in self._topics:
                    get_topic_status_board().on_unknown(topic)
                if self._node is not None:
                    try:
                        self._node.destroy_node()
                    except Exception:
                        logger.debug("Failed to destroy topic monitor node", exc_info=True)
                    self._node = None
                self._running = False

    def _check_topic(self, topic, now):
        with self._lock:
            if topic not in self._topics or self._node is None:
                return
            graph = topic_graph(self._node, topic, self._topic_modes.get(topic, "backend_sub"))
            state = graph["link"]
            board = get_topic_status_board()
            if state == "UNKNOWN":
                self._is_subscribed[topic] = False
                self._zero_since[topic] = None
                board.on_unknown(topic)
            elif state == "OK":
                self._is_subscribed[topic] = True
                self._zero_since[topic] = None
                board.on_subscribe(topic)
            else:
                # Debounce only disappearing sources. Missing bridge endpoints
                # or failed graph queries invalidate the observation immediately.
                if state == "NO_SOURCE" and self._is_subscribed.get(topic):
                    since = self._zero_since.get(topic)
                    if since is None:
                        self._zero_since[topic] = now
                        return
                    if now - since < self._debounce_s:
                        return
                self._is_subscribed[topic] = False
                self._zero_since[topic] = None
                board.on_unsubscribe(topic)


_singleton_monitor = None
_singleton_lock = threading.Lock()


def get_topic_monitor():
    global _singleton_monitor
    with _singleton_lock:
        if _singleton_monitor is None:
            _singleton_monitor = TopicSubscriptionMonitor()
        return _singleton_monitor
