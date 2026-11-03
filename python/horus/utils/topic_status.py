"""Deduplicated subscription observations, separate from data/connection health."""

import threading
import time
from collections import OrderedDict
from dataclasses import dataclass


@dataclass
class _TopicState:
    state: str
    last_changed: float


class TopicStatusBoard:
    """Thread-safe observations; the connection dashboard owns terminal redraws."""

    def __init__(self, max_hz=10.0):
        self._topics = OrderedDict()
        self._lock = threading.RLock()
        self._silent = False
        self._stopped = False

    @property
    def is_silent(self):
        with self._lock:
            return self._silent

    def start(self):
        with self._lock:
            self._stopped = False

    def stop(self):
        with self._lock:
            self._stopped = True

    def set_silent(self, silent):
        with self._lock:
            self._silent = bool(silent)

    def on_subscribe(self, topic):
        self._update(topic, "SUBSCRIBED")

    def on_unsubscribe(self, topic):
        self._update(topic, "UNSUBSCRIBED")

    def on_unknown(self, topic):
        self._update(topic, "UNKNOWN")

    def snapshot(self):
        with self._lock:
            return OrderedDict((topic, item.state) for topic, item in self._topics.items())

    def _update(self, topic, state):
        with self._lock:
            previous = self._topics.get(topic)
            if previous and previous.state == state:
                return
            self._topics[topic] = _TopicState(state, time.monotonic())
            if self._silent or self._stopped:
                return
            # No raw ANSI cursor control or competing live-render thread.
            from .cli import print_info
            print_info(f"Topic: {topic} | {state.capitalize()}")


_singleton_board = None
_singleton_lock = threading.Lock()


def get_topic_status_board():
    global _singleton_board
    with _singleton_lock:
        if _singleton_board is None:
            _singleton_board = TopicStatusBoard()
        return _singleton_board
