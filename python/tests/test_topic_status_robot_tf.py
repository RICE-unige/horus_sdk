import sys
import types

from horus.dataviz.dataviz import DataViz
from horus.robot.robot import Robot, RobotType
from horus.utils import topic_monitor, topic_status


class _FakeRegistry:
    def register_robot(self, robot, dataviz, timeout_sec: float = 10.0):
        return True, {"robot_id": "R1", "assigned_color": "#FF0000"}

    def unregister_robot(self, robot_id: str, timeout_sec: float = 10.0):
        return True, {"message": "ok"}


class _SpyBoard:
    def __init__(self):
        self.events = []

    def on_subscribe(self, topic: str):
        self.events.append(("sub", topic))

    def on_unsubscribe(self, topic: str):
        self.events.append(("unsub", topic))


class _SpyMonitor:
    def __init__(self):
        self.started = False
        self.watched = []
        self.unwatched = []

    def watch_topics(self, topics):
        self.watched.extend(topics)

    def start(self):
        self.started = True

    def unwatch_topics(self, topics, emit_unsubscribed: bool = False):
        self.unwatched.extend(topics)


def test_robot_tf_topic_status_non_tty(monkeypatch):
    monkeypatch.setattr("sys.stdout.isatty", lambda: False)
    monkeypatch.setattr(topic_status, "_singleton_board", None, raising=False)

    fake_registry_module = types.SimpleNamespace(RobotRegistryClient=_FakeRegistry)
    monkeypatch.setitem(
        sys.modules, "horus.bridge.robot_registry", fake_registry_module
    )

    spy_board = _SpyBoard()
    spy_monitor = _SpyMonitor()
    monkeypatch.setattr(topic_status, "get_topic_status_board", lambda: spy_board)
    monkeypatch.setattr(topic_monitor, "get_topic_monitor", lambda: spy_monitor)

    robot = Robot(name="drone_alpha", robot_type=RobotType.AERIAL)
    dataviz = DataViz(name="dv")
    dataviz.add_robot_transform(
        robot_name="drone_alpha", topic="/drone_alpha/tf", frame_id="world"
    )

    ok, _ = robot.register_with_horus(dataviz)
    assert ok
    ok2, _ = robot.unregister_from_horus()
    assert ok2

    assert ("sub", "/drone_alpha/tf") in spy_board.events
    assert ("unsub", "/drone_alpha/tf") in spy_board.events
    assert "/drone_alpha/tf" in spy_monitor.watched
    assert "/drone_alpha/tf" in spy_monitor.unwatched
    assert spy_monitor.started
