import json
import threading
from types import SimpleNamespace

from horus.bridge import robot_registry
from horus.bridge.robot_registry import RobotRegistryClient


class _FakeString:
    def __init__(self):
        self.data = ""


class _AckingPublisher:
    def __init__(self, client):
        self.client = client
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg.data)
        payload = json.loads(msg.data)
        robot_name = payload["robot_name"]
        self.client._ack_by_robot[robot_name] = {
            "robot_name": robot_name,
            "success": True,
            "message": "Unregistered",
        }
        self.client._ack_received.set()


class _SilentPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg.data)


def _client_with_publisher(publisher):
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.node = object()
    client.publisher = publisher
    client.ros_initialized = True
    client._registration_lock = threading.Lock()
    client._ack_received = threading.Event()
    client._last_ack_data = {}
    client._ack_by_robot = {}
    client._ensure_bridge_running = lambda: True
    client._removed_description_cache = []
    client._remove_robot_description_cache = client._removed_description_cache.append
    if hasattr(publisher, "client"):
        publisher.client = client
    return client


def test_unregister_publishes_request_and_waits_for_ack(monkeypatch):
    monkeypatch.setattr(robot_registry, "String", _FakeString, raising=False)
    monkeypatch.setattr(
        robot_registry,
        "rclpy",
        SimpleNamespace(spin_once=lambda node, timeout_sec=0.0: None),
        raising=False,
    )

    placeholder = SimpleNamespace()
    publisher = _AckingPublisher(placeholder)
    client = _client_with_publisher(publisher)

    ok, result = client.unregister_robot("drone_alpha", timeout_sec=0.5)

    assert ok
    assert result["message"] == "Unregistered"
    assert client._removed_description_cache == ["drone_alpha"]
    assert json.loads(publisher.messages[0]) == {
        "action": "unregister",
        "robot_name": "drone_alpha",
    }


def test_unregister_times_out_without_ack(monkeypatch):
    monkeypatch.setattr(robot_registry, "String", _FakeString, raising=False)
    monkeypatch.setattr(
        robot_registry,
        "rclpy",
        SimpleNamespace(spin_once=lambda node, timeout_sec=0.0: None),
        raising=False,
    )

    publisher = _SilentPublisher()
    client = _client_with_publisher(publisher)

    ok, result = client.unregister_robot("drone_alpha", timeout_sec=0.01)

    assert not ok
    assert result["error"] == "Unregistration timeout"
    assert client._removed_description_cache == []
    assert json.loads(publisher.messages[0]) == {
        "action": "unregister",
        "robot_name": "drone_alpha",
    }
