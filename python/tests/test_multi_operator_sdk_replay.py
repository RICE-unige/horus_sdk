"""Tests for multi-operator SDK registration replay publishing."""

import json
import types

from horus.bridge.robot_registry import RobotRegistryClient
from horus.bridge import robot_registry as robot_registry_module


class _FakeString:
    def __init__(self):
        self.data = ""


class _SpyPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(getattr(msg, "data", ""))


def _build_client(monkeypatch):
    monkeypatch.setattr(robot_registry_module, "String", _FakeString, raising=False)
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = True
    client.node = object()
    client.sdk_replay_begin_publisher = _SpyPublisher()
    client.sdk_replay_item_publisher = _SpyPublisher()
    client.sdk_replay_end_publisher = _SpyPublisher()
    client._sdk_replay_burst_attempts = 1
    client._sdk_replay_burst_initial_delay_s = 0.0
    client._sdk_replay_burst_inter_attempt_delay_s = 0.0
    return client


def _make_entries(count):
    entries = []
    for idx in range(count):
        payload = json.dumps({"robot_name": f"bot_{idx + 1}"})
        entries.append((None, None, types.SimpleNamespace(data=payload)))
    return entries


def _decode_messages(messages):
    return [json.loads(raw) for raw in messages]


def test_publish_sdk_registry_replay_once_emits_begin_items_end(monkeypatch):
    client = _build_client(monkeypatch)
    entries = _make_entries(2)
    replay_request = {
        "request_id": "req-123",
        "join_attempt_id": "join-abc",
        "requester_app_id": "joiner-app",
    }

    ok, result = client._publish_sdk_registry_replay_once(entries, replay_request)

    assert ok is True
    assert result["request_id"] == "req-123"
    assert result["join_attempt_id"] == "join-abc"
    assert result["requester_app_id"] == "joiner-app"
    assert result["expected_count"] == 2
    assert result["published_count"] == 2

    begin = _decode_messages(client.sdk_replay_begin_publisher.messages)
    items = _decode_messages(client.sdk_replay_item_publisher.messages)
    end = _decode_messages(client.sdk_replay_end_publisher.messages)

    assert len(begin) == 1
    assert begin[0]["request_id"] == "req-123"
    assert begin[0]["join_attempt_id"] == "join-abc"
    assert begin[0]["expected_count"] == 2

    assert len(items) == 2
    assert [item["index"] for item in items] == [0, 1]
    assert all(item["request_id"] == "req-123" for item in items)
    assert all(item["join_attempt_id"] == "join-abc" for item in items)

    assert len(end) == 1
    assert end[0]["request_id"] == "req-123"
    assert end[0]["join_attempt_id"] == "join-abc"
    assert end[0]["expected_count"] == 2
    assert end[0]["published_count"] == 2


def test_publish_sdk_registry_replay_burst_reuses_request_id_across_attempts(monkeypatch):
    client = _build_client(monkeypatch)
    client._sdk_replay_burst_attempts = 3
    client._sdk_replay_burst_initial_delay_s = 0.15
    client._sdk_replay_burst_inter_attempt_delay_s = 0.25

    sleep_calls = []
    monkeypatch.setattr(robot_registry_module.time, "sleep", lambda seconds: sleep_calls.append(seconds))

    ok, result = client._publish_sdk_registry_replay(
        _make_entries(2),
        {
            "request_id": "req-fixed",
            "join_attempt_id": "join-fixed",
            "requester_app_id": "joiner-app",
        },
    )

    assert ok is True
    assert result["request_id"] == "req-fixed"
    assert result["join_attempt_id"] == "join-fixed"
    assert result["attempt_count"] == 3
    assert result["per_attempt_published_count"] == [2, 2, 2]
    assert result["published_count"] == 2
    assert result["total_published_count"] == 6

    begin = _decode_messages(client.sdk_replay_begin_publisher.messages)
    items = _decode_messages(client.sdk_replay_item_publisher.messages)
    end = _decode_messages(client.sdk_replay_end_publisher.messages)

    assert len(begin) == 3
    assert len(items) == 6
    assert len(end) == 3
    assert {msg["request_id"] for msg in begin} == {"req-fixed"}
    assert {msg["join_attempt_id"] for msg in begin} == {"join-fixed"}
    assert {msg["request_id"] for msg in end} == {"req-fixed"}
    assert {msg["join_attempt_id"] for msg in end} == {"join-fixed"}
    assert all(msg["request_id"] == "req-fixed" for msg in items)
    assert all(msg["join_attempt_id"] == "join-fixed" for msg in items)

    assert sleep_calls == [0.15, 0.25, 0.25]


def test_publish_sdk_registry_replay_handles_zero_entries(monkeypatch):
    client = _build_client(monkeypatch)

    ok, result = client._publish_sdk_registry_replay(
        [],
        {"request_id": "req-zero", "join_attempt_id": "join-zero"},
    )

    assert ok is True
    assert result["request_id"] == "req-zero"
    assert result["join_attempt_id"] == "join-zero"
    assert result["expected_count"] == 0
    assert result["published_count"] == 0
    assert result["attempt_count"] == 1
    assert result["per_attempt_published_count"] == [0]
    assert result["total_published_count"] == 0

    begin = _decode_messages(client.sdk_replay_begin_publisher.messages)
    items = _decode_messages(client.sdk_replay_item_publisher.messages)
    end = _decode_messages(client.sdk_replay_end_publisher.messages)

    assert len(begin) == 1
    assert begin[0]["expected_count"] == 0
    assert len(items) == 0
    assert len(end) == 1
    assert end[0]["expected_count"] == 0
    assert end[0]["published_count"] == 0
