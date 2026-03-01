"""Tests for robot-description chunk request/response transport."""

import json
import tempfile
import os

from horus.bridge import robot_registry as robot_registry_module
from horus.bridge.robot_registry import RobotRegistryClient
from horus.robot import Robot, RobotType


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
    client.ros_initialized = False
    client.node = None
    client.robot_description_chunk_begin_publisher = _SpyPublisher()
    client.robot_description_chunk_item_publisher = _SpyPublisher()
    client.robot_description_chunk_end_publisher = _SpyPublisher()
    client._robot_description_resolver = None
    client._robot_description_by_robot = {}
    client._robot_description_by_id = {}
    return client


def test_robot_description_request_publishes_chunk_triplet(monkeypatch):
    urdf = """
<robot name="test_bot">
  <link name="base_link">
    <collision><geometry><box size="0.2 0.2 0.1"/></geometry></collision>
  </link>
</robot>
    """.strip()
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False, encoding="utf-8") as handle:
        handle.write(urdf)
        urdf_path = handle.name

    try:
        robot = Robot(name="nova", robot_type=RobotType.WHEELED)
        robot.configure_robot_description(urdf_path=urdf_path, base_frame="base_link")
        client = _build_client(monkeypatch)
        container = client._resolve_robot_description_artifact(robot)
        assert container is not None
        artifact = container["artifact"]
        request_id = "req-robot-desc"
        request_payload = {
            "request_id": request_id,
            "robot_name": "nova",
            "description_id": artifact.manifest.description_id,
            "session_id": "session-1",
            "app_id": "joiner-1",
        }

        client._robot_description_request_callback(_FakeStringWithData(json.dumps(request_payload)))

        begin_messages = [json.loads(raw) for raw in client.robot_description_chunk_begin_publisher.messages]
        item_messages = [json.loads(raw) for raw in client.robot_description_chunk_item_publisher.messages]
        end_messages = [json.loads(raw) for raw in client.robot_description_chunk_end_publisher.messages]

        assert len(begin_messages) == 1
        assert begin_messages[0]["request_id"] == request_id
        assert begin_messages[0]["description_id"] == artifact.manifest.description_id
        assert begin_messages[0]["expected_chunks"] == len(artifact.chunks)

        assert len(item_messages) == len(artifact.chunks)
        assert [entry["chunk_index"] for entry in item_messages] == list(range(len(artifact.chunks)))
        assert all(entry["request_id"] == request_id for entry in item_messages)
        assert all(entry["description_id"] == artifact.manifest.description_id for entry in item_messages)

        assert len(end_messages) == 1
        assert end_messages[0]["request_id"] == request_id
        assert end_messages[0]["description_id"] == artifact.manifest.description_id
        assert end_messages[0]["received_count"] == len(artifact.chunks)
    finally:
        if os.path.isfile(urdf_path):
            os.remove(urdf_path)


def test_robot_description_request_ignores_wrong_description_id(monkeypatch):
    client = _build_client(monkeypatch)
    request_payload = {
        "request_id": "req-invalid",
        "robot_name": "nova",
        "description_id": "sha256:not_found",
    }
    client._robot_description_request_callback(_FakeStringWithData(json.dumps(request_payload)))

    assert client.robot_description_chunk_begin_publisher.messages == []
    assert client.robot_description_chunk_item_publisher.messages == []
    assert client.robot_description_chunk_end_publisher.messages == []


class _FakeStringWithData:
    def __init__(self, data):
        self.data = data
