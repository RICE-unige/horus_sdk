"""Opt-in synthetic control/recording through real dual-lane HorusLink TCP.

Uses localhost ports 13980/13981 and an isolated ROS domain set by the runner.
Never publishes to a hardware controller. No registration acknowledgement is
fabricated: this verifies the transport/session path, not headset interaction.
"""
import json
import math
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import sys
import time
import uuid

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("HORUS_TEACH_LINK_TEST") != "1", reason="Opt-in real bridge test")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "examples"))


def tlv(key, value):
    return struct.pack("<HH", key, len(value)) + value


def fields(payload):
    result = {}
    while payload:
        key, size = struct.unpack("<HH", payload[:4])
        result[key], payload = payload[4:4+size], payload[4+size:]
    return result


def json_string(payload):
    size, = struct.unpack("<I", payload[:4])
    return json.loads(payload[4:4+size-1])


class Peer:
    def __init__(self, port):
        self.sockets = [socket.create_connection(("127.0.0.1", port + n), timeout=2) for n in range(2)]
        self.seq, self.messages, self.buffers, self.acks = 0, {}, {s: b"" for s in self.sockets}, set()
        self.hellos, self.clock_responses = [], []
        session = uuid.uuid4().int & ((1 << 64) - 1)
        for index, s in enumerate(self.sockets):
            payload = (tlv(1, b"\x01") + tlv(2, struct.pack("<H", 1)) + tlv(3, b"\x01") +
                       tlv(9, struct.pack("<I", 8*1024*1024)) + tlv(10, struct.pack("<I", 1000)) +
                       tlv(7, bytes([index+1])) + tlv(13, struct.pack("<Q", session)))
            self.send(0, 2, payload, s=s)

    def send(self, channel, kind, payload, s=None, flags=0):
        self.seq += 1
        (s or self.sockets[0]).sendall(struct.pack("<HBBIII", channel, kind, flags, self.seq, 0, len(payload)) + payload)

    def channel(self, channel, topic, *, publish=False, message="std_msgs/msg/String", lane=1):
        self.send(0, 2, tlv(1, bytes([5 if publish else 2])) + tlv(2, struct.pack("<H", 1)) +
                  tlv(4, struct.pack("<H", channel)) + tlv(5, topic.encode()) + tlv(6, message.encode()) +
                  tlv(7, bytes([lane])) + tlv(8, b"\x01"))
        self.wait(lambda: channel in self.acks)

    def publish(self, channel, value):
        text = json.dumps(value, allow_nan=False).encode() + b"\0"
        self.send(channel, 1, struct.pack("<I", len(text)) + text, flags=7)

    def poll(self, delay=.02):
        ready, _, _ = select.select(self.sockets, [], [], delay)
        for s in ready:
            data = s.recv(1024*1024)
            assert data, "Bridge closed connection"
            buffer = self.buffers[s] + data
            while len(buffer) >= 16:
                channel, kind, flags, seq, corr, size = struct.unpack("<HBBIII", buffer[:16])
                if len(buffer) < 16+size: break
                payload, buffer = buffer[16:16+size], buffer[16+size:]
                if kind == 2:
                    values = fields(payload)
                    if values.get(1) == b"\x01":
                        self.hellos.append(values)
                    if values.get(1) == b"\x03":
                        assert values[11] == b"\x01", values
                        self.acks.add(struct.unpack("<H", values[4])[0])
                elif kind == 1:
                    self.messages.setdefault(channel, []).append(payload)
                elif kind == 5 and len(payload) == 20 and payload[:4] == b"HLT1":
                    sent, remote = struct.unpack("<QQ", payload[4:])
                    self.clock_responses.append((sent, time.monotonic_ns() // 1000000, remote))
            self.buffers[s] = buffer

    def value(self, channel):
        data = self.messages.get(channel, [])
        return json_string(data[-1]) if data else {}

    def wait(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.poll()
            if predicate(): return
        pytest.fail(f"TCP timeout; channels={list(self.messages)} state={self.value(1)}")

    def close(self):
        for s in self.sockets: s.close()


@pytest.mark.parametrize("input_version,mode", [(1, "hold_to_run"), (2, "hold_to_run"), (2, "continuous")])
def test_two_episodes_through_bridge_and_controller(tmp_path, input_version, mode):
    from humanoid_teach_demo import make_registration
    from tools.teach_synthetic import NAME, PREFIX, description, SyntheticHumanoid, SyntheticSource
    from horus.teaching.runtime import TeachRuntime
    from horus.teaching.dataset import Episode
    urdf = description()
    path = tmp_path / "robot.urdf"
    path.write_text(urdf)
    extensions = input_version == 2 and mode == "continuous"
    robot, _ = make_registration(path, tmp_path / "episodes", input_version, mode, task_actions=extensions)
    fixture = json.loads((Path(__file__).resolve().parents[3] / "contracts/fixtures/teaching.json").read_text())["input_v2"]
    model = SyntheticHumanoid()
    binary = os.environ["HORUS_BRIDGE_BINARY"]
    log = (tmp_path / "bridge.log").open("w")
    bridge = subprocess.Popen([binary, "--ros-args", "-p", "tcp_ip:=127.0.0.1", "-p", "tcp_port:=13980",
                               "-p", "horuslink_bulk_port:=13981"], stdout=log, stderr=subprocess.STDOUT)
    peer = None
    try:
        time.sleep(1)
        assert bridge.poll() is None
        actions, saves = [], []
        with SyntheticSource(model, urdf), TeachRuntime(robot, adapter=model.adapter(),
                action_handlers={"check_fixture": actions.append} if extensions else None,
                after_save=saves.append if extensions else None) as host:
            peer = Peer(13980)
            peer.channel(1, f"/horus/teach/{NAME}/state")
            peer.channel(2, f"/horus/teach/{NAME}/request", publish=True)
            peer.channel(3, f"/horus/teach/{NAME}/input", publish=True)
            peer.channel(4, "/horus/multi_operator/control_lease_request", publish=True)
            peer.channel(5, "/horus/multi_operator/control_lease_state")
            peer.channel(6, PREFIX + "/front/image_raw", message="sensor_msgs/msg/Image", lane=2)
            peer.channel(7, PREFIX + "/tf", message="tf2_msgs/msg/TFMessage")
            peer.wait(lambda: peer.value(1).get("recording_ready") and not peer.value(1).get("extension_busy") and 6 in peer.messages and 7 in peer.messages)

            def request(action, **values):
                state = peer.value(1)
                item = dict(request_id=str(uuid.uuid4()), owner="test_operator", action=action,
                            config_hash=host.config.digest, session_id=state.get("session_id", ""),
                            expected_revision=state.get("revision", 0))
                item.update(values)
                peer.publish(2, item)
                peer.wait(lambda: any(json_string(p).get("request_id") == item["request_id"]
                                      for p in peer.messages.get(1, [])))
                ack = next(json_string(p) for p in reversed(peer.messages[1])
                           if json_string(p).get("request_id") == item["request_id"])
                assert ack["accepted"], ack

            request("open", **(dict(input_schema_version=2, input_features=fixture["features"], engagement_mode=mode) if input_version == 2 else {}))
            alignment = dict(schema_version=1, alignment_id=str(uuid.uuid4()), tracking_origin_id=str(uuid.uuid4()),
                frame_id="quest_tracking_flu", child_frame_id="teach_calibrated_floor_flu",
                position=[0, 0, 0], rotation=[0, 0, 0, 1])
            request("align", alignment=alignment)
            if extensions:
                request("select_task", task="Through-link task")
                peer.wait(lambda: peer.value(1).get("current_task") == "Through-link task" and not peer.value(1).get("extension_busy"))
            lease = dict(request_id=str(uuid.uuid4()), robot_name=NAME, app_id="test_operator", role="operator",
                         session_id="test_workspace", action="acquire", panel_open=True, teleop_active=True)
            peer.publish(4, lease)
            peer.wait(lambda: bool(peer.value(5).get("leases")))
            generation = peer.value(5)["leases"][0]["authority_generation"]
            peer.wait(lambda: model.lease.get("authority_generation") == generation)
            for episode_index in range(2):
                episode_id = str(uuid.uuid4())
                request("record", episode_id=episode_id, task="Through-link task" if extensions else host.config.task)
                peer.wait(lambda: peer.value(1).get("capture") == "recording")
                peer.publish(4, dict(lease, action="heartbeat", request_id=str(uuid.uuid4())))
                request("engage", authority_generation=generation, alignment_id=alignment["alignment_id"])
                peer.wait(lambda: peer.value(1).get("control") == "armed")
                before, previous_count = model.snapshot(), model.accepted
                sample_count = max(25, min(3000, int(os.environ.get("HORUS_TEACH_LINK_SAMPLES", "25"))))
                for i in range(sample_count):
                    if i % 25 == 0:
                        peer.publish(4, dict(lease, action="heartbeat", request_id=str(uuid.uuid4())))
                        request("heartbeat")
                    state = peer.value(1)
                    sample = dict(owner="test_operator", session_id=state["session_id"], input_epoch=state["input_epoch"],
                                  challenge=state["challenge"], sequence=i+1, tracking_valid=True, clutch=True, ui_focus=False,
                                  left_trigger=.7, right_trigger=.3)
                    if input_version == 2:
                        sample.update(fixture["sample"])
                        sample["source_monotonic_seconds"] = time.monotonic()
                        sample["engagement_mode"] = mode
                        if mode == "continuous":
                            sample.update(left_primary=False, right_primary=False)
                    sample.update(frame_id="quest_tracking_flu", alignment_id=alignment["alignment_id"], tracking_origin_id=alignment["tracking_origin_id"])
                    for hand, sign in (("left", 1), ("right", -1)):
                        sample[hand] = dict(position=[.3 + .01*math.sin(i/20), sign*.25, 1.1], rotation=[0, 0, 0, 1])
                    peer.publish(3, sample)
                    peer.poll(.04)
                    time.sleep(.02)
                assert model.accepted - previous_count >= 10
                assert model.snapshot() != before
                peer.wait(lambda: peer.value(1).get("control") == "engaged")
                if mode == "continuous":
                    peer.publish(3, dict(sample, sequence=sample_count+1, left_primary=True, right_primary=True))
                    peer.wait(lambda: peer.value(1).get("control") == "holding")
                    stopped_count = model.accepted
                    peer.publish(3, dict(sample, sequence=sample_count+2))
                    peer.poll(.1)
                    time.sleep(.1)
                    assert model.accepted == stopped_count and model.mode == "holding"
                request("finish", outcome="success")
                peer.wait(lambda: peer.value(1).get("capture") == "saved" and not peer.value(1).get("extension_busy"))
                assert model.mode == "holding"
                episode = Episode(tmp_path / "episodes" / episode_id)
                if extensions:
                    assert len(saves) == episode_index + 1
                    assert saves[-1].path == episode.path
                    assert peer.value(1)["current_task_counts"]["saved"] == episode_index + 1
                    request("custom_action", action_id="check_fixture")
                    peer.wait(lambda: peer.value(1).get("last_action", {}).get("status") == "completed" and not peer.value(1).get("extension_busy"))
                    assert len(actions) == episode_index + 1
                assert not episode.validate()
                assert episode.manifest["quality"] == "acceptable", episode.manifest
                events = list(episode.events())
                assert sum(e["kind"] == "command_result" for e in events) >= 10
                assert any(e["kind"] == "hold" for e in events)
                if input_version == 2:
                    intents = [e["data"] for e in events if e["kind"] == "operator_intent"]
                    assert len(intents) >= 10
                    for intent in intents:
                        assert intent["head"] == fixture["sample"]["head"]
                        assert intent["left_stick"] == fixture["sample"]["left_stick"]
                        assert intent["right_buttons_valid"] is True
                        assert intent["engagement_mode"] == mode
                        if mode == "continuous":
                            assert intent["left_primary"] is False and intent["right_primary"] is False
                for key in ("front", "left", "right", "tf", "joints"):
                    assert episode.manifest["stream_counts"][key] >= 3
            request("close")
    finally:
        if peer: peer.close()
        bridge.terminate()
        try: bridge.wait(timeout=10)
        except subprocess.TimeoutExpired:
            bridge.kill()
            bridge.wait()
        log.close()
