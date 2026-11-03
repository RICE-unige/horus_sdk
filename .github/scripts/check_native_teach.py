#!/usr/bin/env python3
"""Isolated DDS capture with real CDR, schemas, timestamps and episode validation.

Starts no bridge/robot/app and sends no motion commands. Use a test ROS domain.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import uuid

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from horus.teaching import TeachConfig
from horus.teaching.dataset import Episode
from horus.teaching.recording import EpisodeRecorder


def run(binary, input_v2=False):
    with tempfile.TemporaryDirectory(prefix="horus-native-teach-") as directory:
        root = Path(directory)
        process = subprocess.Popen([binary, directory] + (["--input-v2"] if input_v2 else []), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        node = rclpy.create_node("native_teach_test_peer")
        state = {}
        replies = {}

        def receive(message):
            value = json.loads(message.data)
            state.clear()
            state.update(value)
            if value.get("request_id"):
                replies[value["request_id"]] = value

        node.create_subscription(String, "/horus/teach/native_teach/state", receive, 10)
        request_publisher = node.create_publisher(String, "/horus/teach/native_teach/request", 10)
        source = node.create_publisher(String, "/horus_native_teach/state", qos_profile_sensor_data)
        camera = node.create_publisher(Image, "/horus_native_teach/image", qos_profile_sensor_data)
        retained = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        info = node.create_publisher(CameraInfo, "/horus_native_teach/info", retained)
        transforms = node.create_publisher(TFMessage, "/horus_native_teach/tf_static", retained)
        inputs = node.create_publisher(String, "/horus/teach/native_teach/input", qos_profile_sensor_data)
        leases = node.create_publisher(String, "/horus/multi_operator/control_lease_state", 1)
        generation = "fixture-" + str(uuid.uuid4())
        image = Image(height=2, width=2, encoding="rgb8", step=6, data=bytes(range(12)))
        image.header.stamp.sec = 42
        image.header.stamp.nanosec = 123

        def spin_until(predicate, timeout=6):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(process.communicate())
                source.publish(String(data="native capture"))
                camera.publish(image)
                if input_v2:
                    leases.publish(String(data=json.dumps({"leases": [{"robot_name": "native_teach",
                        "holder_app_id": "test_operator", "authority_generation": generation,
                        "remaining_ms": 3000.0}]})))
                rclpy.spin_once(node, timeout_sec=0.02)
                if predicate():
                    return
            raise AssertionError(f"Timed out; last state: {state}")

        def request(action, **extra):
            identifier = str(uuid.uuid4())
            value = dict(request_id=identifier, action=action, config_hash=state["config_hash"], owner="test_operator",
                         session_id=state.get("session_id", ""), expected_revision=state["revision"], **extra)
            request_publisher.publish(String(data=json.dumps(value)))
            spin_until(lambda: identifier in replies)
            response = replies[identifier]
            assert response["accepted"], response
            return response

        try:
            spin_until(lambda: info.get_subscription_count() > 0 and transforms.get_subscription_count() > 0)
            calibration = CameraInfo(width=2, height=2)
            calibration.header.stamp.sec = 41
            info.publish(calibration)
            for name in ("left", "right"):
                transform = TransformStamped(child_frame_id=name)
                transform.header.frame_id = "base"
                transform.header.stamp.sec = 40
                transform.transform.rotation.w = 1.0
                transforms.publish(TFMessage(transforms=[transform]))
            until = time.monotonic() + .3
            spin_until(lambda: time.monotonic() >= until)
            spin_until(lambda: state.get("recording_ready") is True)
            fixture = json.loads((Path(__file__).resolve().parents[2] / "contracts/fixtures/teaching.json").read_text())["input_v2"]
            request("open", **(dict(input_schema_version=2, input_features=fixture["features"]) if input_v2 else {}))
            episode_id = str(uuid.uuid4())
            request("record", episode_id=episode_id)
            spin_until(lambda: state.get("capture") == "recording")
            end = time.monotonic() + 0.4
            spin_until(lambda: time.monotonic() >= end)
            if input_v2:
                request("align", alignment=dict(schema_version=1, alignment_id=str(uuid.uuid4()),
                    tracking_origin_id=str(uuid.uuid4()), frame_id="quest_tracking_flu",
                    child_frame_id="teach_calibrated_floor_flu", position=[0, 0, 0], rotation=[0, 0, 0, 1]))
                request("engage", authority_generation=generation, alignment_id=state["alignment_id"])
                for sequence in range(1, 9):
                    sample = dict(fixture["sample"], owner="test_operator", session_id=state["session_id"],
                                  input_epoch=state["input_epoch"], challenge=state["challenge"], sequence=sequence)
                    sample.update(frame_id="quest_tracking_flu", alignment_id=state["alignment_id"],
                                  tracking_origin_id=state["tracking_origin_id"])
                    inputs.publish(String(data=json.dumps(sample)))
                    until = time.monotonic() + .06
                    spin_until(lambda: time.monotonic() >= until)
                assert state["control"] == "engaged", state
                # A false validity flag must request Hold, not reuse the last pose.
                sample.update(sequence=9, head_tracking_valid=False, challenge=state["challenge"])
                inputs.publish(String(data=json.dumps(sample)))
                spin_until(lambda: state.get("control") == "holding")
            request("finish", outcome="success")
            spin_until(lambda: state.get("capture") == "saved")
            assert state["quality"] == "acceptable", state
            episode = Episode(root / episode_id)
            issues = episode.validate()
            assert issues == [], issues
            seen = {"state": 0, "image": 0}
            for schema, channel, message in episode.samples(seen):
                assert schema.encoding == "ros2msg"
                seen[channel.topic] += 1
                if channel.topic == "state":
                    assert deserialize_message(message.data, String).data == "native capture"
                else:
                    decoded = deserialize_message(message.data, Image)
                    assert bytes(decoded.data) == bytes(range(12))
                    assert message.publish_time == 42_000_000_123
            assert min(seen.values()) >= 2, seen
            children = set()
            infos = []
            for _, channel, message in episode.samples(["info", "static"]):
                if channel.topic == "info":
                    infos.append(deserialize_message(message.data, CameraInfo))
                    assert message.publish_time == 41_000_000_000
                else:
                    children.update(t.child_frame_id for t in deserialize_message(message.data, TFMessage).transforms)
            assert len(infos) == 1 and infos[0].width == 2 and children == {"left", "right"}, (infos, children)
            snapshots = [e for e in episode.events() if e["kind"] == "calibration_snapshot"]
            assert len(snapshots) == 3, snapshots
            if input_v2:
                events = list(episode.events())
                intents = [e["data"] for e in events if e["kind"] == "operator_intent"]
                commands = [e["data"] for e in events if e["kind"] == "command_result"]
                assert len(intents) >= 4 and len(commands) >= 3, events
                assert all(e["head_tracking_valid"] is True for e in intents)
                for command in commands:
                    for key, value in fixture["sample"].items():
                        assert command["fixture_input"][key] == value, key
                    assert command["fixture_input"]["tools"] == {"grasp": .6, "pinch": .4}
            request("close")
            stdout, stderr = process.communicate(timeout=16)
            assert process.returncode == 0, (stdout, stderr)
            subprocess.run([binary, "--validate", str(root / episode_id)], check=True, timeout=10)
            # The native reader must also accept existing Python-recorded episodes.
            python_id = str(uuid.uuid4())
            recorder = EpisodeRecorder(TeachConfig(recording_root=directory, minimum_free_bytes=1024), "native_teach", python_id, "Python interoperability")
            assert recorder.event("python_recorded", {"value": 42})
            recorder.finish("success")
            subprocess.run([binary, "--validate", str(root / python_id)], check=True, timeout=10)
            print(f"PASS {Path(binary).name} input_v2={input_v2}: raw CDR, source timestamps, schemas, {seen}, bidirectional Python episode validation", flush=True)
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.communicate(timeout=4)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate()
            node.destroy_node()


if __name__ == "__main__":
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not os.environ.get("ROS_DOMAIN_ID"):
        raise SystemExit("Set a dedicated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1")
    rclpy.init()
    try:
        for executable in sys.argv[1:]:
            run(executable)
            run(executable, input_v2=True)
    finally:
        rclpy.shutdown()
