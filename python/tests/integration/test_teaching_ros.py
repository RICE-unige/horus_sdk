"""ROS transport/recording test. Isolate with ROS_DOMAIN_ID and local discovery."""

import json
import threading
import time
import uuid

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("mcap")
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CameraInfo, JointState, Image
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from horus.robot import Robot, RobotType
from horus.teaching import TeachAction, TeachConfig, TeachStream
from horus.teaching.dataset import Episode
from horus.teaching.runtime import TeachRuntime
from rclpy.serialization import deserialize_message


@pytest.mark.parametrize("extensions", [False, True])
def test_ros_native_episode_roundtrip(tmp_path, extensions):
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node("horus_teach_test_source", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin)
    worker.start()
    robot = Robot("teach_test", RobotType.HUMANOID)
    cfg = TeachConfig(str(tmp_path), minimum_free_bytes=1024, provenance="fixture",
        task="Initial task" if extensions else "", task_management=extensions, episode_counts=extensions,
        actions=(TeachAction("reset", "Reset fixture"),) if extensions else (),
        streams=(TeachStream("joints", "/teach_test/joints", "sensor_msgs/msg/JointState", role="measured_state"),
                 TeachStream("front", "/teach_test/image", "sensor_msgs/msg/Image", role="camera")))
    robot.configure_teaching(cfg)
    topics = cfg.registration(robot.name)
    requests = node.create_publisher(String, topics["request_topic"], 10)
    joints = node.create_publisher(JointState, "/teach_test/joints", 10)
    camera = node.create_publisher(Image, "/teach_test/image", 10)
    states = []
    node.create_subscription(String, topics["state_topic"], lambda m: states.append(json.loads(m.data)), 10)
    count = 0

    def publish():
        nonlocal count
        count += 1
        stamp = node.get_clock().now().to_msg()
        msg = JointState(name=["joint"], position=[count * 0.01])
        msg.header.stamp = stamp
        joints.publish(msg)
        frame = Image(height=2, width=2, encoding="rgb8", step=6, data=bytes([255, 0, 0] * 4))
        frame.header.stamp = stamp
        camera.publish(frame)

    node.create_timer(0.03, publish)

    def wait_for(predicate):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if states and predicate(states[-1]): return states[-1]
            time.sleep(0.02)
        pytest.fail(f"No matching state: {states[-1:]}")

    def request(action, **values):
        state = states[-1] if states else {}
        body = dict(request_id=str(uuid.uuid4()), owner="test_operator", action=action,
                    config_hash=cfg.digest, session_id=state.get("session_id", ""),
                    expected_revision=state.get("revision", 0))
        body.update(values)
        requests.publish(String(data=json.dumps(body)))
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            responses = [s for s in states if s.get("request_id") == body["request_id"]]
            if responses:
                assert responses[-1]["accepted"], responses[-1]
                return responses[-1]
            time.sleep(0.03)
        pytest.fail("Request acknowledgement missing")

    try:
        actions, saved = [], []
        with TeachRuntime(robot, action_handlers={"reset": actions.append} if extensions else None,
                          after_save=saved.append if extensions else None):
            wait_for(lambda s: s["recording_ready"] and not s.get("extension_busy"))
            request("open")
            if extensions:
                request("select_task", task="Recorded through DDS")
                wait_for(lambda s: s.get("current_task") == "Recorded through DDS" and not s["extension_busy"])
                request("custom_action", action_id="reset")
                wait_for(lambda s: s.get("last_action", {}).get("status") == "completed" and not s["extension_busy"])
                assert len(actions) == 1 and actions[0].action_id == "reset"
            episode_id = str(uuid.uuid4())
            request("record", episode_id=episode_id, task="Recorded through DDS" if extensions else "")
            wait_for(lambda s: s["capture"] == "recording")
            time.sleep(0.4)
            request("finish", outcome="unknown")
            final = wait_for(lambda s: s["capture"] == "saved" and not s.get("extension_busy"))
            episode = Episode(tmp_path / episode_id)
            assert not episode.validate()
            assert episode.manifest["provenance"] == "fixture"
            assert episode.manifest["stream_counts"]["joints"] >= 5
            assert episode.manifest["stream_counts"]["front"] >= 5
            _, _, frame = next(episode.samples(["front"]))
            decoded = deserialize_message(frame.data, Image)
            assert decoded.width == 2 and decoded.height == 2
            assert bytes(decoded.data) == bytes([255, 0, 0] * 4)
            if extensions:
                assert episode.manifest["task"] == "Recorded through DDS"
                assert len(saved) == 1 and saved[0].path == tmp_path / episode_id
                assert final["current_task_counts"]["saved"] == 1
                assert final["save_hook"]["status"] == "completed"
            request("close")
    finally:
        executor.shutdown()
        worker.join()
        node.destroy_node()
        context.shutdown()


def test_pre_record_calibration_survives_two_ros_episodes(tmp_path):
    context = Context()
    rclpy.init(context=context)
    name = "teach_calibration_" + uuid.uuid4().hex[:8]
    node = rclpy.create_node(name + "_source", context=context)
    qos = QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    info_pub = node.create_publisher(CameraInfo, f"/{name}/info", qos)
    tf_pub = node.create_publisher(TFMessage, f"/{name}/tf_static", qos)
    info = CameraInfo(width=640, height=480)
    info.header.frame_id = "camera_depth_optical_frame"
    info.header.stamp.sec = 123
    info.header.stamp.nanosec = 456
    info.k = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]

    def transform(child, x, seconds):
        value = TransformStamped(child_frame_id=child)
        value.header.frame_id = "pelvis"
        value.header.stamp.sec = seconds
        value.transform.translation.x = x
        value.transform.rotation.w = 1.0
        return value

    # Separate retained messages and an overlapping update must preserve both children.
    first = TFMessage(transforms=[transform("camera", 0.1, 124),
                                  transform("wrist", 0.2, 125)])
    second = TFMessage(transforms=[transform("camera", 0.3, 126)])
    info_pub.publish(info)
    tf_pub.publish(first)
    tf_pub.publish(second)
    config = TeachConfig(str(tmp_path), minimum_free_bytes=1024, streams=(
        TeachStream("info", f"/{name}/info", "sensor_msgs/msg/CameraInfo",
                    required=True, reliability="reliable", durability="transient_local",
                    queue_depth=8, max_gap_seconds=0.1),
        TeachStream("static", f"/{name}/tf_static", "tf2_msgs/msg/TFMessage",
                    required=True, reliability="reliable", durability="transient_local",
                    queue_depth=8, max_gap_seconds=0.1)))
    robot = Robot(name, RobotType.HUMANOID)
    robot.configure_teaching(config)

    def wait_for(predicate):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        pytest.fail("Timed out waiting for retained calibration/capture")

    try:
        with TeachRuntime(robot) as runtime:
            session = runtime.session

            def cached():
                with session.lock:
                    return len(session.calibration_samples) == 3

            wait_for(cached)

            def request(action, **values):
                state = session.state()
                response = session.request(dict(
                    request_id=str(uuid.uuid4()), owner="calibration_test", action=action,
                    config_hash=config.digest, session_id=state["session_id"],
                    expected_revision=state["revision"], **values))
                assert response["accepted"], response

            request("open")
            time.sleep(0.25)  # Old retained calibration is valid, not a live-stream heartbeat.
            assert session.state()["recording_ready"]
            recorded = []
            for _ in range(2):
                started_ns = time.time_ns()
                episode_id = str(uuid.uuid4())
                request("record", episode_id=episode_id)
                wait_for(lambda: session.state()["capture"] == "recording")
                request("finish", outcome="unknown")
                wait_for(lambda: session.state()["capture"] == "saved")
                episode = Episode(tmp_path / episode_id)
                assert episode.validate() == []
                camera = list(episode.samples(["info"]))
                transforms = list(episode.samples(["static"]))
                assert len(camera) == 1 and len(transforms) == 2
                assert deserialize_message(camera[0][2].data, CameraInfo) == info
                assert camera[0][2].publish_time == 123_000_000_456
                assert [deserialize_message(m.data, TFMessage) for _, _, m in transforms] == [first, second]
                snapshots = [e for e in episode.events() if e["kind"] == "calibration_snapshot"]
                assert len(snapshots) == 3
                samples = camera + transforms
                assert all(m.log_time < started_ns for _, _, m in samples)
                recorded.append([(m.log_time, m.publish_time, m.data) for _, _, m in samples])
            assert recorded[0] == recorded[1]
            request("close")
    finally:
        node.destroy_node()
        context.shutdown()
