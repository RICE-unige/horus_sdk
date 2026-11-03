"""Small, explicitly synthetic offline capture for installation/reader checks."""

import math
from pathlib import Path
import time
import uuid

from .config import TeachConfig, TeachStream, TeachCamera
from .recording import EpisodeRecorder, Sample


def create_fixture(root) -> Path:
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState, Image
    from .ros_schema import message_schema

    config = TeachConfig(str(Path(root).expanduser().resolve()), provenance="fixture", task="Offline recorder fixture",
        minimum_free_bytes=1024 * 1024, streams=(
            TeachStream("joints", "/fixture/joints", "sensor_msgs/msg/JointState", role="measured_state"),
            TeachStream("front", "/fixture/front", "sensor_msgs/msg/Image", role="camera")),
        cameras=(TeachCamera("front", stream_id="front"),))
    recorder = EpisodeRecorder(config, "fixture_humanoid", str(uuid.uuid4()), config.task)
    try:
        recorder.event("fixture", {"physical_robot": False, "action_data": "absent"})
        for index in range(25):
            now = time.time_ns()
            pixels = bytes([index * 10, 80, 140]) * (160 * 120)
            messages = {"joints": JointState(name=["shoulder", "elbow"],
                position=[0.2 * math.sin(index / 5), 0.3 * math.cos(index / 5)]),
                "front": Image(width=160, height=120, encoding="rgb8", step=160 * 3, data=pixels)}
            for stream in config.streams:
                msg = messages[stream.stream_id]
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(now, 1_000_000_000)
                recorder.append(Sample(stream.stream_id, serialize_message(msg), now, time.monotonic_ns(),
                    source_ns=now, schema=message_schema(stream.message_type), schema_encoding="ros2msg"))
            time.sleep(0.08)
        recorder.finish("unknown")
    except BaseException:
        recorder.finish("abort", interrupted=True)
        raise
    return recorder.path
