import time
import uuid

import pytest

from horus.teaching import TeachConfig, TeachStream
from horus.teaching.dataset import Episode
from horus.teaching.recording import Sample
from horus.teaching.session import TeachSession
from test_teaching import request, poll


def test_each_episode_retains_original_calibration_and_all_tf_children(tmp_path):
    config = TeachConfig(recording_root=str(tmp_path), minimum_free_bytes=1024, streams=(
        TeachStream("info", "/info", "sensor_msgs/msg/CameraInfo", required=True),
        TeachStream("static", "/tf_static", "tf2_msgs/msg/TFMessage", required=True,
                    reliability="reliable", durability="transient_local"),))
    session = TeachSession(config, "robot")
    now = time.time_ns()
    mono = time.monotonic_ns()
    samples = [Sample("info", b"calibration", now-10**10, mono-10**10, source_ns=123, schema=b"string raw\n", schema_encoding="ros2msg"),
               Sample("static", b"old-a-and-b", now-9*10**9, mono-9*10**9, schema=b"string raw\n", schema_encoding="ros2msg"),
               Sample("static", b"new-a", now-8*10**9, mono-8*10**9, schema=b"string raw\n", schema_encoding="ros2msg")]
    try:
        assert session.request(request(session, "open"))["accepted"]
        session.append(samples[0])
        session.append(samples[1], ["a", "b"])
        session.append(samples[2], ["a"])
        session.health = {"info": time.monotonic()-100, "static": time.monotonic()-100}
        assert session.state()["recording_ready"]
        for _ in range(2):
            episode_id = str(uuid.uuid4())
            assert session.request(request(session, "record", episode_id=episode_id))["accepted"]
            poll(session, "recording")
            assert session.request(request(session, "finish"))["accepted"]
            poll(session, "saved")
            episode = Episode(tmp_path / episode_id)
            assert episode.validate() == []
            captured = list(episode.samples(stream_ids=["info", "static"]))
            assert [m.data for _, _, m in captured] == [s.payload for s in samples]
            assert [m.log_time for _, _, m in captured] == [s.received_unix_ns for s in samples]
            assert captured[0][2].publish_time == 123
            events = [e for e in episode.events() if e["kind"] == "calibration_snapshot"]
            assert len(events) == 3
            assert [e["data"]["original_receipt_monotonic_ns"] for e in events] == [s.received_monotonic_ns for s in samples]
    finally:
        session.close()


def test_calibration_cache_is_bounded_and_rejects_invalid_keys(tmp_path):
    config = TeachConfig(recording_root=str(tmp_path), minimum_free_bytes=1024, streams=(
        TeachStream("static", "/tf_static", "tf2_msgs/msg/TFMessage", durability="transient_local"),))
    session = TeachSession(config, "robot")
    try:
        with pytest.raises(ValueError):
            session.append(Sample("missing", b"", 1, 1))
        value = Sample("static", b"original", 1, 1)
        session.append(value, ["a"])
        for key in ("", None, "x"*513):
            with pytest.raises(ValueError):
                session.append(value, [key])
        assert not session.append(value, [str(i) for i in range(2049)])
        assert len(session.calibration_samples) == 1
        assert session.error and session.quality == "unacceptable"
    finally:
        session.close()
