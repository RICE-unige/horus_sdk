"""Recover complete MCAP records into a NEW episode; never modify the evidence."""

import fcntl
import json
import os
from pathlib import Path
import time
import uuid

from .dataset import validate_mcap
from .recording import atomic_json, file_digest, sync_directory


def recover(path) -> Path:
    from mcap.stream_reader import StreamReader
    from mcap.records import Schema, Channel, Message
    from mcap.writer import Writer

    path = Path(path).resolve()
    if not path.name.endswith(".inprogress"):
        raise ValueError("Recovery accepts interrupted .inprogress directories only")
    with (path / "capture.lock").open("a+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("Capture is still active; do not recover a running episode") from exc
        manifest = json.loads((path / "journal.json").read_text())
        episode_id = str(uuid.uuid4())
        staging = path.parent / (episode_id + ".inprogress")
        staging.mkdir()
        atomic_json(staging / "journal.json", manifest)
        channels, schemas, counts = {}, {0: 0}, {}
        error, last_ns = "", manifest["started_unix_ns"]
        with (path / "data.mcap").open("rb") as source, (staging / "data.mcap").open("xb") as target:
            writer = Writer(target, enable_data_crcs=True)
            writer.start(library="horus-teach-recovery/1")
            try:
                for record in StreamReader(source, validate_crcs=True, record_size_limit=256 * 1024**2).records:
                    if isinstance(record, Schema):
                        if record.id not in schemas:
                            schemas[record.id] = writer.register_schema(record.name, record.encoding, record.data)
                    elif isinstance(record, Channel):
                        if record.id not in channels:
                            channels[record.id] = (writer.register_channel(record.topic, record.message_encoding,
                                schemas[record.schema_id], record.metadata), record.topic)
                    elif isinstance(record, Message):
                        channel, topic = channels[record.channel_id]
                        writer.add_message(channel, record.log_time, record.data, record.publish_time, record.sequence)
                        counts[topic] = counts.get(topic, 0) + 1
                        last_ns = max(last_ns, record.log_time)
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
            writer.finish()
            target.flush()
            os.fsync(target.fileno())
        validate_mcap(staging / "data.mcap")
        manifest.update(episode_id=episode_id, capture="interrupted", quality="unacceptable", outcome="unknown",
            quality_issues=["recovered_interrupted_capture"], export="not_requested",
            duration_ns=max(0, last_ns - manifest["started_unix_ns"]), ended_unix_ns=last_ns,
            stream_counts=counts, stream_statistics={}, recorder_drops=None,
            recovery={"source_episode": path.name, "source_sha256": file_digest(path / "data.mcap"),
                      "read_error": error, "recovered_unix_ns": time.time_ns(), "lost_tail": "unknown"},
            artifacts=[{"path": "data.mcap", "bytes": (staging / "data.mcap").stat().st_size,
                        "sha256": file_digest(staging / "data.mcap")}])
        atomic_json(staging / "manifest.json", manifest)
        destination = path.parent / episode_id
        os.rename(staging, destination)
        sync_directory(path.parent)
        return destination
