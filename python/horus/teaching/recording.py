"""Bounded native-rate MCAP capture and durable, non-overwriting publication."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import shutil
import threading
import time
import uuid

from .config import TeachConfig, canonical


def sync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    with temporary.open("xb") as file:
        file.write(canonical(value))
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, path)
    sync_directory(path.parent)


@dataclass(frozen=True)
class Sample:
    stream_id: str
    payload: bytes
    received_unix_ns: int
    received_monotonic_ns: int
    source_ns: int | None = None
    source_sequence: int | None = None
    encoding: str = "cdr"
    schema: bytes = b""
    schema_encoding: str = ""


class EpisodeRecorder:
    """A dedicated disk worker; producers never wait for encoding or disk IO.

    Queue overflow is retained as quality evidence. It does not masquerade as
    successful capture of every source frame or stop the robot control worker.
    """

    def __init__(self, config: TeachConfig, robot_name: str, episode_id: str, task: str):
        from mcap.writer import Writer, CompressionType

        uuid.UUID(episode_id)
        self.config = config
        self.root = Path(config.recording_root).expanduser()
        self.root.mkdir(parents=True, exist_ok=True)
        if shutil.disk_usage(self.root).free < config.minimum_free_bytes:
            raise OSError("Insufficient recording disk headroom")
        self.destination = self.root / episode_id
        self.path = self.root / (episode_id + ".inprogress")
        if self.destination.exists():
            raise FileExistsError("Episode already finalized; it will not be overwritten")
        self.path.mkdir(exist_ok=False)
        import fcntl
        self.capture_lock = (self.path / "capture.lock").open("a+")
        fcntl.flock(self.capture_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.episode_id = episode_id
        self.condition = threading.Condition()
        self.finish_lock = threading.Lock()
        self.finish_result = None
        self.queue = deque()
        self.queued_bytes = 0
        self.accepting = True
        self.error = ""
        self.dropped = 0
        self.counts = {}
        self.stats = {}
        self.channels = {}
        self.started_monotonic_ns = time.monotonic_ns()
        self.manifest = {
            "schema": "horus.teach.episode.v1", "episode_id": episode_id,
            "robot_name": robot_name, "task": task, "provenance": config.provenance,
            "config": config.to_dict(), "config_hash": config.digest,
            "capture": "recording", "quality": "pending", "outcome": "unknown",
            "export": "not_requested", "started_unix_ns": time.time_ns(),
            "clock": {"receipt": "recorder_unix", "intervals": "recorder_monotonic",
                      "clock_offset_ns": None, "clock_error_ns": None,
                      "boot_id": self._boot_id()},
            "counter_origin": "recorder_subscription", "upstream_drops": None,
            "required_artifacts": ["data.mcap"],
        }
        try:
            atomic_json(self.path / "journal.json", self.manifest)
            self.file = (self.path / "data.mcap").open("xb")
            self.writer = Writer(self.file, chunk_size=256 * 1024, compression=CompressionType.ZSTD, enable_data_crcs=True)
            self.writer.start(profile="", library="horus-teach/1")
            self.thread = threading.Thread(target=self._write_loop, name="horus-teach-recorder", daemon=False)
            self.thread.start()
        except Exception:
            if hasattr(self, "file"): self.file.close()
            self.capture_lock.close()
            raise

    @staticmethod
    def _boot_id():
        path = Path("/proc/sys/kernel/random/boot_id")
        return path.read_text().strip() if path.exists() else "unknown"

    def append(self, sample: Sample) -> bool:
        size = len(sample.payload) + len(sample.schema) + 512
        with self.condition:
            if not self.accepting or self.error:
                return False
            if size > self.config.queue_budget_bytes - self.queued_bytes:
                self.dropped += 1
                return False
            self.queue.append((sample, size))
            self.queued_bytes += size
            self.condition.notify()
            return True

    def event(self, kind: str, data: dict) -> bool:
        return self.append(Sample("__events", canonical({"kind": kind, "data": data}),
                                  time.time_ns(), time.monotonic_ns(), encoding="json"))

    def _write_loop(self):
        try:
            disk_check = time.monotonic()
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.queue or not self.accepting)
                    if not self.queue:
                        break
                    sample, size = self.queue.popleft()
                    self.queued_bytes -= size
                if time.monotonic() - disk_check >= 1:
                    if shutil.disk_usage(self.root).free < self.config.minimum_free_bytes:
                        raise OSError("Recording disk reserve reached")
                    disk_check = time.monotonic()
                    self.file.flush()
                channel = self.channels.get(sample.stream_id)
                if channel is None:
                    stream = next((s for s in self.config.streams if s.stream_id == sample.stream_id), None)
                    schema = self.writer.register_schema(
                        stream.message_type if stream else "horus.teach.event.v1",
                        sample.schema_encoding, sample.schema)
                    channel = self.writer.register_channel(sample.stream_id, sample.encoding, schema,
                                                          {"source_topic": stream.topic if stream else "internal"})
                    self.channels[sample.stream_id] = channel
                sequence = self.counts.get(sample.stream_id, 0)
                self.writer.add_message(channel, sample.received_unix_ns, sample.payload,
                                        sample.source_ns or 0, sequence & 0xffffffff)
                self.counts[sample.stream_id] = sequence + 1
                # Per-message evidence lives beside raw payloads, never changes them.
                timing = self.channels.get("__timing")
                if timing is None:
                    timing = self.writer.register_channel("__timing", "json", 0)
                    self.channels["__timing"] = timing
                self.writer.add_message(timing, sample.received_unix_ns, canonical({
                    "stream_id": sample.stream_id, "receipt_index": sequence,
                    "received_monotonic_ns": sample.received_monotonic_ns,
                    "source_ns": sample.source_ns, "source_sequence": sample.source_sequence,
                }), sample.source_ns or 0, sequence & 0xffffffff)
                stats = self.stats.setdefault(sample.stream_id, {"first_ns": sample.received_monotonic_ns,
                    "last_ns": sample.received_monotonic_ns, "max_gap_ns": 0, "bytes": 0,
                    "source_time_regressions": 0, "source_time_duplicates": 0, "source_last_ns": None})
                stats["max_gap_ns"] = max(stats["max_gap_ns"], sample.received_monotonic_ns - stats["last_ns"])
                stats["last_ns"] = sample.received_monotonic_ns
                stats["bytes"] += len(sample.payload)
                if sample.source_ns and stats["source_last_ns"] is not None:
                    stats["source_time_regressions"] += int(sample.source_ns < stats["source_last_ns"])
                    stats["source_time_duplicates"] += int(sample.source_ns == stats["source_last_ns"])
                if sample.source_ns:
                    stats["source_last_ns"] = sample.source_ns
            self.writer.finish()
            self.file.flush()
            os.fsync(self.file.fileno())
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            self.file.close()
            with self.condition:
                self.accepting = False
                self.queue.clear()
                self.queued_bytes = 0

    def finish(self, outcome: str = "unknown", interrupted: bool = False) -> dict:
        if outcome not in {"unknown", "success", "failure", "abort"}:
            raise ValueError("Invalid episode outcome")
        with self.finish_lock:
            if self.finish_result is not None:
                return self.finish_result
            try:
                self.finish_result = self._finish(outcome, interrupted)
                return self.finish_result
            finally:
                self.capture_lock.close()

    def _finish(self, outcome: str, interrupted: bool) -> dict:
        if outcome not in {"unknown", "success", "failure", "abort"}:
            raise ValueError("Invalid episode outcome")
        with self.condition:
            self.accepting = False
            self.condition.notify_all()
        ended = time.monotonic_ns()
        ended_unix = time.time_ns()
        self.thread.join()
        issues = []
        if self.dropped:
            issues.append(f"recorder_queue_drops:{self.dropped}")
        for stream in self.config.streams:
            stats = self.stats.get(stream.stream_id)
            if stream.required and not stats:
                issues.append(f"missing:{stream.stream_id}")
            elif stream.required and not stream.retained_calibration:
                gap = max(stats["max_gap_ns"], stats["first_ns"] - self.started_monotonic_ns,
                          ended - stats["last_ns"])
                if gap > stream.max_gap_seconds * 1e9:
                    issues.append(f"stream_gap:{stream.stream_id}")
                if stats["source_time_regressions"]:
                    issues.append(f"source_time_regression:{stream.stream_id}")
        if self.error:
            issues.append(self.error)
        if interrupted:
            issues.append("interrupted")
        self.manifest.update(capture="interrupted" if interrupted or self.error else "saved",
            quality="unacceptable" if issues else "acceptable", quality_issues=issues,
            outcome=outcome, ended_unix_ns=ended_unix,
            duration_ns=ended - self.started_monotonic_ns, stream_counts=self.counts,
            stream_statistics=self.stats, recorder_drops=self.dropped)
        if self.error:
            atomic_json(self.path / "manifest.json", self.manifest)
            return self.manifest
        from .dataset import validate_mcap
        validate_mcap(self.path / "data.mcap")
        self.manifest["artifacts"] = [{"path": "data.mcap", "sha256": file_digest(self.path / "data.mcap"),
                                        "bytes": (self.path / "data.mcap").stat().st_size}]
        atomic_json(self.path / "manifest.json", self.manifest)
        # Episode UUID reservation and destination check prevent concurrent reuse.
        if self.destination.exists():
            raise FileExistsError("Destination appeared during capture")
        os.rename(self.path, self.destination)
        sync_directory(self.root)
        self.path = self.destination
        return self.manifest


def file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
