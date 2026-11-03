"""Native episode inspection; no learning framework or cloud service required."""

import json
from pathlib import Path


def validate_mcap(path):
    from mcap.reader import make_reader
    with Path(path).open("rb") as stream:
        reader = make_reader(stream, validate_crcs=True)
        if reader.get_summary() is None:
            raise ValueError("MCAP was not finalized")
        for _ in reader.iter_messages(log_time_order=False):
            pass


class Episode:
    def __init__(self, path):
        self.path = Path(path)
        self.manifest = json.loads((self.path / "manifest.json").read_text())
        if self.manifest.get("schema") != "horus.teach.episode.v1":
            raise ValueError("Unsupported episode schema")

    def validate(self) -> list[str]:
        from .recording import file_digest
        issues = []
        from .config import TeachConfig
        try:
            if TeachConfig.from_dict(self.manifest["config"]).digest != self.manifest["config_hash"]:
                issues.append("Recording configuration hash mismatch")
        except (KeyError, ValueError, TypeError) as exc:
            issues.append("Invalid recording configuration: " + str(exc))
        inventory = self.manifest.get("artifacts", [])
        if {item["path"] for item in inventory} != {"data.mcap"}:
            issues.append("Missing or unsupported artifact inventory")
        for item in inventory:
            if item["path"] != "data.mcap":
                continue
            path = self.path / item["path"]
            if not path.is_file() or path.stat().st_size != item["bytes"] or file_digest(path) != item["sha256"]:
                issues.append("Artifact integrity mismatch: " + item["path"])
        if not issues:
            try:
                validate_mcap(self.path / "data.mcap")
                counts = {}
                for schema, channel, message in self.samples():
                    counts[channel.topic] = counts.get(channel.topic, 0) + 1
                    if channel.message_encoding == "cdr" and (schema is None or schema.encoding not in {"ros2msg", "ros2idl"} or not schema.data):
                        raise ValueError("Missing ROS schema: " + channel.topic)
                for topic, expected in self.manifest.get("stream_counts", {}).items():
                    if counts.get(topic, 0) != expected:
                        issues.append("Stream count mismatch: " + topic)
            except Exception as exc:
                issues.append(f"MCAP validation failed: {exc}")
        return issues

    def samples(self, stream_ids=None, start_ns=None, end_ns=None):
        """Yield schema/channel/raw message without loading an episode in memory."""
        from mcap.reader import make_reader
        with (self.path / "data.mcap").open("rb") as stream:
            yield from make_reader(stream, validate_crcs=True).iter_messages(
                topics=stream_ids, start_time=start_ns, end_time=end_ns)

    def events(self):
        for _, _, message in self.samples(["__events"]):
            yield {"received_unix_ns": message.log_time, **json.loads(message.data)}
