"""Small summary helpers for HORUS experiment output directories."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import mean, median, pstdev, stdev
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Tuple


def percentile(values: Iterable[float], p: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * min(max(p, 0.0), 100.0) / 100.0
    low = int(rank)
    high = min(low + 1, len(ordered) - 1)
    frac = rank - low
    return ordered[low] * (1.0 - frac) + ordered[high] * frac


def _measurement_window_from_events(events_path: Path) -> Optional[Tuple[int, int]]:
    events_path = Path(events_path)
    if not events_path.exists():
        return None

    start: Optional[int] = None
    end: Optional[int] = None
    with events_path.open(encoding="utf-8") as handle:
        for line in handle:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            name = event.get("name")
            try:
                timestamp_ns = int(event.get("timestamp_ns"))
            except (TypeError, ValueError):
                continue
            if name == "measurement_start" and start is None:
                start = timestamp_ns
            elif name == "measurement_end":
                end = timestamp_ns

    if start is None or end is None or end <= start:
        return None
    return start, end


def _measurement_window(run_dir: Path) -> Optional[Tuple[int, int]]:
    return _measurement_window_from_events(Path(run_dir) / "orchestrator_events.ndjson")


def _metric_window(run_dir: Path, file_name: str, default_window: Optional[Tuple[int, int]]) -> Optional[Tuple[int, int]]:
    if file_name == "headset_metrics.csv" or file_name.startswith("headset_"):
        return _measurement_window_from_events(Path(run_dir) / "headset_events.ndjson") or default_window
    return default_window


def _filter_measurement_rows(rows: List[Dict[str, str]], window: Optional[Tuple[int, int]]) -> List[Dict[str, str]]:
    if window is None:
        return rows

    start, end = window
    filtered: List[Dict[str, str]] = []
    for row in rows:
        try:
            timestamp_ns = int(row.get("timestamp_ns", ""))
        except (TypeError, ValueError):
            continue
        if start <= timestamp_ns <= end:
            filtered.append(row)
    return filtered


def _read_csv_rows(path: Path) -> List[Dict[str, str]]:
    with Path(path).open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def _parse_int(value: object, default: int = 0) -> int:
    try:
        if value is None or value == "":
            return default
        text = str(value).strip()
        try:
            return int(text)
        except ValueError:
            return int(float(text))
    except (TypeError, ValueError):
        return default


def _parse_extra_json(row: Mapping[str, str]) -> Dict[str, Any]:
    raw = row.get("extra_json") or ""
    if not raw:
        return {}
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError:
        return {}
    return payload if isinstance(payload, dict) else {}


def _load_clock_sync(run_dir: Path) -> Optional[Dict[str, Any]]:
    path = Path(run_dir) / "clock_sync.json"
    if not path.exists():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _clock_samples(run_dir: Path) -> List[Dict[str, int]]:
    payload = _load_clock_sync(run_dir)
    if not payload:
        return []
    raw_samples = payload.get("samples")
    if not isinstance(raw_samples, list):
        return []
    samples: List[Dict[str, int]] = []
    for item in raw_samples:
        if not isinstance(item, dict):
            continue
        host_send_ns = _parse_int(item.get("host_send_ns"))
        host_receive_ns = _parse_int(item.get("host_receive_ns"))
        host_midpoint_ns = int((host_send_ns + host_receive_ns) / 2)
        offset_ns = _parse_int(item.get("offset_ns"))
        round_trip_ns = _parse_int(item.get("round_trip_ns"))
        headset_receive_monotonic_ns = _parse_int(item.get("headset_receive_monotonic_ns"))
        headset_send_monotonic_ns = _parse_int(item.get("headset_send_monotonic_ns"))
        headset_monotonic_midpoint_ns = 0
        monotonic_offset_ns = 0
        if headset_receive_monotonic_ns > 0 and headset_send_monotonic_ns > 0:
            headset_monotonic_midpoint_ns = int((headset_receive_monotonic_ns + headset_send_monotonic_ns) / 2)
            # Offset is headset_monotonic_time - host_time. Subtract it to place
            # headset monotonic rows on the host/source clock.
            monotonic_offset_ns = headset_monotonic_midpoint_ns - host_midpoint_ns
        if host_midpoint_ns > 0:
            samples.append(
                {
                    "host_midpoint_ns": host_midpoint_ns,
                    "headset_wall_midpoint_ns": host_midpoint_ns + offset_ns,
                    "offset_ns": offset_ns,
                    "round_trip_ns": round_trip_ns,
                    "headset_monotonic_midpoint_ns": headset_monotonic_midpoint_ns,
                    "monotonic_offset_ns": monotonic_offset_ns,
                }
            )
    return sorted(samples, key=lambda sample: sample["host_midpoint_ns"])


def _interpolate_offset(value_ns: int, samples: List[Dict[str, int]], time_key: str, offset_key: str) -> Optional[int]:
    usable = [sample for sample in samples if sample.get(time_key, 0) > 0]
    if value_ns <= 0 or not usable:
        return None
    usable = sorted(usable, key=lambda sample: sample[time_key])
    if len(usable) == 1:
        return usable[0][offset_key]

    first = usable[0]
    last = usable[-1]
    if value_ns <= first[time_key]:
        return first[offset_key]
    if value_ns >= last[time_key]:
        return last[offset_key]

    for left, right in zip(usable, usable[1:]):
        if left[time_key] <= value_ns <= right[time_key]:
            span = max(1, right[time_key] - left[time_key])
            ratio = (value_ns - left[time_key]) / span
            return int(left[offset_key] + (right[offset_key] - left[offset_key]) * ratio)
    return last[offset_key]


def _headset_to_host_time_ns(
    headset_timestamp_ns: int,
    samples: List[Dict[str, int]],
    headset_monotonic_ns: int = 0,
) -> Tuple[Optional[int], str]:
    if not samples:
        return None, ""

    # Prefer Quest monotonic time. Android wall time can be corrected by NTP while
    # an experiment is running; monotonic timestamps stay stable and are recorded
    # in every headset row.
    if headset_monotonic_ns > 0:
        offset = _interpolate_offset(
            headset_monotonic_ns,
            samples,
            "headset_monotonic_midpoint_ns",
            "monotonic_offset_ns",
        )
        if offset is not None:
            return headset_monotonic_ns - offset, "clock_corrected_monotonic"

    if headset_timestamp_ns <= 0:
        return None, ""

    # Fallback for older runs without headset monotonic clock-sync samples.
    offset = _interpolate_offset(
        headset_timestamp_ns,
        samples,
        "headset_wall_midpoint_ns",
        "offset_ns",
    )
    if offset is None:
        return None, ""
    return headset_timestamp_ns - offset, "clock_corrected_wall"


def _has_metric_row(
    run_dir: Path,
    file_name: str,
    predicate: Callable[[Mapping[str, str]], bool],
    window: Optional[Tuple[int, int]],
) -> bool:
    path = Path(run_dir) / file_name
    if not path.exists():
        return False
    try:
        rows = _filter_measurement_rows(_read_csv_rows(path), window)
    except Exception:
        return False
    return any(predicate(row) for row in rows)


def _contains_truthy_key(payload: object, key: str) -> bool:
    if isinstance(payload, Mapping):
        for item_key, item_value in payload.items():
            if str(item_key) == key and bool(item_value):
                return True
            if _contains_truthy_key(item_value, key):
                return True
    elif isinstance(payload, list):
        return any(_contains_truthy_key(item, key) for item in payload)
    return False


def _expects_control_metrics(manifest: Optional[Mapping[str, object]]) -> bool:
    if not manifest:
        return False
    experiment = str(manifest.get("experiment") or "").strip().lower()
    condition = str(manifest.get("condition") or "").strip().lower()
    if experiment.startswith("e6_"):
        return True
    if any(token in condition for token in ("control", "goal_load", "teleop")):
        return True
    return _contains_truthy_key(manifest, "control_load")


def _command_clock_tolerance_ns(run_dir: Path, clock_samples: List[Dict[str, int]]) -> int:
    """Tolerance for pairing headset command rows with bridge publish rows.

    The command row and the bridge publish row are emitted by different processes
    and clocks. If a bridge publish timestamp lands slightly before the corrected
    headset command timestamp, accept it only inside this clock-sync uncertainty
    band. Larger inversions are not paired.
    """
    rtt_candidates = [sample.get("round_trip_ns", 0) for sample in clock_samples]
    payload = _load_clock_sync(run_dir)
    if payload and isinstance(payload.get("summary"), Mapping):
        summary = payload["summary"]
        rtt_candidates.append(_parse_int(summary.get("round_trip_ns_max")))
        rtt_candidates.append(_parse_int(summary.get("round_trip_ns_min")))
    max_rtt_ns = max([value for value in rtt_candidates if value > 0], default=0)
    return max(75_000_000, int(max_rtt_ns / 2) + 20_000_000)


def _iter_command_latency_rows(
    run_dir: Path,
    clock_samples: List[Dict[str, int]],
    default_window: Optional[Tuple[int, int]],
) -> Iterable[Dict[str, object]]:
    command_path = Path(run_dir) / "headset_command_metrics.csv"
    bridge_path = Path(run_dir) / "bridge_metrics.csv"
    if not command_path.exists() or not bridge_path.exists():
        return []

    command_window = _metric_window(run_dir, "headset_command_metrics.csv", default_window)
    try:
        command_rows = _filter_measurement_rows(_read_csv_rows(command_path), command_window)
        bridge_rows = _filter_measurement_rows(_read_csv_rows(bridge_path), default_window)
    except Exception:
        return []

    bridge_by_topic: Dict[str, List[Mapping[str, str]]] = {}
    for row in bridge_rows:
        if (row.get("category") or "") != "unity_to_ros":
            continue
        if (row.get("name") or "") != "published":
            continue
        topic = (row.get("destination") or "").strip()
        timestamp_ns = _parse_int(row.get("timestamp_ns"))
        if not topic or timestamp_ns <= 0:
            continue
        bridge_by_topic.setdefault(topic, []).append(row)
    for rows in bridge_by_topic.values():
        rows.sort(key=lambda item: _parse_int(item.get("timestamp_ns")))

    tolerance_ns = _command_clock_tolerance_ns(run_dir, clock_samples)
    bridge_index_by_topic: Dict[str, int] = {}
    output_rows: List[Dict[str, object]] = []
    for row in sorted(command_rows, key=lambda item: _parse_int(item.get("timestamp_ns"))):
        topic = (row.get("topic") or "").strip()
        if not topic or topic not in bridge_by_topic:
            continue
        headset_ts = _parse_int(row.get("timestamp_ns"))
        headset_monotonic_ns = _parse_int(row.get("monotonic_ns"))
        corrected_ts, correction_method = _headset_to_host_time_ns(headset_ts, clock_samples, headset_monotonic_ns)
        if corrected_ts is None:
            continue

        rows = bridge_by_topic[topic]
        index = bridge_index_by_topic.get(topic, 0)
        while index < len(rows) and _parse_int(rows[index].get("timestamp_ns")) < corrected_ts - tolerance_ns:
            index += 1
        if index >= len(rows):
            bridge_index_by_topic[topic] = index
            continue

        bridge_row = rows[index]
        bridge_index_by_topic[topic] = index + 1
        bridge_ts = _parse_int(bridge_row.get("timestamp_ns"))
        latency_ms = (bridge_ts - corrected_ts) / 1_000_000.0
        notes = f"{correction_method};topic_fifo_bridge_publish"
        if latency_ms < 0:
            notes += ";negative_within_clock_uncertainty"
        output_rows.append(
            {
                "timestamp_ns": bridge_ts,
                "run_id": row.get("run_id", ""),
                "experiment": row.get("experiment", ""),
                "condition": row.get("condition", ""),
                "source": "analysis",
                "category": "command_latency",
                "name": f"{row.get('name') or 'command'}_to_bridge_publish",
                "topic": topic,
                "stream": row.get("stream", ""),
                "seq": row.get("seq", ""),
                "join_strategy": "topic_fifo_bridge_publish",
                "source_timestamp_ns": corrected_ts,
                "headset_timestamp_ns": headset_ts,
                "corrected_headset_timestamp_ns": corrected_ts,
                "latency_ms": f"{latency_ms:.6f}",
                "payload_bytes": bridge_row.get("payload_bytes", ""),
                "notes": notes,
            }
        )
    return output_rows


def write_derived_metrics(run_dir: Path) -> Optional[Path]:
    run_dir = Path(run_dir)
    headset_path = run_dir / "headset_metrics.csv"
    if not headset_path.exists():
        return None

    clock_samples = _clock_samples(run_dir)
    if not clock_samples:
        return None

    default_window = _measurement_window(run_dir)
    headset_window = _metric_window(run_dir, "headset_metrics.csv", default_window)
    try:
        headset_rows = _filter_measurement_rows(_read_csv_rows(headset_path), headset_window)
    except Exception:
        return None

    source_rows_by_key: Dict[Tuple[str, int], Mapping[str, str]] = {}
    source_path = run_dir / "source_metrics.csv"
    if source_path.exists():
        try:
            for row in _filter_measurement_rows(_read_csv_rows(source_path), default_window):
                topic = (row.get("topic") or "").strip()
                seq = _parse_int(row.get("seq"))
                if topic and seq > 0:
                    source_rows_by_key[(topic, seq)] = row
        except Exception:
            source_rows_by_key = {}

    output_path = run_dir / "derived_metrics.csv"
    fieldnames = [
        "timestamp_ns",
        "run_id",
        "experiment",
        "condition",
        "source",
        "category",
        "name",
        "topic",
        "stream",
        "seq",
        "join_strategy",
        "source_timestamp_ns",
        "headset_timestamp_ns",
        "corrected_headset_timestamp_ns",
        "latency_ms",
        "payload_bytes",
        "notes",
    ]
    rows_written = 0
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in headset_rows:
            if (row.get("category") or "") != "camera":
                continue
            name = row.get("name") or ""
            if name not in {"received_compressed", "displayed_compressed", "received_raw", "displayed_raw"}:
                continue
            topic = (row.get("topic") or "").strip()
            headset_ts = _parse_int(row.get("timestamp_ns"))
            headset_monotonic_ns = _parse_int(row.get("monotonic_ns"))
            corrected_ts, correction_method = _headset_to_host_time_ns(headset_ts, clock_samples, headset_monotonic_ns)
            if corrected_ts is None:
                continue

            extra = _parse_extra_json(row)
            source_stamp_ns = _parse_int(extra.get("source_stamp_ns"))
            seq = _parse_int(row.get("seq"))
            join_strategy = "header_stamp"
            # The only sound join is on the source header stamp carried in the headset row's
            # extra_json (`source_stamp_ns`), because it identifies the exact source frame. Do not
            # fall back to (topic, seq): source rows and headset rows use independent counters, so
            # equal sequence numbers can be unrelated frames on real data.
            if source_stamp_ns <= 0:
                continue

            latency_ms = (corrected_ts - source_stamp_ns) / 1_000_000.0
            writer.writerow(
                {
                    "timestamp_ns": corrected_ts,
                    "run_id": row.get("run_id", ""),
                    "experiment": row.get("experiment", ""),
                    "condition": row.get("condition", ""),
                    "source": "analysis",
                    "category": "latency",
                    "name": f"camera_{name}",
                    "topic": topic,
                    "stream": row.get("stream", ""),
                    "seq": seq,
                    "join_strategy": join_strategy,
                    "source_timestamp_ns": source_stamp_ns,
                    "headset_timestamp_ns": headset_ts,
                    "corrected_headset_timestamp_ns": corrected_ts,
                    "latency_ms": f"{latency_ms:.6f}",
                    "payload_bytes": row.get("payload_bytes", ""),
                    "notes": correction_method,
                }
            )
            rows_written += 1

        for derived_row in _iter_command_latency_rows(run_dir, clock_samples, default_window):
            writer.writerow(derived_row)
            rows_written += 1

    return output_path if rows_written > 0 else output_path


def _inter_arrival_stats_ms(timestamps_ns: Iterable[int]) -> Optional[Dict[str, float]]:
    ordered = sorted(int(t) for t in timestamps_ns if t)
    if len(ordered) < 2:
        return None
    deltas = [b - a for a, b in zip(ordered, ordered[1:])]
    return {
        "mean_ms": mean(deltas) / 1_000_000.0,
        "stddev_ms": pstdev(deltas) / 1_000_000.0,
        "p95_ms": percentile(deltas, 95) / 1_000_000.0,
    }


# Minimum matched frames before a jitter (stddev) figure is reportable; below this the sample
# standard deviation is dominated by noise and should not be quoted in the paper.
_JITTER_MIN_SAMPLES = 10


def _latency_stats(values: List[float]) -> Dict[str, float]:
    return {
        "mean": mean(values),
        "stddev": pstdev(values),
        "min": min(values),
        "p50": median(values),
        "p95": percentile(values, 95),
        "p99": percentile(values, 99),
        "max": max(values),
    }


def write_derived_summary(run_dir: Path) -> Optional[Path]:
    """Compute the paper-facing derived metrics the raw column percentiles cannot express:
    per-stream end-to-end latency + latency jitter, throughput (bytes/s), drop_rate
    (1 - delivered/sent), inter-arrival jitter, and the bridge per-lane queueing latency
    (time-in-queue) + drop/evict counts. Written to derived_summary.json."""
    run_dir = Path(run_dir)
    window = _measurement_window(run_dir)
    if window is None:
        return None
    window_s = (window[1] - window[0]) / 1_000_000_000.0
    if window_s <= 0:
        return None

    out: Dict[str, object] = {"window_duration_s": window_s, "streams": {}, "bridge": {}}

    # Clock-offset uncertainty band = half the minimum round-trip (the NTP bound). This is the
    # irreducible per-message latency uncertainty from cross-machine synchronization; report it
    # alongside (combined with) the run-to-run CI so latency error bars are honest (B1.d).
    clock_payload = _load_clock_sync(run_dir)
    clock_uncertainty_ms: Optional[float] = None
    if clock_payload and isinstance(clock_payload.get("summary"), dict):
        rtt_min_ns = _parse_int(clock_payload["summary"].get("round_trip_ns_min"))
        if rtt_min_ns > 0:
            clock_uncertainty_ms = rtt_min_ns / 2 / 1_000_000.0
            out["clock_offset_uncertainty_ms"] = clock_uncertainty_ms

    # Source-clock validity (M3). Latency is only glass-to-glass if the publisher shares the host
    # clock (co-located workload). The orchestrator stamps this into the manifest; surface it so a
    # reader knows whether the latency figures are trustworthy or carry an uncorrected source offset.
    source_clock_synchronized: Optional[bool] = None
    manifest = _load_manifest(run_dir)
    if manifest and isinstance(manifest.get("source_clock"), dict):
        sync_flag = manifest["source_clock"].get("synchronized")
        if isinstance(sync_flag, bool):
            source_clock_synchronized = sync_flag
            out["source_clock_synchronized"] = sync_flag
            if not sync_flag:
                out["latency_warning"] = (
                    "source clock unsynchronized â€” end-to-end latency is NOT glass-to-glass and "
                    "must be excluded or corrected (see manifest source_clock)"
                )

    # Source side: per (stream, topic) sent count, bytes, achieved Hz, inter-arrival jitter.
    source_groups: Dict[str, Dict[str, object]] = {}
    source_path = run_dir / "source_metrics.csv"
    if source_path.exists():
        try:
            for row in _filter_measurement_rows(_read_csv_rows(source_path), window):
                key = f"{(row.get('stream') or '').strip()}|{(row.get('topic') or '').strip()}"
                group = source_groups.setdefault(key, {"count": 0, "bytes": 0, "ts": []})
                group["count"] = int(group["count"]) + 1  # type: ignore[arg-type]
                group["bytes"] = int(group["bytes"]) + _parse_int(row.get("payload_bytes"))  # type: ignore[arg-type]
                timestamp = _parse_int(row.get("timestamp_ns"))
                if timestamp:
                    group["ts"].append(timestamp)  # type: ignore[union-attr]
        except Exception:
            source_groups = {}

    # Headset side: per-topic frame counts split by *endpoint* (M2). `received_*` rows are emitted
    # on every network arrival; `displayed_*` rows only for the frame that survives the headset
    # coalesce-to-latest drain (CameraImageVisualizer drops all-but-newest before presenting). The
    # paper's user-visible drop is displayed/sent, NOT received/sent â€” equating "received" with
    # "shown to the user" hides exactly the headset-side starvation the study is about.
    headset_received: Dict[str, int] = {}
    headset_displayed: Dict[str, int] = {}
    headset_camera_topics: set = set()
    headset_path = run_dir / "headset_metrics.csv"
    if headset_path.exists():
        headset_window = _metric_window(run_dir, "headset_metrics.csv", window)
        try:
            for row in _filter_measurement_rows(_read_csv_rows(headset_path), headset_window):
                if (row.get("category") or "") != "camera":
                    continue
                name = row.get("name") or ""
                topic = (row.get("topic") or "").strip()
                if name.startswith("received"):
                    headset_received[topic] = headset_received.get(topic, 0) + 1
                    headset_camera_topics.add(topic)
                elif name.startswith("displayed"):
                    headset_displayed[topic] = headset_displayed.get(topic, 0) + 1
                    headset_camera_topics.add(topic)
        except Exception:
            headset_received = {}
            headset_displayed = {}
            headset_camera_topics = set()

    # Derived per-frame latency per camera topic, split by endpoint (M1). `displayed_*` rows are the
    # glass-to-glass figure (source stamp -> frame actually presented); `received_*` is network-
    # arrival latency. Pooling the two produced a bimodal blend over ~2x the true sample count, so
    # the reported p95/jitter were meaningless. Keep them separate; report displayed as headline.
    latency_displayed: Dict[str, List[float]] = {}
    latency_received: Dict[str, List[float]] = {}
    latency_ts_displayed: Dict[str, List[int]] = {}
    command_latency: Dict[str, List[float]] = {}
    command_negative_counts: Dict[str, int] = {}
    derived_path = run_dir / "derived_metrics.csv"
    if derived_path.exists():
        try:
            for row in _read_csv_rows(derived_path):
                category = row.get("category") or ""
                topic = (row.get("topic") or "").strip()
                name = row.get("name") or ""
                try:
                    latency = float(row.get("latency_ms") or "")
                except ValueError:
                    continue
                if category == "command_latency":
                    if topic:
                        command_latency.setdefault(topic, []).append(latency)
                        if latency < 0:
                            command_negative_counts[topic] = command_negative_counts.get(topic, 0) + 1
                    continue
                if category != "latency":
                    continue
                if "displayed" in name:
                    latency_displayed.setdefault(topic, []).append(latency)
                    ts = _parse_int(row.get("corrected_headset_timestamp_ns"))
                    if ts:
                        latency_ts_displayed.setdefault(topic, []).append(ts)
                else:
                    # received_* (network arrival); any unlabeled legacy row falls here.
                    latency_received.setdefault(topic, []).append(latency)
        except Exception:
            latency_displayed = {}
            latency_received = {}

    streams: Dict[str, object] = {}
    for key, group in source_groups.items():
        stream, _, topic = key.partition("|")
        count = int(group["count"])  # type: ignore[arg-type]
        byte_total = int(group["bytes"])  # type: ignore[arg-type]
        entry: Dict[str, object] = {
            "sent_count": count,
            "bytes": byte_total,
            "throughput_bytes_per_s": byte_total / window_s,
            "achieved_hz": count / window_s,
        }
        source_ia = _inter_arrival_stats_ms(group["ts"])  # type: ignore[arg-type]
        if source_ia:
            entry["source_inter_arrival"] = source_ia
        # Frame delivery accounting (M2). Report THREE distinct rates, never conflated:
        #   delivery_rate  = received/sent   (reached the headset over the wire)
        #   display_rate   = displayed/sent  (actually presented to the user)
        #   drop_rate      = 1 - display_rate (the headline: frames the user never saw)
        #   headset_coalesce_rate = 1 - displayed/received (shed by the headset coalesce-to-latest)
        # A camera stream with zero headset rows is fully starved (drop_rate=1.0), NOT absent â€” it
        # must stay in the summary so aggregation cannot silently survivor-bias the worst run away.
        is_camera = stream == "camera" or topic in headset_camera_topics
        if is_camera and count > 0:
            received = headset_received.get(topic, 0)
            displayed = headset_displayed.get(topic, 0)
            entry["received_count"] = received
            entry["displayed_count"] = displayed
            entry["received_hz"] = received / window_s
            entry["displayed_hz"] = displayed / window_s
            entry["delivery_rate"] = min(1.0, received / count)
            entry["display_rate"] = min(1.0, displayed / count)
            entry["drop_rate"] = max(0.0, 1.0 - displayed / count)
            if received > 0:
                entry["headset_coalesce_rate"] = max(0.0, 1.0 - displayed / received)
            elif displayed == 0:
                # nothing arrived and nothing displayed: every sent frame was lost upstream.
                entry["headset_coalesce_rate"] = 0.0
        # Glass-to-glass latency = displayed rows (M1). received_* is reported separately as arrival.
        displayed_lat = latency_displayed.get(topic)
        if displayed_lat:
            entry["matched_count"] = len(displayed_lat)
            entry["latency_ms"] = _latency_stats(displayed_lat)
            # Jitter needs a minimum sample or it is noise; use sample stdev and always expose n.
            if len(displayed_lat) >= _JITTER_MIN_SAMPLES:
                entry["latency_jitter_ms_stddev"] = stdev(displayed_lat)
            if clock_uncertainty_ms is not None:
                entry["latency_clock_uncertainty_ms"] = clock_uncertainty_ms
            if source_clock_synchronized is not None:
                entry["latency_source_clock_synchronized"] = source_clock_synchronized
            displayed_ia = _inter_arrival_stats_ms(latency_ts_displayed.get(topic, []))
            if displayed_ia:
                entry["displayed_inter_arrival"] = displayed_ia
        arrival_lat = latency_received.get(topic)
        if arrival_lat:
            entry["arrival_matched_count"] = len(arrival_lat)
            entry["arrival_latency_ms"] = _latency_stats(arrival_lat)
        streams[key] = entry
    out["streams"] = streams

    command_counts: Dict[str, Dict[str, int]] = {}
    command_path = run_dir / "headset_command_metrics.csv"
    if command_path.exists():
        command_window = _metric_window(run_dir, "headset_command_metrics.csv", window)
        try:
            for row in _filter_measurement_rows(_read_csv_rows(command_path), command_window):
                topic = (row.get("topic") or "").strip()
                if not topic:
                    continue
                command_counts.setdefault(topic, {"headset_count": 0, "bridge_publish_count": 0})
                command_counts[topic]["headset_count"] += 1
        except Exception:
            command_counts = {}
    bridge_path = run_dir / "bridge_metrics.csv"
    if bridge_path.exists():
        try:
            for row in _filter_measurement_rows(_read_csv_rows(bridge_path), window):
                if (row.get("category") or "") != "unity_to_ros":
                    continue
                if (row.get("name") or "") != "published":
                    continue
                topic = (row.get("destination") or "").strip()
                if not topic:
                    continue
                if topic in command_counts:
                    command_counts[topic]["bridge_publish_count"] += 1
        except Exception:
            pass

    if command_counts or command_latency:
        command_out: Dict[str, object] = {}
        for topic in sorted(set(command_counts) | set(command_latency)):
            counts = command_counts.get(topic, {"headset_count": 0, "bridge_publish_count": 0})
            entry: Dict[str, object] = dict(counts)
            values = command_latency.get(topic, [])
            if values:
                entry["matched_count"] = len(values)
                entry["headset_to_bridge_publish_ms"] = _latency_stats(values)
                if clock_uncertainty_ms is not None:
                    entry["latency_clock_uncertainty_ms"] = clock_uncertainty_ms
                negative_count = command_negative_counts.get(topic, 0)
                if negative_count:
                    entry["negative_within_clock_uncertainty_count"] = negative_count
            command_out[topic] = entry
        out["commands"] = command_out

    # Bridge: per-lane queueing latency (time-in-queue) + drop/evict counts (B5 instrumentation).
    bridge_path = run_dir / "bridge_metrics.csv"
    if bridge_path.exists():
        lane_time_in_queue: Dict[str, List[float]] = {"realtime": [], "bulk": []}
        lane_depth: Dict[str, List[float]] = {"realtime": [], "bulk": []}
        drop_counts: Dict[str, int] = {"dropped": 0, "evicted": 0}
        try:
            for row in _filter_measurement_rows(_read_csv_rows(bridge_path), window):
                if (row.get("category") or "") != "outbound":
                    continue
                name = row.get("name") or ""
                extra = _parse_extra_json(row)
                lane = extra.get("lane")
                if name == "sent" and lane in lane_time_in_queue:
                    tiq = extra.get("time_in_queue_ms")
                    if isinstance(tiq, (int, float)):
                        lane_time_in_queue[lane].append(float(tiq))
                    realtime_depth = extra.get("realtime_depth")
                    bulk_depth = extra.get("bulk_depth")
                    if isinstance(realtime_depth, (int, float)):
                        lane_depth["realtime"].append(float(realtime_depth))
                    if isinstance(bulk_depth, (int, float)):
                        lane_depth["bulk"].append(float(bulk_depth))
                elif name in drop_counts:
                    drop_counts[name] += 1
        except Exception:
            lane_time_in_queue = {"realtime": [], "bulk": []}
        bridge_out: Dict[str, object] = {
            "dropped_count": drop_counts["dropped"],
            "evicted_count": drop_counts["evicted"],
        }
        for lane, values in lane_time_in_queue.items():
            if values:
                bridge_out[f"{lane}_time_in_queue_ms"] = _latency_stats(values)
        for lane, values in lane_depth.items():
            if values:
                bridge_out[f"{lane}_queue_depth"] = {
                    "mean": mean(values),
                    "p95": percentile(values, 95),
                    "max": max(values),
                }
        out["bridge"] = bridge_out

    path = run_dir / "derived_summary.json"
    path.write_text(json.dumps(out, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def metric_row_count(path: Path, measurement_window: Optional[Tuple[int, int]] = None) -> int:
    rows = _read_csv_rows(Path(path))
    return len(_filter_measurement_rows(rows, measurement_window))


_GROUP_COLUMNS = ("category", "name", "stream", "topic")
_IDENTITY_COLUMNS = {"timestamp_ns", "run_id", "experiment", "condition", "extra_json"}


def _column_stats(rows: List[Dict[str, str]], exclude: set) -> Dict[str, object]:
    numeric_columns = set()
    for row in rows:
        for key, value in row.items():
            if key in exclude:
                continue
            try:
                float(value)
            except Exception:
                continue
            numeric_columns.add(key)

    stats: Dict[str, object] = {}
    for column in sorted(numeric_columns):
        values = []
        for row in rows:
            try:
                values.append(float(row[column]))
            except Exception:
                pass
        if values:
            stats[column] = {
                "min": min(values),
                "mean": mean(values),
                "p50": median(values),
                "p95": percentile(values, 95),
                "p99": percentile(values, 99),
                "max": max(values),
                "stddev": pstdev(values),
                "count": len(values),
            }
    return stats


def summarize_numeric_csv(path: Path, measurement_window: Optional[Tuple[int, int]] = None) -> Dict[str, object]:
    all_rows = _read_csv_rows(Path(path))

    rows = _filter_measurement_rows(all_rows, measurement_window)
    summary: Dict[str, object] = {"row_count": len(rows), "row_count_total": len(all_rows)}
    if measurement_window is not None:
        summary["measurement_window_applied"] = True
    if not rows:
        return summary

    # Group rows by their (category, name, stream, topic) identity before computing percentiles, so
    # incommensurable rows are never pooled into one statistic (e.g. headset tf-received duration_ms=0
    # drowning camera decode times -> a meaningless p50=0.0). Columns that are absent in the file are
    # simply not part of the key.
    group_columns = [c for c in _GROUP_COLUMNS if any(c in row for row in rows)]
    exclude = set(_IDENTITY_COLUMNS) | set(group_columns)

    groups: Dict[str, List[Dict[str, str]]] = {}
    for row in rows:
        key = "|".join((row.get(c) or "") for c in group_columns) if group_columns else ""
        groups.setdefault(key, []).append(row)

    if len(groups) <= 1:
        # Homogeneous file: expose flat per-column stats at the top level (backward compatible).
        only_rows = next(iter(groups.values()))
        for column, stats in _column_stats(only_rows, exclude).items():
            summary[column] = stats
        return summary

    # Heterogeneous file: per-group stats only. The pooled per-column statistic is deliberately NOT
    # emitted, because it would mix unrelated quantities.
    by_group: Dict[str, object] = {}
    for key, group_rows in sorted(groups.items()):
        group_stats = _column_stats(group_rows, exclude)
        group_stats["_row_count"] = len(group_rows)
        by_group[key] = group_stats
    summary["group_keys"] = list(group_columns)
    summary["by_group"] = by_group
    return summary


def _load_manifest(run_dir: Path) -> Optional[Dict[str, object]]:
    manifest_path = Path(run_dir) / "run_manifest.json"
    if not manifest_path.exists():
        return None
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None
    return manifest if isinstance(manifest, dict) else None


def _expected_camera_streams(manifest: Optional[Dict[str, object]]) -> int:
    if not manifest:
        return 0

    candidates: List[object] = [
        manifest.get("camera_streams"),
        manifest.get("camera_stream_count"),
    ]
    extra = manifest.get("extra")
    if isinstance(extra, dict):
        workload = extra.get("workload")
        if isinstance(workload, dict):
            camera = workload.get("camera")
            if isinstance(camera, dict):
                candidates.insert(0, camera.get("streams"))

    for value in candidates:
        try:
            streams = int(value)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            continue
        if streams > 0:
            return streams
    return 0


def _camera_staging(manifest: Optional[Dict[str, object]]) -> Optional[Dict[str, object]]:
    extra = (manifest or {}).get("extra")
    workload = extra.get("workload") if isinstance(extra, dict) else {}
    if not isinstance(workload, dict):
        return None
    staging = workload.get("camera_staging")
    if not staging or not isinstance(staging, dict):
        workload_extra = workload.get("extra")
        staging = workload_extra.get("camera_staging") if isinstance(workload_extra, dict) else None
    if not isinstance(staging, dict):
        return None
    enabled = str(staging.get("enabled", True)).strip().lower() not in {"0", "false", "no", "off"}
    if not enabled:
        return None
    counts: List[int] = []
    for value in staging.get("stream_counts") or staging.get("stages") or []:
        try:
            count = int(value)
        except (TypeError, ValueError):
            continue
        counts.append(max(0, count))
    try:
        duration_s = float(staging.get("stage_duration_s") or 0.0)
    except (TypeError, ValueError):
        duration_s = 0.0
    if not counts or duration_s <= 0.0:
        return None
    return {"stream_counts": counts, "stage_duration_s": duration_s}


def _camera_stream_index_from_topic(manifest: Optional[Dict[str, object]], stream_key: object) -> Optional[int]:
    _, _, topic = str(stream_key or "").partition("|")
    if not topic:
        topic = str(stream_key or "")
    parts = [part for part in topic.split("/") if part]
    robot_part = next((part for part in parts if part.startswith("exp_robot_")), "")
    camera_part = next((part for part in parts if part.startswith("camera_")), "")
    try:
        robot_index = int(robot_part.rsplit("_", 1)[1])
        camera_index = int(camera_part.split("_", 1)[1])
    except (IndexError, ValueError):
        return None
    robot_count = max(1, _parse_int((manifest or {}).get("robot_count"), 1))
    return camera_index * robot_count + robot_index


def _camera_stream_active_duration_s(manifest: Optional[Dict[str, object]], stream_key: object) -> Optional[float]:
    staging = _camera_staging(manifest)
    if not staging:
        return None
    stream_index = _camera_stream_index_from_topic(manifest, stream_key)
    if stream_index is None:
        return None
    counts = staging["stream_counts"]
    stage_duration_s = float(staging["stage_duration_s"])
    assert isinstance(counts, list)
    active_stages = sum(1 for count in counts if int(count) > stream_index)
    if active_stages <= 0:
        return 0.0
    return active_stages * stage_duration_s


def _workload_transport(manifest: Optional[Dict[str, object]]) -> str:
    if not manifest:
        return ""
    direct = manifest.get("transport")
    if direct:
        return str(direct).strip().lower()
    extra = manifest.get("extra")
    if isinstance(extra, dict):
        workload = extra.get("workload")
        if isinstance(workload, dict):
            return str(workload.get("transport") or "").strip().lower()
    return ""


def _is_webrtc_workload(manifest: Optional[Dict[str, object]]) -> bool:
    if "webrtc" in _workload_transport(manifest):
        return True
    condition = str((manifest or {}).get("condition") or "").strip().lower()
    return "webrtc" in condition


def _validate_source_camera_animation(
    run_dir: Path,
    window: Optional[Tuple[int, int]],
    errors: List[str],
    warnings: List[str],
) -> None:
    manifest = _load_manifest(run_dir)
    expected_streams = _expected_camera_streams(manifest)
    if expected_streams <= 0:
        return

    source_path = Path(run_dir) / "source_metrics.csv"
    if not source_path.exists():
        return

    try:
        rows = _filter_measurement_rows(_read_csv_rows(source_path), window)
    except Exception as exc:
        warnings.append(f"source camera animation could not be validated: {exc}")
        return

    topics: Dict[str, Dict[str, object]] = {}
    for row in rows:
        stream = (row.get("stream") or "").strip().lower()
        topic = (row.get("topic") or "").strip()
        if stream != "camera" and "/camera_" not in topic:
            continue

        key = topic or row.get("robot_id") or "camera"
        stats = topics.setdefault(key, {"rows": 0, "phases": set(), "payload_sizes": set()})
        stats["rows"] = int(stats["rows"]) + 1

        notes = (row.get("notes") or "").strip()
        for part in notes.split(";"):
            part = part.strip()
            if part.startswith("frame_phase="):
                phases = stats["phases"]
                assert isinstance(phases, set)
                phases.add(part)

        payload_size = (row.get("payload_bytes") or "").strip()
        if payload_size:
            payload_sizes = stats["payload_sizes"]
            assert isinstance(payload_sizes, set)
            payload_sizes.add(payload_size)

    if not topics:
        errors.append("source_metrics.csv did not contain camera stream rows")
        return

    if len(topics) < expected_streams:
        errors.append(
            "source camera stream count mismatch: "
            f"expected {expected_streams}, observed {len(topics)}"
        )

    for topic, stats in sorted(topics.items()):
        rows_count = int(stats["rows"])
        if rows_count < 3:
            continue
        phases = stats["phases"]
        payload_sizes = stats["payload_sizes"]
        assert isinstance(phases, set)
        assert isinstance(payload_sizes, set)
        if len(phases) < 2 and len(payload_sizes) < 2:
            errors.append(
                "source camera topic did not show animated frame variation "
                f"during measurement window: {topic}"
            )


def _expected_pointcloud_streams(manifest: Optional[Dict[str, object]]) -> int:
    workload = ((manifest or {}).get("extra") or {})
    if isinstance(workload, dict):
        workload = workload.get("workload") or {}
    if not isinstance(workload, dict):
        return 0

    robot_count = _parse_int((manifest or {}).get("robot_count"), 1)
    pointcloud = workload.get("pointcloud") if isinstance(workload.get("pointcloud"), dict) else {}
    points = _parse_int(pointcloud.get("points") if isinstance(pointcloud, dict) else 0)
    hz = _to_float(pointcloud.get("hz") if isinstance(pointcloud, dict) else 0)
    if points <= 0 or hz is None or hz <= 0:
        return 0

    count = max(0, robot_count)
    map_cfg = workload.get("map") if isinstance(workload.get("map"), dict) else {}
    representation = str(map_cfg.get("representation") if isinstance(map_cfg, dict) else "").lower()
    if "pointcloud" in representation:
        count += 1
    return count


def _expected_mesh_streams(manifest: Optional[Dict[str, object]]) -> int:
    workload = ((manifest or {}).get("extra") or {})
    if isinstance(workload, dict):
        workload = workload.get("workload") or {}
    if not isinstance(workload, dict):
        return 0

    map_cfg = workload.get("map") if isinstance(workload.get("map"), dict) else {}
    representation = str(map_cfg.get("representation") if isinstance(map_cfg, dict) else "").lower()
    triangles = _parse_int(map_cfg.get("triangles") if isinstance(map_cfg, dict) else 0)
    chunks = _parse_int(map_cfg.get("chunks") if isinstance(map_cfg, dict) else 0)
    hz = _to_float(map_cfg.get("hz") if isinstance(map_cfg, dict) else 0)
    if "mesh" not in representation or triangles <= 0 or chunks <= 0 or hz is None or hz <= 0:
        return 0
    return 1


def _validate_mesh_delivery(
    run_dir: Path,
    source_window: Optional[Tuple[int, int]],
    headset_window: Optional[Tuple[int, int]],
    errors: List[str],
    warnings: List[str],
) -> None:
    manifest = _load_manifest(run_dir)
    if _expected_mesh_streams(manifest) <= 0:
        return

    source_path = Path(run_dir) / "source_metrics.csv"
    if not source_path.exists():
        return

    try:
        source_rows = _filter_measurement_rows(_read_csv_rows(source_path), source_window)
    except Exception as exc:
        warnings.append(f"source mesh publication could not be validated: {exc}")
        return

    mesh_source_rows = [
        row for row in source_rows
        if (row.get("stream") or "").strip().lower() == "mesh"
    ]
    if not mesh_source_rows:
        errors.append("source_metrics.csv did not contain mesh publication samples during the measurement window")
        return

    headset_path = Path(run_dir) / "headset_metrics.csv"
    if not headset_path.exists():
        return

    try:
        headset_rows = _filter_measurement_rows(_read_csv_rows(headset_path), headset_window)
    except Exception as exc:
        warnings.append(f"headset mesh delivery could not be validated: {exc}")
        return

    received_rows = [
        row for row in headset_rows
        if (row.get("category") or "") == "mesh_map"
        and (row.get("name") or "") in {"marker_received", "marker_array_received"}
    ]
    finalized_rows = [
        row for row in headset_rows
        if (row.get("category") or "") == "mesh_map"
        and (row.get("name") or "") == "finalized"
    ]

    if not received_rows:
        errors.append(
            "headset_metrics.csv did not contain mesh_map marker_received samples during the "
            "measurement window despite source mesh publication"
        )
    if not finalized_rows:
        errors.append(
            "headset_metrics.csv did not contain mesh_map finalized samples during the "
            "measurement window despite source mesh publication"
        )


def _validate_pointcloud_payload_variation(
    run_dir: Path,
    source_window: Optional[Tuple[int, int]],
    headset_window: Optional[Tuple[int, int]],
    errors: List[str],
    warnings: List[str],
) -> None:
    manifest = _load_manifest(run_dir)
    if _expected_pointcloud_streams(manifest) <= 0:
        return

    source_path = Path(run_dir) / "source_metrics.csv"
    if not source_path.exists():
        return

    try:
        source_rows = _filter_measurement_rows(_read_csv_rows(source_path), source_window)
    except Exception as exc:
        warnings.append(f"source pointcloud animation could not be validated: {exc}")
        return

    phases_by_topic: Dict[str, set] = {}
    rows_by_topic: Dict[str, int] = {}
    for row in source_rows:
        stream = (row.get("stream") or "").strip().lower()
        if stream not in {"pointcloud", "map_pointcloud"}:
            continue
        topic = (row.get("topic") or "").strip() or stream
        rows_by_topic[topic] = rows_by_topic.get(topic, 0) + 1
        for part in (row.get("notes") or "").split(";"):
            part = part.strip()
            if part.startswith("frame_phase="):
                phases_by_topic.setdefault(topic, set()).add(part)

    for topic, row_count in sorted(rows_by_topic.items()):
        if row_count >= 3 and len(phases_by_topic.get(topic, set())) < 2:
            errors.append(
                "source pointcloud topic did not show animated payload variation "
                f"during measurement window: {topic}"
            )

    headset_path = Path(run_dir) / "headset_metrics.csv"
    if not headset_path.exists():
        return

    try:
        headset_rows = _filter_measurement_rows(_read_csv_rows(headset_path), headset_window)
    except Exception as exc:
        warnings.append(f"headset pointcloud upload could not be validated: {exc}")
        return

    stats: Dict[str, Dict[str, int]] = {}
    for row in headset_rows:
        if (row.get("category") or "") != "pointcloud":
            continue
        topic = (row.get("topic") or "").strip() or "pointcloud"
        topic_stats = stats.setdefault(topic, {"received": 0, "uploaded": 0, "identical_skipped": 0})
        name = row.get("name") or ""
        if name == "received":
            topic_stats["received"] += 1
        elif name == "uploaded":
            topic_stats["uploaded"] += 1
        elif name == "skipped":
            extra = _parse_extra_json(row)
            if extra.get("skipped_identical_bytes") is True:
                topic_stats["identical_skipped"] += 1

    for topic, topic_stats in sorted(stats.items()):
        received = topic_stats["received"]
        uploaded = topic_stats["uploaded"]
        identical_skipped = topic_stats["identical_skipped"]
        if received >= 3 and uploaded <= 1 and identical_skipped > uploaded:
            errors.append(
                "headset pointcloud uploads were dominated by identical-payload skips "
                f"for {topic}: received={received}, uploaded={uploaded}, "
                f"identical_skipped={identical_skipped}"
            )


def _validate_sample_completeness(
    run_dir: Path,
    window: Optional[Tuple[int, int]],
    warnings: List[str],
) -> None:
    """Flag (as warnings) timestamp non-monotonicity and per-topic sequence gaps in every metrics
    stream, so a silently truncated or reordered capture is visible rather than averaged over."""
    for csv_path in sorted(Path(run_dir).glob("*_metrics.csv")):
        metric_window = _metric_window(run_dir, csv_path.name, window)
        try:
            rows = _filter_measurement_rows(_read_csv_rows(csv_path), metric_window)
        except Exception:
            continue
        if not rows:
            continue

        previous: Optional[int] = None
        out_of_order = 0
        for row in rows:
            timestamp = _parse_int(row.get("timestamp_ns"), -1)
            if timestamp < 0:
                continue
            if previous is not None and timestamp < previous:
                out_of_order += 1
            previous = timestamp
        if out_of_order > 0:
            warnings.append(
                f"{csv_path.name}: {out_of_order} out-of-order timestamp_ns row(s) in the "
                "measurement window (non-monotonic capture)"
            )

        if "seq" in rows[0] and "topic" in rows[0]:
            seqs_by_topic: Dict[str, set] = {}
            for row in rows:
                topic = (row.get("topic") or "").strip()
                seq = _parse_int(row.get("seq"), -1)
                if topic and seq > 0:
                    seqs_by_topic.setdefault(topic, set()).add(seq)
            for topic, seqs in sorted(seqs_by_topic.items()):
                if len(seqs) < 3:
                    continue
                lo, hi = min(seqs), max(seqs)
                missing = (hi - lo + 1) - len(seqs)
                if missing > 0:
                    warnings.append(
                        f"{csv_path.name}: topic {topic} has {missing} missing seq value(s) "
                        f"over [{lo}, {hi}] (sequence gap)"
                    )


def _nested_number(payload: Mapping[str, object], *keys: str) -> Optional[float]:
    current: object = payload
    for key in keys:
        if not isinstance(current, Mapping):
            return None
        current = current.get(key)
    if isinstance(current, bool):
        return None
    if isinstance(current, (int, float)):
        return float(current)
    try:
        return float(str(current))
    except (TypeError, ValueError):
        return None


def _performance_envelope_violations(run_dir: Path) -> List[str]:
    """Classify complete measurements that are valid but outside the pre-registered usability
    envelope. These are not data-quality failures; they define the operating boundary."""
    violations: List[str] = []
    manifest = _load_manifest(run_dir)
    webrtc_workload = _is_webrtc_workload(manifest)

    summary_path = Path(run_dir) / "summary.json"
    if summary_path.exists():
        try:
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            summary = {}
        headset = summary.get("headset_metrics.csv")
        if isinstance(headset, Mapping):
            groups = headset.get("by_group")
            if isinstance(groups, Mapping):
                for key, value in groups.items():
                    if not isinstance(value, Mapping):
                        continue
                    key_text = str(key).lower()
                    if "frame|render" not in key_text and "render" not in key_text:
                        continue
                    fps = _nested_number(value, "value", "p50")
                    if fps is not None and fps < 60.0:
                        violations.append(f"Quest render FPS p50 {fps:.1f} < 60.0")

    derived_path = Path(run_dir) / "derived_summary.json"
    if derived_path.exists():
        try:
            derived = json.loads(derived_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            derived = {}
        streams = derived.get("streams")
        if isinstance(streams, Mapping):
            for stream_key, stream_payload in streams.items():
                if not isinstance(stream_payload, Mapping):
                    continue
                key_text = str(stream_key).lower()
                drop_rate = _nested_number(stream_payload, "drop_rate")
                if drop_rate is not None:
                    displayed_hz = _nested_number(stream_payload, "displayed_hz")
                    if key_text.startswith("camera|") or "|camera" in key_text:
                        active_duration_s = _camera_stream_active_duration_s(manifest, stream_key)
                        displayed_count = _nested_number(stream_payload, "displayed_count")
                        if (
                            active_duration_s is not None
                            and active_duration_s > 0.0
                            and displayed_count is not None
                        ):
                            displayed_hz = displayed_count / active_duration_s
                        # Overview/minimap cameras are intentionally coalesced to latest-frame under
                        # load. The usability target is the frame rate the user actually sees, not
                        # a strict displayed/sent ratio. Teleop/WebRTC streams keep the tighter 30 Hz
                        # target; overview streams need at least 15 Hz.
                        target_hz = 30.0 if webrtc_workload or "teleop" in key_text or "webrtc" in key_text else 15.0
                        if displayed_hz is None:
                            if drop_rate > 0.05:
                                violations.append(f"{stream_key} drop_rate {drop_rate:.1%} > 5%")
                        elif displayed_hz < target_hz:
                            violations.append(
                                f"{stream_key} displayed_hz {displayed_hz:.1f} < {target_hz:.1f}"
                            )
                    elif drop_rate > 0.05:
                        violations.append(f"{stream_key} drop_rate {drop_rate:.1%} > 5%")
        bridge = derived.get("bridge")
        if isinstance(bridge, Mapping):
            dropped = _nested_number(bridge, "dropped_count") or 0.0
            evicted = _nested_number(bridge, "evicted_count") or 0.0
            if dropped > 0 or evicted > 0:
                violations.append(
                    f"bridge dropped/evicted messages observed: dropped={int(dropped)}, evicted={int(evicted)}"
                )

    return violations


def write_summary(run_dir: Path) -> Path:
    run_dir = Path(run_dir)
    write_derived_metrics(run_dir)
    write_derived_summary(run_dir)
    output: Dict[str, object] = {}
    window = _measurement_window(run_dir)
    if window is not None:
        output["_measurement_window"] = {
            "start_timestamp_ns": window[0],
            "end_timestamp_ns": window[1],
            "duration_s": (window[1] - window[0]) / 1_000_000_000.0,
        }
    for csv_path in sorted(run_dir.glob("*_metrics.csv")):
        metric_window = _metric_window(run_dir, csv_path.name, window)
        output[csv_path.name] = summarize_numeric_csv(csv_path, metric_window)
    summary_path = run_dir / "summary.json"
    summary_path.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary_path


def validate_run_directory(run_dir: Path, required_metrics: Mapping[str, int]) -> Dict[str, object]:
    run_dir = Path(run_dir)
    write_derived_metrics(run_dir)
    write_derived_summary(run_dir)
    files: Dict[str, object] = {}
    errors: List[str] = []
    warnings: List[str] = []
    window = _measurement_window(run_dir)
    headset_window = _measurement_window_from_events(run_dir / "headset_events.ndjson")
    manifest = _load_manifest(run_dir)
    if window is not None:
        manifest_path = run_dir / "run_manifest.json"
        if manifest_path.exists():
            try:
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                expected_duration = float(manifest.get("duration_s"))
                measured_duration = (window[1] - window[0]) / 1_000_000_000.0
                tolerance = max(0.5, expected_duration * 0.02)
                if abs(measured_duration - expected_duration) > tolerance:
                    errors.append(
                        "measurement window duration does not match manifest "
                        f"duration_s: expected {expected_duration:.3f}s, measured {measured_duration:.3f}s"
                    )
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                errors.append(f"run_manifest.json duration_s could not be validated: {exc}")
    if headset_window is not None:
        manifest_path = run_dir / "run_manifest.json"
        if manifest_path.exists():
            try:
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                expected_duration = float(manifest.get("duration_s"))
                measured_duration = (headset_window[1] - headset_window[0]) / 1_000_000_000.0
                tolerance = max(1.0, expected_duration * 0.05)
                if abs(measured_duration - expected_duration) > tolerance:
                    warnings.append(
                        "headset measurement window duration differs from manifest "
                        f"duration_s: expected {expected_duration:.3f}s, measured {measured_duration:.3f}s"
                    )
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                warnings.append(f"headset measurement duration could not be validated: {exc}")

    for csv_path in sorted(run_dir.glob("*_metrics.csv")):
        try:
            metric_window = _metric_window(run_dir, csv_path.name, window)
            total_rows = metric_row_count(csv_path)
            measured_rows = metric_row_count(csv_path, metric_window)
            files[csv_path.name] = {
                "exists": True,
                "row_count": measured_rows,
                "row_count_total": total_rows,
            }
        except Exception as exc:
            files[csv_path.name] = {"exists": True, "error": str(exc)}
            errors.append(f"{csv_path.name} could not be read: {exc}")

    for file_name, minimum_rows in required_metrics.items():
        path = run_dir / file_name
        if not path.exists():
            files[file_name] = {"exists": False, "row_count_total": 0}
            errors.append(f"{file_name} is required but was not created")
            continue
        metric_window = _metric_window(run_dir, file_name, window)
        total_rows = metric_row_count(path)
        measured_rows = metric_row_count(path, metric_window)
        files[file_name] = {"exists": True, "row_count": measured_rows, "row_count_total": total_rows}
        rows = measured_rows if metric_window is not None else total_rows
        if rows < minimum_rows:
            scope = "measurement-window" if metric_window is not None else "total"
            errors.append(f"{file_name} is required to contain at least {minimum_rows} {scope} data row(s), found {rows}")

    if window is None:
        warnings.append("measurement_start/measurement_end were not found; summary includes all rows")

    _validate_source_camera_animation(run_dir, window, errors, warnings)
    _validate_pointcloud_payload_variation(run_dir, window, headset_window, errors, warnings)
    _validate_mesh_delivery(run_dir, window, headset_window, errors, warnings)
    _validate_sample_completeness(run_dir, window, warnings)

    headset_required = int(required_metrics.get("headset_metrics.csv", 0) or 0) > 0
    if headset_required:
        clock_sync = _load_clock_sync(run_dir)
        clock_negative_tolerance_ms = 2.0
        if not clock_sync:
            errors.append("clock_sync.json is required for headset latency runs but was not created")
        else:
            samples = clock_sync.get("samples")
            if not isinstance(samples, list) or len(samples) < 2:
                errors.append("clock_sync.json must contain at least two samples for drift checks")
            summary = clock_sync.get("summary")
            if isinstance(summary, dict):
                max_rtt_ns = _parse_int(summary.get("round_trip_ns_max"))
                drift_ns = abs(_parse_int(summary.get("offset_drift_ns")))
                if max_rtt_ns > 500_000_000:
                    errors.append(f"clock sync round trip is too high for reliable latency data: {max_rtt_ns / 1_000_000.0:.3f} ms")
                if max_rtt_ns > 0:
                    # NTP-style clock sync cannot localize a one-way timestamp more tightly than
                    # roughly half the measured RTT. Treat small negative latencies inside that
                    # uncertainty as warnings, but still reject physically impossible values beyond it.
                    clock_negative_tolerance_ms = max(clock_negative_tolerance_ms, (max_rtt_ns / 2_000_000.0) + 2.0)
                if drift_ns > 100_000_000:
                    warnings.append(
                        "headset wall-clock offset drift is high; monotonic headset timestamps are "
                        "used for derived latency when available and corrected latency will be validated "
                        f"against derived rows: {drift_ns / 1_000_000.0:.3f} ms"
                    )

        if not _has_metric_row(
            run_dir,
            "headset_metrics.csv",
            lambda row: (row.get("category") or "") == "frame" and (row.get("name") or "") == "render",
            headset_window,
        ):
            errors.append("headset_metrics.csv did not contain frame/render samples during the headset measurement window")

        expected_camera_streams = _expected_camera_streams(manifest)
        if expected_camera_streams > 0:
            if not _has_metric_row(
                run_dir,
                "headset_metrics.csv",
                lambda row: (row.get("category") or "") == "camera"
                and (row.get("name") or "").startswith("received"),
                headset_window,
            ):
                errors.append(
                    "headset_metrics.csv did not contain camera received samples during the headset measurement window"
                )
            if not _has_metric_row(
                run_dir,
                "headset_metrics.csv",
                lambda row: (row.get("category") or "") == "camera"
                and (row.get("name") or "").startswith("displayed"),
                headset_window,
            ):
                warnings.append(
                    "headset_metrics.csv did not contain camera displayed samples during the headset measurement window"
                )

        if _is_webrtc_workload(manifest):
            webrtc_path = run_dir / "webrtc_metrics.csv"
            if not webrtc_path.exists() or metric_row_count(webrtc_path) <= 0:
                errors.append("webrtc_metrics.csv is required for WebRTC camera runs but was not created")
            else:
                try:
                    webrtc_rows = _read_csv_rows(webrtc_path)
                except Exception as exc:
                    errors.append(f"webrtc_metrics.csv could not be read: {exc}")
                    webrtc_rows = []
                if webrtc_rows and not any(
                    (row.get("category") or "") == "webrtc" and (row.get("name") or "") == "frame_latency_ms"
                    for row in webrtc_rows
                ):
                    errors.append("webrtc_metrics.csv did not contain WebRTC frame latency samples")
                webrtc_window = _metric_window(run_dir, "webrtc_metrics.csv", window)
                if webrtc_window is not None and metric_row_count(webrtc_path, webrtc_window) == 0:
                    warnings.append(
                        "webrtc_metrics.csv contains shutdown-summary rows outside the measurement window; "
                        "use its total rows for connector-side WebRTC latency, not measurement-window row_count"
                    )

        derived_path = run_dir / "derived_metrics.csv"
        if derived_path.exists():
            try:
                derived_rows = _filter_measurement_rows(_read_csv_rows(derived_path), window)
            except Exception as exc:
                errors.append(f"derived_metrics.csv could not be read: {exc}")
                derived_rows = []
            if expected_camera_streams > 0 and not any(row.get("category") == "latency" for row in derived_rows):
                errors.append("derived_metrics.csv did not contain headset camera latency rows")
            negative_display_latencies = []
            negative_auxiliary_latencies = []
            for row in derived_rows:
                if row.get("category") != "latency":
                    continue
                try:
                    value = float(row.get("latency_ms") or "0")
                except ValueError:
                    continue
                if value >= -2.0:
                    continue
                name = row.get("name") or ""
                if "displayed" in name:
                    negative_display_latencies.append(value)
                else:
                    negative_auxiliary_latencies.append(value)
            excessive_display_latencies = [
                value for value in negative_display_latencies if abs(value) > clock_negative_tolerance_ms
            ]
            tolerated_display_latencies = [
                value for value in negative_display_latencies if abs(value) <= clock_negative_tolerance_ms
            ]
            if excessive_display_latencies:
                errors.append(
                    "derived displayed latency contains negative values after clock correction; "
                    f"minimum={min(excessive_display_latencies):.3f} ms exceeds "
                    f"clock uncertainty tolerance {clock_negative_tolerance_ms:.3f} ms"
                )
            if tolerated_display_latencies:
                warnings.append(
                    "derived displayed latency has small negative values within clock-sync uncertainty; "
                    f"minimum={min(tolerated_display_latencies):.3f} ms, "
                    f"tolerance={clock_negative_tolerance_ms:.3f} ms"
                )
            if negative_auxiliary_latencies:
                warnings.append(
                    "derived non-displayed latency contains negative values after clock correction; "
                    f"minimum={min(negative_auxiliary_latencies):.3f} ms; "
                    "do not report these auxiliary arrival values as end-to-end display latency"
                )
        elif manifest and _expected_camera_streams(manifest) > 0:
            errors.append("derived_metrics.csv was not created for a headset camera run")

    if _expects_control_metrics(manifest):
        if not _has_metric_row(
            run_dir,
            "headset_command_metrics.csv",
            lambda row: bool((row.get("topic") or "").strip()),
            headset_window,
        ):
            errors.append(
                "headset_command_metrics.csv did not contain command samples during the headset "
                "measurement window; run E6/E-control trials only after sending teleop and/or goal commands"
            )
        else:
            derived_path = run_dir / "derived_metrics.csv"
            try:
                derived_rows = _filter_measurement_rows(_read_csv_rows(derived_path), window) if derived_path.exists() else []
            except Exception as exc:
                errors.append(f"derived_metrics.csv command latency rows could not be read: {exc}")
                derived_rows = []
            if not any((row.get("category") or "") == "command_latency" for row in derived_rows):
                errors.append(
                    "derived_metrics.csv did not contain command_latency rows; headset commands were "
                    "not matched to bridge unity_to_ros published samples"
                )

    envelope_violations = _performance_envelope_violations(run_dir)
    measurement_valid = not errors
    within_envelope = measurement_valid and not envelope_violations
    result: Dict[str, object] = {
        "ok": measurement_valid,
        "measurement_valid": measurement_valid,
        "within_envelope": within_envelope,
        "degraded": measurement_valid and bool(envelope_violations),
        "envelope_violations": envelope_violations,
        "errors": errors,
        "warnings": warnings,
        "required_metrics": dict(required_metrics),
        "files": files,
    }
    (run_dir / "data_quality.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


# --- Cross-repetition aggregation -------------------------------------------------------------
# A single run's p95 conflates within-run sampling with run-to-run variance. A paper must report
# N>=5 repetitions per condition aggregated to mean +/- 95% CI (and median + IQR). These helpers
# take the per-run derived_summary.json scalars and aggregate them across repetitions.

# Two-tailed t critical values at 95% confidence, by degrees of freedom (df = n - 1). df > 30 -> z.
_T_CRITICAL_95 = {
    1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306,
    9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145, 15: 2.131, 16: 2.120,
    17: 2.110, 18: 2.101, 19: 2.093, 20: 2.086, 21: 2.080, 22: 2.074, 23: 2.069, 24: 2.064,
    25: 2.060, 26: 2.056, 27: 2.052, 28: 2.048, 29: 2.045, 30: 2.042,
}


def _t_critical_95(df: int) -> float:
    if df <= 0:
        return 0.0
    return _T_CRITICAL_95.get(df, 1.96)


def _flatten_numeric(prefix: str, obj: object, out: Dict[str, float]) -> None:
    if isinstance(obj, bool):
        return
    if isinstance(obj, dict):
        for key, value in obj.items():
            _flatten_numeric(f"{prefix}.{key}" if prefix else str(key), value, out)
    elif isinstance(obj, (int, float)):
        out[prefix] = float(obj)


def _run_scalar_metrics(run_dir: Path) -> Dict[str, float]:
    """Flatten a run's derived_summary.json into dotted-key scalar metrics for aggregation."""
    out: Dict[str, float] = {}
    path = Path(run_dir) / "derived_summary.json"
    if path.exists():
        try:
            _flatten_numeric("", json.loads(path.read_text(encoding="utf-8")), out)
        except (OSError, json.JSONDecodeError):
            pass
    return out


def _aggregate_values(values: List[float]) -> Dict[str, float]:
    n = len(values)
    m = mean(values)
    sample_sd = stdev(values) if n > 1 else 0.0
    result: Dict[str, float] = {
        "n": n,
        "mean": m,
        "stddev": sample_sd,
        "median": median(values),
        "p25": percentile(values, 25),
        "p75": percentile(values, 75),
        "min": min(values),
        "max": max(values),
    }
    if n > 1:
        sem = sample_sd / (n ** 0.5)
        half_width = _t_critical_95(n - 1) * sem
        result["ci95_low"] = m - half_width
        result["ci95_high"] = m + half_width
        result["ci95_half_width"] = half_width
    else:
        result["ci95_low"] = m
        result["ci95_high"] = m
        result["ci95_half_width"] = 0.0
    return result


def aggregate_runs(run_dirs: Iterable[Path]) -> Dict[str, object]:
    """Aggregate the per-run derived scalars across repetitions to mean+/-95%CI and median+IQR."""
    per_metric: Dict[str, List[float]] = {}
    contributing = 0
    for run_dir in run_dirs:
        metrics = _run_scalar_metrics(run_dir)
        if not metrics:
            continue
        contributing += 1
        for key, value in metrics.items():
            per_metric.setdefault(key, []).append(value)

    aggregated: Dict[str, object] = {}
    incomplete: List[str] = []
    for key, values in sorted(per_metric.items()):
        stats = _aggregate_values(values)
        stats["n_runs"] = contributing
        # Survivorship guard: a metric present in fewer runs than n_runs means some repetitions
        # produced no value for it (e.g. a fully-starved stream). Surface the coverage so a worst-
        # case run cannot silently vanish from this metric's mean/CI (final-review aggregation bias).
        if contributing and len(values) < contributing:
            stats["coverage"] = len(values) / contributing
            incomplete.append(key)
        aggregated[key] = stats
    result: Dict[str, object] = {"n_runs": contributing, "metrics": aggregated}
    if incomplete:
        result["metrics_missing_in_some_runs"] = sorted(incomplete)
    return result


def aggregate_results_root(results_root: Path, output_dir: Optional[Path] = None) -> List[Path]:
    """Group every run directory under results_root by (experiment, condition) and write one
    aggregate_<experiment>_<condition>.json per group with N-run mean+/-95%CI and median+IQR."""
    results_root = Path(results_root)
    output_dir = Path(output_dir) if output_dir is not None else results_root
    output_dir.mkdir(parents=True, exist_ok=True)

    groups: Dict[Tuple[str, str], List[Path]] = {}
    for run_dir in sorted(p for p in results_root.iterdir() if p.is_dir()):
        manifest = _load_manifest(run_dir)
        if not manifest:
            continue
        experiment = str(manifest.get("experiment") or "unknown")
        condition = str(manifest.get("condition") or "unknown")
        groups.setdefault((experiment, condition), []).append(run_dir)

    written: List[Path] = []
    for (experiment, condition), run_dirs in sorted(groups.items()):
        aggregate = aggregate_runs(run_dirs)
        aggregate["experiment"] = experiment
        aggregate["condition"] = condition
        aggregate["run_ids"] = [run_dir.name for run_dir in run_dirs]
        safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in f"{experiment}_{condition}")
        path = output_dir / f"aggregate_{safe}.json"
        path.write_text(json.dumps(aggregate, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        written.append(path)
    return written


# ---- Saturation-knee detection (map-load limits, M4) --------------------------------------------
#
# A capacity study claims "measurable limits"; that claim has to be *computed*, not asserted. The
# knee is the smallest swept map payload at which the pipeline can no longer keep up, defined by an
# explicit, pre-registered criterion so the result is reproducible and not eyeballed off a plot.

_KNEE_HZ_FLOOR_FRAC = 0.9    # delivered/publish rate must stay within 90% of the declared source Hz
_KNEE_DROP_CEILING = 0.05    # >5% displayed-frame drop is degraded
_KNEE_LATENCY_MULT = 2.0     # glass-to-glass p95 may not exceed 2x the smallest-payload baseline
_KNEE_FPS_FLOOR_FRAC = 0.9   # headset FPS must stay within 90% of the device target


def _to_float(value: object) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    if result != result or result in (float("inf"), float("-inf")):  # NaN / inf guard
        return None
    return result


def _mean_headset_fps(run_dir: Path) -> Optional[float]:
    """Mean measured headset render rate (category=frame,name=render rows) over the window."""
    path = Path(run_dir) / "headset_metrics.csv"
    if not path.exists():
        return None
    window = _metric_window(run_dir, "headset_metrics.csv", _measurement_window(run_dir))
    values: List[float] = []
    try:
        for row in _filter_measurement_rows(_read_csv_rows(path), window):
            if (row.get("category") or "") == "frame" and (row.get("name") or "") == "render":
                fps = _to_float(row.get("value"))
                if fps is not None and fps > 0:
                    values.append(fps)
    except Exception:
        return None
    return mean(values) if values else None


def _run_knee_point(run_dir: Path) -> Optional[Dict[str, object]]:
    """Extract one sweep point {payload, source_hz, achieved_hz, drop_rate, latency_p95, fps,...}
    from a run. Returns None for runs that are not a map-load sweep (no points/triangles)."""
    run_dir = Path(run_dir)
    manifest = _load_manifest(run_dir)
    if not manifest:
        return None
    workload: Dict[str, object] = {}
    extra = manifest.get("extra")
    if isinstance(extra, dict) and isinstance(extra.get("workload"), dict):
        workload = extra["workload"]  # type: ignore[assignment]
    mapcfg = workload.get("map") if isinstance(workload.get("map"), dict) else {}
    pccfg = workload.get("pointcloud") if isinstance(workload.get("pointcloud"), dict) else {}

    triangles = _parse_int(mapcfg.get("triangles"))
    points = _parse_int(pccfg.get("points"))
    if triangles > 0:
        payload, payload_kind = float(triangles), "triangles"
        source_hz = _to_float(mapcfg.get("hz"))
    elif points > 0:
        payload, payload_kind = float(points), "points"
        source_hz = _to_float(pccfg.get("hz")) or _to_float(mapcfg.get("hz"))
    else:
        return None  # not a map-load sweep run

    summary: Dict[str, object] = {}
    summary_path = run_dir / "derived_summary.json"
    if summary_path.exists():
        try:
            loaded = json.loads(summary_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                summary = loaded
        except (OSError, json.JSONDecodeError):
            summary = {}
    streams = summary.get("streams") if isinstance(summary.get("streams"), dict) else {}

    achieved_hz: Optional[float] = None
    best_bytes = -1.0
    worst_drop: Optional[float] = None
    worst_latency: Optional[float] = None
    for entry in streams.values():
        if not isinstance(entry, dict):
            continue
        # The map is by far the highest-throughput stream; use it to judge publish-rate sustain.
        throughput = _to_float(entry.get("throughput_bytes_per_s"))
        if throughput is not None and throughput > best_bytes:
            best_bytes = throughput
            achieved_hz = _to_float(entry.get("achieved_hz"))
        drop = _to_float(entry.get("drop_rate"))
        if drop is not None:
            worst_drop = drop if worst_drop is None else max(worst_drop, drop)
        latency = entry.get("latency_ms")
        if isinstance(latency, dict):
            p95 = _to_float(latency.get("p95"))
            if p95 is not None:
                worst_latency = p95 if worst_latency is None else max(worst_latency, p95)

    return {
        "label": str(manifest.get("condition") or run_dir.name),
        "run_id": str(manifest.get("run_id") or run_dir.name),
        "payload": payload,
        "payload_kind": payload_kind,
        "source_hz": source_hz,
        "achieved_hz": achieved_hz,
        "drop_rate": worst_drop,
        "latency_p95_ms": worst_latency,
        "headset_fps": _mean_headset_fps(run_dir),
        "target_fps": _to_float(manifest.get("target_fps")),
    }


def find_saturation_knee(
    conditions: List[Dict[str, object]],
    *,
    hz_floor_frac: float = _KNEE_HZ_FLOOR_FRAC,
    drop_ceiling: float = _KNEE_DROP_CEILING,
    latency_mult: float = _KNEE_LATENCY_MULT,
    fps_floor_frac: float = _KNEE_FPS_FLOOR_FRAC,
) -> Dict[str, object]:
    """Locate the saturation knee in a monotone payload sweep.

    Each condition is a dict with a numeric ``payload`` (the swept dimension) and any of the
    signals ``source_hz``/``achieved_hz``/``drop_rate``/``latency_p95_ms``/``headset_fps``/
    ``target_fps`` (missing signals are simply not evaluated). A condition is *degraded* if ANY
    available signal crosses its threshold (latency is judged against the smallest-payload
    baseline). The knee is the smallest payload that is degraded AND stays degraded for every
    larger payload, so a lone transient blip is not mistaken for the limit."""
    points = sorted(
        (c for c in conditions if _to_float(c.get("payload")) is not None),
        key=lambda c: _to_float(c.get("payload")),  # type: ignore[arg-type, return-value]
    )
    if not points:
        return {"knee": None, "interpretation": "no conditions with a payload", "conditions": []}

    baseline = points[0]
    baseline_latency = _to_float(baseline.get("latency_p95_ms"))

    evaluated: List[Dict[str, object]] = []
    for c in points:
        reasons: List[str] = []
        source_hz = _to_float(c.get("source_hz"))
        achieved_hz = _to_float(c.get("achieved_hz"))
        if source_hz and achieved_hz is not None and achieved_hz < hz_floor_frac * source_hz:
            reasons.append(
                f"achieved_hz {achieved_hz:.2f} < {hz_floor_frac:.0%} of source {source_hz:.2f}"
            )
        drop = _to_float(c.get("drop_rate"))
        if drop is not None and drop > drop_ceiling:
            reasons.append(f"drop_rate {drop:.1%} > {drop_ceiling:.0%}")
        latency = _to_float(c.get("latency_p95_ms"))
        if baseline_latency and latency is not None and latency > latency_mult * baseline_latency:
            reasons.append(
                f"latency p95 {latency:.1f}ms > {latency_mult:.0f}x baseline {baseline_latency:.1f}ms"
            )
        fps = _to_float(c.get("headset_fps"))
        target_fps = _to_float(c.get("target_fps"))
        if target_fps and fps is not None and fps < fps_floor_frac * target_fps:
            reasons.append(f"headset_fps {fps:.1f} < {fps_floor_frac:.0%} of target {target_fps:.1f}")
        evaluated.append({**c, "degraded": bool(reasons), "reasons": reasons})

    knee: Optional[Dict[str, object]] = None
    for i, c in enumerate(evaluated):
        if c["degraded"] and all(e["degraded"] for e in evaluated[i:]):
            knee = c
            break

    result: Dict[str, object] = {
        "thresholds": {
            "hz_floor_frac": hz_floor_frac,
            "drop_ceiling": drop_ceiling,
            "latency_mult": latency_mult,
            "fps_floor_frac": fps_floor_frac,
        },
        "baseline_label": baseline.get("label"),
        "conditions": [
            {
                "label": e.get("label"),
                "payload": e.get("payload"),
                "payload_kind": e.get("payload_kind"),
                "degraded": e["degraded"],
                "reasons": e["reasons"],
            }
            for e in evaluated
        ],
    }
    if knee is None:
        result["knee"] = None
        result["knee_payload"] = None
        result["interpretation"] = "no sustained saturation within the tested payload range"
    elif knee is evaluated[0]:
        result["knee"] = knee.get("label")
        result["knee_payload"] = knee.get("payload")
        result["interpretation"] = (
            "degraded at the smallest tested payload â€” the true knee is below the swept range; "
            "extend the sweep downward"
        )
    else:
        payload_val = _to_float(knee.get("payload")) or 0.0
        result["knee"] = knee.get("label")
        result["knee_payload"] = knee.get("payload")
        result["interpretation"] = (
            f"sustained saturation begins at {payload_val:.0f} {knee.get('payload_kind')}"
        )
    return result


def compute_experiment_knee(run_dirs: Iterable[Path]) -> Optional[Dict[str, object]]:
    """Build sweep points from a set of run dirs and locate the saturation knee, or None if none
    of the runs are map-load sweeps."""
    conditions = [pt for pt in (_run_knee_point(rd) for rd in run_dirs) if pt is not None]
    if not conditions:
        return None
    return find_saturation_knee(conditions)


def write_knee_reports(results_root: Path, output_dir: Optional[Path] = None) -> List[Path]:
    """Group every map-load-sweep run under results_root by experiment and write one
    knee_<experiment>.json with the computed saturation knee + per-condition degradation reasons."""
    results_root = Path(results_root)
    output_dir = Path(output_dir) if output_dir is not None else results_root

    groups: Dict[str, List[Dict[str, object]]] = {}
    for run_dir in sorted(p for p in results_root.iterdir() if p.is_dir()):
        point = _run_knee_point(run_dir)
        if point is None:
            continue
        manifest = _load_manifest(run_dir)
        experiment = str(manifest.get("experiment") or "unknown") if manifest else "unknown"
        groups.setdefault(experiment, []).append(point)

    written: List[Path] = []
    for experiment, conditions in sorted(groups.items()):
        knee = find_saturation_knee(conditions)
        knee["experiment"] = experiment
        output_dir.mkdir(parents=True, exist_ok=True)
        safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in experiment)
        path = output_dir / f"knee_{safe}.json"
        path.write_text(json.dumps(knee, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        written.append(path)
    return written
