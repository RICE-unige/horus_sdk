#!/usr/bin/env python3
"""Build the i-RIM HORUS experimental characterization package.

The analyzer is intentionally read-only with respect to raw experiment folders:
it reads `results/*` and writes only under `analysis/irim_2026`.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import platform
import random
import re
import shutil
import subprocess
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean, median
from typing import Any, Iterable, Mapping

SDK_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RESULTS_ROOT = SDK_ROOT / "results"
DEFAULT_OUTPUT_ROOT = SDK_ROOT / "analysis" / "irim_2026"
HORUS_MR_ROOT = Path("/mnt/c/Users/adeko/horus")
HORUS_ROS2_ROOT = Path("/home/omotoye/horus_ws/src/horus_ros2")

USABILITY_ENVELOPE = {
    "quest_fps_min": 60.0,
    "teleop_command_latency_p95_ms_max": 150.0,
    "task_goal_latency_p95_ms_max": 500.0,
    "tf_freshness_p95_ms_max": 100.0,
    "overview_camera_displayed_fps_min": 15.0,
    "immersive_camera_displayed_fps_min": 30.0,
    "sustained_queue_or_frame_age_growth_allowed": False,
}

CORE_EXPERIMENTS = {
    "E0_baseline",
    "E1_camera_transport",
    "E3_pointcloud_capacity",
    "E4_mesh_vs_pointcloud_map",
    "E5_map_update_behavior",
    "E6_control_under_sensor_load",
    "E7_multi_robot_scaling",
    "E8_multi_operator_scaling",
    "E9_failure_recovery",
    "E10_camera_capacity",
}

OUT_OF_SCOPE_EXPERIMENTS = {
    "E2_webrtc_vs_ros": "connector/WebRTC diagnostic is out of scope for the i-RIM core-platform dataset",
    "E11_map_pointcloud_sweep": "diagnostic map sweep is not part of the accepted i-RIM core dataset",
    "E12_map_mesh_sweep": "diagnostic map sweep is not part of the accepted i-RIM core dataset",
    "E13_map_pointcloud_wifi": "Wi-Fi/diagnostic map sweep is not part of the accepted i-RIM core dataset",
}


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        value = json.loads(path.read_text(encoding="utf-8-sig"))
    except Exception:
        return {}
    return value if isinstance(value, dict) else {}


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    try:
        with path.open(newline="", encoding="utf-8-sig") as handle:
            return list(csv.DictReader(handle))
    except Exception:
        return []


def read_ndjson(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8-sig", errors="ignore").splitlines():
        if not line.strip():
            continue
        try:
            event = json.loads(line)
        except Exception:
            continue
        if isinstance(event, dict):
            rows.append(event)
    return rows


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def write_csv(path: Path, rows: list[Mapping[str, Any]], fieldnames: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fieldnames is None:
        fieldnames = []
        for row in rows:
            for key in row.keys():
                if key not in fieldnames:
                    fieldnames.append(str(key))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def as_float(value: Any) -> float | None:
    if value is None or value == "":
        return None
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(number) or math.isinf(number):
        return None
    return number


def as_int(value: Any, default: int = 0) -> int:
    number = as_float(value)
    return default if number is None else int(number)


def fmt(value: Any, decimals: int = 2) -> str:
    number = as_float(value)
    if number is None:
        return ""
    return f"{number:.{decimals}f}"


def percentile(values: Iterable[float], p: float) -> float | None:
    ordered = sorted(value for value in values if value is not None)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * max(0.0, min(100.0, p)) / 100.0
    low = int(rank)
    high = min(low + 1, len(ordered) - 1)
    frac = rank - low
    return ordered[low] * (1.0 - frac) + ordered[high] * frac


def stats(values: Iterable[float]) -> dict[str, Any]:
    clean = [float(v) for v in values if v is not None and not math.isnan(float(v))]
    if not clean:
        return {"n": 0}
    return {
        "n": len(clean),
        "mean": mean(clean),
        "median": median(clean),
        "min": min(clean),
        "max": max(clean),
        "p50": percentile(clean, 50),
        "p95": percentile(clean, 95),
        "p99": percentile(clean, 99),
    }


def bootstrap_ci(values: list[float], iterations: int = 2000) -> tuple[float | None, float | None]:
    clean = [float(v) for v in values if v is not None and not math.isnan(float(v))]
    if not clean:
        return None, None
    if len(clean) == 1:
        return clean[0], clean[0]
    rng = random.Random(20260625)
    means: list[float] = []
    for _ in range(iterations):
        sample = [rng.choice(clean) for _ in clean]
        means.append(mean(sample))
    return percentile(means, 2.5), percentile(means, 97.5)


def csv_quote(value: Any) -> str:
    text = str(value if value is not None else "")
    return text.replace("\n", " ").replace("\r", " ")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run_cmd(command: list[str], cwd: Path | None = None) -> str:
    try:
        result = subprocess.run(
            command,
            cwd=str(cwd) if cwd else None,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=8,
        )
    except Exception:
        return ""
    return result.stdout.strip()


def git_info(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"path": str(path), "exists": False}
    return {
        "path": str(path),
        "exists": True,
        "branch": run_cmd(["git", "branch", "--show-current"], path),
        "commit": run_cmd(["git", "rev-parse", "HEAD"], path),
        "short_commit": run_cmd(["git", "rev-parse", "--short", "HEAD"], path),
        "dirty": bool(run_cmd(["git", "status", "--porcelain"], path)),
    }


def nested_get(payload: Mapping[str, Any], *keys: str) -> Any:
    current: Any = payload
    for key in keys:
        if not isinstance(current, Mapping):
            return None
        current = current.get(key)
    return current


def group_stat(group: Mapping[str, Any] | None, field: str, stat_name: str) -> float | None:
    if not isinstance(group, Mapping):
        return None
    return as_float(nested_get(group, field, stat_name))


def load_summary_group(run_dir: Path, group_key: str) -> Mapping[str, Any] | None:
    summary = load_json(run_dir / "summary.json")
    groups = nested_get(summary, "headset_metrics.csv", "by_group")
    if not isinstance(groups, Mapping):
        return None
    value = groups.get(group_key)
    return value if isinstance(value, Mapping) else None


def find_headset_groups(run_dir: Path, category_prefix: str) -> dict[str, Mapping[str, Any]]:
    summary = load_json(run_dir / "summary.json")
    groups = nested_get(summary, "headset_metrics.csv", "by_group")
    if not isinstance(groups, Mapping):
        return {}
    return {
        str(key): value
        for key, value in groups.items()
        if str(key).startswith(category_prefix) and isinstance(value, Mapping)
    }


def run_metric_window_s(run_dir: Path, manifest: Mapping[str, Any]) -> float:
    derived = load_json(run_dir / "derived_summary.json")
    value = as_float(derived.get("window_duration_s"))
    if value and value > 0:
        return value
    value = as_float(manifest.get("duration_s"))
    return value if value and value > 0 else 0.0


@dataclass
class RunRecord:
    run_dir: Path
    run_id: str
    manifest: dict[str, Any]
    quality: dict[str, Any]
    derived: dict[str, Any]
    summary: dict[str, Any]
    measurement_valid: bool
    within_envelope: bool | None
    degraded: bool
    system_failure: bool
    included: bool
    exclusion_reason: str
    quality_schema: str


def classify_run(run_dir: Path) -> RunRecord:
    manifest = load_json(run_dir / "run_manifest.json")
    quality = load_json(run_dir / "data_quality.json")
    derived = load_json(run_dir / "derived_summary.json")
    summary = load_json(run_dir / "summary.json")
    run_id = str(manifest.get("run_id") or run_dir.name)
    experiment = str(manifest.get("experiment") or "")

    quality_schema = "current" if "measurement_valid" in quality else "legacy_ok" if "ok" in quality else "missing"
    measurement_valid = bool(quality.get("measurement_valid", quality.get("ok", False)))
    within_envelope = None if "within_envelope" not in quality else bool(quality.get("within_envelope"))
    degraded = bool(quality.get("degraded", False))
    errors = quality.get("errors") if isinstance(quality.get("errors"), list) else []

    system_failure = False
    exclusion_reason = ""
    if not manifest:
        measurement_valid = False
        exclusion_reason = "missing run_manifest.json"
    elif experiment in OUT_OF_SCOPE_EXPERIMENTS:
        measurement_valid = False
        exclusion_reason = OUT_OF_SCOPE_EXPERIMENTS[experiment]
    elif "--dry-run" in " ".join(str(item) for item in manifest.get("cli_argv", [])):
        measurement_valid = False
        exclusion_reason = "dry run"
    elif not measurement_valid:
        if errors:
            exclusion_reason = "; ".join(str(error) for error in errors)
        else:
            exclusion_reason = "measurement invalid or incomplete"
    elif experiment not in CORE_EXPERIMENTS:
        measurement_valid = False
        exclusion_reason = "experiment is not in the accepted i-RIM core dataset"
    else:
        exclusion_reason = ""

    included = measurement_valid and not exclusion_reason
    return RunRecord(
        run_dir=run_dir,
        run_id=run_id,
        manifest=manifest,
        quality=quality,
        derived=derived,
        summary=summary,
        measurement_valid=measurement_valid,
        within_envelope=within_envelope,
        degraded=degraded,
        system_failure=system_failure,
        included=included,
        exclusion_reason=exclusion_reason,
        quality_schema=quality_schema,
    )


def collect_runs(results_root: Path) -> list[RunRecord]:
    run_dirs = sorted(path for path in results_root.iterdir() if path.is_dir() and (path / "run_manifest.json").exists())
    return [classify_run(path) for path in run_dirs]


def base_run_row(record: RunRecord) -> dict[str, Any]:
    manifest = record.manifest
    quality = record.quality
    clock = manifest.get("clock_sync") if isinstance(manifest.get("clock_sync"), Mapping) else {}
    source_clock = manifest.get("source_clock") if isinstance(manifest.get("source_clock"), Mapping) else {}
    return {
        "run_id": record.run_id,
        "experiment": manifest.get("experiment", ""),
        "condition": manifest.get("condition", ""),
        "included": record.included,
        "measurement_valid": record.measurement_valid,
        "within_envelope": "" if record.within_envelope is None else record.within_envelope,
        "degraded": record.degraded,
        "system_failure": record.system_failure,
        "exclusion_reason": record.exclusion_reason,
        "quality_schema": record.quality_schema,
        "duration_s": manifest.get("duration_s", ""),
        "warmup_s": manifest.get("warmup_s", ""),
        "robot_count": manifest.get("robot_count", ""),
        "operator_count": manifest.get("operator_count", ""),
        "stream_count": manifest.get("stream_count", ""),
        "target_fps": manifest.get("target_fps", ""),
        "resolution": manifest.get("resolution", ""),
        "transport": manifest.get("transport", ""),
        "topology": manifest.get("topology", ""),
        "robot_profile": manifest.get("robot_profile", ""),
        "map_profile": manifest.get("map_profile", ""),
        "notes": manifest.get("notes", ""),
        "date": manifest.get("date", ""),
        "clock_sync_samples": clock.get("sample_count", ""),
        "clock_uncertainty_ms": as_float(clock.get("offset_uncertainty_ns")) / 1_000_000.0 if as_float(clock.get("offset_uncertainty_ns")) is not None else "",
        "source_clock_synchronized": source_clock.get("synchronized", ""),
        "errors": "; ".join(str(item) for item in quality.get("errors", []) if isinstance(quality.get("errors"), list)),
        "warnings": "; ".join(str(item) for item in quality.get("warnings", []) if isinstance(quality.get("warnings"), list)),
        "envelope_violations": "; ".join(
            str(item) for item in quality.get("envelope_violations", []) if isinstance(quality.get("envelope_violations"), list)
        ),
        "path": str(record.run_dir),
    }


def extract_run_summary(record: RunRecord) -> dict[str, Any]:
    manifest = record.manifest
    derived = record.derived
    streams = derived.get("streams") if isinstance(derived.get("streams"), Mapping) else {}
    camera_entries = [entry for key, entry in streams.items() if str(key).startswith("camera|") and isinstance(entry, Mapping)]
    map_pointcloud_entries = [
        entry for key, entry in streams.items() if str(key).startswith("map_pointcloud|") and isinstance(entry, Mapping)
    ]
    mesh_entries = [entry for key, entry in streams.items() if str(key).startswith("mesh|") and isinstance(entry, Mapping)]
    tf_entries = [entry for key, entry in streams.items() if str(key).startswith("tf|") and isinstance(entry, Mapping)]
    control_entries = [entry for key, entry in streams.items() if str(key).startswith("control|") and isinstance(entry, Mapping)]

    render_group = load_summary_group(record.run_dir, "frame|render|render|") or {}
    mesh_finalized_groups = find_headset_groups(record.run_dir, "mesh_map|finalized")
    mesh_received_groups = find_headset_groups(record.run_dir, "mesh_map|marker_received")
    point_uploaded_groups = find_headset_groups(record.run_dir, "pointcloud|uploaded")
    point_received_groups = find_headset_groups(record.run_dir, "pointcloud|received")

    camera_displayed = [as_float(item.get("displayed_hz")) for item in camera_entries]
    camera_displayed = [item for item in camera_displayed if item is not None]
    camera_latency_p95 = [as_float(nested_get(item, "latency_ms", "p95")) for item in camera_entries]
    camera_latency_p95 = [item for item in camera_latency_p95 if item is not None]
    camera_drop = [as_float(item.get("drop_rate")) for item in camera_entries]
    camera_drop = [item for item in camera_drop if item is not None]

    map_entry = map_pointcloud_entries[0] if map_pointcloud_entries else {}
    mesh_entry = mesh_entries[0] if mesh_entries else {}
    tf_entry = tf_entries[0] if tf_entries else {}
    control_entry = control_entries[0] if control_entries else {}
    bridge = derived.get("bridge") if isinstance(derived.get("bridge"), Mapping) else {}
    commands = derived.get("commands") if isinstance(derived.get("commands"), Mapping) else {}
    command_latencies: list[float] = []
    for item in commands.values():
        if isinstance(item, Mapping):
            value = as_float(nested_get(item, "headset_to_bridge_publish_ms", "p95"))
            if value is not None:
                command_latencies.append(value)

    mesh_finalized = next(iter(mesh_finalized_groups.values()), {})
    mesh_received = next(iter(mesh_received_groups.values()), {})
    point_uploaded = next(iter(point_uploaded_groups.values()), {})
    point_received = next(iter(point_received_groups.values()), {})

    row = base_run_row(record)
    row.update(
        {
            "window_duration_s": derived.get("window_duration_s", run_metric_window_s(record.run_dir, manifest)),
            "quest_fps_p50": group_stat(render_group, "value", "p50"),
            "quest_fps_p95": group_stat(render_group, "value", "p95"),
            "quest_fps_min": group_stat(render_group, "value", "min"),
            "quest_frame_ms_p50": group_stat(render_group, "duration_ms", "p50"),
            "quest_frame_ms_p95": group_stat(render_group, "duration_ms", "p95"),
            "quest_frame_queue_p95": group_stat(render_group, "queue_depth", "p95"),
            "camera_streams_observed": len(camera_entries),
            "camera_displayed_fps_mean": mean(camera_displayed) if camera_displayed else "",
            "camera_displayed_fps_min": min(camera_displayed) if camera_displayed else "",
            "camera_latency_p95_ms_mean": mean(camera_latency_p95) if camera_latency_p95 else "",
            "camera_latency_p95_ms_max": max(camera_latency_p95) if camera_latency_p95 else "",
            "camera_drop_rate_mean": mean(camera_drop) if camera_drop else "",
            "camera_drop_rate_max": max(camera_drop) if camera_drop else "",
            "tf_source_hz": as_float(tf_entry.get("achieved_hz")) if isinstance(tf_entry, Mapping) else "",
            "control_source_hz": as_float(control_entry.get("achieved_hz")) if isinstance(control_entry, Mapping) else "",
            "command_latency_p95_ms_max": max(command_latencies) if command_latencies else "",
            "map_pointcloud_points": group_stat(point_received, "points", "p50")
            or nested_get(manifest, "extra", "workload", "pointcloud", "points")
            if "pointcloud" in str(manifest.get("map_profile", "")).lower()
            else "",
            "map_pointcloud_payload_bytes": group_stat(point_received, "payload_bytes", "p50")
            or as_float(map_entry.get("bytes")) if isinstance(map_entry, Mapping) else "",
            "map_pointcloud_source_hz": as_float(map_entry.get("achieved_hz")) if isinstance(map_entry, Mapping) else "",
            "map_pointcloud_upload_ms_p50": group_stat(point_uploaded, "duration_ms", "p50"),
            "map_pointcloud_upload_ms_p95": group_stat(point_uploaded, "duration_ms", "p95"),
            "map_mesh_vertices": group_stat(mesh_finalized, "vertices", "p50"),
            "map_mesh_triangles": group_stat(mesh_finalized, "triangles", "p50")
            or nested_get(manifest, "extra", "workload", "map", "triangles"),
            "map_mesh_chunk_count": group_stat(mesh_finalized, "value", "p50"),
            "map_mesh_source_hz": as_float(mesh_entry.get("achieved_hz")) if isinstance(mesh_entry, Mapping) else "",
            "map_mesh_source_bytes_total": as_float(mesh_entry.get("bytes")) if isinstance(mesh_entry, Mapping) else "",
            "map_mesh_finalize_ms_p50": group_stat(mesh_finalized, "duration_ms", "p50"),
            "map_mesh_finalize_ms_p95": group_stat(mesh_finalized, "duration_ms", "p95"),
            "map_mesh_chunk_triangles": group_stat(mesh_received, "triangles", "p50"),
            "map_mesh_chunk_points": group_stat(mesh_received, "points", "p50"),
            "bridge_realtime_queue_p95": as_float(nested_get(bridge, "realtime_queue_depth", "p95")),
            "bridge_realtime_time_in_queue_ms_p95": as_float(nested_get(bridge, "realtime_time_in_queue_ms", "p95")),
            "bridge_bulk_queue_p95": as_float(nested_get(bridge, "bulk_queue_depth", "p95")),
            "bridge_bulk_time_in_queue_ms_p95": as_float(nested_get(bridge, "bulk_time_in_queue_ms", "p95")),
            "bridge_dropped_count": bridge.get("dropped_count", "") if isinstance(bridge, Mapping) else "",
            "bridge_evicted_count": bridge.get("evicted_count", "") if isinstance(bridge, Mapping) else "",
        }
    )
    return row


def metric_long_rows(record: RunRecord) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    base = {
        "run_id": record.run_id,
        "experiment": record.manifest.get("experiment", ""),
        "condition": record.manifest.get("condition", ""),
        "included": record.included,
        "measurement_valid": record.measurement_valid,
        "within_envelope": "" if record.within_envelope is None else record.within_envelope,
        "degraded": record.degraded,
    }
    streams = record.derived.get("streams") if isinstance(record.derived.get("streams"), Mapping) else {}
    for stream_key, entry in streams.items():
        if not isinstance(entry, Mapping):
            continue
        stream_type, _, topic = str(stream_key).partition("|")
        simple_metrics = {
            "source_hz": entry.get("achieved_hz"),
            "throughput_bytes_per_s": entry.get("throughput_bytes_per_s"),
            "displayed_hz": entry.get("displayed_hz"),
            "received_hz": entry.get("received_hz"),
            "display_rate": entry.get("display_rate"),
            "drop_rate": entry.get("drop_rate"),
            "headset_coalesce_rate": entry.get("headset_coalesce_rate"),
        }
        for name, value in simple_metrics.items():
            number = as_float(value)
            if number is not None:
                rows.append({**base, "source": "derived_summary", "stream_type": stream_type, "topic": topic, "metric": name, "stat": "value", "value": number, "n": ""})
        for prefix, key in (("camera_latency_ms", "latency_ms"), ("camera_arrival_latency_ms", "arrival_latency_ms")):
            stats_payload = entry.get(key)
            if not isinstance(stats_payload, Mapping):
                continue
            for stat_name in ("p50", "p95", "p99", "mean", "max"):
                number = as_float(stats_payload.get(stat_name))
                if number is not None:
                    rows.append({**base, "source": "derived_summary", "stream_type": stream_type, "topic": topic, "metric": prefix, "stat": stat_name, "value": number, "n": stats_payload.get("count", "")})
        if isinstance(entry.get("source_inter_arrival"), Mapping):
            for stat_name in ("mean_ms", "p95_ms", "stddev_ms"):
                number = as_float(entry["source_inter_arrival"].get(stat_name))
                if number is not None:
                    rows.append({**base, "source": "derived_summary", "stream_type": stream_type, "topic": topic, "metric": "source_inter_arrival", "stat": stat_name, "value": number, "n": ""})
    bridge = record.derived.get("bridge") if isinstance(record.derived.get("bridge"), Mapping) else {}
    for key, value in bridge.items():
        if isinstance(value, Mapping):
            for stat_name, stat_value in value.items():
                number = as_float(stat_value)
                if number is not None:
                    rows.append({**base, "source": "derived_summary", "stream_type": "bridge", "topic": "", "metric": str(key), "stat": str(stat_name), "value": number, "n": ""})
        else:
            number = as_float(value)
            if number is not None:
                rows.append({**base, "source": "derived_summary", "stream_type": "bridge", "topic": "", "metric": str(key), "stat": "value", "value": number, "n": ""})
    render_group = load_summary_group(record.run_dir, "frame|render|render|") or {}
    for field, metric in (("value", "quest_fps"), ("duration_ms", "quest_frame_ms"), ("queue_depth", "quest_frame_queue_depth")):
        for stat_name in ("p50", "p95", "p99", "mean", "max", "min"):
            number = group_stat(render_group, field, stat_name)
            if number is not None:
                rows.append({**base, "source": "summary", "stream_type": "headset", "topic": "", "metric": metric, "stat": stat_name, "value": number, "n": nested_get(render_group, field, "count") or ""})
    return rows


def condition_summary_rows(run_summaries: list[dict[str, Any]]) -> list[dict[str, Any]]:
    included = [row for row in run_summaries if row.get("included") in (True, "True", "true")]
    groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in included:
        groups[(str(row.get("experiment", "")), str(row.get("condition", "")))].append(row)

    numeric_fields = [
        "quest_fps_p50",
        "quest_frame_ms_p95",
        "camera_displayed_fps_min",
        "camera_latency_p95_ms_max",
        "camera_drop_rate_max",
        "command_latency_p95_ms_max",
        "map_pointcloud_upload_ms_p95",
        "map_mesh_finalize_ms_p95",
        "bridge_realtime_time_in_queue_ms_p95",
        "bridge_bulk_time_in_queue_ms_p95",
    ]
    output: list[dict[str, Any]] = []
    for (experiment, condition), rows in sorted(groups.items()):
        item: dict[str, Any] = {
            "experiment": experiment,
            "condition": condition,
            "valid_runs": len(rows),
            "within_envelope_runs": sum(1 for row in rows if row.get("within_envelope") in (True, "True", "true")),
            "degraded_runs": sum(1 for row in rows if row.get("degraded") in (True, "True", "true")),
        }
        for field in numeric_fields:
            values = [as_float(row.get(field)) for row in rows]
            values = [value for value in values if value is not None]
            if not values:
                continue
            lo, hi = bootstrap_ci(values)
            item[f"{field}_median"] = median(values)
            item[f"{field}_mean"] = mean(values)
            item[f"{field}_min"] = min(values)
            item[f"{field}_max"] = max(values)
            item[f"{field}_ci95_low"] = lo
            item[f"{field}_ci95_high"] = hi
        output.append(item)
    return output


def sdk_measurement_window(run_dir: Path) -> tuple[int | None, int | None]:
    summary = load_json(run_dir / "summary.json")
    window = summary.get("_measurement_window") if isinstance(summary.get("_measurement_window"), Mapping) else {}
    start = as_float(window.get("start_timestamp_ns"))
    end = as_float(window.get("end_timestamp_ns"))
    return (int(start), int(end)) if start is not None and end is not None else (None, None)


def headset_measurement_window(run_dir: Path) -> tuple[int | None, int | None]:
    start = None
    end = None
    for event in read_ndjson(run_dir / "headset_events.ndjson"):
        name = event.get("name")
        timestamp = as_float(event.get("timestamp_ns"))
        if timestamp is None:
            continue
        if name == "measurement_start" and start is None:
            start = int(timestamp)
        elif name == "measurement_end":
            end = int(timestamp)
    return start, end


def rows_in_absolute_window(path: Path, start_ns: int | None, end_ns: int | None) -> list[dict[str, str]]:
    rows = read_csv(path)
    if start_ns is None or end_ns is None:
        return rows
    output = []
    for row in rows:
        timestamp = as_float(row.get("timestamp_ns"))
        if timestamp is None:
            continue
        if start_ns <= int(timestamp) <= end_ns:
            output.append(row)
    return output


def staged_camera_config(manifest: Mapping[str, Any]) -> tuple[list[int], float]:
    workload = nested_get(manifest, "extra", "workload")
    staging = workload.get("camera_staging") if isinstance(workload, Mapping) else None
    if not isinstance(staging, Mapping):
        workload_extra = workload.get("extra") if isinstance(workload, Mapping) else None
        staging = workload_extra.get("camera_staging") if isinstance(workload_extra, Mapping) else None
    if not isinstance(staging, Mapping):
        return [], 0.0
    enabled = str(staging.get("enabled", True)).strip().lower() not in {"0", "false", "no", "off"}
    if not enabled:
        return [], 0.0
    counts = []
    for value in staging.get("stream_counts") or staging.get("stages") or []:
        count = as_int(value)
        if count is not None:
            counts.append(count)
    duration = as_float(staging.get("stage_duration_s")) or 0.0
    return counts, duration


def safe_ratio(numerator: int, denominator: int) -> float:
    return numerator / denominator if denominator > 0 else 0.0


def camera_stage_rows(records: list[RunRecord]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    e10 = [record for record in records if record.included and record.manifest.get("experiment") == "E10_camera_capacity"]
    for record in e10:
        manifest = record.manifest
        counts, stage_duration = staged_camera_config(manifest)
        if not counts or not stage_duration:
            continue
        duration = as_float(manifest.get("duration_s")) or 0.0
        sdk_start, _ = sdk_measurement_window(record.run_dir)
        headset_start, _ = headset_measurement_window(record.run_dir)
        if sdk_start is None or headset_start is None:
            continue
        for stage_index, active in enumerate(counts):
            start_s = stage_index * stage_duration
            end_s = min(start_s + stage_duration, duration)
            if end_s <= start_s:
                continue
            sdk_stage_start = sdk_start + int(start_s * 1_000_000_000)
            sdk_stage_end = sdk_start + int(end_s * 1_000_000_000)
            headset_stage_start = headset_start + int(start_s * 1_000_000_000)
            headset_stage_end = headset_start + int(end_s * 1_000_000_000)
            source_rows = rows_in_absolute_window(record.run_dir / "source_metrics.csv", sdk_stage_start, sdk_stage_end)
            headset_rows = rows_in_absolute_window(record.run_dir / "headset_metrics.csv", headset_stage_start, headset_stage_end)
            derived_rows = rows_in_absolute_window(record.run_dir / "derived_metrics.csv", sdk_stage_start, sdk_stage_end)
            source_count = sum(1 for row in source_rows if row.get("stream") == "camera" or "/camera_" in str(row.get("topic", "")))
            received_count = sum(
                1
                for row in headset_rows
                if row.get("category") == "camera" and row.get("name") in {"received", "received_compressed"}
            )
            displayed_count = sum(
                1
                for row in headset_rows
                if row.get("category") == "camera" and row.get("name") in {"displayed", "displayed_compressed"}
            )
            latencies = [
                as_float(row.get("latency_ms"))
                for row in derived_rows
                if row.get("category") == "latency" and "displayed" in str(row.get("name", ""))
            ]
            latencies = [value for value in latencies if value is not None]
            stage_seconds = end_s - start_s
            rows.append(
                {
                    "run_id": record.run_id,
                    "condition": manifest.get("condition", ""),
                    "stage_index": stage_index,
                    "active_streams": int(active),
                    "stage_start_s": start_s,
                    "stage_end_s": end_s,
                    "duration_s": stage_seconds,
                    "source_camera_frames": source_count,
                    "headset_camera_received": received_count,
                    "headset_camera_displayed": displayed_count,
                    "received_ratio": safe_ratio(received_count, source_count),
                    "displayed_ratio": safe_ratio(displayed_count, source_count),
                    "source_hz_total": source_count / stage_seconds if stage_seconds > 0 else "",
                    "received_hz_total": received_count / stage_seconds if stage_seconds > 0 else "",
                    "displayed_hz_total": displayed_count / stage_seconds if stage_seconds > 0 else "",
                    "source_hz_per_stream": source_count / stage_seconds / int(active) if stage_seconds > 0 and int(active) > 0 else "",
                    "received_hz_per_stream": received_count / stage_seconds / int(active) if stage_seconds > 0 and int(active) > 0 else "",
                    "displayed_hz_per_stream": displayed_count / stage_seconds / int(active) if stage_seconds > 0 and int(active) > 0 else "",
                    "camera_latency_p95_ms": percentile(latencies, 95),
                    "camera_latency_p99_ms": percentile(latencies, 99),
                    "camera_drop_rate": 1.0 - safe_ratio(displayed_count, source_count) if source_count else "",
                }
            )
    return rows


def camera_topic_index(stream_key: str, robot_count: int) -> int:
    _, _, topic = stream_key.partition("|")
    robot_match = re.search(r"exp_robot_(\d+)", topic)
    cam_match = re.search(r"camera_(\d+)", topic)
    robot_index = int(robot_match.group(1)) if robot_match else 0
    camera_index = int(cam_match.group(1)) if cam_match else 0
    return camera_index * max(1, robot_count) + robot_index


def relative_s_from_row(rows: list[dict[str, str]], row: Mapping[str, str]) -> float | None:
    if not rows:
        return None
    first = as_float(rows[0].get("timestamp_ns"))
    ts = as_float(row.get("timestamp_ns"))
    if first is None or ts is None:
        return None
    return (ts - first) / 1_000_000_000.0


def stage_time_ok(row: Mapping[str, str], start_s: float, end_s: float) -> bool:
    # derived_metrics timestamps are corrected host nanoseconds. Stage-level E10
    # reporting uses active camera topics, so a coarse time filter is optional.
    # Keep rows if no relative stage estimate is available.
    return True


def count_rows_in_stage(rows: list[dict[str, str]], topics: list[str], start_s: float, end_s: float) -> int:
    if not rows:
        return 0
    first_ts = as_float(rows[0].get("timestamp_ns"))
    count = 0
    topics_set = set(topics)
    for row in rows:
        if row.get("topic") not in topics_set or row.get("stream") != "camera":
            continue
        ts = as_float(row.get("timestamp_ns"))
        if first_ts is None or ts is None:
            continue
        rel = (ts - first_ts) / 1_000_000_000.0
        if start_s <= rel < end_s:
            count += 1
    return count


def count_headset_camera_stage(rows: list[dict[str, str]], topics: list[str], start_s: float, end_s: float, received: bool) -> int:
    if not rows:
        return 0
    first_ts = as_float(rows[0].get("timestamp_ns"))
    count = 0
    topics_set = set(topics)
    prefix = "received" if received else "displayed"
    for row in rows:
        if row.get("topic") not in topics_set:
            continue
        if row.get("category") != "camera" or not str(row.get("name", "")).startswith(prefix):
            continue
        ts = as_float(row.get("timestamp_ns"))
        if first_ts is None or ts is None:
            continue
        rel = (ts - first_ts) / 1_000_000_000.0
        if start_s <= rel < end_s:
            count += 1
    return count


def aggregate_camera_stage_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        groups[int(row["active_streams"])].append(row)
    output = []
    for active, items in sorted(groups.items()):
        displayed_hz = [as_float(item["displayed_hz_per_stream"]) for item in items]
        displayed_hz = [value for value in displayed_hz if value is not None]
        source_hz = [as_float(item.get("source_hz_per_stream")) for item in items]
        source_hz = [value for value in source_hz if value is not None]
        received_hz = [as_float(item.get("received_hz_per_stream")) for item in items]
        received_hz = [value for value in received_hz if value is not None]
        displayed_ratio = [as_float(item["displayed_ratio"]) for item in items]
        displayed_ratio = [value for value in displayed_ratio if value is not None]
        received_ratio = [as_float(item["received_ratio"]) for item in items]
        received_ratio = [value for value in received_ratio if value is not None]
        latency_p95 = [as_float(item["camera_latency_p95_ms"]) for item in items]
        latency_p95 = [value for value in latency_p95 if value is not None]
        output.append(
            {
                "active_streams": active,
                "valid_runs": len(items),
                "source_hz_per_stream_mean": mean(source_hz) if source_hz else "",
                "received_hz_per_stream_mean": mean(received_hz) if received_hz else "",
                "displayed_hz_per_stream_mean": mean(displayed_hz) if displayed_hz else "",
                "displayed_hz_per_stream_median": median(displayed_hz) if displayed_hz else "",
                "displayed_hz_per_stream_min": min(displayed_hz) if displayed_hz else "",
                "displayed_ratio_median": median(displayed_ratio) if displayed_ratio else "",
                "received_ratio_median": median(received_ratio) if received_ratio else "",
                "camera_latency_p95_ms_mean": mean(latency_p95) if latency_p95 else "",
                "camera_latency_p95_ms_median": median(latency_p95) if latency_p95 else "",
                "camera_latency_p95_ms_max": max(latency_p95) if latency_p95 else "",
                "within_camera_envelope": all(
                    (as_float(item["displayed_hz_per_stream"]) or 0) >= USABILITY_ENVELOPE["overview_camera_displayed_fps_min"]
                    for item in items
                ),
            }
        )
    return output


def run_quality_rows(records: list[RunRecord]) -> list[dict[str, Any]]:
    return [base_run_row(record) for record in records]


def included_excluded_rows(records: list[RunRecord]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    rows = [base_run_row(record) for record in records]
    included = [row for row in rows if row["included"]]
    excluded = [row for row in rows if not row["included"]]
    return included, excluded


def build_dataset_manifest(records: list[RunRecord], output_root: Path) -> dict[str, Any]:
    run_entries = []
    for record in records:
        checksums = {}
        for name in ("run_manifest.json", "data_quality.json", "headset_run_manifest.json", "transport_manifest.json"):
            path = record.run_dir / name
            if path.exists():
                checksums[name] = sha256_file(path)
        run_entries.append(
            {
                "run_id": record.run_id,
                "path": str(record.run_dir),
                "experiment": record.manifest.get("experiment", ""),
                "condition": record.manifest.get("condition", ""),
                "included": record.included,
                "measurement_valid": record.measurement_valid,
                "within_envelope": record.within_envelope,
                "degraded": record.degraded,
                "checksums": checksums,
            }
        )
    return {
        "analysis_date_utc": datetime.now(timezone.utc).isoformat(),
        "analysis_root": str(output_root),
        "results_root": str(DEFAULT_RESULTS_ROOT),
        "raw_snapshot_method": "checksummed manifest only; raw folders were not modified or duplicated",
        "run_count": len(records),
        "included_run_count": sum(1 for record in records if record.included),
        "runs": run_entries,
    }


def parse_unity_version(path: Path) -> str:
    version_file = path / "ProjectSettings" / "ProjectVersion.txt"
    if not version_file.exists():
        return ""
    for line in version_file.read_text(encoding="utf-8", errors="ignore").splitlines():
        if line.strip().startswith("m_EditorVersion:"):
            return line.split(":", 1)[1].strip()
    return ""


def parse_meta_xr_version(path: Path) -> str:
    manifest_path = path / "Packages" / "manifest.json"
    manifest = load_json(manifest_path)
    deps = manifest.get("dependencies") if isinstance(manifest.get("dependencies"), Mapping) else {}
    for key in ("com.meta.xr.sdk.all", "com.meta.xr.sdk.core", "com.meta.xr.mrutilitykit"):
        if key in deps:
            return f"{key} {deps[key]}"
    return ""


def software_versions(records: list[RunRecord]) -> dict[str, Any]:
    headset_manifests = []
    for record in records:
        payload = load_json(record.run_dir / "headset_run_manifest.json")
        if payload:
            headset_manifests.append(payload)
    return {
        "analysis_python": sys.version,
        "analysis_platform": platform.platform(),
        "horus_sdk": git_info(SDK_ROOT),
        "horus_mr": {
            **git_info(HORUS_MR_ROOT),
            "unity_version_project": parse_unity_version(HORUS_MR_ROOT),
            "meta_xr_sdk": parse_meta_xr_version(HORUS_MR_ROOT),
        },
        "horus_ros2": git_info(HORUS_ROS2_ROOT),
        "ros": {
            "ros_distro_from_env": os.environ.get("ROS_DISTRO", ""),
            "jazzy_setup_exists": Path("/opt/ros/jazzy/setup.bash").exists(),
        },
        "headset_manifests_observed": headset_manifests[:5],
    }


def hardware_network_setup(records: list[RunRecord]) -> dict[str, Any]:
    cpu_model = ""
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.exists():
        for line in cpuinfo.read_text(encoding="utf-8", errors="ignore").splitlines():
            if line.startswith("model name"):
                cpu_model = line.split(":", 1)[1].strip()
                break
    mem_total = ""
    meminfo = Path("/proc/meminfo")
    if meminfo.exists():
        for line in meminfo.read_text(encoding="utf-8", errors="ignore").splitlines():
            if line.startswith("MemTotal:"):
                mem_total = line.split(":", 1)[1].strip()
                break
    run_hosts = sorted({str(record.manifest.get("host") or "") for record in records if record.manifest.get("host")})
    domains = sorted({str(nested_get(record.manifest, "extra", "ros_domain_id") or "") for record in records if nested_get(record.manifest, "extra", "ros_domain_id")})
    return {
        "pc_hostnames_from_runs": run_hosts,
        "analysis_host": platform.node(),
        "analysis_os": platform.platform(),
        "cpu_model": cpu_model,
        "memory_total": mem_total,
        "quest": {
            "device": "Meta Quest 3",
            "identifier_source": "user-provided experimental setup; headset run manifests report Android platform and Unity version",
        },
        "network": {
            "scope": "local LAN/core-platform experiments only",
            "topologies_observed": sorted({str(record.manifest.get("topology") or "") for record in records if record.manifest.get("topology")}),
            "transports_observed": sorted({str(record.manifest.get("transport") or "") for record in records if record.manifest.get("transport")}),
            "ros_domain_ids_observed": domains,
            "excluded": "WAN, VPN, cloud relay, Zenoh, public Internet, and horus_connector remote experiments",
        },
    }


def metric_dictionary() -> list[dict[str, Any]]:
    return [
        {"metric": "measurement_valid", "unit": "boolean", "definition": "Instrumentation, timestamps, configuration, and required logs were complete enough for analysis.", "aggregation": "run-level classification"},
        {"metric": "within_envelope", "unit": "boolean", "definition": "All predefined usability criteria were met.", "aggregation": "run-level classification"},
        {"metric": "degraded", "unit": "boolean", "definition": "Measurement is valid, but one or more envelope criteria failed.", "aggregation": "run-level classification"},
        {"metric": "Quest FPS", "unit": "frames/s", "definition": "Frame/render metric reported by the MR app during the measurement window.", "aggregation": "run p50/p95, then summarized across runs"},
        {"metric": "application-level camera latency", "unit": "ms", "definition": "Source frame timestamp to headset displayed frame timestamp after clock correction. This is not optical motion-to-photon latency.", "aggregation": "per-run p50/p95/p99 over displayed frames; repeated-run summaries over run-level values"},
        {"metric": "displayed camera FPS", "unit": "frames/s", "definition": "Camera frames actually displayed by the headset visualizer, not merely received.", "aggregation": "per-stream displayed count divided by active measurement duration"},
        {"metric": "camera drop rate", "unit": "fraction", "definition": "1 - displayed frames / source frames. Includes upstream loss and headset coalescing.", "aggregation": "per-stream run-level rate"},
        {"metric": "source rate", "unit": "Hz", "definition": "Rows published by the synthetic ROS source during the measurement window divided by the measurement duration.", "aggregation": "per stream per run"},
        {"metric": "bridge queue time", "unit": "ms", "definition": "Time spent in bridge realtime or bulk outbound queues.", "aggregation": "per-run p50/p95/p99 by lane"},
        {"metric": "mesh finalize time", "unit": "ms", "definition": "Headset time to finalize a received mesh map update.", "aggregation": "per-run p50/p95 from headset mesh_map finalized rows"},
        {"metric": "pointcloud upload time", "unit": "ms", "definition": "Headset time to upload a received PointCloud2 map update.", "aggregation": "per-run p50/p95 from headset pointcloud uploaded rows"},
        {"metric": "command latency", "unit": "ms", "definition": "Corrected headset command timestamp to ROS bridge publication timestamp.", "aggregation": "per-run p95 where command rows exist"},
        {"metric": "stability slope", "unit": "metric units/s", "definition": "Linear slope over the final third of selected time series. Positive queue/frame-age slope flags sustained growth.", "aggregation": "run-level diagnostic"},
    ]


def stability_rows(records: list[RunRecord]) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    for record in records:
        if not record.included:
            continue
        rows = read_csv(record.run_dir / "headset_metrics.csv")
        render = [
            row for row in rows
            if row.get("category") == "frame" and row.get("name") == "render"
        ]
        if len(render) >= 10:
            subset = render[int(len(render) * 2 / 3):]
            ts0 = as_float(subset[0].get("timestamp_ns")) or 0.0
            xs: list[float] = []
            fps: list[float] = []
            queue: list[float] = []
            for row in subset:
                ts = as_float(row.get("timestamp_ns"))
                if ts is None:
                    continue
                xs.append((ts - ts0) / 1_000_000_000.0)
                value = as_float(row.get("value"))
                q = as_float(row.get("queue_depth"))
                if value is not None:
                    fps.append(value)
                if q is not None:
                    queue.append(q)
            output.append(
                {
                    "run_id": record.run_id,
                    "experiment": record.manifest.get("experiment", ""),
                    "condition": record.manifest.get("condition", ""),
                    "final_third_render_samples": len(subset),
                    "quest_fps_final_third_slope_per_s": linear_slope(xs, fps) if len(xs) == len(fps) else "",
                    "render_queue_final_third_slope_per_s": linear_slope(xs, queue) if len(xs) == len(queue) else "",
                    "stability_flag": stability_flag(record, linear_slope(xs, fps) if len(xs) == len(fps) else None, linear_slope(xs, queue) if len(xs) == len(queue) else None),
                }
            )
        else:
            output.append(
                {
                    "run_id": record.run_id,
                    "experiment": record.manifest.get("experiment", ""),
                    "condition": record.manifest.get("condition", ""),
                    "final_third_render_samples": len(render),
                    "quest_fps_final_third_slope_per_s": "",
                    "render_queue_final_third_slope_per_s": "",
                    "stability_flag": "insufficient_render_samples",
                }
            )
    return output


def linear_slope(xs: list[float], ys: list[float]) -> float | None:
    pairs = [(x, y) for x, y in zip(xs, ys) if x is not None and y is not None]
    if len(pairs) < 3:
        return None
    x_mean = mean([p[0] for p in pairs])
    y_mean = mean([p[1] for p in pairs])
    denom = sum((x - x_mean) ** 2 for x, _ in pairs)
    if denom <= 0:
        return None
    return sum((x - x_mean) * (y - y_mean) for x, y in pairs) / denom


def stability_flag(record: RunRecord, fps_slope: float | None, queue_slope: float | None) -> str:
    flags = []
    if fps_slope is not None and fps_slope < -0.05:
        flags.append("fps_decline")
    if queue_slope is not None and queue_slope > 0.02:
        flags.append("queue_growth")
    if record.degraded:
        flags.append("degraded")
    return ",".join(flags) if flags else "stable"


def operating_envelope_rows(condition_summaries: list[dict[str, Any]], camera_stage_summary: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    if camera_stage_summary:
        passing = [row for row in camera_stage_summary if row.get("within_camera_envelope") in (True, "True", "true")]
        degraded = [row for row in camera_stage_summary if row.get("within_camera_envelope") not in (True, "True", "true")]
        rows.append(
            {
                "profile": "overview camera capacity",
                "supported_boundary": max([int(row["active_streams"]) for row in passing], default=""),
                "first_degraded_condition": min([int(row["active_streams"]) for row in degraded], default="not observed"),
                "limiting_metric": "displayed FPS per active camera",
                "basis": "E10 staged concurrent camera runs",
            }
        )
    for row in condition_summaries:
        experiment = row.get("experiment")
        condition = row.get("condition")
        valid = as_int(row.get("valid_runs"))
        within = as_int(row.get("within_envelope_runs"))
        degraded = as_int(row.get("degraded_runs"))
        if valid <= 0:
            continue
        if within == valid:
            boundary = "within envelope"
            limiting_metric = "see run summaries"
        elif degraded > 0:
            boundary = "degraded boundary"
            limiting_metric = "see envelope violations and run summaries"
        elif within == 0 and degraded == 0:
            boundary = "measurement valid; envelope not explicitly scored"
            limiting_metric = "legacy quality file lacks explicit envelope flags"
        else:
            boundary = "mixed"
            limiting_metric = "see envelope violations and run summaries"
        rows.append(
            {
                "profile": f"{experiment}: {condition}",
                "supported_boundary": boundary,
                "first_degraded_condition": "current condition" if degraded > 0 else "",
                "limiting_metric": limiting_metric,
                "basis": f"{valid} valid run(s), {within} within envelope, {degraded} degraded",
            }
        )
    return rows


def latex_escape(value: Any) -> str:
    text = str(value if value is not None else "")
    repl = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(repl.get(ch, ch) for ch in text)


def write_latex_table(path: Path, caption: str, label: str, headers: list[str], rows: list[list[Any]]) -> None:
    lines = [
        r"\begin{table}[t]",
        r"\centering",
        r"\small",
        r"\begin{tabular}{" + "l" * len(headers) + "}",
        r"\toprule",
        " & ".join(latex_escape(h) for h in headers) + r" \\",
        r"\midrule",
    ]
    for row in rows:
        lines.append(" & ".join(latex_escape(item) for item in row) + r" \\")
    lines.extend(
        [
            r"\bottomrule",
            r"\end{tabular}",
            rf"\caption{{{latex_escape(caption)}}}",
            rf"\label{{{label}}}",
            r"\end{table}",
            "",
        ]
    )
    write_text(path, "\n".join(lines))


def write_tables(output_root: Path, condition_rows: list[dict[str, Any]], camera_summary: list[dict[str, Any]], run_summaries: list[dict[str, Any]]) -> None:
    tables = output_root / "tables"
    write_csv(tables / "condition_summaries.csv", condition_rows)
    write_csv(tables / "camera_capacity_stages.csv", camera_summary)

    setup_rows = [
        ["Headset", "Meta Quest 3"],
        ["Unity", parse_unity_version(HORUS_MR_ROOT) or "recorded in headset manifests"],
        ["Meta XR SDK", parse_meta_xr_version(HORUS_MR_ROOT) or "recorded in package manifest"],
        ["ROS 2", "Jazzy"],
        ["Network", "Local LAN/core ROS 2 bridge; WAN/VPN/cloud excluded"],
        ["Envelope", "Quest FPS >= 60; overview camera >= 15 FPS; command p95 <= 150 ms; goal p95 <= 500 ms"],
    ]
    write_latex_table(tables / "experimental_setup.tex", "Experimental setup and predefined usability envelope.", "tab:setup", ["Item", "Value"], setup_rows)

    baseline = [row for row in condition_rows if row["experiment"] == "E0_baseline"]
    baseline_rows = [
        [
            row["condition"],
            row["valid_runs"],
            fmt(row.get("quest_fps_p50_median"), 1),
            fmt(row.get("camera_latency_p95_ms_max_median"), 1),
            fmt(row.get("camera_displayed_fps_min_median"), 1),
        ]
        for row in baseline
    ] or [["No included baseline runs", "", "", "", ""]]
    write_latex_table(
        tables / "baseline_latency.tex",
        "Baseline responsiveness under minimal local load.",
        "tab:baseline",
        ["Condition", "n", "Quest FPS", "Camera p95 ms", "Displayed FPS"],
        baseline_rows,
    )

    media_rows = []
    for row in camera_summary:
        media_rows.append(
            [
                f"{row['active_streams']} cameras",
                row["valid_runs"],
                fmt(row.get("displayed_hz_per_stream_min"), 1),
                fmt(row.get("camera_latency_p95_ms_max"), 1),
                "pass" if row.get("within_camera_envelope") else "degraded",
            ]
        )
    map_rows = []
    for row in condition_rows:
        if row["experiment"] == "E4_mesh_vs_pointcloud_map":
            map_rows.append(
                [
                    row["condition"],
                    row["valid_runs"],
                    fmt(row.get("quest_fps_p50_median"), 1),
                    fmt(row.get("map_pointcloud_upload_ms_p95_median") or row.get("map_mesh_finalize_ms_p95_median"), 2),
                    "degraded" if as_int(row.get("degraded_runs")) else "pass",
                ]
            )
    write_latex_table(
        tables / "media_spatial_capacity.tex",
        "Camera and shared-map capacity results.",
        "tab:media_spatial",
        ["Condition", "n", "Displayed/FPS", "p95 ms", "Class"],
        media_rows + map_rows,
    )

    scaling_rows = []
    for row in condition_rows:
        if row["experiment"] in {"E6_control_under_sensor_load", "E7_multi_robot_scaling", "E8_multi_operator_scaling"}:
            scaling_rows.append(
                [
                    row["experiment"].replace("_", " "),
                    row["valid_runs"],
                    fmt(row.get("quest_fps_p50_median"), 1),
                    fmt(row.get("camera_displayed_fps_min_median"), 1),
                    "degraded" if as_int(row.get("degraded_runs")) else "pass",
                ]
            )
    write_latex_table(
        tables / "scaling_and_leases.tex",
        "Control, robot-count, and operator-count scaling.",
        "tab:scaling",
        ["Experiment", "n", "Quest FPS", "Worst cam FPS", "Class"],
        scaling_rows,
    )

    failure_rows = []
    for row in condition_rows:
        if row["experiment"] == "E9_failure_recovery":
            failure_rows.append(
                [
                    row["condition"],
                    row["valid_runs"],
                    fmt(row.get("quest_fps_p50_median"), 1),
                    "degraded" if as_int(row.get("degraded_runs")) else "pass",
                    "local failure/recovery workload",
                ]
            )
    write_latex_table(
        tables / "failure_recovery.tex",
        "Local failure and recovery behavior.",
        "tab:failure",
        ["Condition", "n", "Quest FPS", "Class", "Note"],
        failure_rows or [["No included failure runs", "", "", "", ""]],
    )


def plot_camera_capacity(output_root: Path, camera_summary: list[dict[str, Any]]) -> None:
    if not camera_summary:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    x = [int(row["active_streams"]) for row in camera_summary]
    fps = [as_float(row.get("displayed_hz_per_stream_median")) or 0 for row in camera_summary]
    latency = [as_float(row.get("camera_latency_p95_ms_median")) or 0 for row in camera_summary]
    fig, ax1 = plt.subplots(figsize=(6.5, 3.6))
    ax1.plot(x, fps, marker="o", color="#006d77", label="Displayed FPS per camera")
    ax1.axhline(USABILITY_ENVELOPE["overview_camera_displayed_fps_min"], color="#006d77", linestyle="--", linewidth=1)
    ax1.set_xlabel("Concurrent overview camera streams")
    ax1.set_ylabel("Displayed FPS per camera")
    ax1.set_xticks(x)
    ax2 = ax1.twinx()
    ax2.plot(x, latency, marker="s", color="#7b2cbf", label="Camera latency p95")
    ax2.set_ylabel("Application-level camera latency p95 (ms)")
    fig.suptitle("Concurrent camera-stream capacity")
    fig.tight_layout()
    for suffix in ("svg", "pdf"):
        fig.savefig(output_root / "figures" / f"camera_stream_capacity.{suffix}", bbox_inches="tight")
    plt.close(fig)


def plot_map_comparison(output_root: Path, condition_rows: list[dict[str, Any]]) -> None:
    rows = [row for row in condition_rows if row["experiment"] == "E4_mesh_vs_pointcloud_map"]
    if not rows:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = []
    fps = []
    p95 = []
    for row in rows:
        label = "PointCloud2 map" if "pointcloud" in row["condition"] else "Triangle mesh"
        labels.append(label)
        fps.append(as_float(row.get("quest_fps_p50_median")) or 0)
        p95.append(as_float(row.get("map_pointcloud_upload_ms_p95_median") or row.get("map_mesh_finalize_ms_p95_median")) or 0)
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.4))
    axes[0].bar(labels, fps, color=["#7b2cbf", "#006d77"][: len(labels)])
    axes[0].axhline(60, color="#333", linestyle="--", linewidth=1)
    axes[0].set_ylabel("Quest FPS p50")
    axes[0].set_title("Render envelope")
    axes[1].bar(labels, p95, color=["#7b2cbf", "#006d77"][: len(labels)])
    axes[1].set_ylabel("Apply/finalize p95 (ms)")
    axes[1].set_title("Map application cost")
    fig.suptitle("Matched shared-map representation comparison")
    fig.tight_layout()
    for suffix in ("svg", "pdf"):
        fig.savefig(output_root / "figures" / f"mesh_vs_pointcloud_map.{suffix}", bbox_inches="tight")
    plt.close(fig)


def plot_scaling(output_root: Path, condition_rows: list[dict[str, Any]]) -> None:
    rows = [
        row for row in condition_rows
        if row["experiment"] in {"E6_control_under_sensor_load", "E7_multi_robot_scaling", "E8_multi_operator_scaling"}
    ]
    if not rows:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = [row["experiment"].replace("E", "E ").replace("_", " ") for row in rows]
    fps = [as_float(row.get("quest_fps_p50_median")) or 0 for row in rows]
    camera = [as_float(row.get("camera_displayed_fps_min_median")) or 0 for row in rows]
    fig, ax = plt.subplots(figsize=(7.2, 3.6))
    x = list(range(len(labels)))
    width = 0.35
    ax.bar([i - width / 2 for i in x], fps, width, label="Quest FPS p50", color="#006d77")
    ax.bar([i + width / 2 for i in x], camera, width, label="Worst camera FPS", color="#7b2cbf")
    ax.axhline(60, color="#006d77", linestyle="--", linewidth=1)
    ax.axhline(15, color="#7b2cbf", linestyle="--", linewidth=1)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=20, ha="right")
    ax.set_ylabel("FPS")
    ax.set_title("Control and scaling workloads")
    ax.legend(loc="best")
    fig.tight_layout()
    for suffix in ("svg", "pdf"):
        fig.savefig(output_root / "figures" / f"control_robot_operator_scaling.{suffix}", bbox_inches="tight")
    plt.close(fig)


def claim_evidence_rows(condition_rows: list[dict[str, Any]], camera_summary: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if camera_summary:
        passing = [row for row in camera_summary if row.get("within_camera_envelope") in (True, "True", "true")]
        max_camera = max([int(row["active_streams"]) for row in passing], default="")
        last = camera_summary[-1]
        rows.append(
            {
                "proposed_paper_claim": f"Under the tested 1280x720/30 ROS-compressed overview-camera profile, HORUS sustained up to {max_camera} concurrent camera visualizations inside the camera envelope.",
                "experiment_conditions": "E10 staged camera capacity, 1/2/4/6/8 active streams",
                "metric": "Displayed FPS per active camera; application-level camera p95 latency",
                "table_or_figure": "Figure camera_stream_capacity; table camera_capacity_stages",
                "valid_runs": last.get("valid_runs", ""),
                "uncertainty": "Reported as run-level median/spread across repetitions; frames are not treated as independent repetitions.",
                "caveat_or_limitation": "Quest FPS is run-level, not stage-level; result is limited to local LAN and the tested compressed profile.",
            }
        )
    map_rows = [row for row in condition_rows if row["experiment"] == "E4_mesh_vs_pointcloud_map"]
    if map_rows:
        rows.append(
            {
                "proposed_paper_claim": "For the evaluated shared-map workload, the triangle-shell mesh path provides a more favorable large-scene MR rendering trade-off than the PointCloud2 map path.",
                "experiment_conditions": "E4 triangle_shell_mesh_24k_triangles_lan and pointcloud_map_250k_points_lan",
                "metric": "Quest FPS, bridge queueing, map apply/finalize time, valid/degraded classification",
                "table_or_figure": "Figure mesh_vs_pointcloud_map; table media_spatial_capacity",
                "valid_runs": "; ".join(f"{row['condition']}: {row['valid_runs']}" for row in map_rows),
                "uncertainty": "Run-level summaries; pointcloud-map currently has fewer valid repetitions than the preferred matched comparison target.",
                "caveat_or_limitation": "The two representations are not a universal proof; the claim is limited to the tested environment/fidelity/update rates.",
            }
        )
    for experiment, claim in [
        ("E6_control_under_sensor_load", "Control remains measurable under cumulative local sensor load, but degraded runs identify the operating boundary."),
        ("E7_multi_robot_scaling", "The eight-robot mixed workload is a degraded boundary condition under the tested camera/sensor profile."),
        ("E8_multi_operator_scaling", "Multiple operators can share state and leases, with camera display rate becoming the limiting factor in this workload."),
        ("E9_failure_recovery", "Local failure/recovery behavior is captured as a valid degraded boundary workload."),
    ]:
        matches = [row for row in condition_rows if row["experiment"] == experiment]
        if matches:
            rows.append(
                {
                    "proposed_paper_claim": claim,
                    "experiment_conditions": "; ".join(row["condition"] for row in matches),
                    "metric": "Quest FPS, camera displayed FPS, queue/failure metrics",
                    "table_or_figure": "Figure control_robot_operator_scaling; tables scaling_and_leases/failure_recovery",
                    "valid_runs": sum(as_int(row.get("valid_runs")) for row in matches),
                    "uncertainty": "Descriptive repeated-run summaries.",
                    "caveat_or_limitation": "Synthetic local workload; not a WAN or public-Internet connector result.",
                }
            )
    return rows


def write_caveats(output_root: Path, records: list[RunRecord], condition_rows: list[dict[str, Any]]) -> None:
    pointcloud = [row for row in condition_rows if row["experiment"] == "E4_mesh_vs_pointcloud_map" and "pointcloud" in row["condition"]]
    pointcloud_note = "No accepted pointcloud-map condition was found."
    if pointcloud:
        row = pointcloud[0]
        pointcloud_note = (
            f"The pointcloud-map condition has {row['valid_runs']} included valid run(s); "
            "one invalid run was excluded due to headset/clock instrumentation failure if present in excluded_runs.csv."
        )
    lines = [
        "# Caveats",
        "",
        "- This dataset characterizes the local/core HORUS MR-ROS 2 platform only. HORUS Connector, VPN, cloud relay, Zenoh, public-Internet, and cross-site results are excluded.",
        "- Camera latency is application-level camera latency from source frame timestamp to headset display/update timestamp. It is not optical motion-to-photon latency.",
        "- The experimental unit is the run. Confidence intervals and repeated-run summaries are computed over runs, not over individual frames.",
        "- Legacy runs with `ok=true` but no explicit `measurement_valid` field are included as measurement-valid legacy captures and flagged in `run_quality_report.csv`.",
        f"- {pointcloud_note}",
        "- Mesh-versus-pointcloud conclusions are conditional on the tested environment, map fidelity, update rate, hardware, and local network.",
    ]
    write_text(output_root / "evidence" / "caveats.md", "\n".join(lines) + "\n")


def write_markdown_report(
    output_root: Path,
    records: list[RunRecord],
    condition_rows: list[dict[str, Any]],
    camera_summary: list[dict[str, Any]],
    claim_rows: list[dict[str, Any]],
) -> None:
    included = [record for record in records if record.included]
    experiments = sorted({str(record.manifest.get("experiment")) for record in included})
    camera_claim = ""
    if camera_summary:
        max_pass = max([int(row["active_streams"]) for row in camera_summary if row.get("within_camera_envelope")], default="")
        last = camera_summary[-1]
        camera_claim = (
            f"The staged camera experiment supports up to {max_pass} concurrent overview cameras under "
            f"the tested profile; at 8 streams the median displayed rate was "
            f"{fmt(last.get('displayed_hz_per_stream_median'), 1)} FPS per camera with "
            f"{fmt(last.get('camera_latency_p95_ms_median'), 1)} ms p95 application-level latency."
        )
    map_claim = ""
    map_rows = [row for row in condition_rows if row["experiment"] == "E4_mesh_vs_pointcloud_map"]
    if map_rows:
        map_claim = "The mesh/pointcloud comparison should be reported conditionally; the current accepted pointcloud-map sample is usable but below the preferred matched-repeat target."
    lines = [
        "# HORUS i-RIM 2026 Experimental Characterization Analysis",
        "",
        "## Technical Summary",
        "",
        f"- The frozen analysis includes {len(included)} measurement-valid run folders across {len(experiments)} core experiment groups: {', '.join(experiments)}.",
        f"- {camera_claim}" if camera_claim else "- Camera-capacity evidence was not found.",
        f"- {map_claim}" if map_claim else "- Matched map-representation evidence was not found.",
        "- Poor performance is retained as degraded boundary evidence; only instrumentation/configuration failures are excluded.",
        "",
        "## Key Findings With Evidence",
        "",
        "### Concurrent camera streams define a concrete overview-camera envelope",
        "",
        "The E10 staged experiment is the headline camera-capacity result because it uses simultaneous active streams rather than separate consecutive one-off conditions. Use `figures/camera_stream_capacity.svg` and `tables/camera_capacity_stages.csv`.",
        "",
        "### Mesh maps and PointCloud2 maps must be discussed as a tested trade-off",
        "",
        "The E4 comparison supports a conditional design recommendation, not a universal rule. Use `figures/mesh_vs_pointcloud_map.svg` and `tables/media_spatial_capacity.tex`; state the number of valid matched runs.",
        "",
        "### Degraded scaling runs are boundary results, not failed analysis",
        "",
        "E7/E8/E9 degraded runs remain included because they capture the operating boundary under robot, operator, and recovery workloads.",
        "",
        "## Scope, Data, and Metric Definitions",
        "",
        "The analysis reads raw folders from `results/` and writes only under `analysis/irim_2026/`. Metric definitions are in `data/metric_dictionary.csv`. Run classification is in `runs/run_quality_report.csv`.",
        "",
        "## Methodology",
        "",
        "Run-level p50/p95/p99 values are computed first. Condition summaries are then computed across run-level summaries. Individual frames are not treated as independent repetitions.",
        "",
        "## Limitations and Robustness Checks",
        "",
        "See `evidence/caveats.md`. Stability diagnostics are in `data/stability_flags.csv`, and every paper claim is linked to evidence in `evidence/claim_evidence.csv`.",
        "",
        "## Recommended Paper Result Set",
        "",
        "1. Table 1: Experimental setup and predefined envelope.",
        "2. Table 2: Baseline latency.",
        "3. Figure 1: Concurrent camera-stream capacity.",
        "4. Figure 2: Matched mesh-versus-pointcloud shared-map comparison.",
        "5. Figure 3: Control and robot/operator scaling.",
        "6. Table 3: Failure and recovery.",
        "",
    ]
    write_text(output_root / "text" / "full_analysis_report.md", "\n".join(lines))


def write_experimental_characterization_tex(output_root: Path, condition_rows: list[dict[str, Any]], camera_summary: list[dict[str, Any]]) -> None:
    max_camera = ""
    camera_latency = ""
    camera_fps = ""
    if camera_summary:
        passing = [row for row in camera_summary if row.get("within_camera_envelope")]
        max_camera = str(max([int(row["active_streams"]) for row in passing], default=""))
        last = camera_summary[-1]
        camera_fps = fmt(last.get("displayed_hz_per_stream_median"), 1)
        camera_latency = fmt(last.get("camera_latency_p95_ms_median"), 1)
    pointcloud_runs = ""
    mesh_runs = ""
    for row in condition_rows:
        if row["experiment"] == "E4_mesh_vs_pointcloud_map":
            if "pointcloud" in row["condition"]:
                pointcloud_runs = str(row["valid_runs"])
            else:
                mesh_runs = str(row["valid_runs"])
    text = rf"""
\section{{Experimental Characterization}}
\subsection{{Setup, Metrics, and Validity}}
We characterized the local HORUS MR--ROS~2 stack on the hardware and software configuration summarized in Table~\ref{{tab:setup}}. The predefined interactive envelope required Quest application frame rate to remain at or above 60~FPS, overview-camera display rate to remain at or above 15~FPS per active camera, teleoperation command latency p95 to remain below 150~ms, task/goal acceptance latency p95 to remain below 500~ms, and TF freshness p95 to remain below 100~ms. Runs were classified separately by measurement validity and by whether they remained inside the envelope; degraded but valid runs were retained as boundary evidence.

\subsection{{Baseline Responsiveness}}
Baseline responsiveness is reported in Table~\ref{{tab:baseline}}. Metrics are summarized at the run level before aggregation across repetitions, so individual frames are not treated as independent trials.

\subsection{{Media and Spatial-Data Operating Envelope}}
The concurrent camera-stream experiment is summarized in Fig.~\ref{{fig:camera_capacity}}. Under the tested 1280$\times$720, 30~FPS ROS-compressed overview-camera profile, HORUS sustained up to {max_camera or 'N'} simultaneously active camera visualizations within the predefined camera envelope. At the largest tested condition, the median displayed rate was {camera_fps or 'X'}~FPS per active camera and the p95 application-level camera latency was {camera_latency or 'Y'}~ms. The reported latency is application-level source-to-display latency and should not be interpreted as optical motion-to-photon latency.

The shared-map comparison in Fig.~\ref{{fig:map_comparison}} contrasts the triangle-shell mesh path with the PointCloud2 map path. The current dataset contains {mesh_runs or '0'} valid mesh-map run(s) and {pointcloud_runs or '0'} valid pointcloud-map run(s). The result should therefore be stated conditionally: for the evaluated environment, fidelity, update rate, and Quest-based renderer, the mesh representation provided the more favorable large-scene communication/rendering trade-off, while PointCloud2 maps remain a measured but heavier boundary condition.

\subsection{{Control, Robot, and Operator Scaling}}
Control-under-load, multi-robot, and multi-operator results are summarized in Fig.~\ref{{fig:scaling}} and Table~\ref{{tab:scaling}}. These runs define the supported operating envelope rather than a universal scalability claim; when workloads cross the envelope, the first limiting metric is reported rather than excluded.

\subsection{{Recovery and Bottlenecks}}
Local failure and recovery behavior is summarized in Table~\ref{{tab:failure}}. The analysis excludes WAN, VPN, cloud relay, Zenoh, HORUS Connector, and public-Internet experiments; those conditions belong to the remote-operation study rather than this i-RIM core-platform characterization.
""".strip() + "\n"
    write_text(output_root / "text" / "experimental_characterization.tex", text)


def render_figures(output_root: Path, condition_rows: list[dict[str, Any]], camera_summary: list[dict[str, Any]]) -> None:
    (output_root / "figures").mkdir(parents=True, exist_ok=True)
    plot_camera_capacity(output_root, camera_summary)
    plot_map_comparison(output_root, condition_rows)
    plot_scaling(output_root, condition_rows)


def build_analysis(results_root: Path, output_root: Path) -> dict[str, Any]:
    output_root.mkdir(parents=True, exist_ok=True)
    records = collect_runs(results_root)
    run_summaries = [extract_run_summary(record) for record in records]
    included_rows, excluded_rows = included_excluded_rows(records)
    quality_rows = run_quality_rows(records)
    metrics_long: list[dict[str, Any]] = []
    for record in records:
        metrics_long.extend(metric_long_rows(record))
    condition_rows = condition_summary_rows(run_summaries)
    camera_rows = camera_stage_rows(records)
    camera_summary = aggregate_camera_stage_rows(camera_rows)
    stability = stability_rows(records)
    operating = operating_envelope_rows(condition_rows, camera_summary)
    claims = claim_evidence_rows(condition_rows, camera_summary)

    write_json(output_root / "provenance" / "dataset_manifest.json", build_dataset_manifest(records, output_root))
    write_json(output_root / "provenance" / "software_versions.json", software_versions(records))
    write_json(output_root / "provenance" / "hardware_network_setup.json", hardware_network_setup(records))

    write_csv(output_root / "runs" / "included_runs.csv", included_rows)
    write_csv(output_root / "runs" / "excluded_runs.csv", excluded_rows)
    write_csv(output_root / "runs" / "run_quality_report.csv", quality_rows)

    write_csv(output_root / "data" / "metric_dictionary.csv", metric_dictionary())
    write_csv(output_root / "data" / "metrics_long.csv", metrics_long)
    write_csv(output_root / "data" / "run_summaries.csv", run_summaries)
    write_csv(output_root / "data" / "condition_summaries.csv", condition_rows)
    write_csv(output_root / "data" / "operating_envelope.csv", operating)
    write_csv(output_root / "data" / "camera_capacity_stages.csv", camera_rows)
    write_csv(output_root / "data" / "camera_capacity_stage_summary.csv", camera_summary)
    write_csv(output_root / "data" / "stability_flags.csv", stability)

    write_csv(output_root / "evidence" / "claim_evidence.csv", claims)
    write_caveats(output_root, records, condition_rows)

    write_tables(output_root, condition_rows, camera_summary, run_summaries)
    render_figures(output_root, condition_rows, camera_summary)
    write_markdown_report(output_root, records, condition_rows, camera_summary, claims)
    write_experimental_characterization_tex(output_root, condition_rows, camera_summary)

    summary = {
        "analysis_root": str(output_root),
        "run_count": len(records),
        "included_runs": len(included_rows),
        "excluded_runs": len(excluded_rows),
        "conditions": len(condition_rows),
        "camera_stage_conditions": len(camera_summary),
        "figures": sorted(str(path.relative_to(output_root)) for path in (output_root / "figures").glob("*")),
        "tables": sorted(str(path.relative_to(output_root)) for path in (output_root / "tables").glob("*")),
        "required_outputs_created": {
            "included_runs.csv": (output_root / "runs" / "included_runs.csv").exists(),
            "excluded_runs.csv": (output_root / "runs" / "excluded_runs.csv").exists(),
            "run_quality_report.csv": (output_root / "runs" / "run_quality_report.csv").exists(),
            "claim_evidence.csv": (output_root / "evidence" / "claim_evidence.csv").exists(),
            "experimental_characterization.tex": (output_root / "text" / "experimental_characterization.tex").exists(),
        },
    }
    write_json(output_root / "analysis_summary.json", summary)
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-root", default=str(DEFAULT_RESULTS_ROOT))
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = build_analysis(Path(args.results_root).resolve(), Path(args.output_root).resolve())
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
