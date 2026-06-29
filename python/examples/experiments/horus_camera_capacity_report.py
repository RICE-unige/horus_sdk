#!/usr/bin/env python3
"""Summarize E10 camera-capacity run folders."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
from typing import Any


SDK_ROOT = Path(__file__).resolve().parents[3]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "runs",
        nargs="*",
        type=Path,
        help="Run folders to summarize. Defaults to results/e10_camera_capacity_*.",
    )
    parser.add_argument("--results-root", type=Path, default=SDK_ROOT / "results")
    parser.add_argument("--include-incomplete", action="store_true")
    parser.add_argument("--output", type=Path, default=None, help="Optional CSV output path.")
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8-sig"))


def headset_window(run_dir: Path) -> tuple[int | None, int | None]:
    events_path = run_dir / "headset_events.ndjson"
    if not events_path.exists():
        return None, None
    start = None
    end = None
    for line in events_path.read_text(encoding="utf-8-sig").splitlines():
        if not line.strip():
            continue
        event = json.loads(line)
        if event.get("name") == "measurement_start" and start is None:
            start = int(event["timestamp_ns"])
        elif event.get("name") == "measurement_end":
            end = int(event["timestamp_ns"])
    return start, end


def sdk_window(run_dir: Path) -> tuple[int | None, int | None]:
    summary = load_json(run_dir / "summary.json")
    window = summary.get("_measurement_window") or {}
    start = window.get("start_timestamp_ns")
    end = window.get("end_timestamp_ns")
    return (int(start), int(end)) if start is not None and end is not None else (None, None)


def rows_in_window(path: Path, start: int | None, end: int | None) -> list[dict[str, str]]:
    if not path.exists():
        return []
    rows: list[dict[str, str]] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            if start is not None and end is not None:
                try:
                    timestamp = int(float(row.get("timestamp_ns") or "0"))
                except ValueError:
                    continue
                if timestamp < start or timestamp > end:
                    continue
            rows.append(row)
    return rows


def staged_camera_config(manifest: dict[str, Any]) -> dict[str, Any]:
    workload = ((manifest.get("extra") or {}).get("workload") or {})
    staging = workload.get("camera_staging") if isinstance(workload, dict) else None
    if not staging or not isinstance(staging, dict):
        workload_extra = workload.get("extra") if isinstance(workload, dict) else {}
        staging = workload_extra.get("camera_staging") if isinstance(workload_extra, dict) else {}
    if not isinstance(staging, dict):
        return {}
    enabled = str(staging.get("enabled", True)).strip().lower() not in {"0", "false", "no", "off"}
    if not enabled:
        return {}
    stream_counts = []
    for value in staging.get("stream_counts") or staging.get("stages") or []:
        try:
            stream_counts.append(int(value))
        except (TypeError, ValueError):
            continue
    try:
        stage_duration_s = float(staging.get("stage_duration_s") or 0.0)
    except (TypeError, ValueError):
        stage_duration_s = 0.0
    if not stream_counts or stage_duration_s <= 0.0:
        return {}
    return {"stream_counts": stream_counts, "stage_duration_s": stage_duration_s}


def ratio(numerator: int, denominator: int) -> float:
    return 0.0 if denominator <= 0 else numerator / denominator


def percentile(values: list[float], p: float) -> float | None:
    ordered = sorted(values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * min(max(p, 0.0), 100.0) / 100.0
    low = int(rank)
    high = min(low + 1, len(ordered) - 1)
    frac = rank - low
    return ordered[low] * (1.0 - frac) + ordered[high] * frac


def numeric(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def summarize_window(
    run_dir: Path,
    manifest: dict[str, Any],
    quality: dict[str, Any],
    derived: dict[str, Any],
    *,
    sdk_start: int | None,
    sdk_end: int | None,
    headset_start: int | None,
    headset_end: int | None,
    stage_index: int | str = "",
    active_streams: int | str | None = None,
    stage_start_s: float | str = "",
    stage_end_s: float | str = "",
) -> dict[str, Any]:
    workload = ((manifest.get("extra") or {}).get("workload") or {})
    camera = workload.get("camera") or {}

    source_rows = rows_in_window(run_dir / "source_metrics.csv", sdk_start, sdk_end)
    headset_rows = rows_in_window(run_dir / "headset_metrics.csv", headset_start, headset_end)
    source_camera = [row for row in source_rows if row.get("stream") == "camera" or "/camera_" in (row.get("topic") or "")]
    headset_camera_received = [
        row for row in headset_rows
        if row.get("category") == "camera" and row.get("name") in {"received", "received_compressed"}
    ]
    headset_camera_displayed = [
        row for row in headset_rows
        if row.get("category") == "camera" and row.get("name") in {"displayed", "displayed_compressed"}
    ]

    source_count = len(source_camera)
    received_count = len(headset_camera_received)
    displayed_count = len(headset_camera_displayed)
    if isinstance(stage_start_s, (int, float)) and isinstance(stage_end_s, (int, float)):
        duration_s = max(0.0, float(stage_end_s) - float(stage_start_s))
    else:
        duration_s = numeric(manifest.get("duration_s")) or 0.0
    displayed_hz_total = displayed_count / duration_s if duration_s > 0 else 0.0
    streams = numeric(active_streams if active_streams not in (None, "") else camera.get("streams", manifest.get("stream_count", ""))) or 0.0
    displayed_hz_per_stream = displayed_hz_total / streams if streams > 0 else 0.0

    latency_values: list[float] = []
    for row in rows_in_window(run_dir / "derived_metrics.csv", sdk_start, sdk_end):
        if row.get("category") != "latency":
            continue
        if "displayed" not in (row.get("name") or ""):
            continue
        value = numeric(row.get("latency_ms"))
        if value is not None:
            latency_values.append(value)
    p95_latency = percentile(latency_values, 95)
    stage_drop_rate = 1.0 - ratio(displayed_count, source_count)

    return {
        "run_id": run_dir.name,
        "condition": manifest.get("condition", ""),
        "stage_index": stage_index,
        "active_streams": active_streams if active_streams not in (None, "") else camera.get("streams", manifest.get("stream_count", "")),
        "stage_start_s": f"{stage_start_s:.3f}" if isinstance(stage_start_s, (int, float)) else stage_start_s,
        "stage_end_s": f"{stage_end_s:.3f}" if isinstance(stage_end_s, (int, float)) else stage_end_s,
        "measurement_valid": quality.get("measurement_valid", quality.get("ok", False)),
        "within_envelope": quality.get("within_envelope", False),
        "degraded": quality.get("degraded", False),
        "streams": camera.get("streams", manifest.get("stream_count", "")),
        "resolution": camera.get("resolution", manifest.get("resolution", "")),
        "target_fps": camera.get("fps", manifest.get("target_fps", "")),
        "duration_s": manifest.get("duration_s", ""),
        "source_camera_frames": source_count,
        "headset_camera_received": received_count,
        "headset_camera_displayed": displayed_count,
        "received_ratio": f"{ratio(received_count, source_count):.4f}",
        "displayed_ratio": f"{ratio(displayed_count, source_count):.4f}",
        "displayed_hz_total": f"{displayed_hz_total:.3f}",
        "displayed_hz_per_stream": f"{displayed_hz_per_stream:.3f}",
        "camera_latency_p95_ms_max": f"{p95_latency:.3f}" if p95_latency is not None else "",
        "camera_drop_rate_max": f"{stage_drop_rate:.4f}" if source_count > 0 else "",
    }


def summarize_run(run_dir: Path) -> list[dict[str, Any]]:
    manifest = load_json(run_dir / "run_manifest.json")
    quality = load_json(run_dir / "data_quality.json")
    derived = load_json(run_dir / "derived_summary.json")
    sdk_start, sdk_end = sdk_window(run_dir)
    headset_start, headset_end = headset_window(run_dir)
    staging = staged_camera_config(manifest)
    if not staging or sdk_start is None or headset_start is None:
        return [
            summarize_window(
                run_dir,
                manifest,
                quality,
                derived,
                sdk_start=sdk_start,
                sdk_end=sdk_end,
                headset_start=headset_start,
                headset_end=headset_end,
            )
        ]

    duration_s = numeric(manifest.get("duration_s")) or 0.0
    stage_duration_s = float(staging["stage_duration_s"])
    rows: list[dict[str, Any]] = []
    for index, stream_count in enumerate(staging["stream_counts"]):
        start_s = index * stage_duration_s
        end_s = min(duration_s, (index + 1) * stage_duration_s)
        if end_s <= start_s:
            continue
        rows.append(
            summarize_window(
                run_dir,
                manifest,
                quality,
                derived,
                sdk_start=sdk_start + int(start_s * 1_000_000_000),
                sdk_end=sdk_start + int(end_s * 1_000_000_000),
                headset_start=headset_start + int(start_s * 1_000_000_000),
                headset_end=headset_start + int(end_s * 1_000_000_000),
                stage_index=index,
                active_streams=stream_count,
                stage_start_s=start_s,
                stage_end_s=end_s,
            )
        )
    return rows


def main() -> int:
    args = parse_args()
    runs = [run.resolve() for run in (args.runs or sorted(args.results_root.glob("e10_camera_capacity_*"))) if run.is_dir()]
    if not args.include_incomplete:
        runs = [run for run in runs if (run / "data_quality.json").exists()]
    rows = [row for run in runs for row in summarize_run(run)]
    if not rows:
        print("No completed E10 camera-capacity run folders found.", file=sys.stderr)
        return 1
    fieldnames = list(rows[0].keys())
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(args.output)
    else:
        writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
