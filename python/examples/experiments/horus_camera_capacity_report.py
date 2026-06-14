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


def ratio(numerator: int, denominator: int) -> float:
    return 0.0 if denominator <= 0 else numerator / denominator


def summarize_run(run_dir: Path) -> dict[str, Any]:
    manifest = load_json(run_dir / "run_manifest.json")
    quality = load_json(run_dir / "data_quality.json")
    workload = ((manifest.get("extra") or {}).get("workload") or {})
    camera = workload.get("camera") or {}
    sdk_start, sdk_end = sdk_window(run_dir)
    headset_start, headset_end = headset_window(run_dir)

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
    return {
        "run_id": run_dir.name,
        "condition": manifest.get("condition", ""),
        "quality_ok": quality.get("ok", False),
        "streams": camera.get("streams", manifest.get("stream_count", "")),
        "resolution": camera.get("resolution", manifest.get("resolution", "")),
        "target_fps": camera.get("fps", manifest.get("target_fps", "")),
        "duration_s": manifest.get("duration_s", ""),
        "source_camera_frames": source_count,
        "headset_camera_received": received_count,
        "headset_camera_displayed": displayed_count,
        "received_ratio": f"{ratio(received_count, source_count):.4f}",
        "displayed_ratio": f"{ratio(displayed_count, source_count):.4f}",
    }


def main() -> int:
    args = parse_args()
    runs = [run.resolve() for run in (args.runs or sorted(args.results_root.glob("e10_camera_capacity_*"))) if run.is_dir()]
    if not args.include_incomplete:
        runs = [run for run in runs if (run / "data_quality.json").exists()]
    rows = [summarize_run(run) for run in runs]
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
