"""Small summary helpers for HORUS experiment output directories."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import median
from typing import Dict, Iterable, List, Mapping, Optional, Tuple


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


def metric_row_count(path: Path, measurement_window: Optional[Tuple[int, int]] = None) -> int:
    rows = _read_csv_rows(Path(path))
    return len(_filter_measurement_rows(rows, measurement_window))


def summarize_numeric_csv(path: Path, measurement_window: Optional[Tuple[int, int]] = None) -> Dict[str, object]:
    all_rows = _read_csv_rows(Path(path))

    rows = _filter_measurement_rows(all_rows, measurement_window)
    summary: Dict[str, object] = {"row_count": len(rows), "row_count_total": len(all_rows)}
    if measurement_window is not None:
        summary["measurement_window_applied"] = True
    if not rows:
        return summary

    numeric_columns = set()
    for row in rows:
        for key, value in row.items():
            if key in {"timestamp_ns", "run_id", "experiment", "condition"}:
                continue
            try:
                float(value)
            except Exception:
                continue
            numeric_columns.add(key)

    for column in sorted(numeric_columns):
        values = []
        for row in rows:
            try:
                values.append(float(row[column]))
            except Exception:
                pass
        if values:
            summary[column] = {
                "min": min(values),
                "p50": median(values),
                "p95": percentile(values, 95),
                "p99": percentile(values, 99),
                "max": max(values),
            }
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

    candidates: List[object] = [manifest.get("stream_count")]
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


def write_summary(run_dir: Path) -> Path:
    run_dir = Path(run_dir)
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
    files: Dict[str, object] = {}
    errors: List[str] = []
    warnings: List[str] = []
    window = _measurement_window(run_dir)
    headset_window = _measurement_window_from_events(run_dir / "headset_events.ndjson")
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

    result: Dict[str, object] = {
        "ok": not errors,
        "errors": errors,
        "warnings": warnings,
        "required_metrics": dict(required_metrics),
        "files": files,
    }
    (run_dir / "data_quality.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result
