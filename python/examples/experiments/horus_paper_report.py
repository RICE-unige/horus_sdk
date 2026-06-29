#!/usr/bin/env python3
"""Build aggregate HORUS paper-result tables from measured experiment runs."""

from __future__ import annotations

import argparse
import csv
import html
import json
from pathlib import Path
from statistics import median
import sys
from typing import Any, Iterable

SDK_ROOT = Path(__file__).resolve().parents[3]
PYTHON_ROOT = SDK_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))


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


def load_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def measurement_window(run_dir: Path, events_file: str = "orchestrator_events.ndjson") -> tuple[int, int] | None:
    path = run_dir / events_file
    if not path.exists():
        return None
    start: int | None = None
    end: int | None = None
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            try:
                event = json.loads(line)
                timestamp_ns = int(event.get("timestamp_ns"))
            except Exception:
                continue
            name = event.get("name")
            if name == "measurement_start" and start is None:
                start = timestamp_ns
            elif name == "measurement_end":
                end = timestamp_ns
    if start is None or end is None or end <= start:
        return None
    return start, end


def in_window(row: dict[str, str], window: tuple[int, int] | None) -> bool:
    if window is None:
        return True
    try:
        timestamp_ns = int(row.get("timestamp_ns", ""))
    except Exception:
        return False
    return window[0] <= timestamp_ns <= window[1]


def as_float(value: str | None) -> float | None:
    try:
        return float(value) if value not in (None, "") else None
    except Exception:
        return None


def collect_run_dirs(results_root: Path, explicit_runs: list[str]) -> list[Path]:
    if explicit_runs:
        return [Path(run).resolve() for run in explicit_runs]
    return sorted(path for path in results_root.iterdir() if (path / "run_manifest.json").exists())


def run_index_rows(run_dirs: list[Path]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for run_dir in run_dirs:
        manifest = load_json(run_dir / "run_manifest.json")
        quality = load_json(run_dir / "data_quality.json")
        rows.append(
            {
                "run_id": run_dir.name,
                "experiment": manifest.get("experiment", ""),
                "condition": manifest.get("condition", ""),
                "measurement_valid": bool(quality.get("measurement_valid", quality.get("ok", False))),
                "within_envelope": bool(quality.get("within_envelope", False)),
                "degraded": bool(quality.get("degraded", False)),
                "duration_s": manifest.get("duration_s", ""),
                "warmup_s": manifest.get("warmup_s", ""),
                "robot_count": manifest.get("robot_count", ""),
                "operator_count": manifest.get("operator_count", ""),
                "stream_count": manifest.get("stream_count", ""),
                "transport": manifest.get("transport", ""),
                "topology": manifest.get("topology", ""),
                "resolution": manifest.get("resolution", ""),
                "target_fps": manifest.get("target_fps", ""),
                "robot_profile": manifest.get("robot_profile", ""),
                "map_profile": manifest.get("map_profile", ""),
                "path": str(run_dir),
            }
        )
    return rows


def metric_summary_rows(run_dirs: list[Path]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, ...], dict[str, list[float] | int]] = {}
    for run_dir in run_dirs:
        manifest = load_json(run_dir / "run_manifest.json")
        quality = load_json(run_dir / "data_quality.json")
        default_window = measurement_window(run_dir)
        headset_window = measurement_window(run_dir, "headset_events.ndjson") or default_window
        for metrics_path in sorted(run_dir.glob("*_metrics.csv")):
            window = headset_window if metrics_path.name.startswith("headset") else default_window
            try:
                rows = list(csv.DictReader(metrics_path.open(newline="", encoding="utf-8-sig")))
            except Exception:
                continue
            for row in rows:
                if not in_window(row, window):
                    continue
                key = (
                    run_dir.name,
                    str(manifest.get("experiment", "")),
                    str(manifest.get("condition", "")),
                    str(bool(quality.get("measurement_valid", quality.get("ok", False)))),
                    str(bool(quality.get("within_envelope", False))),
                    str(bool(quality.get("degraded", False))),
                    metrics_path.name,
                    row.get("category", "") or "",
                    row.get("name", "") or "",
                    row.get("stream", "") or "",
                    row.get("topic", "") or "",
                )
                bucket = groups.setdefault(
                    key,
                    {"count": 0, "value": [], "duration_ms": [], "latency_ms": [], "payload_bytes": []},
                )
                bucket["count"] = int(bucket["count"]) + 1
                for field in ("value", "duration_ms", "latency_ms", "payload_bytes"):
                    value = as_float(row.get(field))
                    if value is not None:
                        cast_bucket = bucket[field]
                        assert isinstance(cast_bucket, list)
                        cast_bucket.append(value)

    output: list[dict[str, Any]] = []
    for key, values in sorted(groups.items()):
        (
            run_id,
            experiment,
            condition,
            measurement_valid,
            within_envelope,
            degraded,
            file_name,
            category,
            name,
            stream,
            topic,
        ) = key
        row: dict[str, Any] = {
            "run_id": run_id,
            "experiment": experiment,
            "condition": condition,
            "measurement_valid": measurement_valid,
            "within_envelope": within_envelope,
            "degraded": degraded,
            "file": file_name,
            "category": category,
            "name": name,
            "stream": stream,
            "topic": topic,
            "count": values["count"],
        }
        for field in ("value", "duration_ms", "latency_ms", "payload_bytes"):
            samples = values[field]
            assert isinstance(samples, list)
            if samples:
                row[f"{field}_p50"] = median(samples)
                row[f"{field}_p95"] = percentile(samples, 95)
                row[f"{field}_p99"] = percentile(samples, 99)
                row[f"{field}_max"] = max(samples)
        output.append(row)
    return output


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


PAPER_TABLES: tuple[tuple[str, str, tuple[str, ...], tuple[str, ...]], ...] = (
    (
        "baseline_latency",
        "Baseline latency",
        ("E0_baseline",),
        ("latency", "tf", "state", "task", "goal", "teleop", "lease", "registration", "join", "replay"),
    ),
    (
        "camera_media",
        "Camera and media",
        ("E1_camera_transport", "E10_camera_capacity"),
        ("camera", "video", "frame", "fps", "latency", "bitrate", "drop", "image"),
    ),
    (
        "sensor_map_capacity",
        "Sensor and shared-map capacity",
        ("E3_pointcloud_capacity", "E4_mesh_vs_pointcloud_map", "E5_map_update_behavior"),
        ("laser", "pointcloud", "point_cloud", "map", "mesh", "grid", "path", "waypoint", "marker", "label"),
    ),
    (
        "control_under_load",
        "Control under sensor load",
        ("E6_control_under_sensor_load",),
        ("control", "teleop", "task", "goal", "lease", "tf", "stale", "drop", "latency"),
    ),
    (
        "scaling",
        "Multi-robot and multi-operator scaling",
        ("E7_multi_robot_scaling", "E8_multi_operator_scaling"),
        ("robot", "operator", "fps", "latency", "cpu", "memory", "registry", "lease", "state"),
    ),
    (
        "failure_recovery",
        "Local failure and recovery",
        ("E9_failure_recovery",),
        ("failure", "recovery", "stale", "blocked", "restart", "disconnect", "unregister", "lease"),
    ),
    (
        "map_load_limits",
        "Shared-map load limits (saturation sweep)",
        ("E11_map_pointcloud_sweep", "E12_map_mesh_sweep"),
        ("map", "pointcloud", "point_cloud", "mesh", "octomap", "marker", "outbound",
         "bulk", "realtime", "frame", "fps", "latency", "drop"),
    ),
)


def metric_text(row: dict[str, Any]) -> str:
    return " ".join(
        str(row.get(field, ""))
        for field in ("file", "category", "name", "stream", "topic", "condition")
    ).lower()


def paper_table_rows(
    metric_rows: list[dict[str, Any]],
    experiments: tuple[str, ...],
    keywords: tuple[str, ...],
) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    for row in metric_rows:
        if str(row.get("measurement_valid", "")).lower() != "true":
            continue
        if row.get("experiment") not in experiments:
            continue
        haystack = metric_text(row)
        if keywords and not any(keyword in haystack for keyword in keywords):
            continue
        output.append(row)
    return output


def preferred_numeric(row: dict[str, Any]) -> tuple[str, float] | None:
    for field in (
        "latency_ms_p95",
        "duration_ms_p95",
        "value_p95",
        "payload_bytes_p95",
        "latency_ms_p50",
        "duration_ms_p50",
        "value_p50",
        "payload_bytes_p50",
        "count",
    ):
        value = as_float(str(row.get(field, "")))
        if value is not None:
            return field, value
    return None


def write_markdown_table(path: Path, title: str, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text(
            f"# {title}\n\nNo measurement-valid measured rows matched this table yet.\n",
            encoding="utf-8",
        )
        return
    columns = [
        "experiment",
        "condition",
        "category",
        "name",
        "stream",
        "topic",
        "count",
        "latency_ms_p50",
        "latency_ms_p95",
        "latency_ms_p99",
        "duration_ms_p50",
        "duration_ms_p95",
        "duration_ms_p99",
        "value_p50",
        "value_p95",
        "value_p99",
        "payload_bytes_p95",
    ]
    lines = [
        f"# {title}",
        "",
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join("---" for _ in columns) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(row.get(column, "")) for column in columns) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_svg_bar_chart(path: Path, title: str, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    width = 960
    height = 420
    margin_left = 70
    margin_bottom = 110
    plot_width = width - margin_left - 30
    plot_height = height - 80 - margin_bottom

    points: list[tuple[str, str, float]] = []
    for row in rows[:24]:
        metric = preferred_numeric(row)
        if metric is None:
            continue
        field, value = metric
        label_parts = [
            str(row.get("condition", "")) or str(row.get("experiment", "")),
            str(row.get("name", "")) or str(row.get("category", "")),
        ]
        label = " / ".join(part for part in label_parts if part)
        points.append((label[:42], field, value))

    if not points:
        body = (
            f'<text x="{width / 2}" y="{height / 2}" text-anchor="middle" '
            'font-family="Arial, sans-serif" font-size="18" fill="#233">'
            "No measurement-valid measured rows matched this figure yet."
            "</text>"
        )
    else:
        max_value = max(value for _, _, value in points) or 1.0
        bar_gap = 8
        bar_width = max(8, (plot_width - bar_gap * (len(points) - 1)) / len(points))
        bars: list[str] = []
        labels: list[str] = []
        for index, (label, field, value) in enumerate(points):
            bar_height = (value / max_value) * plot_height
            x = margin_left + index * (bar_width + bar_gap)
            y = 70 + (plot_height - bar_height)
            bars.append(
                f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_width:.1f}" height="{bar_height:.1f}" '
                'fill="#4267b2" />'
            )
            bars.append(
                f'<title>{html.escape(label)} | {html.escape(field)} = {value:.3f}</title>'
            )
            labels.append(
                f'<text x="{x + bar_width / 2:.1f}" y="{height - 96}" transform="rotate(55 {x + bar_width / 2:.1f} {height - 96})" '
                'font-family="Arial, sans-serif" font-size="10" fill="#233">'
                f"{html.escape(label)}</text>"
            )
        body = "\n".join(
            [
                f'<line x1="{margin_left}" y1="{70 + plot_height}" x2="{width - 30}" y2="{70 + plot_height}" stroke="#888" />',
                f'<line x1="{margin_left}" y1="70" x2="{margin_left}" y2="{70 + plot_height}" stroke="#888" />',
                f'<text x="{margin_left - 10}" y="72" text-anchor="end" font-family="Arial, sans-serif" font-size="11" fill="#233">{max_value:.2f}</text>',
                *bars,
                *labels,
            ]
        )

    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">\n'
        '<rect width="100%" height="100%" fill="#fff" />\n'
        f'<text x="{width / 2}" y="34" text-anchor="middle" font-family="Arial, sans-serif" '
        f'font-size="22" font-weight="700" fill="#122">{html.escape(title)}</text>\n'
        f"{body}\n"
        "</svg>\n"
    )
    path.write_text(svg, encoding="utf-8")


def write_paper_artifacts(output_dir: Path, metric_rows: list[dict[str, Any]]) -> list[tuple[str, int]]:
    artifacts: list[tuple[str, int]] = []
    for slug, title, experiments, keywords in PAPER_TABLES:
        rows = paper_table_rows(metric_rows, experiments, keywords)
        write_csv(output_dir / "tables" / f"{slug}.csv", rows)
        write_markdown_table(output_dir / "tables" / f"{slug}.md", title, rows)
        write_svg_bar_chart(output_dir / "figures" / f"{slug}.svg", title, rows)
        artifacts.append((slug, len(rows)))
    return artifacts


def write_knee_artifacts(output_dir: Path, results_root: Path) -> list[tuple[str, Any]]:
    """Compute the saturation knee for each map-load sweep (E11/E12/E13) and render it as both
    JSON (per experiment) and one combined markdown table. Returns (experiment, knee_payload)."""
    try:
        from horus.experiments.analysis import write_knee_reports
    except Exception:
        return []

    knee_paths = write_knee_reports(results_root, output_dir / "tables")
    summary: list[tuple[str, Any]] = []
    lines = [
        "# Shared-map load limits — saturation knee",
        "",
        "Knee = smallest swept payload that is degraded AND stays degraded for every larger payload,",
        "under the pre-registered criterion (achieved_hz < 90% of source, OR drop_rate > 5%, OR",
        "glass-to-glass p95 > 2x baseline, OR headset FPS < 90% of target).",
        "",
        "| Experiment | Knee | Knee payload | Interpretation |",
        "| --- | --- | ---: | --- |",
    ]
    for knee_path in knee_paths:
        knee = load_json(knee_path)
        experiment = str(knee.get("experiment", knee_path.stem))
        knee_label = knee.get("knee")
        knee_payload = knee.get("knee_payload")
        interpretation = str(knee.get("interpretation", ""))
        lines.append(
            f"| {experiment} | {knee_label if knee_label is not None else 'none'} "
            f"| {knee_payload if knee_payload is not None else '—'} | {interpretation} |"
        )
        summary.append((experiment, knee_payload))
        conditions = knee.get("conditions")
        if isinstance(conditions, list) and conditions:
            lines.extend(["", f"### {experiment} — per-condition", "",
                          "| Condition | Payload | Degraded | Reasons |",
                          "| --- | ---: | :---: | --- |"])
            for cond in conditions:
                if not isinstance(cond, dict):
                    continue
                reasons = cond.get("reasons") or []
                reasons_text = "; ".join(str(r) for r in reasons) if reasons else "—"
                lines.append(
                    f"| {cond.get('label')} | {cond.get('payload')} "
                    f"| {'yes' if cond.get('degraded') else 'no'} | {reasons_text} |"
                )
            lines.append("")

    table_path = output_dir / "tables" / "map_load_knees.md"
    table_path.parent.mkdir(parents=True, exist_ok=True)
    if not knee_paths:
        table_path.write_text(
            "# Shared-map load limits — saturation knee\n\n"
            "No map-load sweep runs (E11/E12/E13) present yet.\n",
            encoding="utf-8",
        )
    else:
        table_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary


def write_markdown(path: Path, run_rows: list[dict[str, Any]], metric_rows: list[dict[str, Any]]) -> None:
    experiments = sorted({str(row.get("experiment", "")) for row in run_rows if row.get("experiment")})
    valid_runs = sum(1 for row in run_rows if row.get("measurement_valid"))
    envelope_runs = sum(1 for row in run_rows if row.get("within_envelope"))
    degraded_runs = sum(1 for row in run_rows if row.get("degraded"))
    lines = [
        "# HORUS Experiment Paper Report",
        "",
        "This report is generated from measured run folders. It does not invent missing values.",
        "",
        f"- Run folders scanned: {len(run_rows)}",
        f"- Measurement-valid runs: {valid_runs}",
        f"- Within-envelope runs: {envelope_runs}",
        f"- Degraded but valid runs: {degraded_runs}",
        f"- Experiments present: {', '.join(experiments) if experiments else 'none'}",
        "",
        "## Run Validity",
        "",
        "| Experiment | Runs | Measurement-valid | Within envelope | Degraded |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for experiment in experiments:
        rows = [row for row in run_rows if row.get("experiment") == experiment]
        lines.append(
            f"| {experiment} | {len(rows)} "
            f"| {sum(1 for row in rows if row.get('measurement_valid'))} "
            f"| {sum(1 for row in rows if row.get('within_envelope'))} "
            f"| {sum(1 for row in rows if row.get('degraded'))} |"
        )

    lines.extend(
        [
            "",
            "## Output Files",
            "",
            "- `paper_run_index.csv`: run metadata, config, and quality status.",
            "- `paper_metric_summary.csv`: per-run metric summaries grouped by file/category/name/stream/topic.",
            "- `tables/*.csv` and `tables/*.md`: paper-oriented tables for each experiment group.",
            "- `figures/*.svg`: dependency-free measured-data visual summaries for each experiment group.",
            "- `tables/map_load_knees.md` and `tables/knee_*.json`: computed saturation knee for the E11/E12/E13 map-load sweeps.",
            "- `paper_report.md`: this human-readable overview.",
            "",
            "Use only rows where `measurement_valid` is `True` for paper numbers unless a table explicitly states a diagnostic or source-only run. Treat `within_envelope` and `degraded` as the operating-envelope classification.",
            "",
            "## Expected Paper Tables",
            "",
            "- Baseline latency: filter `paper_metric_summary.csv` for `E0_baseline` and command/TF/registration metrics.",
            "- Camera/media: filter `E1_camera_transport` and `E10_camera_capacity`.",
            "- Sensor/map capacity: filter `E3_pointcloud_capacity`, `E4_mesh_vs_pointcloud_map`, and `E5_map_update_behavior`.",
            "- Control under load: filter `E6_control_under_sensor_load`.",
            "- Multi-robot/operator scaling: filter `E7_multi_robot_scaling` and `E8_multi_operator_scaling`.",
            "- Failure/recovery: filter `E9_failure_recovery` and the failure/orchestrator event files.",
            "- Shared-map load limits: filter `E11_map_pointcloud_sweep` and `E12_map_mesh_sweep`; the computed saturation knee is in `tables/map_load_knees.md` and `tables/knee_*.json`.",
            "",
            f"Metric groups summarized: {len(metric_rows)}",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="*", help="Optional explicit run folders. Defaults to results/*.")
    parser.add_argument("--results-root", default=str(SDK_ROOT / "results"))
    parser.add_argument("--output-dir", default=str(SDK_ROOT / "results" / "paper_report"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_dirs = collect_run_dirs(Path(args.results_root).resolve(), args.runs)
    output_dir = Path(args.output_dir).resolve()
    run_rows = run_index_rows(run_dirs)
    metric_rows = metric_summary_rows(run_dirs)

    write_csv(output_dir / "paper_run_index.csv", run_rows)
    write_csv(output_dir / "paper_metric_summary.csv", metric_rows)
    artifact_counts = write_paper_artifacts(output_dir, metric_rows)
    knee_summary = write_knee_artifacts(output_dir, Path(args.results_root).resolve())
    write_markdown(output_dir / "paper_report.md", run_rows, metric_rows)
    write_csv(
        output_dir / "paper_artifact_index.csv",
        [{"artifact": artifact, "matched_metric_rows": count} for artifact, count in artifact_counts],
    )

    if knee_summary:
        print(f"map-load knees computed: {len(knee_summary)} ({', '.join(e for e, _ in knee_summary)})")
    print(output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
