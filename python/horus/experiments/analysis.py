"""Small summary helpers for HORUS experiment output directories."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import median
from typing import Dict, Iterable, List


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


def summarize_numeric_csv(path: Path) -> Dict[str, object]:
    rows: List[Dict[str, str]] = []
    with Path(path).open(newline="", encoding="utf-8") as handle:
        rows.extend(csv.DictReader(handle))

    summary: Dict[str, object] = {"row_count": len(rows)}
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


def write_summary(run_dir: Path) -> Path:
    run_dir = Path(run_dir)
    output: Dict[str, object] = {}
    for csv_path in sorted(run_dir.glob("*_metrics.csv")):
        output[csv_path.name] = summarize_numeric_csv(csv_path)
    summary_path = run_dir / "summary.json"
    summary_path.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary_path
