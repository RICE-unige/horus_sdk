"""Metrics writers used by HORUS benchmark sources and analysis tools."""

from __future__ import annotations

import csv
from dataclasses import asdict, is_dataclass
import json
from pathlib import Path
import time
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence


COMMON_FIELDS = ("timestamp_ns", "run_id", "experiment", "condition")


def now_ns() -> int:
    return time.time_ns()


def monotonic_ns() -> int:
    return time.monotonic_ns()


def normalize_row(row: Any) -> Dict[str, Any]:
    if is_dataclass(row):
        row = asdict(row)
    if isinstance(row, Mapping):
        return dict(row)
    raise TypeError(f"Metric rows must be mappings or dataclasses, got {type(row)!r}")


class CsvMetricWriter:
    """Append metrics to a CSV with stable common fields.

    The writer intentionally accepts dictionaries so benchmark scripts can add
    experiment-specific columns without changing the core package.
    """

    def __init__(
        self,
        path: Path,
        *,
        run_id: str,
        experiment: str,
        condition: str,
        fieldnames: Sequence[str],
    ) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.run_id = run_id
        self.experiment = experiment
        self.condition = condition
        self.fieldnames = self._merge_fields(fieldnames)
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.fieldnames, extrasaction="ignore")
        self._writer.writeheader()

    def _merge_fields(self, fieldnames: Sequence[str]) -> Sequence[str]:
        fields = list(COMMON_FIELDS)
        for field in fieldnames:
            if field not in fields:
                fields.append(field)
        return fields

    def write(self, row: Any, *, timestamp_ns: Optional[int] = None) -> None:
        data = normalize_row(row)
        data.setdefault("timestamp_ns", timestamp_ns if timestamp_ns is not None else now_ns())
        data.setdefault("run_id", self.run_id)
        data.setdefault("experiment", self.experiment)
        data.setdefault("condition", self.condition)
        self._writer.writerow(data)

    def writerows(self, rows: Iterable[Any]) -> None:
        for row in rows:
            self.write(row)

    def flush(self) -> None:
        self._file.flush()

    def close(self) -> None:
        self._file.flush()
        self._file.close()

    def __enter__(self) -> "CsvMetricWriter":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


class NdjsonEventWriter:
    """Write event records as newline-delimited JSON."""

    def __init__(
        self,
        path: Path,
        *,
        run_id: str,
        experiment: str,
        condition: str,
        source: str,
    ) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.run_id = run_id
        self.experiment = experiment
        self.condition = condition
        self.source = source
        self._file = self.path.open("w", encoding="utf-8")

    def write(self, event: Mapping[str, Any], *, timestamp_ns: Optional[int] = None) -> None:
        payload: Dict[str, Any] = dict(event)
        payload.setdefault("timestamp_ns", timestamp_ns if timestamp_ns is not None else now_ns())
        payload.setdefault("run_id", self.run_id)
        payload.setdefault("experiment", self.experiment)
        payload.setdefault("condition", self.condition)
        payload.setdefault("source", self.source)
        self._file.write(json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n")

    def flush(self) -> None:
        self._file.flush()

    def close(self) -> None:
        self._file.flush()
        self._file.close()

    def __enter__(self) -> "NdjsonEventWriter":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
