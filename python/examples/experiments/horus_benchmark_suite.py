#!/usr/bin/env python3
"""Create HORUS experiment run folders and source-side baseline metrics."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

SDK_ROOT = Path(__file__).resolve().parents[3]
PYTHON_ROOT = SDK_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from horus.experiments.analysis import write_summary
from horus.experiments.manifest import ExperimentManifest, RunIdentity, default_run_id
from horus.experiments.metrics import CsvMetricWriter, NdjsonEventWriter
from horus.experiments.synthetic_robot import iter_source_baseline_rows
from horus.experiments.workloads import load_workload_config


SOURCE_FIELDS = (
    "robot_id",
    "stream",
    "seq",
    "source_hz",
    "payload_bytes",
    "pose_x",
    "pose_y",
    "pose_z",
    "yaw",
    "points",
    "point_step",
    "resolution",
    "encoding",
    "bitrate_mbps",
    "representation",
    "chunks",
    "vertices",
    "triangles",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parent / "configs" / "e0_baseline.json"),
        help="JSON workload config.",
    )
    parser.add_argument(
        "--results-root",
        default=str(SDK_ROOT / "results"),
        help="Directory where benchmark run folders are written.",
    )
    parser.add_argument("--run-id", default="", help="Override generated run id.")
    parser.add_argument("--duration", type=float, default=None, help="Effective measured duration for this run.")
    parser.add_argument("--warmup", type=float, default=None, help="Effective warmup duration for this run.")
    parser.add_argument("--samples", type=int, default=0, help="Synthetic baseline samples to emit.")
    parser.add_argument("--notes", default="", help="Optional run notes.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config_path = Path(args.config).resolve()
    workload = load_workload_config(config_path)
    duration_s = workload.duration_s if args.duration is None else args.duration
    warmup_s = workload.warmup_s if args.warmup is None else args.warmup
    run_id = args.run_id or default_run_id(
        workload.experiment,
        workload.condition,
        workload.repetition,
    )
    run_dir = Path(args.results_root).resolve() / run_id

    identity = RunIdentity(
        run_id=run_id,
        experiment=workload.experiment,
        condition=workload.condition,
        repetition=workload.repetition,
    )
    manifest = ExperimentManifest.from_identity(
        identity,
        duration_s=duration_s,
        warmup_s=warmup_s,
        robot_count=workload.robot_count,
        operator_count=workload.operator_count,
        stream_count=workload.stream_count,
        transport=workload.transport,
        topology=workload.topology,
        resolution=workload.resolution,
        target_fps=workload.target_fps,
        robot_profile=workload.robot_profile,
        map_profile=workload.map_profile,
        notes=args.notes,
        repo_root=SDK_ROOT,
        extra={
            "workload": workload.to_dict(),
            "config_path": str(config_path),
            "configured_duration_s": workload.duration_s,
            "configured_warmup_s": workload.warmup_s,
            "effective_duration_s": duration_s,
            "effective_warmup_s": warmup_s,
            "total_source_duration_s": warmup_s + duration_s,
        },
    )
    manifest.save(run_dir)

    with NdjsonEventWriter(
        run_dir / "events.ndjson",
        run_id=run_id,
        experiment=workload.experiment,
        condition=workload.condition,
        source="horus_sdk",
    ) as events:
        events.write({"event": "run_created", "config": str(config_path)})

    with CsvMetricWriter(
        run_dir / "source_metrics.csv",
        run_id=run_id,
        experiment=workload.experiment,
        condition=workload.condition,
        fieldnames=SOURCE_FIELDS,
    ) as writer:
        for row in iter_source_baseline_rows(workload, samples=max(0, args.samples)):
            writer.write(row)

    # Create empty files for the other layers so all run folders have the same shape.
    for name in ("bridge_metrics.csv", "headset_metrics.csv", "webrtc_metrics.csv", "command_metrics.csv"):
        path = run_dir / name
        if not path.exists():
            path.write_text("timestamp_ns,run_id,experiment,condition\n", encoding="utf-8")

    write_summary(run_dir)
    print(run_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
