#!/usr/bin/env python3
"""Run multi-condition HORUS experiment sweeps."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys


EXPERIMENT_DIR = Path(__file__).resolve().parent
ORCHESTRATOR = EXPERIMENT_DIR / "horus_experiment_orchestrator.py"

SUITES = {
    "camera_capacity": [
        "e10_camera_capacity_ros_01stream",
        "e10_camera_capacity_ros_02stream",
        "e10_camera_capacity_ros_04stream",
        "e10_camera_capacity_ros_06stream",
        "e10_camera_capacity_ros_08stream",
    ],
    "pointcloud_map_capacity": [
        "e11_map_pointcloud_sweep_0100k",
        "e11_map_pointcloud_sweep_0250k",
        "e11_map_pointcloud_sweep_0500k",
        "e11_map_pointcloud_sweep_1000k",
    ],
    "mesh_map_capacity": [
        "e12_map_mesh_sweep_024k",
        "e12_map_mesh_sweep_096k",
        "e12_map_mesh_sweep_192k",
        "e12_map_mesh_sweep_384k",
    ],
    "map_capacity": [
        "e11_map_pointcloud_sweep_0100k",
        "e11_map_pointcloud_sweep_0250k",
        "e11_map_pointcloud_sweep_0500k",
        "e11_map_pointcloud_sweep_1000k",
        "e12_map_mesh_sweep_024k",
        "e12_map_mesh_sweep_096k",
        "e12_map_mesh_sweep_192k",
        "e12_map_mesh_sweep_384k",
    ],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a named HORUS experiment sweep. Arguments after the suite name "
            "are passed through to every condition run."
        )
    )
    parser.add_argument("suite", choices=sorted(SUITES))
    parser.add_argument("orchestrator_args", nargs=argparse.REMAINDER)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    conditions = SUITES[args.suite]
    for index, condition in enumerate(conditions, start=1):
        print(flush=True)
        print(f"[sweep] {args.suite}: condition {index}/{len(conditions)}: {condition}", flush=True)
        command = [sys.executable, str(ORCHESTRATOR), condition, *args.orchestrator_args]
        completed = subprocess.run(command, text=True, check=False)
        if completed.returncode != 0:
            print(f"[sweep] stopping after {condition} failed with {completed.returncode}", file=sys.stderr)
            return int(completed.returncode)
    print(flush=True)
    print(f"[sweep] {args.suite}: complete", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
