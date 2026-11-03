#!/usr/bin/env python3
"""Run the native Quest nvblox mesh pipeline and register it with HORUS.

This is the experiment entry point. It starts the full-geometry nvblox block adapter,
registers a first-class workspace mesh (not a fake robot camera), and keeps both
processes alive until Ctrl+C.
"""

from __future__ import annotations

import argparse
import atexit
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import math

from horus.robot import WorkspaceVisualization, is_registration_cancelled, register_entities


ADAPTER = Path(__file__).resolve().parent / "tools" / "nvblox_mesh_to_horus.py"


def parse_float(value: str) -> float:
    return float(str(value).strip().replace(",", "."))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default="/nvblox_node/mesh")
    parser.add_argument("--output-topic", default="/horus/nvblox/mesh")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--workspace-scale", type=parse_float, default=0.1)
    parser.add_argument("--blocks-per-axis", type=int, default=4)
    parser.add_argument("--max-pages", type=int, default=1024)
    parser.add_argument("--max-triangles", type=int, default=2_000_000)
    parser.add_argument("--max-publish-pages-per-second", type=float, default=90.0)
    parser.add_argument("--max-publish-mib-per-second", type=float, default=8.0)
    parser.add_argument("--input-reliability", choices=("auto", "reliable", "best_effort"), default="auto")
    parser.add_argument("--max-upload-ms-per-frame", type=float, default=2.5)
    parser.add_argument("--color-mode", choices=("auto", "source", "normals", "fallback"), default="auto")
    parser.add_argument("--no-launch-adapter", action="store_true")
    args = parser.parse_args()
    if not 0.001 <= args.workspace_scale <= 10.0:
        parser.error("--workspace-scale must be between 0.001 and 10")
    if not 1 <= args.blocks_per_axis <= 8:
        parser.error("--blocks-per-axis must be between 1 and 8")
    if not 8 <= args.max_pages <= 1024:
        parser.error("--max-pages must be between 8 and 1024")
    if not 0.25 <= args.max_upload_ms_per_frame <= 8.0:
        parser.error("--max-upload-ms-per-frame must be between 0.25 and 8")
    if not math.isfinite(args.max_publish_mib_per_second) or args.max_publish_mib_per_second <= 0:
        parser.error("--max-publish-mib-per-second must be finite and > 0")
    return args


def start_adapter(args: argparse.Namespace) -> subprocess.Popen[str]:
    command = [
        sys.executable,
        str(ADAPTER),
        "--input-topic",
        args.input_topic,
        "--output-topic",
        args.output_topic,
        "--blocks-per-axis",
        str(args.blocks_per_axis),
        "--max-pages",
        str(args.max_pages),
        "--max-publish-pages-per-second",
        str(args.max_publish_pages_per_second),
        "--max-publish-mib-per-second",
        str(args.max_publish_mib_per_second),
        "--input-reliability",
        args.input_reliability,
        "--color-mode",
        args.color_mode,
    ]
    process = subprocess.Popen(command, env=os.environ.copy(), text=True)
    time.sleep(1.0)
    if process.poll() is not None:
        raise RuntimeError(f"nvblox adapter exited with code {process.returncode}")
    return process


def stop_process(process: subprocess.Popen[str] | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()


def main() -> int:
    args = parse_args()
    adapter: subprocess.Popen[str] | None = None
    if not args.no_launch_adapter:
        adapter = start_adapter(args)
        atexit.register(stop_process, adapter)

    resource = WorkspaceVisualization("nvblox_map")
    dataviz = resource.create_dataviz("nvblox_map")
    dataviz.add_nvblox_mesh(
        topic=args.output_topic,
        frame_id=args.frame_id,
        render_options={
            "max_triangles": int(args.max_triangles),
            "max_chunk_renderers": int(args.max_pages),
            "max_upload_milliseconds_per_frame": float(args.max_upload_ms_per_frame),
        },
    )

    try:
        success, result = register_entities(
            [resource],
            datavizs=[dataviz],
            workspace_scale=float(args.workspace_scale),
            compass_enabled=False,
            keep_alive=True,
        )
    finally:
        stop_process(adapter)

    if success:
        return 0
    if is_registration_cancelled(result):
        print("HORUS registration monitor stopped.")
        return 0
    raise SystemExit(f"HORUS nvblox registration failed: {result}")


if __name__ == "__main__":
    raise SystemExit(main())
