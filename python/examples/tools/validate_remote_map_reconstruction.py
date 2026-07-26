#!/usr/bin/env python3
"""Render offline views of the exact surfel reconstruction used by HORUS MR."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from horus.remote_rendering import (
    COW_LADY_DEPTH_FAR_BASE,
    COW_LADY_DEPTH_NEAR_BASE,
    COW_LADY_VIEW_SPECS,
    ETH3D_COURTYARD_DEPTH_FAR_BASE,
    ETH3D_COURTYARD_DEPTH_NEAR_BASE,
    ETH3D_COURTYARD_VIEW_SPECS,
    build_cow_lady_rgbd,
    build_eth3d_courtyard_rgbd,
    render_colored_points,
    unproject_multiview_surfels,
)


def render_validation_sheet(
    points: np.ndarray,
    colors: np.ndarray,
    cameras: tuple[tuple[tuple[float, float, float], float, float], ...],
    output: Path,
    near_m: float,
    far_m: float,
) -> None:
    frames = []
    for position, yaw, pitch in cameras:
        color, _ = render_colored_points(
            points,
            colors,
            640,
            360,
            point_radius=2,
            near_m=near_m,
            far_m=far_m,
            camera_position=np.asarray(position, dtype=np.float32),
            camera_yaw_deg=yaw,
            camera_pitch_deg=pitch,
        )
        frames.append(color)
    sheet = np.concatenate(frames, axis=1)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), cv2.cvtColor(sheet, cv2.COLOR_RGB2BGR))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=("cow_lady", "eth3d_courtyard"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--width", type=int, default=1440)
    parser.add_argument("--height", type=int, default=810)
    parser.add_argument("--columns", type=int, default=240)
    parser.add_argument("--rows", type=int, default=135)
    parser.add_argument("--dataset-path", default="")
    args = parser.parse_args()

    if args.scene == "cow_lady":
        color, depth, _ = build_cow_lady_rgbd(
            args.width, args.height, ply_path=args.dataset_path or None
        )
        views = COW_LADY_VIEW_SPECS
        near_m, far_m = COW_LADY_DEPTH_NEAR_BASE, COW_LADY_DEPTH_FAR_BASE
        cameras = (
            ((-5.5, 3.0, -5.5), 45.0, 8.0),
            ((5.5, 3.0, -5.5), 315.0, 8.0),
            ((0.0, 6.5, -0.5), 0.0, 72.0),
        )
    else:
        color, depth, _ = build_eth3d_courtyard_rgbd(
            args.width, args.height, dataset_root=args.dataset_path or None
        )
        views = ETH3D_COURTYARD_VIEW_SPECS
        near_m, far_m = ETH3D_COURTYARD_DEPTH_NEAR_BASE, ETH3D_COURTYARD_DEPTH_FAR_BASE
        cameras = (
            ((-21.0, 12.0, -21.0), 42.0, 14.0),
            ((21.0, 12.0, -21.0), 318.0, 14.0),
            ((0.0, 22.0, 0.0), 0.0, 70.0),
        )

    points, colors = unproject_multiview_surfels(
        color,
        depth,
        views,
        columns=args.columns,
        rows=args.rows,
    )
    render_validation_sheet(points, colors, cameras, args.output, near_m, far_m * 2.0)
    print(
        f"[validation] scene={args.scene} surfels={len(points):,} "
        f"valid_atlas={float(np.isfinite(depth).mean()):.3f} output={args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
