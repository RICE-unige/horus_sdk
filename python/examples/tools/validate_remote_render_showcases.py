#!/usr/bin/env python3
"""Render deterministic CUDA previews for every ETH3D HORUS showcase."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from horus.remote_rendering import (
    CudaPointRenderer,
    DynamicCameraPose,
    ETH3D_REMOTE_SCENE_IDS,
    iter_eth3d_remote_scene_lod_chunks,
    iter_eth3d_remote_scene_chunks,
    prepare_eth3d_remote_scene,
    prepare_eth3d_remote_scene_lod,
    projection_from_vertical_fov,
)


def _initial_pose(scene) -> DynamicCameraPose:
    half_pitch = np.deg2rad(scene.initial_pitch_degrees) * 0.5
    return DynamicCameraPose(
        scene.initial_position,
        (float(np.sin(half_pitch)), 0.0, 0.0, float(np.cos(half_pitch))),
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path.home() / ".cache" / "horus" / "remote_map_previews",
    )
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--point-radius", type=int, default=2)
    parser.add_argument("--source-voxel-size", type=float, default=0.0125)
    parser.add_argument("--full-source", action="store_true")
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    projection = projection_from_vertical_fov(96.0, args.width / args.height)

    for scene_id in ETH3D_REMOTE_SCENE_IDS:
        scene = prepare_eth3d_remote_scene(scene_id)
        if args.full_source:
            point_count = scene.point_count
            chunks = iter_eth3d_remote_scene_chunks(scene)
        else:
            lod_path, point_count = prepare_eth3d_remote_scene_lod(
                scene,
                voxel_size_m=args.source_voxel_size,
            )
            chunks = iter_eth3d_remote_scene_lod_chunks(lod_path)
        pose = _initial_pose(scene)
        print(
            f"[load] {scene_id}: {point_count:,} render points "
            f"({scene.point_count:,} source) from {scene.root}",
            flush=True,
        )
        with CudaPointRenderer(point_count) as renderer:
            offset = 0
            for points, colors in chunks:
                renderer.upload(points, colors, offset=offset)
                offset += len(points)
            color, depth, elapsed_ms = renderer.render_views(
                (pose.position,),
                (pose.rotation,),
                args.width,
                args.height,
                vertical_fov_deg=96.0,
                projections=(projection,),
                near_m=scene.depth_near_m,
                far_m=scene.depth_far_m,
                point_radius=args.point_radius,
            )
        valid_fraction = float(np.isfinite(depth[0]).mean())
        path = output / f"{scene_id}.png"
        cv2.imwrite(str(path), cv2.cvtColor(color[0], cv2.COLOR_RGB2BGR))
        print(
            f"[ok] {scene_id}: valid={valid_fraction:.3f} "
            f"cuda_ms={elapsed_ms:.2f} preview={path}",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
