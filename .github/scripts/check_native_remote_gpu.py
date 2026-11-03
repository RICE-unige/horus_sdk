#!/usr/bin/env python3
"""Qualify native point/mesh bindings against Python on a real CUDA device."""
import argparse
import json
from pathlib import Path
import subprocess
from unittest.mock import patch

import numpy as np
from horus.remote_rendering.cuda_renderer import CudaPointRenderer
from horus.remote_rendering.cuda_mesh_renderer import CudaMeshRenderer


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--point-library", type=Path, required=True)
    p.add_argument("--mesh-library", type=Path, required=True)
    p.add_argument("binaries", nargs="+")
    args = p.parse_args()
    points = np.array([[-.5, 0, 2], [0, 0, 2], [.5, 0, 2]], dtype=np.float32)
    colors = np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255]], dtype=np.uint8)
    vertices = np.array([[-1, -1, 2], [1, -1, 2], [1, 1, 2], [-1, 1, 2]], dtype=np.float32)
    faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)
    tints = np.array([[255, 80, 20], [20, 80, 255]], dtype=np.uint8)
    arguments = dict(camera_positions=[[-.1, 0, 0], [.1, 0, 0]],
                     camera_rotations=[[0, 0, 0, 1]] * 2, width=32, height=24,
                     vertical_fov_deg=90, near_m=.1, far_m=10, point_radius=1,
                     projections=[[1, 1, 0, 0]] * 2)
    with patch("horus.remote_rendering.cuda_renderer.build_cuda_renderer_library", return_value=args.point_library):
        with CudaPointRenderer(len(points)) as renderer:
            renderer.upload(points, colors)
            point = renderer.render_views(**arguments)
    with patch("horus.remote_rendering.cuda_mesh_renderer.build_cuda_mesh_renderer_library", return_value=args.mesh_library):
        with CudaMeshRenderer(vertices, faces, tints) as renderer:
            mesh = renderer.render_views(**arguments)
    for executable in args.binaries:
        completed = subprocess.run([executable, str(args.point_library), str(args.mesh_library)],
                                   text=True, capture_output=True, timeout=30, check=True)
        result = json.loads(completed.stdout)
        for key, reference in (("points", point), ("mesh", mesh)):
            rgb, depth, _ = reference
            actual_rgb = np.asarray(result[key]["rgb"], dtype=np.uint8).reshape(rgb.shape)
            actual_depth = np.asarray([np.inf if x is None else x for x in result[key]["depth"]],
                                      dtype=np.float32).reshape(depth.shape)
            assert np.array_equal(actual_rgb, rgb), f"{executable}: {key} RGB differs"
            assert np.array_equal(actual_depth, depth), f"{executable}: {key} depth differs"
            assert np.count_nonzero(np.isfinite(depth)) > 10, "Blank GPU result"
            assert np.isfinite(result[key]["cuda_ms"])
        print(json.dumps({"binary": executable, "result": "passed", "point_rgb_depth": "exact",
                          "mesh_rgb_depth": "exact", "device": "CUDA"}))


if __name__ == "__main__":
    main()
