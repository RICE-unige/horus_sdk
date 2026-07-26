#!/usr/bin/env python3
"""Create a cached triangle surface for stable Cow and Lady remote rendering."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import venv


DEFAULT_SOURCE = (
    Path.home()
    / ".cache"
    / "horus"
    / "voxblox_cow_lady"
    / "extras"
    / "cow_and_lady_gt.ply"
)
WORKER_ENV = Path.home() / ".cache" / "horus" / "remote_meshing_venv"
OPEN3D_VERSION = "0.19.0"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Reconstruct the Cow and Lady point fixture as a colored triangle "
            "surface for temporally stable remote rendering."
        )
    )
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--voxel-size", type=float, default=0.02)
    parser.add_argument("--normal-radius", type=float, default=0.08)
    parser.add_argument("--normal-neighbors", type=int, default=50)
    parser.add_argument("--orientation-neighbors", type=int, default=40)
    parser.add_argument("--poisson-depth", type=int, default=9)
    parser.add_argument("--density-quantile", type=float, default=0.015)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.output is None:
        args.output = args.source.with_name("cow_and_lady_surface.npz")
    if args.voxel_size <= 0.0 or args.normal_radius <= 0.0:
        parser.error("voxel and normal radii must be positive")
    if args.normal_neighbors < 3 or args.orientation_neighbors < 3:
        parser.error("normal neighbor counts must be at least three")
    if not 6 <= args.poisson_depth <= 12:
        parser.error("--poisson-depth must be between 6 and 12")
    if not 0.0 <= args.density_quantile < 0.25:
        parser.error("--density-quantile must be in [0, 0.25)")
    return args


def ensure_worker_environment(args: argparse.Namespace) -> None:
    if args.worker:
        return
    python = WORKER_ENV / "bin" / "python"
    if not python.is_file():
        print(f"[surface] creating isolated Open3D environment at {WORKER_ENV}")
        venv.EnvBuilder(with_pip=True).create(WORKER_ENV)
    probe = subprocess.run(
        [str(python), "-c", "import open3d"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if probe.returncode != 0:
        print(f"[surface] installing Open3D {OPEN3D_VERSION} (one-time download)")
        subprocess.run(
            [
                str(python),
                "-m",
                "pip",
                "install",
                f"open3d=={OPEN3D_VERSION}",
            ],
            check=True,
        )
    command = [str(python), str(Path(__file__).resolve()), "--worker"]
    for name in (
        "source",
        "output",
        "voxel_size",
        "normal_radius",
        "normal_neighbors",
        "orientation_neighbors",
        "poisson_depth",
        "density_quantile",
    ):
        command.extend((f"--{name.replace('_', '-')}", str(getattr(args, name))))
    if args.force:
        command.append("--force")
    os.execv(str(python), command)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def reconstruct(args: argparse.Namespace) -> None:
    import numpy as np
    import open3d as o3d

    source = args.source.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(
            f"Cow and Lady point fixture is missing: {source}. "
            "Run fetch_voxblox_cow_lady.py first."
        )
    if output.is_file() and not args.force:
        print(f"[surface] cached mesh already exists: {output}")
        return

    print(f"[surface] loading {source}")
    cloud = o3d.io.read_point_cloud(str(source))
    if len(cloud.points) == 0 or len(cloud.colors) != len(cloud.points):
        raise RuntimeError("source PLY must contain colored points")
    cloud = cloud.voxel_down_sample(args.voxel_size)
    print(f"[surface] estimating normals for {len(cloud.points):,} points")
    cloud.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=args.normal_radius,
            max_nn=args.normal_neighbors,
        )
    )
    cloud.orient_normals_consistent_tangent_plane(args.orientation_neighbors)

    print(f"[surface] reconstructing Poisson surface at depth {args.poisson_depth}")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        cloud,
        depth=args.poisson_depth,
        scale=1.05,
        linear_fit=False,
        n_threads=-1,
    )
    densities = np.asarray(densities)
    if args.density_quantile > 0.0:
        threshold = float(np.quantile(densities, args.density_quantile))
        mesh.remove_vertices_by_mask(densities < threshold)
    mesh = mesh.crop(cloud.get_axis_aligned_bounding_box())
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()

    vertices = np.asarray(mesh.vertices, dtype=np.float32)
    faces = np.asarray(mesh.triangles, dtype=np.uint32)
    vertex_colors = np.asarray(mesh.vertex_colors, dtype=np.float32)
    if len(vertices) == 0 or len(faces) == 0:
        raise RuntimeError("surface reconstruction produced an empty mesh")
    if len(vertex_colors) != len(vertices):
        raise RuntimeError("surface reconstruction did not preserve vertex colors")
    face_colors = np.ascontiguousarray(
        np.clip(vertex_colors[faces].mean(axis=1) * 255.0, 0.0, 255.0),
        dtype=np.uint8,
    )
    metadata = json.dumps(
        {
            "format": "horus.remote_surface.v1",
            "source": str(source),
            "source_sha256": file_sha256(source),
            "open3d_version": o3d.__version__,
            "voxel_size": args.voxel_size,
            "normal_radius": args.normal_radius,
            "normal_neighbors": args.normal_neighbors,
            "orientation_neighbors": args.orientation_neighbors,
            "poisson_depth": args.poisson_depth,
            "density_quantile": args.density_quantile,
        },
        sort_keys=True,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".part")
    with temporary.open("wb") as target:
        np.savez_compressed(
            target,
            vertices=vertices,
            faces=faces,
            face_colors=face_colors,
            metadata=np.asarray(metadata),
        )
    temporary.replace(output)
    print(
        f"[surface] saved {len(vertices):,} vertices and {len(faces):,} "
        f"triangles to {output}"
    )


def main() -> int:
    args = parse_args()
    ensure_worker_environment(args)
    reconstruct(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
