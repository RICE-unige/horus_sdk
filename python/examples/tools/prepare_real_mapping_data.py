#!/usr/bin/env python3
"""Prepare separate full-detail replay exports; never edit original robot data."""

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import urllib.request

import numpy as np
from plyfile import PlyData

if __package__:
    from .real_mapping_data import DEFAULT_ROOT, octomap_file
else:
    from real_mapping_data import DEFAULT_ROOT, octomap_file


OCTOMAP_URL = "http://ais.informatik.uni-freiburg.de/projects/datasets/octomap/freiburg1_360.ot.gz"


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def export_mesh(path, output, provenance):
    ply = PlyData.read(path)
    vertices = np.column_stack([ply["vertex"][key] for key in ("x", "y", "z")]).astype(np.float32)
    colors = np.column_stack([ply["vertex"][key] for key in ("red", "green", "blue")]).astype(np.uint8)
    triangles = ply["face"]["vertex_indices"]
    if any(len(face) != 3 for face in triangles):
        raise ValueError("Only original triangular surfaces are accepted; no implicit triangulation")
    faces = np.stack(triangles).astype(np.int32)
    np.savez(output, vertices=vertices, colors=colors, faces=faces)
    return dict(path=str(output), source=str(path), source_sha256=digest(path), sha256=digest(output),
                provenance=provenance, vertices=len(vertices), triangles=len(faces),
                frame="map", conversion="PLY to float32/indexed arrays; no triangle removal or color replacement")


def fetch_octomap(root):
    archive, output = root / "freiburg1_360.ot.gz", root / "freiburg1_360.ot"
    if not archive.exists():
        partial = archive.with_suffix(".partial")
        with urllib.request.urlopen(OCTOMAP_URL, timeout=30) as response, partial.open("wb") as target:
            size = 0
            while chunk := response.read(1024 * 1024):
                size += len(chunk)
                if size > 32 * 1024**2:
                    raise ValueError("Unexpected OctoMap download size")
                target.write(chunk)
        partial.replace(archive)
    if not output.exists():
        partial = output.with_suffix(".partial")
        with gzip.open(archive, "rb") as source, partial.open("wb") as target:
            size = 0
            while chunk := source.read(1024 * 1024):
                size += len(chunk)
                if size > 256 * 1024**2:
                    raise ValueError("Unexpected expanded OctoMap size")
                target.write(chunk)
        octomap_file(partial)
        partial.replace(output)
    tree, resolution, binary, _ = octomap_file(output)
    return dict(path=str(output), source_url=OCTOMAP_URL, archive_sha256=digest(archive), sha256=digest(output),
                provenance="University of Freiburg RGBD freiburg1_360: physical Kinect office capture aligned by RGBDSLAM",
                tree=tree, resolution=resolution, binary=binary, frame="map",
                conversion="Original published ColorOcTree; unchanged resolution and tree bytes",
                license="Check upstream dataset terms before redistribution; local evaluation copy only",
                transport_note="Upstream dataset server serves HTTP only; digest records this download, not upstream authentication")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--g1-session", required=True, type=Path)
    parser.add_argument("--spot-mesh", required=True, type=Path)
    parser.add_argument("--output", default=DEFAULT_ROOT, type=Path)
    args = parser.parse_args()
    root = args.output.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    if (root / "manifest.json").exists():
        raise ValueError("Prepared manifest already exists; use a new --output to preserve provenance")
    if shutil.disk_usage(root).free < 1024**3:
        raise RuntimeError("At least 1 GiB free space required")
    session = args.g1_session.expanduser().resolve()
    report = json.loads((session / "report.json").read_text())
    if report.get("status") != "complete" or report.get("input_audit", {}).get("errors"):
        raise ValueError("G1 session is not a complete, audited replay")
    g1_mesh = export_mesh(session / "mapping/mesh.ply", root / "g1-mesh.npz",
                         "nvblox surface reconstructed from the physical G1 2026-10-05 walk; nominal unqualified extrinsics")
    g1_mesh["source_frame"] = "g1_spatial/map"
    with np.load(session / "pointcloud.npz", allow_pickle=False) as cloud:
        np.savez(root / "g1-points.npz", points=cloud["points"], colors=cloud["colors"])
        count = len(cloud["points"])
    point_record = dict(path=str(root / "g1-points.npz"), source=str(session / "pointcloud.npz"),
        source_sha256=digest(session / "pointcloud.npz"), sha256=digest(root / "g1-points.npz"), points=count,
        provenance="Registered G1 LiDAR scan aggregate with measured RGB where observed; neutral elsewhere",
        conversion="All points in existing 1 cm aggregate retained; no new sampling", frame="map",
        source_frame="g1_spatial/map")
    bag = dict(path=str(session / "outputs"), source=str(session / "report.json"),
        provenance="Recorded nvblox/registered-scan outputs of physical G1 bag replay; not a live robot",
        mesh_topic="/g1_spatial_nvblox/mesh", points_topic="/g1_spatial/map_scan", frame="g1_spatial/map")
    spot_mesh = export_mesh(args.spot_mesh.resolve(), root / "spot-mesh.npz",
                           "nvblox reconstruction exported by the physical Spot observation pipeline; source RGB may be neutral")
    spot_mesh["source_frame"] = "spot/vision"
    cow = Path.home() / ".cache/horus/voxblox_cow_lady/extras/cow_and_lady_gt.ply"
    ply = PlyData.read(cow)
    xyz = np.column_stack([ply["vertex"][key] for key in ("x", "y", "z")]).astype(np.float32)
    rgb = np.column_stack([ply["vertex"][key] for key in ("red", "green", "blue")]).astype(np.uint8)
    np.savez(root / "cow-lady-points.npz", points=xyz, colors=rgb)
    cow_record = dict(path=str(root / "cow-lady-points.npz"), source=str(cow), source_sha256=digest(cow),
        sha256=digest(root / "cow-lady-points.npz"), points=len(xyz), frame="map",
        provenance="ETH Zurich Cow and Lady physical Leica laser scan, CC BY 4.0",
        source_url="https://doi.org/10.3929/ethz-b-000721636", conversion="All original scan vertices and colors")
    manifest = dict(schema="horus.real_mapping.v1", datasets={
        "g1": dict(mesh=g1_mesh, points=point_record, bag=bag),
        "spot": dict(mesh=spot_mesh), "cow-lady": dict(points=cow_record),
        "freiburg": dict(octomap=fetch_octomap(root))})
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
