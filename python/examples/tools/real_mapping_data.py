"""Validated, provenance-labelled map exports used by the real-data demos."""

import json
from pathlib import Path

import numpy as np


DEFAULT_ROOT = Path.home() / ".cache/horus/real_mapping"


def record(dataset, kind, root=DEFAULT_ROOT):
    manifest = root / "manifest.json"
    if not manifest.is_file():
        raise RuntimeError(f"Real mapping data is not prepared: {manifest}. See MAPPING_VISUALIZATION_TESTING.md.")
    data = json.loads(manifest.read_text())
    if data.get("schema") != "horus.real_mapping.v1":
        raise ValueError("Unsupported real mapping manifest")
    try:
        result = data["datasets"][dataset][kind]
    except KeyError as exc:
        raise ValueError(f"No {kind} data for {dataset}; no synthetic fallback") from exc
    path = Path(result["path"]).expanduser()
    if not path.exists():
        raise FileNotFoundError(path)
    return result


def arrays(path, mesh=False):
    with np.load(path, allow_pickle=False) as data:
        points = np.ascontiguousarray(data["vertices" if mesh else "points"], dtype=np.float32)
        raw_colors = data["colors"]
        if raw_colors.dtype != np.uint8:
            raise ValueError("Source RGB must be uint8; implicit color rescaling is not allowed")
        colors = np.ascontiguousarray(raw_colors)
        faces = data["faces"] if mesh else None
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError("Real map has empty/invalid coordinates")
    if colors.shape != points.shape:
        raise ValueError("Real map requires one source RGB color per vertex")
    if mesh and (not np.issubdtype(faces.dtype, np.integer)
                 or faces.ndim != 2 or faces.shape[1] != 3 or not len(faces)
                 or faces.min() < 0 or faces.max() >= len(points)):
        raise ValueError("Real map has invalid triangle indices")
    if mesh:
        if faces.max() > np.iinfo(np.int32).max:
            raise ValueError("Real map exceeds supported index range")
        faces = np.ascontiguousarray(faces, dtype=np.int32)
    return points, colors, faces


def octomap_file(path):
    """Strip only the official OctoMap file header, preserving the tree bytes."""
    with Path(path).open("rb") as source:
        first = source.readline().strip()
        if first not in (b"# Octomap OcTree file", b"# Octomap OcTree binary file"):
            raise ValueError("Not an OctoMap tree file")
        metadata = {}
        for _ in range(100):
            line = source.readline().strip()
            if line == b"data":
                break
            if line and not line.startswith(b"#"):
                key, value = line.decode("ascii").split(maxsplit=1)
                metadata[key] = value
        else:
            raise ValueError("OctoMap header is incomplete")
        payload = source.read()
    resolution = float(metadata["res"])
    if not payload or not np.isfinite(resolution) or resolution <= 0:
        raise ValueError("OctoMap tree is empty or has invalid resolution")
    return metadata["id"], resolution, first.endswith(b"binary file"), payload
