from __future__ import annotations

import importlib.util
import struct
import sys
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "examples"
    / "tools"
    / "ply_to_horus_mesh_marker.py"
)
SPEC = importlib.util.spec_from_file_location("_ply_mesh_transport", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_indexed_binary_payload_deduplicates_shared_vertices() -> None:
    points = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
    ]
    colors = [(0.25, 0.5, 0.75, 1.0)] * len(points)

    payload = MODULE.build_indexed_binary_payload(7, points, colors)
    magic, version, chunk_id, vertex_count, index_count, has_color = struct.unpack_from(
        "<IIIIIB3x", payload, 0
    )

    assert magic == 0x484D5348
    assert version == 1
    assert chunk_id == 7
    assert vertex_count == 4
    assert index_count == 6
    assert has_color == 1
    assert len(payload) == 24 + vertex_count * 16 + index_count * 4


def test_spatial_order_preserves_geometry_and_color_pairing() -> None:
    points = [
        (10.0, 0.0, 0.0),
        (11.0, 0.0, 0.0),
        (10.0, 1.0, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
    ]
    colors = [(1.0, 0.0, 0.0, 1.0)] * 3 + [(0.0, 1.0, 0.0, 1.0)] * 3
    geometry = MODULE.MeshGeometry(
        points=points,
        colors=colors,
        triangle_count=2,
        source_vertex_count=6,
    )

    ordered = MODULE.spatially_order_geometry(geometry)

    assert ordered.triangle_count == geometry.triangle_count
    assert sorted(zip(ordered.points, ordered.colors)) == sorted(zip(points, colors))
