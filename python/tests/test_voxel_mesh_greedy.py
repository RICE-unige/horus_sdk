"""Tests for greedy voxel surface mesh generation helpers."""

import numpy as np

from horus.utils.voxel_mesh import build_greedy_surface_mesh


def _naive_triangle_count(voxels: np.ndarray) -> int:
    occ = {tuple(v.tolist()) for v in voxels}
    tri = 0
    for x, y, z in occ:
        for dx, dy, dz in (
            (1, 0, 0),
            (-1, 0, 0),
            (0, 1, 0),
            (0, -1, 0),
            (0, 0, 1),
            (0, 0, -1),
        ):
            if (x + dx, y + dy, z + dz) not in occ:
                tri += 2
    return tri


def test_greedy_surface_mesh_returns_non_empty_for_simple_block():
    xs, ys, zs = np.mgrid[0:2, 0:2, 0:1]
    voxels = np.stack((xs.ravel(), ys.ravel(), zs.ravel()), axis=1).astype(np.int32)

    result = build_greedy_surface_mesh(
        voxels=voxels,
        voxel_size=0.1,
        voxel_colors=None,
        max_triangles=0,
    )

    assert result.triangle_count > 0
    assert len(result.vertices) == result.triangle_count * 3


def test_greedy_surface_mesh_is_deterministic():
    xs, ys, zs = np.mgrid[0:4, 0:3, 0:2]
    voxels = np.stack((xs.ravel(), ys.ravel(), zs.ravel()), axis=1).astype(np.int32)
    colors = np.tile(np.array([[120, 180, 210]], dtype=np.uint8), (voxels.shape[0], 1))

    first = build_greedy_surface_mesh(
        voxels=voxels,
        voxel_size=0.1,
        voxel_colors=colors,
        max_triangles=0,
    )
    second = build_greedy_surface_mesh(
        voxels=voxels,
        voxel_size=0.1,
        voxel_colors=colors,
        max_triangles=0,
    )

    assert first.triangle_count == second.triangle_count
    assert first.vertices == second.vertices
    assert first.colors == second.colors


def test_greedy_surface_mesh_reduces_triangles_vs_naive_on_grid():
    xs, ys, zs = np.mgrid[0:6, 0:6, 0:2]
    voxels = np.stack((xs.ravel(), ys.ravel(), zs.ravel()), axis=1).astype(np.int32)

    naive_triangles = _naive_triangle_count(voxels)
    result = build_greedy_surface_mesh(
        voxels=voxels,
        voxel_size=0.1,
        voxel_colors=None,
        max_triangles=0,
    )

    assert result.triangle_count < naive_triangles


def test_greedy_surface_mesh_respects_triangle_cap():
    xs, ys, zs = np.mgrid[0:8, 0:8, 0:4]
    voxels = np.stack((xs.ravel(), ys.ravel(), zs.ravel()), axis=1).astype(np.int32)

    result = build_greedy_surface_mesh(
        voxels=voxels,
        voxel_size=0.1,
        voxel_colors=None,
        max_triangles=10,
    )

    assert result.triangle_count <= 10


def test_greedy_surface_mesh_returns_colors_when_provided():
    xs, ys, zs = np.mgrid[0:3, 0:3, 0:1]
    voxels = np.stack((xs.ravel(), ys.ravel(), zs.ravel()), axis=1).astype(np.int32)
    colors = np.zeros((voxels.shape[0], 3), dtype=np.uint8)
    colors[:, 0] = 200
    colors[:, 1] = 120
    colors[:, 2] = 80

    result = build_greedy_surface_mesh(
        voxels=voxels,
        voxel_size=0.1,
        voxel_colors=colors,
        max_triangles=0,
    )

    assert result.colors is not None
    assert len(result.colors) == len(result.vertices)
