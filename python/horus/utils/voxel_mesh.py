"""Voxel surface meshing helpers for 3D map conversion."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class GreedyMeshResult:
    """Triangle-list mesh output in voxel/world coordinates."""

    vertices: List[Tuple[float, float, float]]
    colors: Optional[List[Tuple[int, int, int]]]
    triangle_count: int


PlaneKey = Tuple[int, int, int]  # axis, side (-1|1), plane index
CellKey = Tuple[int, int]
ColorTuple = Tuple[int, int, int]


def build_greedy_surface_mesh(
    voxels: np.ndarray,
    voxel_size: float,
    voxel_colors: Optional[np.ndarray] = None,
    max_triangles: int = 0,
    color_quant_step: int = 8,
) -> GreedyMeshResult:
    """Build a greedy-merged triangle list for exposed voxel surfaces.

    Args:
        voxels: Nx3 integer voxel coordinates.
        voxel_size: edge size of each voxel in meters.
        voxel_colors: Optional Nx3 uint8 per-voxel colors.
        max_triangles: Triangle cap (0 = unlimited).
        color_quant_step: Quantization step used after region color averaging.
    """

    if voxels is None or voxels.size == 0:
        return GreedyMeshResult(vertices=[], colors=None, triangle_count=0)

    if voxel_colors is not None and len(voxel_colors) != len(voxels):
        voxel_colors = None

    occ_colors: Dict[Tuple[int, int, int], Optional[ColorTuple]] = {}
    if voxel_colors is not None:
        colors_u8 = np.clip(voxel_colors, 0, 255).astype(np.uint8)
        for idx, raw in enumerate(voxels):
            key = (int(raw[0]), int(raw[1]), int(raw[2]))
            rgb = colors_u8[idx]
            occ_colors[key] = (int(rgb[0]), int(rgb[1]), int(rgb[2]))
    else:
        for raw in voxels:
            key = (int(raw[0]), int(raw[1]), int(raw[2]))
            occ_colors[key] = None

    planes = _collect_exposed_face_planes(occ_colors)
    vertices: List[Tuple[float, float, float]] = []
    colors: Optional[List[ColorTuple]] = [] if voxel_colors is not None else None
    triangle_count = 0

    for plane_key in sorted(planes.keys()):
        if max_triangles > 0 and triangle_count >= max_triangles:
            break

        rects = _greedy_rectangles_for_plane(
            planes[plane_key],
            color_quant_step=max(1, int(color_quant_step)),
        )
        axis, side, plane_k = plane_key

        for u0, v0, width, height, rgb in rects:
            if max_triangles > 0 and triangle_count + 2 > max_triangles:
                break

            quad = _quad_vertices(axis, plane_k, u0, v0, width, height)
            if side < 0:
                tri_indices = ((0, 2, 1), (0, 3, 2))
            else:
                tri_indices = ((0, 1, 2), (0, 2, 3))

            for i0, i1, i2 in tri_indices:
                p0 = quad[i0] * voxel_size
                p1 = quad[i1] * voxel_size
                p2 = quad[i2] * voxel_size
                vertices.append((float(p0[0]), float(p0[1]), float(p0[2])))
                vertices.append((float(p1[0]), float(p1[1]), float(p1[2])))
                vertices.append((float(p2[0]), float(p2[1]), float(p2[2])))

                if colors is not None:
                    c = rgb if rgb is not None else (180, 184, 198)
                    colors.extend((c, c, c))

            triangle_count += 2

    return GreedyMeshResult(vertices=vertices, colors=colors, triangle_count=triangle_count)


def _collect_exposed_face_planes(
    occ_colors: Dict[Tuple[int, int, int], Optional[ColorTuple]],
) -> Dict[PlaneKey, Dict[CellKey, Optional[ColorTuple]]]:
    planes: Dict[PlaneKey, Dict[CellKey, Optional[ColorTuple]]] = {}

    for (x, y, z), color in occ_colors.items():
        # +X / -X
        if (x + 1, y, z) not in occ_colors:
            planes.setdefault((0, 1, x + 1), {})[(y, z)] = color
        if (x - 1, y, z) not in occ_colors:
            planes.setdefault((0, -1, x), {})[(y, z)] = color

        # +Y / -Y
        if (x, y + 1, z) not in occ_colors:
            planes.setdefault((1, 1, y + 1), {})[(x, z)] = color
        if (x, y - 1, z) not in occ_colors:
            planes.setdefault((1, -1, y), {})[(x, z)] = color

        # +Z / -Z
        if (x, y, z + 1) not in occ_colors:
            planes.setdefault((2, 1, z + 1), {})[(x, y)] = color
        if (x, y, z - 1) not in occ_colors:
            planes.setdefault((2, -1, z), {})[(x, y)] = color

    return planes


def _greedy_rectangles_for_plane(
    cells: Dict[CellKey, Optional[ColorTuple]],
    color_quant_step: int,
) -> List[Tuple[int, int, int, int, Optional[ColorTuple]]]:
    if not cells:
        return []

    u_values = [cell[0] for cell in cells]
    v_values = [cell[1] for cell in cells]
    min_u = min(u_values)
    max_u = max(u_values)
    min_v = min(v_values)
    max_v = max(v_values)

    width = max_u - min_u + 1
    height = max_v - min_v + 1

    present = np.zeros((height, width), dtype=np.bool_)
    color_grid = np.zeros((height, width, 3), dtype=np.int32)
    has_color = np.zeros((height, width), dtype=np.bool_)

    for (u, v), color in cells.items():
        uu = u - min_u
        vv = v - min_v
        present[vv, uu] = True
        if color is not None:
            has_color[vv, uu] = True
            color_grid[vv, uu, 0] = int(color[0])
            color_grid[vv, uu, 1] = int(color[1])
            color_grid[vv, uu, 2] = int(color[2])

    visited = np.zeros((height, width), dtype=np.bool_)
    rects: List[Tuple[int, int, int, int, Optional[ColorTuple]]] = []

    for v0 in range(height):
        for u0 in range(width):
            if not present[v0, u0] or visited[v0, u0]:
                continue

            run_w = 1
            while (u0 + run_w) < width and present[v0, u0 + run_w] and not visited[v0, u0 + run_w]:
                run_w += 1

            run_h = 1
            while (v0 + run_h) < height:
                row_ok = True
                for x in range(u0, u0 + run_w):
                    if not present[v0 + run_h, x] or visited[v0 + run_h, x]:
                        row_ok = False
                        break
                if not row_ok:
                    break
                run_h += 1

            for vv in range(v0, v0 + run_h):
                for uu in range(u0, u0 + run_w):
                    visited[vv, uu] = True

            block_has_color = has_color[v0 : v0 + run_h, u0 : u0 + run_w]
            color_value: Optional[ColorTuple] = None
            if np.any(block_has_color):
                rgb_block = color_grid[v0 : v0 + run_h, u0 : u0 + run_w]
                valid_mask = block_has_color.astype(np.bool_)
                r = int(np.rint(np.mean(rgb_block[:, :, 0][valid_mask])))
                g = int(np.rint(np.mean(rgb_block[:, :, 1][valid_mask])))
                b = int(np.rint(np.mean(rgb_block[:, :, 2][valid_mask])))
                color_value = (
                    _quantize_channel(r, color_quant_step),
                    _quantize_channel(g, color_quant_step),
                    _quantize_channel(b, color_quant_step),
                )

            rects.append(
                (
                    min_u + u0,
                    min_v + v0,
                    run_w,
                    run_h,
                    color_value,
                )
            )

    return rects


def _quad_vertices(
    axis: int,
    plane_k: int,
    u0: int,
    v0: int,
    width: int,
    height: int,
) -> np.ndarray:
    u1 = u0 + width
    v1 = v0 + height

    if axis == 0:
        # Plane X=const, local axes Y (u) and Z (v)
        return np.array(
            [
                [plane_k, u0, v0],
                [plane_k, u1, v0],
                [plane_k, u1, v1],
                [plane_k, u0, v1],
            ],
            dtype=np.float32,
        )

    if axis == 1:
        # Plane Y=const, local axes Z then X (to keep +Y winding correct)
        return np.array(
            [
                [u0, plane_k, v0],
                [u0, plane_k, v1],
                [u1, plane_k, v1],
                [u1, plane_k, v0],
            ],
            dtype=np.float32,
        )

    # axis == 2 -> Plane Z=const, local axes X (u) and Y (v)
    return np.array(
        [
            [u0, v0, plane_k],
            [u1, v0, plane_k],
            [u1, v1, plane_k],
            [u0, v1, plane_k],
        ],
        dtype=np.float32,
    )


def _quantize_channel(value: int, step: int) -> int:
    if step <= 1:
        return int(np.clip(value, 0, 255))
    quantized = int(np.rint(value / float(step)) * step)
    return int(np.clip(quantized, 0, 255))
