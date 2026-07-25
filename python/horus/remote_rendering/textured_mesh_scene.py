"""Detailed textured triangle-mesh assets for PC-side HORUS rendering."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path

import numpy as np

from .scene_catalog import MESH_SCENES


DEFAULT_SPONZA_ROOT = (
    Path.home() / ".cache" / "horus" / "remote_render_assets" / "mesh" / "sponza"
)


@dataclass(frozen=True)
class TexturedTriangleMesh:
    vertices: np.ndarray
    faces: np.ndarray
    uvs: np.ndarray
    face_tints: np.ndarray
    face_texture_rects: np.ndarray
    texture_atlas: np.ndarray


def resolve_textured_mesh(
    scene_id: str,
    path: str | Path | None = None,
) -> Path:
    """Resolve a catalog mesh or an explicitly supplied glTF/OBJ source."""
    if scene_id not in MESH_SCENES:
        raise ValueError(f"unknown textured mesh scene: {scene_id}")
    candidate = Path(path).expanduser() if path else MESH_SCENES[scene_id].cache_path
    if candidate.is_dir():
        preferred_names = {
            "sponza_mesh": ("Sponza.gltf", "sponza.obj"),
            "san_miguel_mesh": (
                "San_Miguel.obj",
                "san_miguel.obj",
                "SanMiguel.obj",
            ),
        }[scene_id]
        files = tuple(item for item in candidate.rglob("*") if item.is_file())
        by_name = {item.name: item for item in files}
        candidate = next(
            (by_name[name] for name in preferred_names if name in by_name),
            next(
                (
                    item
                    for item in files
                    if item.suffix.lower() in {".gltf", ".glb", ".obj"}
                    and "low" not in item.name.lower()
                ),
                Path(),
            ),
        )
    if not candidate.is_file():
        raise FileNotFoundError(
            f"{MESH_SCENES[scene_id].label} mesh is missing: {candidate}. Run "
            "'python3 python/examples/tools/fetch_remote_render_advanced_maps.py "
            f"--scene {scene_id}' first."
        )
    return candidate


def resolve_sponza_gltf(path: str | Path | None = None) -> Path:
    """Backward-compatible Sponza resolver."""
    return resolve_textured_mesh("sponza_mesh", path)


def _image_key(image: np.ndarray) -> str:
    digest = hashlib.sha256()
    digest.update(str(image.shape).encode("ascii"))
    digest.update(image.tobytes())
    return digest.hexdigest()


def _material_texture_and_tint(material) -> tuple[np.ndarray, np.ndarray]:
    image = getattr(material, "baseColorTexture", None)
    if image is None:
        image = getattr(material, "image", None)
    if image is None:
        pixels = np.full((4, 4, 3), 255, dtype=np.uint8)
    elif hasattr(image, "convert"):
        pixels = np.asarray(image.convert("RGB"), dtype=np.uint8)
    else:
        pixels = np.asarray(image, dtype=np.uint8)
        if pixels.ndim != 3 or pixels.shape[2] < 3:
            raise ValueError("mesh material texture must contain RGB pixels")
        pixels = pixels[:, :, :3]
    factor_value = getattr(material, "baseColorFactor", None)
    if factor_value is None:
        factor_value = getattr(material, "diffuse", None)
    if factor_value is None:
        factor_value = getattr(material, "main_color", None)
    if factor_value is None:
        factor_value = (255, 255, 255, 255)
    factor = np.asarray(factor_value, dtype=np.float32).reshape(-1)
    if len(factor) < 3:
        factor = np.asarray((255.0, 255.0, 255.0), dtype=np.float32)
    elif float(np.max(factor[:3])) <= 1.0:
        factor = factor * 255.0
    return np.ascontiguousarray(pixels), np.clip(factor[:3], 0.0, 255.0)


def _build_texture_atlas(
    textures: dict[str, np.ndarray],
) -> tuple[np.ndarray, dict[str, tuple[int, int, int, int]]]:
    if not textures:
        textures = {"white": np.full((4, 4, 3), 255, dtype=np.uint8)}
    padding = 2
    entries = sorted(
        textures.items(),
        key=lambda item: (-item[1].shape[0], -item[1].shape[1], item[0]),
    )
    padded_area = sum(
        (image.shape[0] + padding * 2) * (image.shape[1] + padding * 2)
        for _, image in entries
    )
    widest = max(image.shape[1] + padding * 2 for _, image in entries)
    mip_alignment = 16
    target_width = max(widest, int(math.ceil(math.sqrt(padded_area))))
    target_width = int(math.ceil(target_width / mip_alignment)) * mip_alignment

    placements: dict[str, tuple[int, int, int, int]] = {}
    cursor_x = 0
    cursor_y = 0
    shelf_height = 0
    used_width = 0
    for key, image in entries:
        height, width = image.shape[:2]
        padded_width = width + padding * 2
        padded_height = height + padding * 2
        if cursor_x > 0 and cursor_x + padded_width > target_width:
            cursor_y += shelf_height
            cursor_x = 0
            shelf_height = 0
        placements[key] = (
            cursor_x + padding,
            cursor_y + padding,
            width,
            height,
        )
        cursor_x += padded_width
        shelf_height = max(shelf_height, padded_height)
        used_width = max(used_width, cursor_x)

    used_height = cursor_y + shelf_height
    atlas_width = int(math.ceil(used_width / mip_alignment)) * mip_alignment
    atlas_height = int(math.ceil(used_height / mip_alignment)) * mip_alignment
    atlas = np.zeros((atlas_height, atlas_width, 3), dtype=np.uint8)
    rectangles: dict[str, tuple[int, int, int, int]] = {}
    for key, image in entries:
        x, y, width, height = placements[key]
        atlas[y : y + height, x : x + width] = image
        atlas[y - padding : y, x : x + width] = image[:1]
        atlas[y + height : y + height + padding, x : x + width] = image[-1:]
        atlas[y - padding : y + height + padding, x - padding : x] = atlas[
            y - padding : y + height + padding, x : x + 1
        ]
        atlas[
            y - padding : y + height + padding,
            x + width : x + width + padding,
        ] = atlas[y - padding : y + height + padding, x + width - 1 : x + width]
        rectangles[key] = (x, y, width, height)
    return np.ascontiguousarray(atlas), rectangles


def load_textured_mesh_scene(
    scene_id: str,
    path: str | Path | None = None,
    *,
    world_scale: float = 1.0,
) -> TexturedTriangleMesh:
    """Load a complete catalog scene as indexed triangles and texture data."""
    try:
        import trimesh
    except ImportError as exc:
        raise RuntimeError(
            "Textured mesh rendering requires trimesh. Run "
            "'python3 python/examples/tools/install_remote_render_dependencies.py'."
        ) from exc

    source = resolve_textured_mesh(scene_id, path)
    scene = trimesh.load(source, force="scene", process=False)
    records = []
    textures: dict[str, np.ndarray] = {}

    for node_name in scene.graph.nodes_geometry:
        transform, geometry_name = scene.graph.get(node_name)
        geometry = scene.geometry[geometry_name]
        if len(geometry.vertices) == 0 or len(geometry.faces) == 0:
            continue
        visual = geometry.visual
        uv = getattr(visual, "uv", None)
        if uv is None or len(uv) != len(geometry.vertices):
            uv = np.zeros((len(geometry.vertices), 2), dtype=np.float32)
        else:
            uv = np.asarray(uv, dtype=np.float32).copy()
            # glTF UVs use a bottom-left convention; decoded images use top-left.
            uv[:, 1] = 1.0 - uv[:, 1]

        material = getattr(visual, "material", None)
        image, tint = _material_texture_and_tint(material)
        key = _image_key(image)
        textures.setdefault(key, image)
        vertices = trimesh.transform_points(
            np.asarray(geometry.vertices, dtype=np.float64),
            np.asarray(transform, dtype=np.float64),
        )
        records.append(
            (
                np.asarray(vertices, dtype=np.float32),
                np.asarray(geometry.faces, dtype=np.uint32),
                np.asarray(uv, dtype=np.float32),
                key,
                tint,
            )
        )

    if not records:
        raise RuntimeError(
            f"{MESH_SCENES[scene_id].label} contained no triangle geometry: {source}"
        )

    atlas, rectangles = _build_texture_atlas(textures)
    all_vertices = []
    all_faces = []
    all_uvs = []
    all_tints = []
    all_rectangles = []
    vertex_offset = 0
    light = np.asarray((-0.34, 0.86, -0.38), dtype=np.float32)
    light /= np.linalg.norm(light)

    for vertices, faces, uv, key, material_tint in records:
        transformed = np.asarray(vertices, dtype=np.float32)
        all_vertices.append(transformed)
        all_faces.append(faces + vertex_offset)
        all_uvs.append(uv)
        vertex_offset += len(transformed)

        triangles = transformed[faces]
        normals = np.cross(
            triangles[:, 1] - triangles[:, 0],
            triangles[:, 2] - triangles[:, 0],
        )
        lengths = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = np.divide(
            normals,
            np.maximum(lengths, 1e-8),
            out=np.zeros_like(normals),
        )
        illumination = 0.78 + 0.22 * np.abs(normals @ light)
        linear_tint = (
            material_tint[None, :] / 255.0
        ) * illumination[:, None]
        tints = np.clip(
            np.power(np.clip(linear_tint, 0.0, 1.0), 1.0 / 2.2) * 255.0,
            0.0,
            255.0,
        ).astype(np.uint8)
        all_tints.append(tints)
        rectangle = np.asarray(rectangles[key], dtype=np.uint32)
        all_rectangles.append(np.repeat(rectangle[None, :], len(faces), axis=0))

    vertices = np.concatenate(all_vertices, axis=0)
    faces = np.concatenate(all_faces, axis=0)
    uvs = np.concatenate(all_uvs, axis=0)
    face_tints = np.concatenate(all_tints, axis=0)
    face_texture_rects = np.concatenate(all_rectangles, axis=0)

    bounds_min = vertices.min(axis=0)
    bounds_max = vertices.max(axis=0)
    vertices[:, 0] -= (bounds_min[0] + bounds_max[0]) * 0.5
    vertices[:, 1] -= bounds_min[1]
    vertices[:, 2] -= (bounds_min[2] + bounds_max[2]) * 0.5
    vertices *= float(world_scale)

    return TexturedTriangleMesh(
        vertices=np.ascontiguousarray(vertices, dtype=np.float32),
        faces=np.ascontiguousarray(faces, dtype=np.uint32),
        uvs=np.ascontiguousarray(uvs, dtype=np.float32),
        face_tints=np.ascontiguousarray(face_tints, dtype=np.uint8),
        face_texture_rects=np.ascontiguousarray(face_texture_rects, dtype=np.uint32),
        texture_atlas=atlas,
    )


def load_sponza_textured_mesh(
    path: str | Path | None = None,
    *,
    world_scale: float = 1.0,
) -> TexturedTriangleMesh:
    """Backward-compatible Sponza loader."""
    return load_textured_mesh_scene(
        "sponza_mesh",
        path,
        world_scale=world_scale,
    )
