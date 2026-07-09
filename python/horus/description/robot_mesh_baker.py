"""Mesh normalization and baking helpers for robot-description visual meshes."""

from __future__ import annotations

import base64
import hashlib
import json
import logging
import math
import os
import shutil
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from .robot_description_models import MeshAsset

logger = logging.getLogger(__name__)

try:
    from PIL import Image
except Exception:  # pragma: no cover - Pillow is optional for texture color hints.
    Image = None


def _float_list(values: np.ndarray) -> List[float]:
    return [float(value) for value in values.tolist()]


def _pack_float32(values: np.ndarray) -> str:
    return base64.b64encode(np.asarray(values, dtype=np.float32).tobytes()).decode("ascii")


def _pack_int32(values: np.ndarray) -> str:
    return base64.b64encode(np.asarray(values, dtype=np.int32).tobytes()).decode("ascii")


def _pack_color32(values: np.ndarray) -> str:
    clipped = np.clip(np.asarray(values, dtype=np.float32), 0.0, 1.0)
    packed = np.rint(clipped * 255.0).astype(np.uint8)
    return base64.b64encode(packed.tobytes()).decode("ascii")


def _normalize_vector_rows(values: np.ndarray) -> np.ndarray:
    if values.size == 0:
        return values.astype(np.float32, copy=False)
    lengths = np.linalg.norm(values, axis=1)
    safe = np.where(lengths > 1e-12, lengths, 1.0)
    normalized = values / safe[:, None]
    normalized[lengths <= 1e-12] = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    return normalized.astype(np.float32, copy=False)


def _rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return np.array(
        [
            [(cy * cp), (cy * sp * sr) - (sy * cr), (cy * sp * cr) + (sy * sr)],
            [(sy * cp), (sy * sp * sr) + (cy * cr), (sy * sp * cr) - (cy * sr)],
            [(-sp), cp * sr, cp * cr],
        ],
        dtype=np.float32,
    )


def _triangulate_face(indices: List[int]) -> List[Tuple[int, int, int]]:
    if len(indices) < 3:
        return []
    if len(indices) == 3:
        return [(indices[0], indices[1], indices[2])]
    triangles: List[Tuple[int, int, int]] = []
    first = indices[0]
    for offset in range(1, len(indices) - 1):
        triangles.append((first, indices[offset], indices[offset + 1]))
    return triangles


@dataclass(frozen=True)
class SourceMeshData:
    vertices: np.ndarray
    faces: np.ndarray
    normals: Optional[np.ndarray]
    color_rgb: Optional[np.ndarray]
    colors: Optional[np.ndarray] = None


@dataclass(frozen=True)
class CombinedMeshData:
    vertices: np.ndarray
    faces: np.ndarray
    normals: np.ndarray
    bounds_min: np.ndarray
    bounds_max: np.ndarray
    color_rgb: Optional[np.ndarray]


class RobotMeshBaker:
    """Normalize URDF mesh sources into compact baked assets."""

    _DEFAULT_TOTAL_TRIANGLE_BUDGET = 90000
    _DEFAULT_MAX_TRIANGLES_PER_ASSET = 60000
    _RUNTIME_HIGH_TOTAL_TRIANGLE_BUDGET = 240000
    _RUNTIME_HIGH_MAX_TRIANGLES_PER_ASSET = 150000

    def __init__(self, cache_root: Optional[str] = None):
        self._cache_root = Path(
            cache_root or os.path.expanduser("~/.cache/horus/robot_description_meshes")
        )
        self._cache_root.mkdir(parents=True, exist_ok=True)
        self._source_cache: Dict[str, SourceMeshData] = {}
        self._decimated_cache: Dict[str, SourceMeshData] = {}
        self._texture_color_cache: Dict[str, Optional[np.ndarray]] = {}

    def build_combined_asset(
        self,
        mesh_entries: List[Dict[str, object]],
        description_id_seed: str,
        target_triangles: Optional[int] = _DEFAULT_TOTAL_TRIANGLE_BUDGET,
        mesh_mode: str = "preview_mesh",
    ) -> Optional[Tuple[MeshAsset, List[float], List[float]]]:
        if not mesh_entries:
            return None

        combined_vertices: List[np.ndarray] = []
        combined_faces: List[np.ndarray] = []
        combined_normals: List[np.ndarray] = []
        combined_colors: List[np.ndarray] = []
        color_accumulator = np.zeros(3, dtype=np.float64)
        color_samples = 0
        vertex_offset = 0

        normalized_mode = str(mesh_mode or "preview_mesh").strip().lower()
        if normalized_mode == "max_quality_mesh":
            normalized_mode = "runtime_high_mesh"
        if normalized_mode not in {"preview_mesh", "runtime_high_mesh", "collision_only"}:
            normalized_mode = "preview_mesh"

        target_triangles = None if target_triangles is None else max(2000, int(target_triangles))
        if normalized_mode == "runtime_high_mesh" and target_triangles is None:
            target_triangles = self._RUNTIME_HIGH_TOTAL_TRIANGLE_BUDGET
        elif normalized_mode == "preview_mesh" and target_triangles is None:
            target_triangles = self._DEFAULT_TOTAL_TRIANGLE_BUDGET

        if normalized_mode == "runtime_high_mesh":
            per_asset_min = 5000
            per_asset_max = self._RUNTIME_HIGH_MAX_TRIANGLES_PER_ASSET
        else:
            per_asset_min = 2000
            per_asset_max = self._DEFAULT_MAX_TRIANGLES_PER_ASSET

        source_triangle_counts: List[int] = []
        resolved_sources: List[Optional[SourceMeshData]] = []
        for entry in mesh_entries:
            source_path = Path(str(entry.get("mesh_path") or "")).resolve()
            if not source_path.is_file():
                resolved_sources.append(None)
                source_triangle_counts.append(0)
                continue
            source_mesh = self._load_source_mesh(source_path)
            resolved_sources.append(source_mesh)
            source_triangle_counts.append(len(source_mesh.faces) if source_mesh is not None else 0)

        total_source_triangles = max(0, sum(source_triangle_counts))

        for entry, raw_source_mesh, raw_triangle_count in zip(mesh_entries, resolved_sources, source_triangle_counts):
            source_uri = str(entry.get("mesh_uri") or "").strip()
            if not source_uri:
                continue

            source_path = Path(str(entry.get("mesh_path") or "")).resolve()
            if not source_path.is_file():
                continue

            if raw_source_mesh is None:
                raw_source_mesh = self._load_source_mesh(source_path)
                raw_triangle_count = len(raw_source_mesh.faces) if raw_source_mesh is not None else 0
            if raw_source_mesh is None:
                continue

            target_for_entry = None
            if target_triangles is not None and total_source_triangles > 0:
                proportional_target = int(math.ceil(target_triangles * (float(raw_triangle_count) / float(total_source_triangles))))
                target_for_entry = max(per_asset_min, min(per_asset_max, proportional_target))
                if raw_triangle_count <= target_for_entry:
                    target_for_entry = None

            source_mesh = self._load_source_mesh(source_path, target_faces=target_for_entry)
            if source_mesh is None:
                source_mesh = raw_source_mesh
            if source_mesh is None:
                continue

            vertices = source_mesh.vertices
            faces = source_mesh.faces
            normals = source_mesh.normals

            if normals is None or len(normals) != len(vertices):
                normals = self._compute_vertex_normals(vertices, faces)

            scale_xyz = np.asarray(entry.get("mesh_scale_xyz") or [1.0, 1.0, 1.0], dtype=np.float32)
            visual_translation = np.asarray(entry.get("origin_xyz") or [0.0, 0.0, 0.0], dtype=np.float32)
            visual_rpy = np.asarray(entry.get("origin_rpy") or [0.0, 0.0, 0.0], dtype=np.float32)
            link_translation = np.asarray(entry.get("link_xyz") or [0.0, 0.0, 0.0], dtype=np.float32)
            link_rpy = np.asarray(entry.get("link_rpy") or [0.0, 0.0, 0.0], dtype=np.float32)

            local_rotation = _rotation_matrix(float(visual_rpy[0]), float(visual_rpy[1]), float(visual_rpy[2]))
            link_rotation = _rotation_matrix(float(link_rpy[0]), float(link_rpy[1]), float(link_rpy[2]))
            scaled_vertices = vertices * scale_xyz[None, :]
            local_vertices = (scaled_vertices @ local_rotation.T) + visual_translation[None, :]
            world_vertices = (local_vertices @ link_rotation.T) + link_translation[None, :]

            rotated_normals = (normals @ local_rotation.T) @ link_rotation.T
            rotated_normals = _normalize_vector_rows(rotated_normals)

            combined_vertices.append(world_vertices.astype(np.float32, copy=False))
            combined_normals.append(rotated_normals.astype(np.float32, copy=False))
            combined_faces.append((faces + vertex_offset).astype(np.int32, copy=False))
            vertex_offset += len(world_vertices)

            entry_color = entry.get("color_rgb")
            entry_fallback_color = entry.get("fallback_color_rgb")
            source_color = None
            source_colors = None
            if isinstance(entry_color, (list, tuple)) and len(entry_color) >= 3:
                source_color = np.asarray(entry_color[:3], dtype=np.float32)
            elif source_mesh.colors is not None and len(source_mesh.colors) == len(vertices):
                source_colors = np.asarray(source_mesh.colors, dtype=np.float32)
                if source_colors.ndim == 2 and source_colors.shape[1] >= 3:
                    source_color = np.clip(source_colors[:, :3].mean(axis=0), 0.0, 1.0).astype(np.float32)
                else:
                    source_colors = None
            elif source_mesh.color_rgb is not None and len(source_mesh.color_rgb) >= 3:
                source_color = np.asarray(source_mesh.color_rgb[:3], dtype=np.float32)
            elif isinstance(entry_fallback_color, (list, tuple)) and len(entry_fallback_color) >= 3:
                source_color = np.asarray(entry_fallback_color[:3], dtype=np.float32)

            if source_color is None:
                source_color = np.asarray([0.78, 0.78, 0.76], dtype=np.float32)
            else:
                color_accumulator += source_color.astype(np.float64)
                color_samples += 1

            if source_colors is not None:
                if source_colors.shape[1] >= 4:
                    entry_colors = source_colors[:, :4]
                else:
                    alpha = np.ones((len(source_colors), 1), dtype=np.float32)
                    entry_colors = np.concatenate((source_colors[:, :3], alpha), axis=1)
                entry_colors = np.clip(entry_colors, 0.0, 1.0)
            else:
                entry_colors = np.tile(
                    np.concatenate(
                        [
                            np.clip(source_color, 0.0, 1.0),
                            np.array([1.0], dtype=np.float32),
                        ]
                    ),
                    (len(world_vertices), 1),
                )
            combined_colors.append(entry_colors.astype(np.float32, copy=False))

        if not combined_vertices or not combined_faces:
            return None

        vertices = np.concatenate(combined_vertices, axis=0)
        faces = np.concatenate(combined_faces, axis=0)
        normals = _normalize_vector_rows(np.concatenate(combined_normals, axis=0))
        colors = np.concatenate(combined_colors, axis=0) if combined_colors else None
        bounds_min = vertices.min(axis=0)
        bounds_max = vertices.max(axis=0)
        mean_color = None
        if color_samples > 0:
            mean_color = np.clip(color_accumulator / float(color_samples), 0.0, 1.0).astype(np.float32)

        payload_seed = {
            "description_id_seed": description_id_seed,
            "mesh_baker_version": "visual_materials_v2",
            "mesh_mode": normalized_mode,
            "target_triangles": int(target_triangles) if target_triangles is not None else 0,
            "mesh_entries": [
                {
                    "mesh_uri": str(entry.get("mesh_uri") or ""),
                    "mesh_path": str(entry.get("mesh_path") or ""),
                    "link_name": str(entry.get("link_name") or ""),
                    "origin_xyz": list(entry.get("origin_xyz") or [0.0, 0.0, 0.0]),
                    "origin_rpy": list(entry.get("origin_rpy") or [0.0, 0.0, 0.0]),
                    "link_xyz": list(entry.get("link_xyz") or [0.0, 0.0, 0.0]),
                    "link_rpy": list(entry.get("link_rpy") or [0.0, 0.0, 0.0]),
                    "mesh_scale_xyz": list(entry.get("mesh_scale_xyz") or [1.0, 1.0, 1.0]),
                    "color_rgb": list(entry.get("color_rgb") or []),
                    "fallback_color_rgb": list(entry.get("fallback_color_rgb") or []),
                }
                for entry in mesh_entries
            ],
            "triangle_count": int(len(faces)),
        }
        mesh_id = "mesh:" + hashlib.sha256(
            json.dumps(payload_seed, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()

        asset = MeshAsset(
            mesh_id=mesh_id,
            vertex_count=int(len(vertices)),
            triangle_count=int(len(faces)),
            positions_b64=_pack_float32(vertices.reshape(-1)),
            normals_b64=_pack_float32(normals.reshape(-1)),
            indices_b64=_pack_int32(faces.reshape(-1)),
            bounds_min=_float_list(bounds_min.astype(np.float32)),
            bounds_max=_float_list(bounds_max.astype(np.float32)),
            color_rgb=_float_list(mean_color) if mean_color is not None else [],
            colors_b64=_pack_color32(colors.reshape(-1, 4)) if colors is not None else "",
        )
        return asset, _float_list(bounds_min.astype(np.float32)), _float_list(bounds_max.astype(np.float32))

    def _load_source_mesh(self, source_path: Path, target_faces: Optional[int] = None) -> Optional[SourceMeshData]:
        source_hash = self._hash_file(source_path)
        cache_key = f"{source_path.suffix.lower()}:{source_hash}"
        cached = self._source_cache.get(cache_key)
        if cached is None:
            if source_path.suffix.lower() == ".dae":
                cached = self._parse_dae_mesh(source_path)
            else:
                normalized_path = self._normalize_to_obj_if_needed(source_path)
                if normalized_path is None or not normalized_path.is_file():
                    return None

                vertices, faces, color_rgb, colors = self._parse_obj_mesh(normalized_path)
                if vertices is None or faces is None or len(vertices) == 0 or len(faces) == 0:
                    return None

                cached = SourceMeshData(vertices=vertices, faces=faces, normals=None, color_rgb=color_rgb, colors=colors)

            if cached is None or len(cached.vertices) == 0 or len(cached.faces) == 0:
                return None
            self._source_cache[cache_key] = cached

        if target_faces is None or len(cached.faces) <= int(target_faces):
            return cached

        decimated_key = f"{cache_key}:decimate:{int(target_faces)}"
        decimated = self._decimated_cache.get(decimated_key)
        if decimated is not None:
            return decimated

        normalized_path = None if source_path.suffix.lower() == ".dae" else self._normalize_to_obj_if_needed(source_path)
        if normalized_path is not None and normalized_path.is_file():
            decimated_path = self._decimate_obj_with_blender(normalized_path, int(target_faces))
            if decimated_path is not None and decimated_path.is_file():
                vertices, faces, _, colors = self._parse_obj_mesh(decimated_path)
                if vertices is not None and faces is not None and len(vertices) > 0 and len(faces) > 0:
                    decimated = SourceMeshData(
                        vertices=vertices,
                        faces=faces,
                        normals=None,
                        color_rgb=cached.color_rgb,
                        colors=colors,
                    )
                    self._decimated_cache[decimated_key] = decimated
                    return decimated

        decimated = self._decimate_source_mesh(cached, int(target_faces))
        self._decimated_cache[decimated_key] = decimated
        return decimated

    def _resolve_blender_path(self) -> Optional[str]:
        blender_path = os.environ.get("HORUS_SDK_BLENDER_PATH", "").strip()
        if blender_path and os.path.isfile(blender_path):
            return blender_path

        direct_blender = shutil.which("blender")
        if direct_blender:
            return direct_blender

        windows_root = Path("/mnt/c/Program Files/Blender Foundation")
        if windows_root.is_dir():
            candidates = sorted(windows_root.glob("Blender */blender.exe"), reverse=True)
            for candidate in candidates:
                if candidate.is_file():
                    return str(candidate)

        return None

    @staticmethod
    def _is_windows_executable_path(path_value: str) -> bool:
        normalized = str(path_value or "").strip().lower()
        return normalized.endswith(".exe")

    def _to_blender_cli_path(self, blender_path: str, path_value: Path) -> str:
        if not self._is_windows_executable_path(blender_path):
            return str(path_value)

        try:
            converted = subprocess.run(
                ["wslpath", "-w", str(path_value)],
                check=True,
                capture_output=True,
                text=True,
                timeout=10.0,
            )
            windows_path = converted.stdout.strip()
            if windows_path:
                return windows_path
        except Exception:
            logger.debug("Failed to convert WSL path %s for Windows Blender", path_value, exc_info=True)
        return str(path_value)

    def _normalize_to_obj_if_needed(self, source_path: Path) -> Optional[Path]:
        suffix = source_path.suffix.lower()
        if suffix == ".obj":
            return source_path
        if suffix == ".stl":
            return self._convert_stl_to_obj(source_path)

        blender_path = self._resolve_blender_path()
        if not blender_path:
            return None

        output_path = self._cache_root / f"{self._hash_file(source_path)}.obj"
        if output_path.is_file():
            return output_path

        script_path = self._cache_root / "blender_obj_convert.py"
        if not script_path.is_file():
            script_path.write_text(
                "\n".join(
                    [
                        "import bpy",
                        "import sys",
                        "argv = sys.argv[sys.argv.index('--') + 1:]",
                        "src, dst = argv[0], argv[1]",
                        "bpy.ops.wm.read_factory_settings(use_empty=True)",
                        "if src.lower().endswith('.obj'): bpy.ops.wm.obj_import(filepath=src)",
                        "elif src.lower().endswith('.stl'): bpy.ops.wm.stl_import(filepath=src)",
                        "else: bpy.ops.wm.collada_import(filepath=src)",
                        "bpy.ops.wm.obj_export(filepath=dst, export_materials=False)",
                    ]
                ),
                encoding="utf-8",
            )

        with tempfile.TemporaryDirectory(prefix="horus_mesh_baker_") as tmp_dir:
            process = subprocess.run(
                [
                    blender_path,
                    "--background",
                    "--python",
                    self._to_blender_cli_path(blender_path, script_path),
                    "--",
                    self._to_blender_cli_path(blender_path, source_path),
                    self._to_blender_cli_path(blender_path, output_path),
                ],
                cwd=tmp_dir,
                check=False,
                capture_output=True,
                text=True,
                timeout=240.0,
            )
        if process.returncode != 0 or not output_path.is_file():
            return None
        return output_path

    def _decimate_obj_with_blender(self, source_path: Path, target_faces: int) -> Optional[Path]:
        blender_path = self._resolve_blender_path()
        if not blender_path:
            return None

        target_faces = max(64, int(target_faces))
        output_path = self._cache_root / f"{self._hash_file(source_path)}_decimate_{target_faces}.obj"
        if output_path.is_file():
            return output_path

        script_path = self._cache_root / "blender_obj_decimate.py"
        if not script_path.is_file():
            script_path.write_text(
                "\n".join(
                    [
                        "import bpy",
                        "import sys",
                        "argv = sys.argv[sys.argv.index('--') + 1:]",
                        "src, dst, target = argv[0], argv[1], max(1, int(argv[2]))",
                        "bpy.ops.wm.read_factory_settings(use_empty=True)",
                        "if src.lower().endswith('.obj'): bpy.ops.wm.obj_import(filepath=src)",
                        "elif src.lower().endswith('.stl'): bpy.ops.wm.stl_import(filepath=src)",
                        "else: bpy.ops.wm.collada_import(filepath=src)",
                        "mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']",
                        "if not mesh_objects: raise RuntimeError('No mesh objects imported for decimation.')",
                        "bpy.ops.object.select_all(action='DESELECT')",
                        "for obj in mesh_objects: obj.select_set(True)",
                        "bpy.context.view_layer.objects.active = mesh_objects[0]",
                        "if len(mesh_objects) > 1: bpy.ops.object.join()",
                        "obj = bpy.context.view_layer.objects.active",
                        "poly_count = max(1, len(obj.data.polygons))",
                        "if poly_count > target:",
                        "    ratio = max(0.001, min(1.0, float(target) / float(poly_count)))",
                        "    modifier = obj.modifiers.new(name='HorusDecimate', type='DECIMATE')",
                        "    modifier.decimate_type = 'COLLAPSE'",
                        "    modifier.ratio = ratio",
                        "    modifier.use_collapse_triangulate = True",
                        "    bpy.ops.object.modifier_apply(modifier=modifier.name)",
                        "bpy.ops.wm.obj_export(filepath=dst, export_materials=False)",
                    ]
                ),
                encoding='utf-8',
            )

        with tempfile.TemporaryDirectory(prefix='horus_mesh_decimate_') as tmp_dir:
            process = subprocess.run(
                [
                    blender_path,
                    '--background',
                    '--python',
                    self._to_blender_cli_path(blender_path, script_path),
                    '--',
                    self._to_blender_cli_path(blender_path, source_path),
                    self._to_blender_cli_path(blender_path, output_path),
                    str(target_faces),
                ],
                cwd=tmp_dir,
                check=False,
                capture_output=True,
                text=True,
                timeout=600.0,
            )
        if process.returncode != 0 or not output_path.is_file():
            return None
        return output_path

    def _convert_stl_to_obj(self, source_path: Path) -> Optional[Path]:
        output_path = self._cache_root / f"{self._hash_file(source_path)}.obj"
        if output_path.is_file():
            return output_path

        with source_path.open("rb") as handle:
            header = handle.read(80)
            triangle_count_raw = handle.read(4)
            if len(triangle_count_raw) != 4:
                return None
            triangle_count = struct.unpack("<I", triangle_count_raw)[0]
            expected_size = 84 + (triangle_count * 50)
            file_size = source_path.stat().st_size
            is_binary = file_size == expected_size

        try:
            if is_binary:
                return self._binary_stl_to_obj(source_path, output_path)
            return self._ascii_stl_to_obj(source_path, output_path)
        except Exception:
            logger.debug("Failed to convert STL mesh %s to OBJ", source_path, exc_info=True)
            return None

    def _binary_stl_to_obj(self, source_path: Path, output_path: Path) -> Path:
        vertices: List[Tuple[float, float, float]] = []
        faces: List[Tuple[int, int, int]] = []
        vertex_map: Dict[Tuple[float, float, float], int] = {}

        with source_path.open("rb") as handle:
            handle.seek(80)
            triangle_count = struct.unpack("<I", handle.read(4))[0]
            for _ in range(triangle_count):
                record = handle.read(50)
                if len(record) != 50:
                    break
                coords = struct.unpack("<12fH", record)
                triangle_indices: List[int] = []
                for vertex_offset in range(3):
                    vertex = (
                        float(coords[3 + (vertex_offset * 3)]),
                        float(coords[4 + (vertex_offset * 3)]),
                        float(coords[5 + (vertex_offset * 3)]),
                    )
                    index = vertex_map.get(vertex)
                    if index is None:
                        index = len(vertices) + 1
                        vertex_map[vertex] = index
                        vertices.append(vertex)
                    triangle_indices.append(index)
                faces.append((triangle_indices[0], triangle_indices[1], triangle_indices[2]))

        with output_path.open("w", encoding="utf-8") as handle:
            handle.write("# horus stl->obj conversion\n")
            for vertex in vertices:
                handle.write(f"v {vertex[0]} {vertex[1]} {vertex[2]}\n")
            for face in faces:
                handle.write(f"f {face[0]} {face[1]} {face[2]}\n")
        return output_path

    def _ascii_stl_to_obj(self, source_path: Path, output_path: Path) -> Path:
        vertices: List[Tuple[float, float, float]] = []
        faces: List[Tuple[int, int, int]] = []
        vertex_map: Dict[Tuple[float, float, float], int] = {}

        with source_path.open("r", encoding="utf-8", errors="ignore") as handle:
            current: List[int] = []
            for raw_line in handle:
                line = raw_line.strip()
                if not line.startswith("vertex "):
                    continue
                _, x_raw, y_raw, z_raw = line.split()[:4]
                vertex = (float(x_raw), float(y_raw), float(z_raw))
                index = vertex_map.get(vertex)
                if index is None:
                    index = len(vertices) + 1
                    vertex_map[vertex] = index
                    vertices.append(vertex)
                current.append(index)
                if len(current) == 3:
                    faces.append((current[0], current[1], current[2]))
                    current = []

        with output_path.open("w", encoding="utf-8") as handle:
            handle.write("# horus stl->obj conversion\n")
            for vertex in vertices:
                handle.write(f"v {vertex[0]} {vertex[1]} {vertex[2]}\n")
            for face in faces:
                handle.write(f"f {face[0]} {face[1]} {face[2]}\n")
        return output_path

    def _parse_dae_mesh(self, source_path: Path) -> Optional[SourceMeshData]:
        try:
            root = ET.parse(source_path).getroot()
        except Exception:
            logger.debug("Failed to parse DAE mesh %s", source_path, exc_info=True)
            return None

        namespace = ""
        if root.tag.startswith("{") and "}" in root.tag:
            namespace = root.tag[1:].split("}", 1)[0]

        def tag(name: str) -> str:
            return f"{{{namespace}}}{name}" if namespace else name

        def parse_matrix(matrix_text: Optional[str]) -> np.ndarray:
            values = np.fromstring(matrix_text or "", sep=" ", dtype=np.float32)
            if values.size != 16:
                return np.eye(4, dtype=np.float32)
            # COLLADA stores the Blender-exported node matrix in row-major textual order.
            return values.reshape((4, 4)).astype(np.float32, copy=False)

        def local_node_matrix(node_el: ET.Element) -> np.ndarray:
            matrix = np.eye(4, dtype=np.float32)
            for child in list(node_el):
                if child.tag == tag("matrix"):
                    matrix = matrix @ parse_matrix(child.text)
                elif child.tag == tag("translate"):
                    values = np.fromstring(child.text or "", sep=" ", dtype=np.float32)
                    if values.size >= 3:
                        translate = np.eye(4, dtype=np.float32)
                        translate[:3, 3] = values[:3]
                        matrix = matrix @ translate
                elif child.tag == tag("scale"):
                    values = np.fromstring(child.text or "", sep=" ", dtype=np.float32)
                    if values.size >= 3:
                        scale = np.eye(4, dtype=np.float32)
                        scale[0, 0] = values[0]
                        scale[1, 1] = values[1]
                        scale[2, 2] = values[2]
                        matrix = matrix @ scale
                elif child.tag == tag("rotate"):
                    values = np.fromstring(child.text or "", sep=" ", dtype=np.float32)
                    if values.size >= 4:
                        axis = values[:3]
                        norm = np.linalg.norm(axis)
                        if norm > 1e-12:
                            axis = axis / norm
                            angle = math.radians(float(values[3]))
                            c = math.cos(angle)
                            s = math.sin(angle)
                            t = 1.0 - c
                            x, y, z = float(axis[0]), float(axis[1]), float(axis[2])
                            rotation = np.array(
                                [
                                    [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0],
                                    [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0],
                                    [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0],
                                    [0.0, 0.0, 0.0, 1.0],
                                ],
                                dtype=np.float32,
                            )
                            matrix = matrix @ rotation
            return matrix

        def parse_color(color_text: Optional[str]) -> Optional[np.ndarray]:
            values = np.fromstring(color_text or "", sep=" ", dtype=np.float32)
            if values.size < 3:
                return None
            return np.clip(values[:3], 0.0, 1.0).astype(np.float32, copy=False)

        def srgb_to_linear(values: np.ndarray) -> np.ndarray:
            clipped = np.clip(values, 0.0, 1.0)
            return np.where(
                clipped <= 0.04045,
                clipped / 12.92,
                np.power((clipped + 0.055) / 1.055, 2.4),
            ).astype(np.float32, copy=False)

        def resolve_image_path(raw_path: Optional[str]) -> Optional[Path]:
            path_text = str(raw_path or "").strip()
            if not path_text:
                return None
            if path_text.startswith("file://"):
                path_text = path_text[7:]
            candidate = Path(path_text)
            if not candidate.is_absolute():
                candidate = source_path.parent / candidate
            return candidate

        def texture_mean_color(texture_path: Optional[Path]) -> Optional[np.ndarray]:
            if texture_path is None or Image is None:
                return None
            cache_key = str(texture_path)
            if cache_key in self._texture_color_cache:
                return self._texture_color_cache[cache_key]
            color = None
            if texture_path.is_file():
                try:
                    with Image.open(texture_path) as image:
                        rgba = image.convert("RGBA")
                        rgba.thumbnail((128, 128))
                        pixels = np.asarray(rgba, dtype=np.float32) / 255.0
                        if pixels.size > 0:
                            rgb = pixels[..., :3].reshape((-1, 3))
                            rgb_linear = srgb_to_linear(rgb)
                            alpha = pixels[..., 3].reshape((-1, 1))
                            total_alpha = float(alpha.sum())
                            if total_alpha > 1e-6:
                                color = np.clip((rgb_linear * alpha).sum(axis=0) / total_alpha, 0.0, 1.0).astype(np.float32)
                            else:
                                color = np.clip(rgb_linear.mean(axis=0), 0.0, 1.0).astype(np.float32)
                except Exception:
                    logger.debug("Failed to sample DAE texture color %s", texture_path, exc_info=True)
            self._texture_color_cache[cache_key] = color
            return color

        image_paths: Dict[str, Path] = {}
        for image_el in root.findall(f".//{tag('image')}"):
            image_ref = str(image_el.attrib.get("id", "")).strip()
            image_name = str(image_el.attrib.get("name", "")).strip()
            init_from_el = image_el.find(tag("init_from"))
            image_path = resolve_image_path(init_from_el.text if init_from_el is not None else "")
            if image_path is None:
                continue
            if image_ref:
                image_paths[image_ref] = image_path
            if image_name:
                image_paths[image_name] = image_path

        effect_colors: Dict[str, np.ndarray] = {}
        effect_texture_colors: Dict[str, np.ndarray] = {}
        for effect_el in root.findall(f".//{tag('effect')}"):
            effect_id = str(effect_el.attrib.get("id", "")).strip()
            if not effect_id:
                continue

            surface_images: Dict[str, str] = {}
            sampler_surfaces: Dict[str, str] = {}
            for newparam_el in effect_el.findall(f".//{tag('newparam')}"):
                sid = str(newparam_el.attrib.get("sid", "")).strip()
                if not sid:
                    continue
                surface_el = newparam_el.find(tag("surface"))
                if surface_el is not None:
                    init_from_el = surface_el.find(tag("init_from"))
                    image_ref = str(init_from_el.text or "").strip() if init_from_el is not None else ""
                    if image_ref:
                        surface_images[sid] = image_ref
                sampler_el = newparam_el.find(tag("sampler2D"))
                if sampler_el is not None:
                    source_el = sampler_el.find(tag("source"))
                    surface_ref = str(source_el.text or "").strip() if source_el is not None else ""
                    if surface_ref:
                        sampler_surfaces[sid] = surface_ref

            color = None
            diffuse_el = effect_el.find(f".//{tag('diffuse')}")
            if diffuse_el is not None:
                color_el = diffuse_el.find(tag("color"))
                color = parse_color(color_el.text if color_el is not None else None)
                if color is None:
                    texture_el = diffuse_el.find(tag("texture"))
                    texture_ref = str(texture_el.attrib.get("texture", "")).strip() if texture_el is not None else ""
                    surface_ref = sampler_surfaces.get(texture_ref, texture_ref)
                    image_ref = surface_images.get(surface_ref, surface_ref)
                    texture_path = image_paths.get(image_ref)
                    texture_color = texture_mean_color(texture_path)
                    if texture_color is not None:
                        effect_texture_colors[effect_id] = texture_color
            if color is None:
                emission_el = effect_el.find(f".//{tag('emission')}/{tag('color')}")
                emission_color = parse_color(emission_el.text if emission_el is not None else None)
                if emission_color is not None and float(np.max(emission_color)) > 1e-4:
                    color = emission_color
            if color is not None:
                effect_colors[effect_id] = color

        material_colors: Dict[str, np.ndarray] = {}
        for material_el in root.findall(f".//{tag('material')}"):
            material_id = str(material_el.attrib.get("id", "")).strip()
            if not material_id:
                continue
            instance_effect_el = material_el.find(f".//{tag('instance_effect')}")
            effect_ref = ""
            if instance_effect_el is not None:
                effect_ref = str(instance_effect_el.attrib.get("url", "")).strip().lstrip("#")
            if effect_ref and effect_ref in effect_colors:
                material_colors[material_id] = effect_colors[effect_ref]
            elif effect_ref and effect_ref in effect_texture_colors:
                material_colors[material_id] = effect_texture_colors[effect_ref]

        def instance_material_colors(
            instance_el: ET.Element,
        ) -> Tuple[Dict[str, np.ndarray], Optional[np.ndarray]]:
            symbol_colors: Dict[str, np.ndarray] = {}
            fallback_color = None
            for material_el in instance_el.findall(f".//{tag('instance_material')}"):
                symbol = str(material_el.attrib.get("symbol", "")).strip()
                target = str(material_el.attrib.get("target", "")).strip().lstrip("#")
                if target and target in material_colors:
                    color = material_colors[target]
                    if symbol:
                        symbol_colors[symbol] = color
                    symbol_colors[target] = color
                    if fallback_color is None:
                        fallback_color = color
            return symbol_colors, fallback_color

        def vertex_colors_from_face_materials(
            vertex_count: int,
            faces: np.ndarray,
            face_materials: List[str],
            symbol_colors: Dict[str, np.ndarray],
            fallback_color: Optional[np.ndarray],
        ) -> Optional[np.ndarray]:
            if vertex_count <= 0 or len(faces) == 0:
                return None
            if not face_materials and fallback_color is None:
                return None
            color_sums = np.zeros((vertex_count, 3), dtype=np.float64)
            color_counts = np.zeros(vertex_count, dtype=np.float64)
            for face_index, face in enumerate(faces):
                material_symbol = face_materials[face_index] if face_index < len(face_materials) else ""
                color = symbol_colors.get(material_symbol)
                if color is None:
                    color = fallback_color
                if color is None:
                    continue
                for vertex_index in face:
                    if 0 <= int(vertex_index) < vertex_count:
                        color_sums[int(vertex_index)] += color.astype(np.float64)
                        color_counts[int(vertex_index)] += 1.0
            if not np.any(color_counts > 0.0):
                return None
            safe_counts = np.where(color_counts > 0.0, color_counts, 1.0)
            colors = color_sums / safe_counts[:, None]
            if fallback_color is not None:
                missing = color_counts <= 0.0
                if np.any(missing):
                    colors[missing] = fallback_color.astype(np.float64)
            alpha = np.ones((vertex_count, 1), dtype=np.float32)
            return np.concatenate((np.clip(colors, 0.0, 1.0).astype(np.float32), alpha), axis=1)

        geometry_meshes: Dict[str, Tuple[np.ndarray, np.ndarray, List[str]]] = {}

        for geometry_el in root.findall(f".//{tag('geometry')}"):
            geometry_id = str(geometry_el.attrib.get("id", "")).strip()
            mesh_el = geometry_el.find(tag("mesh"))
            if mesh_el is None or not geometry_id:
                continue

            source_arrays: Dict[str, np.ndarray] = {}
            for source_el in mesh_el.findall(tag("source")):
                source_id = str(source_el.attrib.get("id", "")).strip()
                float_array_el = source_el.find(tag("float_array"))
                if not source_id or float_array_el is None:
                    continue
                raw_values = np.fromstring(float_array_el.text or "", sep=" ", dtype=np.float32)
                if raw_values.size < 3:
                    continue
                accessor_el = source_el.find(f".//{tag('accessor')}")
                stride = 3
                if accessor_el is not None:
                    try:
                        stride = max(1, int(accessor_el.attrib.get("stride", "3")))
                    except (TypeError, ValueError):
                        stride = 3
                usable_count = (raw_values.size // stride) * stride
                if usable_count < 3:
                    continue
                values = raw_values[:usable_count].reshape((-1, stride))[:, :3]
                source_arrays[source_id] = values.astype(np.float32, copy=False)

            vertices_to_source: Dict[str, str] = {}
            for vertices_el in mesh_el.findall(tag("vertices")):
                vertices_id = str(vertices_el.attrib.get("id", "")).strip()
                if not vertices_id:
                    continue
                for input_el in vertices_el.findall(tag("input")):
                    if str(input_el.attrib.get("semantic", "")).upper() != "POSITION":
                        continue
                    source_ref = str(input_el.attrib.get("source", "")).strip().lstrip("#")
                    if source_ref:
                        vertices_to_source[vertices_id] = source_ref

            geometry_vertices: Optional[np.ndarray] = None
            geometry_faces: List[Tuple[int, int, int]] = []
            geometry_face_materials: List[str] = []

            for primitive_name in ("triangles", "polylist"):
                for primitive_el in mesh_el.findall(tag(primitive_name)):
                    inputs = primitive_el.findall(tag("input"))
                    if not inputs:
                        continue
                    primitive_material = str(primitive_el.attrib.get("material", "")).strip()

                    stride = 1
                    vertex_offset_in_primitive = 0
                    position_source_id = ""
                    for input_el in inputs:
                        try:
                            input_offset = int(input_el.attrib.get("offset", "0"))
                        except (TypeError, ValueError):
                            input_offset = 0
                        stride = max(stride, input_offset + 1)
                        semantic = str(input_el.attrib.get("semantic", "")).upper()
                        if semantic not in {"VERTEX", "POSITION"}:
                            continue
                        vertex_offset_in_primitive = input_offset
                        source_ref = str(input_el.attrib.get("source", "")).strip().lstrip("#")
                        position_source_id = vertices_to_source.get(source_ref, source_ref)

                    if not position_source_id or position_source_id not in source_arrays:
                        continue

                    positions = source_arrays[position_source_id]
                    if geometry_vertices is None:
                        geometry_vertices = positions
                    elif geometry_vertices is not positions:
                        # Rare multi-position-source geometry. Keep it simple by appending this source.
                        local_base = len(geometry_vertices)
                        geometry_vertices = np.vstack((geometry_vertices, positions))
                    else:
                        local_base = 0
                    if geometry_vertices is positions:
                        local_base = 0

                    p_el = primitive_el.find(tag("p"))
                    if p_el is None or not (p_el.text or "").strip():
                        continue
                    raw_indices = np.fromstring(p_el.text or "", sep=" ", dtype=np.int64)
                    if raw_indices.size < stride * 3:
                        continue
                    tuples = raw_indices[: (raw_indices.size // stride) * stride].reshape((-1, stride))
                    vertex_indices = tuples[:, vertex_offset_in_primitive].astype(np.int64, copy=False) + int(local_base)

                    if primitive_name == "triangles":
                        usable = (vertex_indices.size // 3) * 3
                        for tri in vertex_indices[:usable].reshape((-1, 3)):
                            geometry_faces.append((int(tri[0]), int(tri[1]), int(tri[2])))
                            geometry_face_materials.append(primitive_material)
                    else:
                        vcount_el = primitive_el.find(tag("vcount"))
                        if vcount_el is None or not (vcount_el.text or "").strip():
                            continue
                        counts = np.fromstring(vcount_el.text or "", sep=" ", dtype=np.int64)
                        cursor = 0
                        for count in counts:
                            count_value = int(count)
                            if count_value < 3 or cursor + count_value > len(vertex_indices):
                                cursor += max(0, count_value)
                                continue
                            polygon = [int(value) for value in vertex_indices[cursor: cursor + count_value]]
                            triangles = _triangulate_face(polygon)
                            geometry_faces.extend(triangles)
                            geometry_face_materials.extend([primitive_material] * len(triangles))
                            cursor += count_value

            if geometry_vertices is None or not geometry_faces:
                continue

            geometry_meshes[geometry_id] = (
                geometry_vertices.astype(np.float32, copy=False),
                np.asarray(geometry_faces, dtype=np.int32),
                geometry_face_materials,
            )

        if not geometry_meshes:
            return None

        instances: List[Tuple[str, np.ndarray, Dict[str, np.ndarray], Optional[np.ndarray]]] = []

        def visit_node(node_el: ET.Element, parent_matrix: np.ndarray) -> None:
            node_matrix = parent_matrix @ local_node_matrix(node_el)
            for instance_el in node_el.findall(tag("instance_geometry")):
                geometry_ref = str(instance_el.attrib.get("url", "")).strip().lstrip("#")
                if geometry_ref:
                    symbol_colors, fallback_color = instance_material_colors(instance_el)
                    instances.append((geometry_ref, node_matrix.copy(), symbol_colors, fallback_color))
            for child_node_el in node_el.findall(tag("node")):
                visit_node(child_node_el, node_matrix)

        scene_root = root.find(f".//{tag('library_visual_scenes')}/{tag('visual_scene')}")
        if scene_root is not None:
            for node_el in scene_root.findall(tag("node")):
                visit_node(node_el, np.eye(4, dtype=np.float32))

        if not instances:
            instances = [
                (geometry_id, np.eye(4, dtype=np.float32), {}, None)
                for geometry_id in geometry_meshes.keys()
            ]

        all_vertices: List[np.ndarray] = []
        all_faces: List[np.ndarray] = []
        all_colors: List[np.ndarray] = []
        color_accumulator = np.zeros(3, dtype=np.float64)
        color_samples = 0
        vertex_offset = 0
        for geometry_id, transform, symbol_colors, fallback_color in instances:
            source_mesh = geometry_meshes.get(geometry_id)
            if source_mesh is None:
                continue
            source_vertices, source_faces, source_face_materials = source_mesh
            homogeneous = np.concatenate(
                (
                    source_vertices.astype(np.float32, copy=False),
                    np.ones((len(source_vertices), 1), dtype=np.float32),
                ),
                axis=1,
            )
            transformed = (homogeneous @ transform.T)[:, :3].astype(np.float32, copy=False)
            all_vertices.append(transformed)
            all_faces.append(source_faces.astype(np.int32, copy=False) + vertex_offset)
            vertex_offset += int(len(transformed))
            source_colors = vertex_colors_from_face_materials(
                len(source_vertices),
                source_faces,
                source_face_materials,
                symbol_colors,
                fallback_color,
            )
            if source_colors is not None and len(source_colors) == len(source_vertices):
                color_accumulator += source_colors[:, :3].mean(axis=0).astype(np.float64)
                color_samples += 1
                all_colors.append(source_colors.astype(np.float32, copy=False))

        if not all_vertices or not all_faces:
            return None

        vertices = np.vstack(all_vertices).astype(np.float32, copy=False)
        faces = np.vstack(all_faces).astype(np.int32, copy=False)
        colors = None
        if all_colors and sum(len(color_rows) for color_rows in all_colors) == len(vertices):
            colors = np.vstack(all_colors).astype(np.float32, copy=False)
        mean_color = None
        if color_samples > 0:
            mean_color = np.clip(color_accumulator / float(color_samples), 0.0, 1.0).astype(np.float32)
        return SourceMeshData(vertices=vertices, faces=faces, normals=None, color_rgb=mean_color, colors=colors)

    def _decimate_source_mesh(self, source_mesh: SourceMeshData, target_faces: int) -> SourceMeshData:
        face_count = int(len(source_mesh.faces))
        if target_faces <= 0 or face_count <= target_faces:
            return source_mesh

        vertices = source_mesh.vertices.astype(np.float32, copy=False)
        faces = source_mesh.faces.astype(np.int32, copy=False)
        bounds_min = vertices.min(axis=0)
        bounds_max = vertices.max(axis=0)
        extent = np.maximum(bounds_max - bounds_min, 1e-6)

        max_extent = float(np.max(extent))
        base_resolution = max(8, int(math.ceil(math.sqrt(float(max(1, target_faces))) * 1.5)))
        grid = np.maximum(
            2,
            np.ceil((extent / max(max_extent, 1e-6)) * float(base_resolution)).astype(np.int64),
        )

        quantized = np.floor((vertices - bounds_min[None, :]) / (extent[None, :] / grid[None, :])).astype(np.int64)
        quantized = np.minimum(np.maximum(quantized, 0), grid[None, :] - 1)
        cell_keys = (
            (quantized[:, 0] * (grid[1] * grid[2]))
            + (quantized[:, 1] * grid[2])
            + quantized[:, 2]
        )
        unique_keys, inverse = np.unique(cell_keys, return_inverse=True)
        clustered_vertices = np.zeros((len(unique_keys), 3), dtype=np.float64)
        np.add.at(clustered_vertices, inverse, vertices.astype(np.float64, copy=False))
        counts = np.bincount(inverse, minlength=len(unique_keys)).astype(np.float64)
        counts[counts <= 0.0] = 1.0
        clustered_vertices = (clustered_vertices / counts[:, None]).astype(np.float32, copy=False)
        clustered_colors = None
        if source_mesh.colors is not None and len(source_mesh.colors) == len(vertices):
            raw_colors = np.asarray(source_mesh.colors, dtype=np.float32)
            if raw_colors.ndim == 2 and raw_colors.shape[1] >= 3:
                color_width = 4 if raw_colors.shape[1] >= 4 else 3
                clustered_colors = np.zeros((len(unique_keys), color_width), dtype=np.float64)
                np.add.at(clustered_colors, inverse, raw_colors[:, :color_width].astype(np.float64, copy=False))
                clustered_colors = (clustered_colors / counts[:, None]).astype(np.float32, copy=False)

        remapped_faces = inverse[faces].astype(np.int32, copy=False)
        non_degenerate = (
            (remapped_faces[:, 0] != remapped_faces[:, 1])
            & (remapped_faces[:, 1] != remapped_faces[:, 2])
            & (remapped_faces[:, 0] != remapped_faces[:, 2])
        )
        remapped_faces = remapped_faces[non_degenerate]
        if len(remapped_faces) == 0:
            return source_mesh

        if len(remapped_faces) > target_faces:
            rng = np.random.default_rng(0)
            keep = np.sort(rng.choice(len(remapped_faces), size=int(target_faces), replace=False))
            remapped_faces = remapped_faces[keep]

        unique_vertices, compact_inverse = np.unique(remapped_faces.reshape(-1), return_inverse=True)
        vertices = clustered_vertices[unique_vertices].astype(np.float32, copy=False)
        faces = compact_inverse.reshape((-1, 3)).astype(np.int32, copy=False)
        normals = self._compute_vertex_normals(vertices, faces)
        colors = None
        if clustered_colors is not None:
            colors = np.clip(clustered_colors[unique_vertices], 0.0, 1.0).astype(np.float32, copy=False)
        return SourceMeshData(
            vertices=vertices,
            faces=faces,
            normals=normals,
            color_rgb=source_mesh.color_rgb,
            colors=colors,
        )

    def _parse_obj_mesh(
        self,
        source_path: Path,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        vertices: List[Tuple[float, float, float]] = []
        vertex_colors: List[Tuple[float, float, float, float]] = []
        faces: List[Tuple[int, int, int]] = []
        color_rgb: Optional[np.ndarray] = None
        material_colors: Dict[str, np.ndarray] = {}
        active_material: Optional[str] = None

        with source_path.open("r", encoding="utf-8", errors="ignore") as handle:
            for raw_line in handle:
                if not raw_line:
                    continue
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("v "):
                    parts = line.split()
                    if len(parts) >= 4:
                        vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
                        if len(parts) >= 7:
                            vertex_colors.append(
                                (
                                    float(parts[4]),
                                    float(parts[5]),
                                    float(parts[6]),
                                    float(parts[7]) if len(parts) >= 8 else 1.0,
                                )
                            )
                        else:
                            vertex_colors.append((math.nan, math.nan, math.nan, math.nan))
                    continue
                if line.startswith("mtllib "):
                    mtllib_name = line.split(maxsplit=1)[1].strip()
                    material_colors.update(self._parse_mtl_colors(source_path.parent / mtllib_name))
                    continue
                if line.startswith("usemtl "):
                    active_material = line.split(maxsplit=1)[1].strip()
                    if color_rgb is None and active_material in material_colors:
                        color_rgb = material_colors[active_material]
                    continue
                if line.startswith("f "):
                    tokens = line.split()[1:]
                    face_indices: List[int] = []
                    for token in tokens:
                        head = token.split("/")[0]
                        if not head:
                            continue
                        index = int(head)
                        if index < 0:
                            index = len(vertices) + index + 1
                        face_indices.append(index - 1)
                    faces.extend(_triangulate_face(face_indices))

        if not vertices or not faces:
            return None, None, None, None

        colors = None
        if len(vertex_colors) == len(vertices):
            raw_colors = np.asarray(vertex_colors, dtype=np.float32)
            if raw_colors.ndim == 2 and raw_colors.shape[1] >= 4:
                valid = np.isfinite(raw_colors[:, :3]).all(axis=1)
                if np.any(valid):
                    fallback = color_rgb if color_rgb is not None else np.asarray([0.78, 0.78, 0.76], dtype=np.float32)
                    raw_colors[~valid, :3] = fallback[:3]
                    raw_colors[~valid, 3] = 1.0
                    colors = np.clip(raw_colors[:, :4], 0.0, 1.0).astype(np.float32, copy=False)
                    if color_rgb is None:
                        color_rgb = np.clip(colors[:, :3].mean(axis=0), 0.0, 1.0).astype(np.float32, copy=False)

        return (
            np.asarray(vertices, dtype=np.float32),
            np.asarray(faces, dtype=np.int32),
            color_rgb.astype(np.float32, copy=False) if color_rgb is not None else None,
            colors,
        )

    def _parse_mtl_colors(self, mtl_path: Path) -> Dict[str, np.ndarray]:
        if not mtl_path.is_file():
            return {}

        material_colors: Dict[str, np.ndarray] = {}
        current_name: Optional[str] = None
        with mtl_path.open("r", encoding="utf-8", errors="ignore") as handle:
            for raw_line in handle:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("newmtl "):
                    current_name = line.split(maxsplit=1)[1].strip()
                    continue
                if line.startswith("Kd ") and current_name:
                    parts = line.split()
                    if len(parts) >= 4:
                        try:
                            material_colors[current_name] = np.asarray(
                                [float(parts[1]), float(parts[2]), float(parts[3])],
                                dtype=np.float32,
                            )
                        except Exception:
                            logger.debug("Ignoring malformed Kd material color in %s: %s", mtl_path, line, exc_info=True)
        return material_colors

    def _simplify_mesh(
        self,
        vertices: np.ndarray,
        faces: np.ndarray,
        target_faces: int,
    ) -> Tuple[np.ndarray, np.ndarray]:
        if len(faces) <= target_faces or len(vertices) == 0:
            return vertices.astype(np.float32, copy=False), faces.astype(np.int32, copy=False)

        bounds_min = vertices.min(axis=0)
        bounds_max = vertices.max(axis=0)
        extent = np.maximum(bounds_max - bounds_min, 1e-6)
        diagonal = float(np.linalg.norm(extent))
        low = diagonal / 8192.0
        high = diagonal / 8.0
        best_vertices = vertices.astype(np.float32, copy=False)
        best_faces = faces.astype(np.int32, copy=False)

        for _ in range(12):
            cell = max(low, (low + high) * 0.5)
            quantized = np.floor((vertices - bounds_min[None, :]) / cell + 0.5).astype(np.int64)
            unique_cells, inverse = np.unique(quantized, axis=0, return_inverse=True)
            if len(unique_cells) < 3:
                high = cell
                continue

            counts = np.bincount(inverse)
            collapsed = np.zeros((len(unique_cells), 3), dtype=np.float64)
            for axis in range(3):
                collapsed[:, axis] = np.bincount(inverse, weights=vertices[:, axis]) / np.maximum(counts, 1)

            remapped_faces = inverse[faces]
            degenerate_mask = (
                (remapped_faces[:, 0] != remapped_faces[:, 1]) &
                (remapped_faces[:, 1] != remapped_faces[:, 2]) &
                (remapped_faces[:, 0] != remapped_faces[:, 2])
            )
            remapped_faces = remapped_faces[degenerate_mask]
            if len(remapped_faces) == 0:
                high = cell
                continue

            canonical = np.sort(remapped_faces, axis=1)
            _, unique_indices = np.unique(canonical, axis=0, return_index=True)
            remapped_faces = remapped_faces[np.sort(unique_indices)]

            if len(remapped_faces) > target_faces:
                low = cell
                continue

            best_vertices = collapsed.astype(np.float32)
            best_faces = remapped_faces.astype(np.int32)
            high = cell

        return best_vertices, best_faces

    def _compute_vertex_normals(self, vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
        if len(vertices) == 0 or len(faces) == 0:
            return np.zeros((len(vertices), 3), dtype=np.float32)

        triangles = vertices[faces]
        face_normals = np.cross(
            triangles[:, 1] - triangles[:, 0],
            triangles[:, 2] - triangles[:, 0],
        )
        face_normals = _normalize_vector_rows(face_normals)
        vertex_normals = np.zeros((len(vertices), 3), dtype=np.float32)
        for corner in range(3):
            np.add.at(vertex_normals, faces[:, corner], face_normals)
        return _normalize_vector_rows(vertex_normals)

    @staticmethod
    def _hash_file(source_path: Path) -> str:
        digest = hashlib.sha256()
        with source_path.open("rb") as handle:
            while True:
                chunk = handle.read(1024 * 1024)
                if not chunk:
                    break
                digest.update(chunk)
        return digest.hexdigest()
