"""Mesh normalization and baking helpers for robot-description visual meshes."""

from __future__ import annotations

import base64
import hashlib
import json
import math
import os
import shutil
import struct
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from .robot_description_models import MeshAsset


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
            source_color = None
            if isinstance(entry_color, (list, tuple)) and len(entry_color) >= 3:
                source_color = np.asarray(entry_color[:3], dtype=np.float32)
            elif source_mesh.color_rgb is not None and len(source_mesh.color_rgb) >= 3:
                source_color = np.asarray(source_mesh.color_rgb[:3], dtype=np.float32)

            if source_color is None:
                source_color = np.asarray([0.78, 0.78, 0.76], dtype=np.float32)
            else:
                color_accumulator += source_color.astype(np.float64)
                color_samples += 1

            entry_colors = np.tile(np.concatenate([np.clip(source_color, 0.0, 1.0), np.array([1.0], dtype=np.float32)]), (len(world_vertices), 1))
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
            normalized_path = self._normalize_to_obj_if_needed(source_path)
            if normalized_path is None or not normalized_path.is_file():
                return None

            vertices, faces, color_rgb = self._parse_obj_mesh(normalized_path)
            if vertices is None or faces is None or len(vertices) == 0 or len(faces) == 0:
                return None

            cached = SourceMeshData(vertices=vertices, faces=faces, normals=None, color_rgb=color_rgb)
            self._source_cache[cache_key] = cached

        if target_faces is None or len(cached.faces) <= int(target_faces):
            return cached

        decimated_key = f"{cache_key}:decimate:{int(target_faces)}"
        decimated = self._decimated_cache.get(decimated_key)
        if decimated is not None:
            return decimated

        normalized_path = self._normalize_to_obj_if_needed(source_path)
        if normalized_path is None or not normalized_path.is_file():
            return cached

        decimated_path = self._decimate_obj_with_blender(normalized_path, int(target_faces))
        if decimated_path is None or not decimated_path.is_file():
            return cached

        vertices, faces, _ = self._parse_obj_mesh(decimated_path)
        if vertices is None or faces is None or len(vertices) == 0 or len(faces) == 0:
            return cached

        decimated = SourceMeshData(vertices=vertices, faces=faces, normals=None, color_rgb=cached.color_rgb)
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
            pass
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

    def _parse_obj_mesh(
        self,
        source_path: Path,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        vertices: List[Tuple[float, float, float]] = []
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
            return None, None, None

        return (
            np.asarray(vertices, dtype=np.float32),
            np.asarray(faces, dtype=np.int32),
            color_rgb.astype(np.float32, copy=False) if color_rgb is not None else None,
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
                            pass
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
