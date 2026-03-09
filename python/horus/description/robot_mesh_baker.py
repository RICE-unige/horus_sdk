"""Mesh normalization and baking helpers for robot-description visual meshes."""

from __future__ import annotations

import base64
import hashlib
import json
import math
import os
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

    _DEFAULT_TOTAL_TRIANGLE_BUDGET = 36000
    _DEFAULT_MAX_TRIANGLES_PER_ASSET = 16000

    def __init__(self, cache_root: Optional[str] = None):
        self._cache_root = Path(
            cache_root or os.path.expanduser("~/.cache/horus/robot_description_meshes")
        )
        self._cache_root.mkdir(parents=True, exist_ok=True)
        self._source_cache: Dict[str, SourceMeshData] = {}

    def build_combined_asset(
        self,
        mesh_entries: List[Dict[str, object]],
        description_id_seed: str,
        target_triangles: int = _DEFAULT_TOTAL_TRIANGLE_BUDGET,
    ) -> Optional[Tuple[MeshAsset, List[float], List[float]]]:
        if not mesh_entries:
            return None

        combined_vertices: List[np.ndarray] = []
        combined_faces: List[np.ndarray] = []
        combined_normals: List[np.ndarray] = []
        color_accumulator = np.zeros(3, dtype=np.float64)
        color_samples = 0
        vertex_offset = 0

        target_triangles = max(1000, int(target_triangles))
        per_entry_budget = max(
            512,
            min(
                self._DEFAULT_MAX_TRIANGLES_PER_ASSET,
                int(math.ceil(target_triangles / max(1, len(mesh_entries)))),
            ),
        )

        for entry in mesh_entries:
            source_uri = str(entry.get("mesh_uri") or "").strip()
            if not source_uri:
                continue

            source_path = Path(str(entry.get("mesh_path") or "")).resolve()
            if not source_path.is_file():
                continue

            source_mesh = self._load_source_mesh(source_path)
            if source_mesh is None:
                continue

            vertices = source_mesh.vertices
            faces = source_mesh.faces
            normals = source_mesh.normals

            if len(faces) > per_entry_budget:
                vertices, faces = self._simplify_mesh(vertices, faces, per_entry_budget)
                normals = None

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
            if isinstance(entry_color, (list, tuple)) and len(entry_color) >= 3:
                color_accumulator += np.asarray(entry_color[:3], dtype=np.float64)
                color_samples += 1

        if not combined_vertices or not combined_faces:
            return None

        vertices = np.concatenate(combined_vertices, axis=0)
        faces = np.concatenate(combined_faces, axis=0)
        normals = _normalize_vector_rows(np.concatenate(combined_normals, axis=0))
        bounds_min = vertices.min(axis=0)
        bounds_max = vertices.max(axis=0)
        mean_color = None
        if color_samples > 0:
            mean_color = np.clip(color_accumulator / float(color_samples), 0.0, 1.0).astype(np.float32)

        payload_seed = {
            "description_id_seed": description_id_seed,
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
        )
        return asset, _float_list(bounds_min.astype(np.float32)), _float_list(bounds_max.astype(np.float32))

    def _load_source_mesh(self, source_path: Path) -> Optional[SourceMeshData]:
        source_hash = self._hash_file(source_path)
        cache_key = f"{source_path.suffix.lower()}:{source_hash}"
        cached = self._source_cache.get(cache_key)
        if cached is not None:
            return cached

        normalized_path = self._normalize_to_obj_if_needed(source_path)
        if normalized_path is None or not normalized_path.is_file():
            return None

        vertices, faces = self._parse_obj_mesh(normalized_path)
        if vertices is None or faces is None or len(vertices) == 0 or len(faces) == 0:
            return None

        source_mesh = SourceMeshData(vertices=vertices, faces=faces, normals=None)
        self._source_cache[cache_key] = source_mesh
        return source_mesh

    def _normalize_to_obj_if_needed(self, source_path: Path) -> Optional[Path]:
        suffix = source_path.suffix.lower()
        if suffix == ".obj":
            return source_path
        if suffix == ".stl":
            return self._convert_stl_to_obj(source_path)

        blender_path = os.environ.get("HORUS_SDK_BLENDER_PATH", "").strip()
        if not blender_path or not os.path.isfile(blender_path):
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
                        "bpy.ops.wm.obj_import(filepath=src) if src.lower().endswith('.obj') else bpy.ops.wm.collada_import(filepath=src)",
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
                    str(script_path),
                    "--",
                    str(source_path),
                    str(output_path),
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

    def _parse_obj_mesh(self, source_path: Path) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        vertices: List[Tuple[float, float, float]] = []
        faces: List[Tuple[int, int, int]] = []

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
            return None, None

        return (
            np.asarray(vertices, dtype=np.float32),
            np.asarray(faces, dtype=np.int32),
        )

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
