"""ctypes wrapper for the native CUDA triangle renderer."""

from __future__ import annotations

import ctypes
import hashlib
import os
from pathlib import Path
import shutil
import subprocess

import numpy as np


class CudaMeshRendererError(RuntimeError):
    pass


def _resolve_nvcc() -> str | None:
    compiler = shutil.which("nvcc")
    if compiler:
        return compiler
    candidates: list[Path] = []
    for variable in ("CUDA_HOME", "CUDA_PATH"):
        root = os.environ.get(variable)
        if root:
            candidates.append(Path(root) / "bin" / "nvcc")
    candidates.extend((Path("/usr/local/cuda/bin/nvcc"), Path("/opt/cuda/bin/nvcc")))
    candidates.extend(sorted(Path("/usr/local").glob("cuda-*/bin/nvcc"), reverse=True))
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None


def build_cuda_mesh_renderer_library() -> Path:
    source = Path(__file__).with_name("cuda_mesh_renderer.cu")
    if not source.is_file():
        raise CudaMeshRendererError(f"CUDA mesh renderer source is missing: {source}")
    compiler = _resolve_nvcc()
    if not compiler:
        raise CudaMeshRendererError("nvcc is required for remote mesh rendering")
    digest = hashlib.sha256(source.read_bytes()).hexdigest()[:16]
    cache_root = Path.home() / ".cache" / "horus" / "remote_renderer"
    library = cache_root / f"libhorus_cuda_mesh_{digest}.so"
    if library.is_file():
        return library
    cache_root.mkdir(parents=True, exist_ok=True)
    temporary = library.with_suffix(".so.part")
    command = [
        compiler,
        "-O3",
        "--shared",
        "-Xcompiler=-fPIC",
        "-arch=native",
        str(source),
        "-o",
        str(temporary),
    ]
    completed = subprocess.run(command, text=True, capture_output=True, check=False)
    if completed.returncode != 0:
        raise CudaMeshRendererError(
            "CUDA mesh renderer compilation failed:\n"
            + (completed.stderr or completed.stdout)
        )
    temporary.replace(library)
    return library


class CudaMeshRenderer:
    """Rasterize an indexed triangle mesh into exact stereo color and depth."""

    def __init__(
        self,
        vertices: np.ndarray,
        faces: np.ndarray,
        colors: np.ndarray,
        *,
        uvs: np.ndarray | None = None,
        texture_atlas: np.ndarray | None = None,
        texture_rects: np.ndarray | None = None,
    ) -> None:
        vertices = np.ascontiguousarray(vertices, dtype=np.float32)
        faces = np.ascontiguousarray(faces, dtype=np.uint32)
        colors = np.ascontiguousarray(colors, dtype=np.uint8)
        if vertices.ndim != 2 or vertices.shape[1] != 3:
            raise ValueError("vertices must have shape (N, 3)")
        if faces.ndim != 2 or faces.shape[1] != 3:
            raise ValueError("faces must have shape (M, 3)")
        if colors.shape != (len(faces), 3):
            raise ValueError("colors must have shape (M, 3)")
        if len(vertices) == 0 or len(faces) == 0:
            raise ValueError("mesh must contain vertices and faces")
        if int(faces.max()) >= len(vertices):
            raise ValueError("face index exceeds the vertex buffer")
        if uvs is None:
            uvs = np.zeros((len(vertices), 2), dtype=np.float32)
        uvs = np.ascontiguousarray(uvs, dtype=np.float32)
        if uvs.shape != (len(vertices), 2):
            raise ValueError("uvs must have shape (N, 2)")
        if texture_atlas is None:
            texture_atlas = np.full((1, 1, 3), 255, dtype=np.uint8)
        texture_atlas = np.ascontiguousarray(texture_atlas, dtype=np.uint8)
        if texture_atlas.ndim != 3 or texture_atlas.shape[2] != 3:
            raise ValueError("texture_atlas must have shape (H, W, 3)")
        if texture_rects is None:
            texture_rects = np.repeat(
                np.asarray(((0, 0, 1, 1),), dtype=np.uint32),
                len(faces),
                axis=0,
            )
        texture_rects = np.ascontiguousarray(texture_rects, dtype=np.uint32)
        if texture_rects.shape != (len(faces), 4):
            raise ValueError("texture_rects must have shape (M, 4)")
        atlas_height, atlas_width = texture_atlas.shape[:2]
        if (
            np.any(texture_rects[:, 2:] == 0)
            or np.any(texture_rects[:, 0] + texture_rects[:, 2] > atlas_width)
            or np.any(texture_rects[:, 1] + texture_rects[:, 3] > atlas_height)
        ):
            raise ValueError("texture rectangle exceeds the atlas")

        self.vertex_count = len(vertices)
        self.face_count = len(faces)
        self._library = ctypes.CDLL(str(build_cuda_mesh_renderer_library()))
        self._configure_api()
        self._handle = ctypes.c_void_p()
        self._check(
            self._library.horus_cuda_mesh_create(
                self.vertex_count,
                self.face_count,
                vertices.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                faces.ctypes.data_as(ctypes.POINTER(ctypes.c_uint)),
                uvs.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                colors.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte)),
                texture_rects.ctypes.data_as(ctypes.POINTER(ctypes.c_uint)),
                texture_atlas.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte)),
                int(atlas_width),
                int(atlas_height),
                ctypes.byref(self._handle),
            ),
            "create",
        )

    def _configure_api(self) -> None:
        library = self._library
        library.horus_cuda_mesh_last_error.restype = ctypes.c_char_p
        library.horus_cuda_mesh_create.argtypes = [
            ctypes.c_size_t,
            ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_uint),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_ubyte),
            ctypes.POINTER(ctypes.c_uint),
            ctypes.POINTER(ctypes.c_ubyte),
            ctypes.c_int,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_void_p),
        ]
        library.horus_cuda_mesh_create.restype = ctypes.c_int
        library.horus_cuda_mesh_render_views.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.POINTER(ctypes.c_ubyte),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
        ]
        library.horus_cuda_mesh_render_views.restype = ctypes.c_int
        library.horus_cuda_mesh_destroy.argtypes = [ctypes.c_void_p]
        library.horus_cuda_mesh_destroy.restype = None

    def _check(self, code: int, operation: str) -> None:
        if int(code) == 0:
            return
        raw = self._library.horus_cuda_mesh_last_error()
        detail = raw.decode("utf-8", "replace") if raw else "unknown CUDA error"
        raise CudaMeshRendererError(f"CUDA mesh renderer {operation} failed: {detail}")

    def render_views(
        self,
        camera_positions,
        camera_rotations,
        width: int,
        height: int,
        *,
        vertical_fov_deg: float,
        projections=None,
        near_m: float,
        far_m: float,
        point_radius: int = 0,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        del point_radius
        from .dynamic_stream import quaternion_to_matrix

        positions = np.ascontiguousarray(camera_positions, dtype=np.float32)
        rotations = np.ascontiguousarray(camera_rotations, dtype=np.float32)
        if positions.ndim != 2 or positions.shape[1] != 3:
            raise ValueError("camera_positions must have shape (N, 3)")
        if rotations.shape != (len(positions), 4):
            raise ValueError("camera_rotations must have shape (N, 4)")
        world_to_camera = np.ascontiguousarray(
            np.stack(
                [quaternion_to_matrix(rotation).T for rotation in rotations],
                axis=0,
            ),
            dtype=np.float32,
        )
        if projections is None:
            tangent = np.tan(np.deg2rad(vertical_fov_deg) * 0.5)
            projection_y = 1.0 / tangent
            projection_x = projection_y / (float(width) / float(height))
            projections = [(projection_x, projection_y, 0.0, 0.0)] * len(positions)
        projections = np.ascontiguousarray(projections, dtype=np.float32)
        if projections.shape != (len(positions), 4):
            raise ValueError("projections must have shape (N, 4)")
        color = np.empty((len(positions), int(height), int(width), 3), dtype=np.uint8)
        depth = np.empty((len(positions), int(height), int(width)), dtype=np.float32)
        elapsed = ctypes.c_float()
        self._check(
            self._library.horus_cuda_mesh_render_views(
                self._handle,
                positions.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                world_to_camera.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                projections.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                len(positions),
                int(width),
                int(height),
                float(vertical_fov_deg),
                float(near_m),
                float(far_m),
                color.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte)),
                depth.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                ctypes.byref(elapsed),
            ),
            "render",
        )
        return color, depth, float(elapsed.value)

    def close(self) -> None:
        if getattr(self, "_handle", None) and self._handle.value:
            self._library.horus_cuda_mesh_destroy(self._handle)
            self._handle = ctypes.c_void_p()

    def __enter__(self) -> "CudaMeshRenderer":
        return self

    def __exit__(self, *_args) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
