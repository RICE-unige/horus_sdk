"""ctypes wrapper for the native CUDA colored-point renderer."""

from __future__ import annotations

import ctypes
import hashlib
import os
from pathlib import Path
import shutil
import subprocess

import numpy as np


class CudaRendererError(RuntimeError):
    pass


def _resolve_nvcc() -> str | None:
    compiler = shutil.which("nvcc")
    if compiler:
        return compiler
    candidates = []
    for variable in ("CUDA_HOME", "CUDA_PATH"):
        root = os.environ.get(variable)
        if root:
            candidates.append(Path(root) / "bin" / "nvcc")
    candidates.extend(
        (
            Path("/usr/local/cuda/bin/nvcc"),
            Path("/opt/cuda/bin/nvcc"),
        )
    )
    candidates.extend(sorted(Path("/usr/local").glob("cuda-*/bin/nvcc"), reverse=True))
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None


def build_cuda_renderer_library() -> Path:
    source = Path(__file__).with_name("cuda_point_renderer.cu")
    if not source.is_file():
        raise CudaRendererError(f"CUDA renderer source is missing: {source}")
    compiler = _resolve_nvcc()
    if not compiler:
        raise CudaRendererError("nvcc is required for dynamic remote-map rendering")
    digest = hashlib.sha256(source.read_bytes()).hexdigest()[:16]
    cache_root = Path.home() / ".cache" / "horus" / "remote_renderer"
    library = cache_root / f"libhorus_cuda_points_{digest}.so"
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
        raise CudaRendererError(
            "CUDA renderer compilation failed:\n" + (completed.stderr or completed.stdout)
        )
    temporary.replace(library)
    return library


class CudaPointRenderer:
    def __init__(self, point_count: int) -> None:
        self.point_count = int(point_count)
        if self.point_count <= 0:
            raise ValueError("point_count must be positive")
        self._library = ctypes.CDLL(str(build_cuda_renderer_library()))
        self._configure_api()
        self._handle = ctypes.c_void_p()
        self._check(
            self._library.horus_cuda_create(self.point_count, ctypes.byref(self._handle)),
            "create",
        )
        self._uploaded = 0

    def _configure_api(self) -> None:
        library = self._library
        library.horus_cuda_last_error.restype = ctypes.c_char_p
        library.horus_cuda_create.argtypes = [ctypes.c_size_t, ctypes.POINTER(ctypes.c_void_p)]
        library.horus_cuda_create.restype = ctypes.c_int
        library.horus_cuda_upload.argtypes = [
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_ubyte),
        ]
        library.horus_cuda_upload.restype = ctypes.c_int
        library.horus_cuda_render.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_ubyte),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
        ]
        library.horus_cuda_render.restype = ctypes.c_int
        library.horus_cuda_render_views.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_ubyte),
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
        ]
        library.horus_cuda_render_views.restype = ctypes.c_int
        library.horus_pack_views.argtypes = [
            ctypes.POINTER(ctypes.c_ubyte),
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.POINTER(ctypes.c_ubyte),
        ]
        library.horus_pack_views.restype = ctypes.c_int
        library.horus_cuda_destroy.argtypes = [ctypes.c_void_p]
        library.horus_cuda_destroy.restype = None

    def _check(self, code: int, operation: str) -> None:
        if int(code) == 0:
            return
        raw = self._library.horus_cuda_last_error()
        detail = raw.decode("utf-8", "replace") if raw else "unknown CUDA error"
        raise CudaRendererError(f"CUDA renderer {operation} failed: {detail}")

    def upload(self, points: np.ndarray, colors: np.ndarray, offset: int | None = None) -> None:
        points = np.ascontiguousarray(points, dtype=np.float32)
        colors = np.ascontiguousarray(colors, dtype=np.uint8)
        if points.ndim != 2 or points.shape[1] != 3 or colors.shape != points.shape:
            raise ValueError("points and colors must both have shape (N, 3)")
        start = self._uploaded if offset is None else int(offset)
        if start < 0 or start + len(points) > self.point_count:
            raise ValueError("CUDA upload exceeds the allocated point buffer")
        self._check(
            self._library.horus_cuda_upload(
                self._handle,
                start,
                len(points),
                points.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                colors.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte)),
            ),
            "upload",
        )
        self._uploaded = max(self._uploaded, start + len(points))

    def render(
        self,
        camera_position,
        camera_rotation,
        width: int,
        height: int,
        *,
        vertical_fov_deg: float,
        near_m: float,
        far_m: float,
        point_radius: int = 1,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        if self._uploaded != self.point_count:
            raise CudaRendererError(
                f"renderer contains {self._uploaded:,}/{self.point_count:,} uploaded points"
            )
        from .dynamic_stream import quaternion_to_matrix

        position = np.ascontiguousarray(camera_position, dtype=np.float32)
        if position.shape != (3,):
            raise ValueError("camera_position must contain three values")
        camera_to_world = quaternion_to_matrix(camera_rotation)
        world_to_camera = np.ascontiguousarray(camera_to_world.T, dtype=np.float32)
        color = np.empty((int(height), int(width), 3), dtype=np.uint8)
        depth = np.empty((int(height), int(width)), dtype=np.float32)
        elapsed = ctypes.c_float()
        self._check(
            self._library.horus_cuda_render(
                self._handle,
                position.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                world_to_camera.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                int(width),
                int(height),
                float(vertical_fov_deg),
                float(near_m),
                float(far_m),
                int(point_radius),
                color.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte)),
                depth.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                ctypes.byref(elapsed),
            ),
            "render",
        )
        return color, depth, float(elapsed.value)

    def render_views(
        self,
        camera_positions,
        camera_rotations,
        width: int,
        height: int,
        *,
        vertical_fov_deg: float,
        near_m: float,
        far_m: float,
        point_radius: int = 1,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        if self._uploaded != self.point_count:
            raise CudaRendererError(
                f"renderer contains {self._uploaded:,}/{self.point_count:,} uploaded points"
            )
        from .dynamic_stream import quaternion_to_matrix

        positions = np.ascontiguousarray(camera_positions, dtype=np.float32)
        rotations = np.ascontiguousarray(camera_rotations, dtype=np.float32)
        if positions.ndim != 2 or positions.shape[1] != 3:
            raise ValueError("camera_positions must have shape (N, 3)")
        if rotations.shape != (len(positions), 4):
            raise ValueError("camera_rotations must have shape (N, 4)")
        if not 1 <= len(positions) <= 12:
            raise ValueError("between one and twelve camera views are supported")
        matrices = np.ascontiguousarray(
            np.stack([quaternion_to_matrix(rotation).T for rotation in rotations]),
            dtype=np.float32,
        )
        color = np.empty((len(positions), int(height), int(width), 3), dtype=np.uint8)
        depth = np.empty((len(positions), int(height), int(width)), dtype=np.float32)
        elapsed = ctypes.c_float()
        self._check(
            self._library.horus_cuda_render_views(
                self._handle,
                positions.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                matrices.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                len(positions),
                int(width),
                int(height),
                float(vertical_fov_deg),
                float(near_m),
                float(far_m),
                int(point_radius),
                color.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte)),
                depth.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                ctypes.byref(elapsed),
            ),
            "multiview render",
        )
        return color, depth, float(elapsed.value)

    def pack_views(
        self,
        colors: np.ndarray,
        depth: np.ndarray,
        *,
        near_m: float,
        far_m: float,
    ) -> np.ndarray:
        colors = np.ascontiguousarray(colors, dtype=np.uint8)
        depth = np.ascontiguousarray(depth, dtype=np.float32)
        if colors.ndim != 4 or colors.shape[0] != 4 or colors.shape[3] != 3:
            raise ValueError("colors must have shape (4, H, W, 3)")
        if depth.shape != colors.shape[:3]:
            raise ValueError("depth must have shape (4, H, W)")
        tile_height, tile_width = depth.shape[1:]
        packed = np.empty((tile_height * 2, tile_width * 4, 3), dtype=np.uint8)
        self._check(
            self._library.horus_pack_views(
                colors.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte)),
                depth.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                4,
                tile_width,
                tile_height,
                float(near_m),
                float(far_m),
                packed.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte)),
            ),
            "packed-atlas conversion",
        )
        return packed

    def close(self) -> None:
        if getattr(self, "_handle", None):
            self._library.horus_cuda_destroy(self._handle)
            self._handle = None

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
