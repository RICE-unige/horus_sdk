"""Real dense-map sources for the native HORUS remote renderer.

The Cow and Lady ground-truth point cloud is published by ETH Zurich under
CC BY 4.0: https://doi.org/10.3929/ethz-b-000721636
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

from .synthetic_scene import (
    CAMERA_PITCH_DEGREES,
    CAMERA_POSITION,
    DEPTH_FAR_METERS,
    DEPTH_NEAR_METERS,
    VERTICAL_FOV_DEGREES,
    encode_luma_depth,
)


DEFAULT_COW_LADY_PLY = (
    Path.home()
    / ".cache"
    / "horus"
    / "voxblox_cow_lady"
    / "extras"
    / "cow_and_lady_gt.ply"
)
COW_LADY_DEPTH_NEAR_BASE = 0.3
COW_LADY_DEPTH_FAR_BASE = 20.0


@dataclass(frozen=True)
class RemoteRenderViewSpec:
    name: str
    position: tuple[float, float, float]
    yaw_degrees: float
    pitch_degrees: float
    atlas_x: float
    atlas_y: float
    atlas_width: float
    atlas_height: float


_THIRD = 1.0 / 3.0


def _atlas_view(
    name: str,
    position: tuple[float, float, float],
    yaw_degrees: float,
    pitch_degrees: float,
    column: int,
    row: int,
) -> RemoteRenderViewSpec:
    return RemoteRenderViewSpec(
        name,
        position,
        yaw_degrees,
        pitch_degrees,
        column * _THIRD,
        row * _THIRD,
        _THIRD,
        _THIRD,
    )


def view_spec_camera_pose(
    view: RemoteRenderViewSpec,
    *,
    world_scale: float = 1.0,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Return the camera-to-world pose used by both CPU and CUDA rasterizers."""
    half_yaw = math.radians(view.yaw_degrees) * 0.5
    half_pitch = math.radians(view.pitch_degrees) * 0.5
    sy, cy = math.sin(half_yaw), math.cos(half_yaw)
    sx, cx = math.sin(half_pitch), math.cos(half_pitch)
    position = tuple(float(value) * world_scale for value in view.position)
    rotation = (cy * sx, sy * cx, -sy * sx, cy * cx)
    return position, rotation


# These views form an immutable scene proxy. Eight ring cameras capture the
# exterior from every azimuth and the ninth captures horizontal/upper surfaces.
# Each tile remains 16:9 when the atlas is 16:9, so the registered projection
# and the PC rasterizer use identical intrinsics.
COW_LADY_VIEW_SPECS = (
    _atlas_view("front", (0.0, 2.2, 4.0), 180.0, 15.0, 0, 0),
    _atlas_view("front_right", (2.828, 2.2, 2.828), 225.0, 15.0, 1, 0),
    _atlas_view("right", (4.0, 2.2, 0.0), 270.0, 15.0, 2, 0),
    _atlas_view("back_right", (2.828, 2.2, -2.828), 315.0, 15.0, 0, 1),
    _atlas_view("back", (0.0, 2.2, -4.0), 0.0, 15.0, 1, 1),
    _atlas_view("back_left", (-2.828, 2.2, -2.828), 45.0, 15.0, 2, 1),
    _atlas_view("left", (-4.0, 2.2, 0.0), 90.0, 15.0, 0, 2),
    _atlas_view("front_left", (-2.828, 2.2, 2.828), 135.0, 15.0, 1, 2),
    _atlas_view("interior_high", (0.0, 2.4, -1.0), 0.0, 55.0, 2, 2),
)

DEFAULT_ETH3D_COURTYARD_ROOT = (
    Path.home()
    / ".cache"
    / "horus"
    / "eth3d_courtyard"
    / "scan_clean"
    / "courtyard"
    / "scan_clean"
)
ETH3D_COURTYARD_POINT_COUNT = 37_795_990
ETH3D_COURTYARD_SOURCE_SCALE = 0.7
ETH3D_COURTYARD_CENTER = np.array((10.45, -1.84, 0.14), dtype=np.float32)
ETH3D_COURTYARD_DEPTH_NEAR_BASE = 1.0
ETH3D_COURTYARD_DEPTH_FAR_BASE = 60.0
ETH3D_COURTYARD_VIEW_SPECS = (
    _atlas_view("south", (0.0, 12.0, -30.0), 0.0, 17.0, 0, 0),
    _atlas_view("south_west", (-21.213, 12.0, -21.213), 45.0, 17.0, 1, 0),
    _atlas_view("west", (-30.0, 12.0, 0.0), 90.0, 17.0, 2, 0),
    _atlas_view("north_west", (-21.213, 12.0, 21.213), 135.0, 17.0, 0, 1),
    _atlas_view("north", (0.0, 12.0, 30.0), 180.0, 17.0, 1, 1),
    _atlas_view("north_east", (21.213, 12.0, 21.213), 225.0, 17.0, 2, 1),
    _atlas_view("east", (30.0, 12.0, 0.0), 270.0, 17.0, 0, 2),
    _atlas_view("south_east", (21.213, 12.0, -21.213), 315.0, 17.0, 1, 2),
    _atlas_view("overhead", (0.0, 40.0, 0.0), 0.0, 89.0, 2, 2),
)
ETH3D_COURTYARD_SCANS = (
    (
        "scan2.ply",
        np.array(
            (
                (1.0, -2.38419e-07, -5.96047e-08, 3.59469),
                (2.38419e-07, 1.0, 2.38419e-07, 0.562818),
                (5.96046e-08, -2.38419e-07, 1.0, 1.80679),
                (0.0, 0.0, 0.0, 1.0),
            ),
            dtype=np.float32,
        ),
    ),
    (
        "scan1.ply",
        np.array(
            (
                (0.996548, 0.0830159, -5.48683e-05, 3.81589),
                (-0.0830159, 0.996548, 0.000287318, -7.23808),
                (7.85309e-05, -0.000281772, 1.0, 1.86659),
                (0.0, 0.0, 0.0, 1.0),
            ),
            dtype=np.float32,
        ),
    ),
)

ETH3D_REMOTE_SCENES = {
    "delivery_area": {
        "label": "ETH3D Delivery Area",
        "archive_url": "https://www.eth3d.net/data/delivery_area_scan_clean.7z",
        "target_extent_m": 28.0,
    },
    "electro": {
        "label": "ETH3D Electro",
        "archive_url": "https://www.eth3d.net/data/electro_scan_clean.7z",
        "target_extent_m": 32.0,
    },
    "facade": {
        "label": "ETH3D Facade",
        "archive_url": "https://www.eth3d.net/data/facade_scan_clean.7z",
        "target_extent_m": 32.0,
    },
    "playground": {
        "label": "ETH3D Playground",
        "archive_url": "https://www.eth3d.net/data/playground_scan_clean.7z",
        "target_extent_m": 32.0,
    },
    "terrains": {
        "label": "ETH3D Terrains",
        "archive_url": "https://www.eth3d.net/data/terrains_scan_clean.7z",
        "target_extent_m": 28.0,
    },
}
ETH3D_REMOTE_SCENE_IDS = tuple(ETH3D_REMOTE_SCENES)
DEFAULT_ETH3D_REMOTE_ROOT = Path.home() / ".cache" / "horus" / "eth3d_remote_maps"


@dataclass(frozen=True)
class Eth3dRemoteScene:
    scene_id: str
    root: Path
    scans: tuple[tuple[str, np.ndarray], ...]
    point_count: int
    center: np.ndarray
    source_bounds_min: np.ndarray
    source_bounds_max: np.ndarray
    world_scale: float
    depth_near_m: float
    depth_far_m: float
    initial_position: tuple[float, float, float]
    initial_pitch_degrees: float
    fingerprint: str


REMOTE_RENDER_LOD_DTYPE = np.dtype(
    [
        ("position", "<f4", (3,)),
        ("color", "u1", (3,)),
    ],
    align=False,
)
REMOTE_RENDER_LOD_HEADER_BYTES = 20


PLY_SCALAR_TYPES = {
    "char": "i1",
    "uchar": "u1",
    "int8": "i1",
    "uint8": "u1",
    "short": "<i2",
    "ushort": "<u2",
    "int16": "<i2",
    "uint16": "<u2",
    "int": "<i4",
    "uint": "<u4",
    "int32": "<i4",
    "uint32": "<u4",
    "float": "<f4",
    "float32": "<f4",
    "double": "<f8",
    "float64": "<f8",
}


def resolve_cow_lady_ply(path: str | Path | None = None) -> Path:
    resolved = Path(path).expanduser() if path else DEFAULT_COW_LADY_PLY
    if not resolved.is_file():
        raise FileNotFoundError(
            f"Cow and Lady point cloud not found at {resolved}. Run "
            "'python3 python/examples/tools/fetch_voxblox_cow_lady.py' first."
        )
    return resolved


def resolve_eth3d_courtyard_root(path: str | Path | None = None) -> Path:
    resolved = Path(path).expanduser() if path else DEFAULT_ETH3D_COURTYARD_ROOT
    missing = [name for name, _ in ETH3D_COURTYARD_SCANS if not (resolved / name).is_file()]
    if missing:
        raise FileNotFoundError(
            f"ETH3D Courtyard scan is incomplete at {resolved}; missing {', '.join(missing)}. "
            "Run python/examples/tools/fetch_eth3d_courtyard.py first."
        )
    return resolved


def load_colored_vertex_ply(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Load all colored vertices from a binary little-endian PLY."""
    source = Path(path).expanduser()
    records, names = open_vertex_ply_records(source)
    points = np.column_stack((records["x"], records["y"], records["z"])).astype(
        np.float32,
        copy=False,
    )
    if {"red", "green", "blue"}.issubset(names):
        colors = np.column_stack(
            (records["red"], records["green"], records["blue"])
        ).astype(np.uint8, copy=False)
    else:
        colors = np.full((len(records), 3), (184, 188, 192), dtype=np.uint8)
    finite = np.isfinite(points).all(axis=1)
    return np.asarray(points[finite]), np.asarray(colors[finite])


def open_vertex_ply_records(path: str | Path) -> tuple[np.memmap, set[str]]:
    source = Path(path).expanduser()
    vertex_count = 0
    vertex_properties: list[tuple[str, str]] = []
    current_element = ""
    encoding = ""
    header_bytes = 0
    with source.open("rb") as handle:
        if handle.readline().strip() != b"ply":
            raise ValueError(f"not a PLY file: {source}")
        header_bytes = handle.tell()
        while True:
            raw = handle.readline()
            if not raw:
                raise ValueError("PLY header ended before end_header")
            header_bytes += len(raw)
            line = raw.decode("ascii", "strict").strip()
            fields = line.split()
            if not fields:
                continue
            if fields[0] == "format":
                encoding = fields[1]
            elif fields[0] == "element":
                current_element = fields[1]
                if current_element == "vertex":
                    vertex_count = int(fields[2])
            elif fields[0] == "property" and current_element == "vertex":
                if fields[1] == "list":
                    raise ValueError("list-valued vertex properties are unsupported")
                vertex_properties.append((fields[2], fields[1]))
            elif fields[0] == "end_header":
                break

    if encoding != "binary_little_endian":
        raise ValueError(f"unsupported PLY encoding '{encoding}'")
    if vertex_count <= 0:
        raise ValueError("PLY contains no vertices")
    names = {name for name, _ in vertex_properties}
    if not {"x", "y", "z"}.issubset(names):
        raise ValueError("PLY vertices must contain x, y and z")
    dtype_fields = []
    for name, scalar_type in vertex_properties:
        if scalar_type not in PLY_SCALAR_TYPES:
            raise ValueError(f"unsupported PLY scalar type '{scalar_type}'")
        dtype_fields.append((name, PLY_SCALAR_TYPES[scalar_type]))
    records = np.memmap(
        source,
        mode="r",
        dtype=np.dtype(dtype_fields),
        offset=header_bytes,
        shape=(vertex_count,),
    )
    return records, names


def resolve_eth3d_remote_scene_root(
    scene_id: str,
    path: str | Path | None = None,
) -> Path:
    if scene_id not in ETH3D_REMOTE_SCENES:
        raise ValueError(
            f"unknown ETH3D scene {scene_id!r}; expected one of "
            f"{', '.join(ETH3D_REMOTE_SCENE_IDS)}"
        )
    search_root = (
        Path(path).expanduser()
        if path
        else DEFAULT_ETH3D_REMOTE_ROOT / scene_id / "scan_clean"
    )
    if (search_root / "scan_alignment.mlp").is_file():
        return search_root.resolve()
    if search_root.is_dir():
        matches = sorted(search_root.rglob("scan_alignment.mlp"))
        for match in matches:
            if match.parent.name == "scan_clean":
                return match.parent.resolve()
        if matches:
            return matches[0].parent.resolve()
    raise FileNotFoundError(
        f"ETH3D {scene_id} is not available below {search_root}. Run "
        "'python3 python/examples/tools/fetch_remote_render_maps.py "
        f"--scene {scene_id}' first."
    )


def _read_eth3d_scan_alignment(
    root: Path,
) -> tuple[tuple[str, np.ndarray], ...]:
    project = ET.parse(root / "scan_alignment.mlp")
    scans: list[tuple[str, np.ndarray]] = []
    for mesh in project.findall(".//MLMesh"):
        filename = (mesh.attrib.get("filename") or "").strip()
        matrix_node = mesh.find("MLMatrix44")
        if not filename or matrix_node is None or not matrix_node.text:
            continue
        matrix_values = np.fromstring(matrix_node.text, sep=" ", dtype=np.float32)
        if matrix_values.size != 16:
            raise ValueError(
                f"invalid alignment matrix for {filename} in "
                f"{root / 'scan_alignment.mlp'}"
            )
        scan_path = (root / filename).resolve()
        if root.resolve() not in scan_path.parents or not scan_path.is_file():
            raise FileNotFoundError(f"aligned ETH3D scan is missing: {scan_path}")
        scans.append((filename, matrix_values.reshape(4, 4)))
    if not scans:
        raise ValueError(f"no aligned scans found in {root / 'scan_alignment.mlp'}")
    return tuple(scans)


def _eth3d_scene_fingerprint(
    root: Path,
    scans: tuple[tuple[str, np.ndarray], ...],
) -> list[dict[str, int | str]]:
    fingerprint = []
    for filename, _ in scans:
        stat = (root / filename).stat()
        fingerprint.append(
            {
                "filename": filename,
                "bytes": stat.st_size,
                "mtime_ns": stat.st_mtime_ns,
            }
        )
    return fingerprint


def _scan_eth3d_scene_bounds(
    root: Path,
    scans: tuple[tuple[str, np.ndarray], ...],
    *,
    chunk_size: int = 1_000_000,
) -> tuple[int, np.ndarray, np.ndarray]:
    bounds_min = np.full(3, np.inf, dtype=np.float64)
    bounds_max = np.full(3, -np.inf, dtype=np.float64)
    point_count = 0
    for filename, alignment in scans:
        records, _ = open_vertex_ply_records(root / filename)
        for start in range(0, len(records), chunk_size):
            end = min(len(records), start + chunk_size)
            points = np.column_stack(
                (
                    records["x"][start:end],
                    records["y"][start:end],
                    records["z"][start:end],
                )
            ).astype(np.float32)
            points = points @ alignment[:3, :3].T + alignment[:3, 3]
            finite = np.isfinite(points).all(axis=1)
            points = points[finite]
            if not len(points):
                continue
            point_count += len(points)
            bounds_min = np.minimum(bounds_min, points.min(axis=0))
            bounds_max = np.maximum(bounds_max, points.max(axis=0))
    if point_count == 0 or not np.isfinite((bounds_min, bounds_max)).all():
        raise ValueError(f"ETH3D scene at {root} contains no finite points")
    return point_count, bounds_min, bounds_max


def prepare_eth3d_remote_scene(
    scene_id: str,
    path: str | Path | None = None,
    *,
    dataset_scale: float = 1.0,
) -> Eth3dRemoteScene:
    if not np.isfinite(dataset_scale) or not 0.1 <= dataset_scale <= 10.0:
        raise ValueError("dataset_scale must be finite and between 0.1 and 10")
    root = resolve_eth3d_remote_scene_root(scene_id, path)
    scans = _read_eth3d_scan_alignment(root)
    source_fingerprint = _eth3d_scene_fingerprint(root, scans)
    fingerprint = hashlib.sha256(
        json.dumps(
            source_fingerprint,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    cache_path = root.parent / "horus_remote_scene_stats_v1.json"
    cached = None
    try:
        cached = json.loads(cache_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        pass
    if cached and cached.get("fingerprint") == source_fingerprint:
        point_count = int(cached["point_count"])
        bounds_min = np.asarray(cached["bounds_min"], dtype=np.float64)
        bounds_max = np.asarray(cached["bounds_max"], dtype=np.float64)
    else:
        point_count, bounds_min, bounds_max = _scan_eth3d_scene_bounds(root, scans)
        cache_path.write_text(
            json.dumps(
                {
                    "scene_id": scene_id,
                    "fingerprint": source_fingerprint,
                    "point_count": point_count,
                    "bounds_min": bounds_min.tolist(),
                    "bounds_max": bounds_max.tolist(),
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )

    extents = bounds_max - bounds_min
    source_extent = max(float(extents.max()), 1e-6)
    target_extent = (
        float(ETH3D_REMOTE_SCENES[scene_id]["target_extent_m"]) * dataset_scale
    )
    world_scale = target_extent / source_extent
    converted_extents = np.asarray(
        (extents[0], extents[2], extents[1]),
        dtype=np.float64,
    ) * world_scale
    horizontal_extent = max(float(converted_extents[0]), float(converted_extents[2]))
    vertical_extent = float(converted_extents[1])
    camera_height = max(1.6, vertical_extent * 0.55)
    camera_distance = max(4.0, horizontal_extent * 0.35)
    camera_target_height = vertical_extent * 0.3
    initial_position = (
        0.0,
        camera_height,
        -camera_distance,
    )
    initial_pitch_degrees = math.degrees(
        math.atan2(camera_height - camera_target_height, camera_distance)
    )
    diagonal = float(np.linalg.norm(converted_extents))
    center = (bounds_min + bounds_max) * 0.5
    # ETH3D is Z-up. Preserve horizontal centering but place the physical
    # scan floor at Unity map y=0 so the remote scene rests on the HORUS
    # workspace instead of straddling it.
    center[2] = bounds_min[2]
    return Eth3dRemoteScene(
        scene_id=scene_id,
        root=root,
        scans=scans,
        point_count=point_count,
        center=center.astype(np.float32),
        source_bounds_min=bounds_min.astype(np.float32),
        source_bounds_max=bounds_max.astype(np.float32),
        world_scale=world_scale,
        depth_near_m=max(0.1, target_extent / 250.0),
        depth_far_m=max(40.0, diagonal * 2.5),
        initial_position=initial_position,
        initial_pitch_degrees=initial_pitch_degrees,
        fingerprint=fingerprint,
    )


def iter_eth3d_remote_scene_chunks(
    scene: Eth3dRemoteScene,
    *,
    chunk_size: int = 1_000_000,
):
    chunk_size = max(10_000, int(chunk_size))
    for filename, alignment in scene.scans:
        records, names = open_vertex_ply_records(scene.root / filename)
        has_color = {"red", "green", "blue"}.issubset(names)
        for start in range(0, len(records), chunk_size):
            end = min(len(records), start + chunk_size)
            points = np.column_stack(
                (
                    records["x"][start:end],
                    records["y"][start:end],
                    records["z"][start:end],
                )
            ).astype(np.float32)
            points = points @ alignment[:3, :3].T + alignment[:3, 3]
            finite = np.isfinite(points).all(axis=1)
            points = points[finite]
            converted = np.empty_like(points)
            converted[:, 0] = (points[:, 0] - scene.center[0]) * scene.world_scale
            converted[:, 1] = (points[:, 2] - scene.center[2]) * scene.world_scale
            converted[:, 2] = (points[:, 1] - scene.center[1]) * scene.world_scale
            if has_color:
                colors = np.column_stack(
                    (
                        records["red"][start:end],
                        records["green"][start:end],
                        records["blue"][start:end],
                    )
                ).astype(np.uint8)[finite]
            else:
                colors = np.full(
                    (len(converted), 3),
                    (184, 188, 192),
                    dtype=np.uint8,
                )
            yield converted, colors


def _remote_render_preprocessor_binary() -> Path:
    repository_root = Path(__file__).resolve().parents[3]
    tool_root = repository_root / "rust" / "remote_render_preprocessor"
    manifest = tool_root / "Cargo.toml"
    if not manifest.is_file():
        raise RuntimeError(f"remote-render Rust preprocessor is missing: {manifest}")
    binary = tool_root / "target" / "release" / "horus-remote-render-preprocessor"
    sources = (manifest, tool_root / "src" / "main.rs")
    if binary.is_file() and binary.stat().st_mtime >= max(
        source.stat().st_mtime for source in sources
    ):
        return binary
    cargo = shutil.which("cargo")
    if cargo is None:
        candidate = Path.home() / ".cargo" / "bin" / "cargo"
        cargo = str(candidate) if candidate.is_file() else None
    if cargo is None:
        raise RuntimeError(
            "Rust cargo is required to prepare remote-render point-cloud LOD caches"
        )
    subprocess.run(
        (
            cargo,
            "build",
            "--release",
            "--manifest-path",
            str(manifest),
        ),
        cwd=tool_root,
        check=True,
    )
    if not binary.is_file():
        raise RuntimeError(f"Rust preprocessor did not produce {binary}")
    return binary


def _read_remote_render_lod_header(path: Path) -> tuple[int, float]:
    with path.open("rb") as stream:
        header = stream.read(REMOTE_RENDER_LOD_HEADER_BYTES)
    if len(header) != REMOTE_RENDER_LOD_HEADER_BYTES or header[:4] != b"HRL1":
        raise RuntimeError(f"invalid HORUS remote-render LOD cache: {path}")
    version = int.from_bytes(header[4:8], "little")
    if version != 1:
        raise RuntimeError(f"unsupported HORUS remote-render LOD version {version}")
    voxel_size = float(np.frombuffer(header[8:12], dtype="<f4")[0])
    point_count = int.from_bytes(header[12:20], "little")
    expected_bytes = REMOTE_RENDER_LOD_HEADER_BYTES + (
        point_count * REMOTE_RENDER_LOD_DTYPE.itemsize
    )
    if point_count <= 0 or path.stat().st_size != expected_bytes:
        raise RuntimeError(f"incomplete HORUS remote-render LOD cache: {path}")
    return point_count, voxel_size


def prepare_eth3d_remote_scene_lod(
    scene: Eth3dRemoteScene,
    *,
    voxel_size_m: float = 0.0125,
) -> tuple[Path, int]:
    """Build or reuse a deterministic render LOD while retaining full source data."""
    voxel_size_m = float(voxel_size_m)
    if not np.isfinite(voxel_size_m) or not 0.001 <= voxel_size_m <= 0.25:
        raise ValueError("voxel_size_m must be between 0.001 and 0.25 meters")
    cache_root = scene.root.parent / "horus_render_lod"
    cache_root.mkdir(parents=True, exist_ok=True)
    voxel_micrometers = int(round(voxel_size_m * 1_000_000.0))
    cache_path = cache_root / (
        f"{scene.scene_id}_{scene.fingerprint[:16]}_{voxel_micrometers}um.hrl"
    )
    if cache_path.is_file():
        point_count, cached_voxel_size = _read_remote_render_lod_header(cache_path)
        if abs(cached_voxel_size - voxel_size_m) <= 1e-6:
            return cache_path, point_count
        cache_path.unlink()

    binary = _remote_render_preprocessor_binary()
    process = subprocess.Popen(
        (str(binary), f"{voxel_size_m:.9f}", str(cache_path)),
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    try:
        if process.stdin is None:
            raise RuntimeError("failed to open Rust preprocessor input")
        for points, colors in iter_eth3d_remote_scene_chunks(scene):
            records = np.empty(len(points), dtype=REMOTE_RENDER_LOD_DTYPE)
            records["position"] = points
            records["color"] = colors
            process.stdin.write(records.tobytes(order="C"))
        process.stdin.close()
        stderr = (
            process.stderr.read().decode("utf-8", "replace")
            if process.stderr
            else ""
        )
        return_code = process.wait()
    except Exception:
        process.kill()
        process.wait()
        cache_path.unlink(missing_ok=True)
        raise
    if return_code != 0:
        cache_path.unlink(missing_ok=True)
        raise RuntimeError(
            "Rust remote-render preprocessing failed"
            + (f":\n{stderr.strip()}" if stderr.strip() else "")
        )
    if stderr.strip():
        print(f"[remote-map-source] {stderr.strip()}", flush=True)
    point_count, _ = _read_remote_render_lod_header(cache_path)
    return cache_path, point_count


def iter_eth3d_remote_scene_lod_chunks(
    path: str | Path,
    *,
    chunk_size: int = 1_000_000,
):
    source = Path(path).expanduser().resolve()
    point_count, _ = _read_remote_render_lod_header(source)
    records = np.memmap(
        source,
        mode="r",
        dtype=REMOTE_RENDER_LOD_DTYPE,
        offset=REMOTE_RENDER_LOD_HEADER_BYTES,
        shape=(point_count,),
    )
    chunk_size = max(10_000, int(chunk_size))
    for start in range(0, point_count, chunk_size):
        end = min(point_count, start + chunk_size)
        yield (
            np.ascontiguousarray(records["position"][start:end]),
            np.ascontiguousarray(records["color"][start:end]),
        )


def prepare_cow_lady_points(
    points: np.ndarray,
    *,
    world_scale: float = 1.0,
) -> np.ndarray:
    """Convert the scan's Z-up frame into the renderer's Y-up map frame."""
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    lower = np.quantile(points, 0.01, axis=0)
    upper = np.quantile(points, 0.99, axis=0)
    center_x = 0.5 * (lower[0] + upper[0])
    center_y = 0.5 * (lower[1] + upper[1])
    floor_z = lower[2]
    converted = np.empty_like(points, dtype=np.float32)
    converted[:, 0] = (points[:, 0] - center_x) * world_scale
    converted[:, 1] = (points[:, 2] - floor_z) * world_scale
    converted[:, 2] = (points[:, 1] - center_y) * world_scale
    return converted


def render_colored_points(
    points: np.ndarray,
    colors: np.ndarray,
    width: int,
    height: int,
    *,
    point_radius: int = 1,
    near_m: float = DEPTH_NEAR_METERS,
    far_m: float = DEPTH_FAR_METERS,
    vertical_fov_deg: float = VERTICAL_FOV_DEGREES,
    camera_position: np.ndarray = CAMERA_POSITION,
    camera_pitch_deg: float = CAMERA_PITCH_DEGREES,
    camera_yaw_deg: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Rasterize every point into a colored metric camera-Z image."""
    color = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.full((height, width), np.inf, dtype=np.float32)
    rasterize_colored_points_into(
        color,
        depth,
        points,
        colors,
        point_radius=point_radius,
        near_m=near_m,
        far_m=far_m,
        vertical_fov_deg=vertical_fov_deg,
        camera_position=camera_position,
        camera_pitch_deg=camera_pitch_deg,
        camera_yaw_deg=camera_yaw_deg,
    )
    return color, depth


def rasterize_colored_points_into(
    color: np.ndarray,
    depth: np.ndarray,
    points: np.ndarray,
    colors: np.ndarray,
    *,
    point_radius: int = 1,
    near_m: float = DEPTH_NEAR_METERS,
    far_m: float = DEPTH_FAR_METERS,
    vertical_fov_deg: float = VERTICAL_FOV_DEGREES,
    camera_position: np.ndarray = CAMERA_POSITION,
    camera_pitch_deg: float = CAMERA_PITCH_DEGREES,
    camera_yaw_deg: float = 0.0,
) -> None:
    if points.shape != colors.shape or points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points and colors must both have shape (N, 3)")
    if depth.ndim != 2 or color.shape != (*depth.shape, 3):
        raise ValueError("color and depth targets have incompatible shapes")
    height, width = depth.shape
    point_radius = max(0, min(4, int(point_radius)))
    pitch = math.radians(camera_pitch_deg)
    yaw = math.radians(camera_yaw_deg)
    world_to_yaw_camera = np.array(
        (
            (math.cos(yaw), 0.0, math.sin(yaw)),
            (0.0, 1.0, 0.0),
            (-math.sin(yaw), 0.0, math.cos(yaw)),
        ),
        dtype=np.float32,
    )
    yaw_to_pitched_camera = np.array(
        (
            (1.0, 0.0, 0.0),
            (0.0, math.cos(pitch), -math.sin(pitch)),
            (0.0, math.sin(pitch), math.cos(pitch)),
        ),
        dtype=np.float32,
    )
    camera_points = (
        (points - np.asarray(camera_position, dtype=np.float32))
        @ world_to_yaw_camera
        @ yaw_to_pitched_camera
    )
    z = camera_points[:, 2]
    focal = height * 0.5 / math.tan(math.radians(vertical_fov_deg) * 0.5)
    screen_x = width * 0.5 + focal * camera_points[:, 0] / z
    screen_y = height * 0.5 - focal * camera_points[:, 1] / z
    margin = point_radius + 1
    visible = (
        (z >= near_m)
        & (z <= far_m)
        & (screen_x >= -margin)
        & (screen_x < width + margin)
        & (screen_y >= -margin)
        & (screen_y < height + margin)
    )
    x = np.floor(screen_x[visible]).astype(np.int32)
    y = np.floor(screen_y[visible]).astype(np.int32)
    z = z[visible]
    visible_colors = colors[visible]
    order = np.argsort(z, kind="stable")
    x = x[order]
    y = y[order]
    z = z[order]
    visible_colors = visible_colors[order]

    depth_flat = depth.reshape(-1)
    color_flat = color.reshape(-1, 3)
    offsets = [
        (dx, dy)
        for dy in range(-point_radius, point_radius + 1)
        for dx in range(-point_radius, point_radius + 1)
        if dx * dx + dy * dy <= point_radius * point_radius + 1
    ]
    for dx, dy in offsets:
        target_x = x + dx
        target_y = y + dy
        in_bounds = (
            (target_x >= 0)
            & (target_x < width)
            & (target_y >= 0)
            & (target_y < height)
        )
        target = target_y[in_bounds] * width + target_x[in_bounds]
        candidate_z = z[in_bounds]
        candidate_color = visible_colors[in_bounds]
        unique_target, first = np.unique(target, return_index=True)
        unique_z = candidate_z[first]
        nearer = unique_z < depth_flat[unique_target]
        selected = unique_target[nearer]
        depth_flat[selected] = unique_z[nearer]
        color_flat[selected] = candidate_color[first[nearer]]


def fill_small_depth_holes(
    color: np.ndarray,
    depth: np.ndarray,
    iterations: int = 2,
) -> tuple[np.ndarray, np.ndarray]:
    """Extend nearby scan samples into only small rasterization holes."""
    filled_color = color.copy()
    filled_depth = depth.copy()
    for _ in range(max(0, int(iterations))):
        missing = ~np.isfinite(filled_depth)
        if not np.any(missing):
            break
        candidate_depth = np.full_like(filled_depth, np.inf)
        candidate_color = np.zeros_like(filled_color)
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            shifted_depth = np.roll(filled_depth, (dy, dx), axis=(0, 1))
            shifted_color = np.roll(filled_color, (dy, dx), axis=(0, 1))
            if dy < 0:
                shifted_depth[dy:] = np.inf
            elif dy > 0:
                shifted_depth[:dy] = np.inf
            if dx < 0:
                shifted_depth[:, dx:] = np.inf
            elif dx > 0:
                shifted_depth[:, :dx] = np.inf
            nearer = shifted_depth < candidate_depth
            candidate_depth[nearer] = shifted_depth[nearer]
            candidate_color[nearer] = shifted_color[nearer]
        fill = missing & np.isfinite(candidate_depth)
        filled_depth[fill] = candidate_depth[fill]
        filled_color[fill] = candidate_color[fill]
    return filled_color, filled_depth


def render_multiview_atlas(
    points: np.ndarray,
    colors: np.ndarray,
    width: int,
    height: int,
    views: tuple[RemoteRenderViewSpec, ...],
    *,
    point_radius: int,
    world_scale: float,
    near_base: float,
    far_base: float,
) -> tuple[np.ndarray, np.ndarray]:
    color_atlas = np.zeros((height, width, 3), dtype=np.uint8)
    depth_atlas = np.full((height, width), np.inf, dtype=np.float32)
    for view in views:
        x0 = int(round(view.atlas_x * width))
        view_width = int(round(view.atlas_width * width))
        view_height = int(round(view.atlas_height * height))
        y0 = height - int(round((view.atlas_y + view.atlas_height) * height))
        color, depth = render_colored_points(
            points,
            colors,
            view_width,
            view_height,
            point_radius=point_radius,
            near_m=near_base * world_scale,
            far_m=far_base * world_scale,
            camera_position=np.asarray(view.position, dtype=np.float32) * world_scale,
            camera_pitch_deg=view.pitch_degrees,
            camera_yaw_deg=view.yaw_degrees,
        )
        color, depth = fill_small_depth_holes(color, depth, iterations=2)
        color_atlas[y0:y0 + view_height, x0:x0 + view_width] = color
        depth_atlas[y0:y0 + view_height, x0:x0 + view_width] = depth
    return color_atlas, depth_atlas


def unproject_multiview_surfels(
    color_atlas: np.ndarray,
    depth_atlas: np.ndarray,
    views: tuple[RemoteRenderViewSpec, ...],
    *,
    columns: int,
    rows: int,
    world_scale: float = 1.0,
    vertical_fov_deg: float = VERTICAL_FOV_DEGREES,
) -> tuple[np.ndarray, np.ndarray]:
    """Reconstruct the same surfel centers sampled by the Quest shader.

    Atlas rectangles use Unity's bottom-left texture coordinates. NumPy images
    use top-left row coordinates, so the row conversion here is intentional.
    This helper is used by validation tooling and tests to keep the PC and
    Quest reconstruction contracts numerically aligned.
    """
    if depth_atlas.ndim != 2 or color_atlas.shape != (*depth_atlas.shape, 3):
        raise ValueError("color and depth atlases have incompatible shapes")
    columns = int(columns)
    rows = int(rows)
    if columns <= 0 or rows <= 0:
        raise ValueError("surfel grid dimensions must be positive")

    atlas_height, atlas_width = depth_atlas.shape
    local_u = (np.arange(columns, dtype=np.float32) + 0.5) / columns
    local_v = (np.arange(rows, dtype=np.float32) + 0.5) / rows
    grid_u, grid_v = np.meshgrid(local_u, local_v)
    tan_half_fov = math.tan(math.radians(vertical_fov_deg) * 0.5)
    reconstructed_points: list[np.ndarray] = []
    reconstructed_colors: list[np.ndarray] = []

    for view in views:
        atlas_u = view.atlas_x + grid_u * view.atlas_width
        atlas_v = view.atlas_y + grid_v * view.atlas_height
        pixel_x = np.clip(np.floor(atlas_u * atlas_width).astype(np.int32), 0, atlas_width - 1)
        pixel_y = np.clip(
            atlas_height - 1 - np.floor(atlas_v * atlas_height).astype(np.int32),
            0,
            atlas_height - 1,
        )
        sampled_depth = depth_atlas[pixel_y, pixel_x]
        valid = np.isfinite(sampled_depth)
        if not np.any(valid):
            continue

        view_aspect = (
            view.atlas_width * atlas_width / max(view.atlas_height * atlas_height, 1.0)
        )
        depth = sampled_depth[valid]
        local = np.column_stack(
            (
                (grid_u[valid] * 2.0 - 1.0) * depth * tan_half_fov * view_aspect,
                (grid_v[valid] * 2.0 - 1.0) * depth * tan_half_fov,
                depth,
            )
        ).astype(np.float32)

        pitch = math.radians(view.pitch_degrees)
        yaw = math.radians(view.yaw_degrees)
        camera_to_world = np.array(
            (
                (math.cos(yaw), math.sin(yaw) * math.sin(pitch), math.sin(yaw) * math.cos(pitch)),
                (0.0, math.cos(pitch), -math.sin(pitch)),
                (-math.sin(yaw), math.cos(yaw) * math.sin(pitch), math.cos(yaw) * math.cos(pitch)),
            ),
            dtype=np.float32,
        )
        world = local @ camera_to_world.T
        world += np.asarray(view.position, dtype=np.float32) * world_scale
        reconstructed_points.append(world)
        reconstructed_colors.append(color_atlas[pixel_y[valid], pixel_x[valid]])

    if not reconstructed_points:
        return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8)
    return (
        np.concatenate(reconstructed_points, axis=0),
        np.concatenate(reconstructed_colors, axis=0),
    )


def build_cow_lady_rgbd(
    width: int,
    height: int,
    *,
    ply_path: str | Path | None = None,
    point_radius: int = 1,
    world_scale: float = 1.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    source = resolve_cow_lady_ply(ply_path)
    points, colors = load_colored_vertex_ply(source)
    points = prepare_cow_lady_points(points, world_scale=world_scale)
    color, depth = render_multiview_atlas(
        points,
        colors,
        width,
        height,
        COW_LADY_VIEW_SPECS,
        point_radius=point_radius,
        world_scale=world_scale,
        near_base=COW_LADY_DEPTH_NEAR_BASE,
        far_base=COW_LADY_DEPTH_FAR_BASE,
    )
    encoded_depth = encode_luma_depth(
        depth,
        near_m=COW_LADY_DEPTH_NEAR_BASE * world_scale,
        far_m=COW_LADY_DEPTH_FAR_BASE * world_scale,
    )
    packed = np.ascontiguousarray(np.concatenate((color, encoded_depth), axis=1))
    return color, depth, packed


def iter_eth3d_courtyard_chunks(
    root: str | Path | None = None,
    *,
    world_scale: float = 1.0,
    chunk_size: int = 500_000,
):
    source_root = resolve_eth3d_courtyard_root(root)
    scale = ETH3D_COURTYARD_SOURCE_SCALE * world_scale
    for filename, alignment in ETH3D_COURTYARD_SCANS:
        records, names = open_vertex_ply_records(source_root / filename)
        has_color = {"red", "green", "blue"}.issubset(names)
        for start in range(0, len(records), max(10_000, int(chunk_size))):
            end = min(len(records), start + max(10_000, int(chunk_size)))
            points = np.column_stack(
                (records["x"][start:end], records["y"][start:end], records["z"][start:end])
            ).astype(np.float32)
            points = points @ alignment[:3, :3].T + alignment[:3, 3]
            finite = np.isfinite(points).all(axis=1)
            points = points[finite]
            converted = np.empty_like(points)
            converted[:, 0] = (points[:, 0] - ETH3D_COURTYARD_CENTER[0]) * scale
            converted[:, 1] = (points[:, 2] - ETH3D_COURTYARD_CENTER[2]) * scale
            converted[:, 2] = (points[:, 1] - ETH3D_COURTYARD_CENTER[1]) * scale
            if has_color:
                colors = np.column_stack(
                    (
                        records["red"][start:end],
                        records["green"][start:end],
                        records["blue"][start:end],
                    )
                ).astype(np.uint8)[finite]
            else:
                colors = np.full((len(converted), 3), (184, 188, 192), dtype=np.uint8)
            yield converted, colors


def build_eth3d_courtyard_rgbd(
    width: int,
    height: int,
    *,
    dataset_root: str | Path | None = None,
    point_radius: int = 1,
    world_scale: float = 1.0,
    use_cache: bool = True,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    source_root = resolve_eth3d_courtyard_root(dataset_root)
    cache_root = source_root.parent / "horus_prepared"
    cache_path = cache_root / (
        f"courtyard_uvbottom_v2_{width}x{height}_r{int(point_radius)}_s{world_scale:.4f}.npz"
    )
    if use_cache and cache_path.is_file():
        with np.load(cache_path) as cached:
            color = np.asarray(cached["color"], dtype=np.uint8)
            depth = np.asarray(cached["depth"], dtype=np.float32)
    else:
        targets = []
        for view in ETH3D_COURTYARD_VIEW_SPECS:
            view_width = int(round(width * view.atlas_width))
            view_height = int(round(height * view.atlas_height))
            targets.append(
                (
                    np.zeros((view_height, view_width, 3), dtype=np.uint8),
                    np.full((view_height, view_width), np.inf, dtype=np.float32),
                )
            )
        processed = 0
        for points, colors in iter_eth3d_courtyard_chunks(
            source_root,
            world_scale=world_scale,
        ):
            for view, (view_color, view_depth) in zip(ETH3D_COURTYARD_VIEW_SPECS, targets):
                rasterize_colored_points_into(
                    view_color,
                    view_depth,
                    points,
                    colors,
                    point_radius=point_radius,
                    near_m=ETH3D_COURTYARD_DEPTH_NEAR_BASE * world_scale,
                    far_m=ETH3D_COURTYARD_DEPTH_FAR_BASE * world_scale,
                    camera_position=np.asarray(view.position, dtype=np.float32) * world_scale,
                    camera_pitch_deg=view.pitch_degrees,
                    camera_yaw_deg=view.yaw_degrees,
                )
            processed += len(points)
            if processed % 5_000_000 < len(points):
                print(
                    f"[remote-render] ETH3D Courtyard processed {processed:,}/"
                    f"{ETH3D_COURTYARD_POINT_COUNT:,} points",
                    flush=True,
                )
        color = np.zeros((height, width, 3), dtype=np.uint8)
        depth = np.full((height, width), np.inf, dtype=np.float32)
        for view, (view_color, view_depth) in zip(ETH3D_COURTYARD_VIEW_SPECS, targets):
            view_color, view_depth = fill_small_depth_holes(view_color, view_depth, iterations=2)
            x0 = int(round(view.atlas_x * width))
            y0 = height - int(round((view.atlas_y + view.atlas_height) * height))
            color[y0:y0 + view_color.shape[0], x0:x0 + view_color.shape[1]] = view_color
            depth[y0:y0 + view_depth.shape[0], x0:x0 + view_depth.shape[1]] = view_depth
        if use_cache:
            cache_root.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(cache_path, color=color, depth=depth)

    encoded_depth = encode_luma_depth(
        depth,
        near_m=ETH3D_COURTYARD_DEPTH_NEAR_BASE * world_scale,
        far_m=ETH3D_COURTYARD_DEPTH_FAR_BASE * world_scale,
    )
    packed = np.ascontiguousarray(np.concatenate((color, encoded_depth), axis=1))
    return color, depth, packed
