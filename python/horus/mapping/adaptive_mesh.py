"""Serialized-worker HMSH v2 adaptive views, with complete coarse context.

Call ``ensure_native()`` during offline preflight, before constructing a store.
No method on the store compiles code, starts threads, or imports ROS. All methods
must be called by one serialized owner. Budgets count serialized HMSH bytes, not
renderer-specific GPU allocations. Over-budget context is returned intact with
an explicit failure status; it is never culled to make the figures look smaller.

Sphere size[0] is its diameter. Box sizes are full extents, with an XYZW rotation.
``min_fraction`` is the minimum uniform geometric scale of a region. All minimum
cores must fit together before any are granted. Weighted round-robin growth then
expands nested volumes in 0.05 scale increments. Every tile intersecting a grant
is fine; overlapping grants share tiles. Nothing is clipped or transmitted twice.
"""

from __future__ import annotations

import ctypes
from copy import deepcopy
from dataclasses import dataclass, field
import math
from pathlib import Path
import re
import struct
import sys
import time
from types import MappingProxyType

import numpy as np

from .mesh_protocol import (
    HMSH_MAGIC, HMSH_VERSION_LIVE, HMSH_V2_HEADER,
    HMSH_OPERATION_UPSERT, HMSH_OPERATION_DELETE, HMSH_OPERATION_CLEAR,
    MeshPageUpdate, decode_live_header,
)
from .native.build_adaptive_mesh import build_library, library_path


_LIBRARIES: dict[str, ctypes.CDLL] = {}
_MAX_UINT64 = (1 << 64) - 1
_MAX_CLIENTS = 4
_CLIENT_TTL = 15.0
_MAX_REGIONS = 64
_MAX_CHUNK_ID = 0x3fffffff
_OPERATIONS = {HMSH_OPERATION_UPSERT, HMSH_OPERATION_DELETE, HMSH_OPERATION_CLEAR}


def _load_native(path: Path) -> ctypes.CDLL:
    key = str(path)
    if key not in _LIBRARIES:
        try:
            library = ctypes.CDLL(key)
        except OSError as error:
            raise RuntimeError("Run ensure_native() during adaptive mesh preflight first") from error
        library.horus_adaptive_mesh_abi.restype = ctypes.c_uint
        if library.horus_adaptive_mesh_abi() != 1:
            raise RuntimeError("Unsupported adaptive mesh native ABI")
        library.horus_adaptive_mesh.argtypes = [
            ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t,
            ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_size_t), ctypes.c_float, ctypes.c_float,
            ctypes.c_float, ctypes.POINTER(ctypes.c_float),
        ]
        library.horus_adaptive_mesh.restype = ctypes.c_int
        library.horus_adaptive_mesh_diagnostics.argtypes = [
            *library.horus_adaptive_mesh.argtypes, ctypes.POINTER(ctypes.c_size_t)]
        library.horus_adaptive_mesh_diagnostics.restype = ctypes.c_int
        library.horus_adaptive_mesh_baseline_diagnostics.argtypes = library.horus_adaptive_mesh_diagnostics.argtypes
        library.horus_adaptive_mesh_baseline_diagnostics.restype = ctypes.c_int
        _LIBRARIES[key] = library
    return _LIBRARIES[key]


def ensure_native() -> Path:
    """Build and load-check the local vendored library; offline preflight only."""
    path = build_library()
    _load_native(path)
    return path


@dataclass(frozen=True)
class AdaptiveMeshPage:
    chunk_id: int
    page_key: tuple[int, int, int]
    source_epoch: int
    source_revision: int
    fine: bytes
    coarse: bytes
    bounds: tuple[tuple[float, float, float], tuple[float, float, float]]
    fine_vertices: int
    coarse_vertices: int
    fine_triangles: int
    coarse_triangles: int
    simplification_error_m: float
    welded_vertices: int
    boundary_vertices: int
    degenerate_triangles: int = 0
    native_simplified_triangles: int = 0
    native_fallback_reason: str = ""
    native_simplification_error_m: float = 0.
    restored_components: int = 0
    restored_triangles: int = 0
    largest_restored_component: int = 0
    source_components: int = 0

    @property
    def cached_bytes(self) -> int:
        return len(self.fine) + (0 if self.coarse is self.fine else len(self.coarse))


@dataclass
class _Client:
    epoch: int
    source_epoch: int | None
    last_seen: float
    revisions: dict[int, int] = field(default_factory=dict)
    signatures: dict[int, tuple] = field(default_factory=dict)
    inventory: dict[int, MeshPageUpdate] = field(default_factory=dict)
    status: dict = field(default_factory=dict)


@dataclass(frozen=True)
class _Region:
    id: str
    shape: str
    center: np.ndarray
    half_size: np.ndarray
    rotation: np.ndarray
    quaternion: tuple[float, float, float, float]
    priority: float
    min_fraction: float


def _integer(value, name: str, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")
    return value


def _vector(value, count: int, name: str) -> np.ndarray:
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError(f"{name} must have {count} finite components")
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value):
        raise ValueError(f"{name} must contain finite numbers")
    try:
        result = np.asarray(value, dtype=np.float64)
    except (OverflowError, ValueError) as error:
        raise ValueError(f"{name} must contain finite numbers") from error
    if not np.isfinite(result).all():
        raise ValueError(f"{name} must contain finite numbers")
    return result


def _regions(values: list[dict]) -> list[_Region]:
    if not isinstance(values, list) or len(values) > _MAX_REGIONS:
        raise ValueError(f"regions must be a list of at most {_MAX_REGIONS} regions")
    result, identifiers = [], set()
    for item in values:
        if not isinstance(item, dict):
            raise ValueError("region must be an object")
        identifier = item.get("id")
        if not isinstance(identifier, str) or not identifier or len(identifier) > 128 or identifier in identifiers:
            raise ValueError("region id must be a unique nonempty string of at most 128 characters")
        identifiers.add(identifier)
        shape = item.get("shape")
        if not isinstance(shape, str) or shape not in {"sphere", "box", "oriented_box"}:
            raise ValueError("region shape must be sphere or box")
        center = _vector(item.get("center"), 3, "region center")
        size = _vector(item.get("size"), 3, "region size")
        if np.any(size <= 0):
            raise ValueError("region sizes must be positive")
        quaternion = _vector(item.get("rotation", [0, 0, 0, 1]), 4, "region rotation")
        norm = math.hypot(*quaternion)
        if not math.isfinite(norm) or norm < 1e-12:
            raise ValueError("region rotation must be a nonzero quaternion")
        x, y, z, w = quaternion / norm
        rotation = np.array([
            [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)],
        ])
        priority, fraction = item.get("priority", 0), item.get("min_fraction", 0.25)
        if (isinstance(priority, bool) or not isinstance(priority, (int, float)) or
                not 0 <= priority <= 1e6 or not math.isfinite(priority)):
            raise ValueError("region priority must be finite and between 0 and 1000000")
        if (isinstance(fraction, bool) or not isinstance(fraction, (int, float)) or
                not 0 <= fraction <= 1 or not math.isfinite(fraction)):
            raise ValueError("region min_fraction must be between 0 and 1")
        result.append(_Region(identifier, shape, center, size * 0.5, rotation,
                              tuple(map(float, quaternion / norm)),
                              float(priority), float(fraction)))
    return result


def _intersects(region: _Region, bounds) -> bool:
    low, high = np.asarray(bounds[0]), np.asarray(bounds[1])
    if region.shape == "sphere":
        delta = region.center - np.clip(region.center, low, high)
        # hypot avoids overflowing valid large map coordinates.
        return math.hypot(*delta) <= region.half_size[0]
    center = low * 0.5 + high * 0.5
    extent = high * 0.5 - low * 0.5
    delta = region.center - center
    axes = [*np.eye(3), *region.rotation.T]
    axes.extend(np.cross(a, b) for a in np.eye(3) for b in region.rotation.T)
    for axis in axes:
        if np.dot(axis, axis) < 1e-20:
            continue
        radius = np.dot(extent, np.abs(axis))
        radius += np.dot(region.half_size, np.abs(region.rotation.T @ axis))
        if abs(np.dot(delta, axis)) > radius + 1e-10:
            return False
    return True


def _activation(region: _Region, bounds: np.ndarray) -> np.ndarray:
    """Minimum uniform region scale intersecting each page AABB (closed sets)."""
    low, high = bounds[:, 0], bounds[:, 1]
    if region.shape == "sphere":
        delta = region.center - np.clip(region.center, low, high)
        return np.hypot.reduce(delta, axis=1) / region.half_size[0]
    center = low * .5 + high * .5
    extent = high * .5 - low * .5
    axes = np.array([*np.eye(3), *region.rotation.T,
        *(np.cross(a, b) for a in np.eye(3) for b in region.rotation.T)])
    axes = axes[np.einsum("ij,ij->i", axes, axes) >= 1e-20]
    distance = np.abs((region.center - center) @ axes.T)
    aabb_radius = extent @ np.abs(axes.T)
    region_radius = np.abs(axes @ region.rotation) @ region.half_size
    return np.maximum(0., ((distance - aabb_radius - 1e-10) / region_radius).max(axis=1))


def _view_payload(payload: bytes, epoch: int, revision: int) -> bytes:
    return payload[:24] + struct.pack("<QQ", epoch, revision) + payload[40:]


def _update(payload: bytes) -> MeshPageUpdate:
    header = decode_live_header(payload)
    return MeshPageUpdate(header.chunk_id, header.page_key, header.epoch,
                          header.revision, header.operation, payload,
                          header.vertex_count, header.index_count // 3)


def _clear(epoch: int) -> MeshPageUpdate:
    return _update(HMSH_V2_HEADER.pack(HMSH_MAGIC, HMSH_VERSION_LIVE, 0, 0, 0, 0,
                                     epoch, 0, HMSH_OPERATION_CLEAR, 0, 0, 0, 0))


class AdaptiveMeshStore:
    """A bounded original-page cache and up to four independent 15-second views.

    ``ingest`` returns False for malformed, stale or over-capacity source data;
    ``last_rejection`` explains why, and the previous source/view state survives.
    Invalid view arguments raise ValueError. Exhausting the hard store byte cap
    raises MemoryError rather than returning a partial inventory. Snapshot repair
    replays the last committed client view, even while newer source data arrives.
    """

    def __init__(self, *, max_pages: int = 1024,
                 max_cache_bytes: int = 512 * 1024 * 1024,
                 max_payload_bytes: int = 16 * 1024 * 1024):
        self.max_pages = _integer(max_pages, "max_pages", 1)
        if self.max_pages > 1024:
            raise ValueError("max_pages cannot exceed 1024 HMSH slots")
        self.max_payload_bytes = _integer(max_payload_bytes, "max_payload_bytes", HMSH_V2_HEADER.size)
        self.max_cache_bytes = _integer(max_cache_bytes, "max_cache_bytes", HMSH_V2_HEADER.size)
        if sys.byteorder != "little":
            raise RuntimeError("Adaptive mesh native input requires a little-endian host")
        self._native = _load_native(library_path())
        self._pages: dict[int, AdaptiveMeshPage] = {}
        self._tombstones: dict[int, bytes] = {}
        self._source_revisions: dict[int, int] = {}
        self._clients: dict[str, _Client] = {}
        self.source_epoch: int | None = None
        self.last_rejection = ""
        self._epoch_clock = time.time_ns()
        self._last_now = -math.inf

    @property
    def pages(self):
        return MappingProxyType(self._pages)

    @property
    def cached_bytes(self) -> int:
        return (sum(page.cached_bytes for page in self._pages.values()) +
                sum(map(len, self._tombstones.values())) +
                sum(len(update.payload) for client in self._clients.values()
                    for update in client.inventory.values()))

    def _decode(self, payload: bytes):
        if not isinstance(payload, bytes) or not HMSH_V2_HEADER.size <= len(payload) <= self.max_payload_bytes:
            raise ValueError("invalid HMSH payload length")
        fields = HMSH_V2_HEADER.unpack_from(payload)
        magic, version, chunk, x, y, z, epoch, revision, operation, color, reserved, vertices, indices = fields
        if magic != HMSH_MAGIC or version != HMSH_VERSION_LIVE:
            raise ValueError("expected HMSH v2")
        if operation not in _OPERATIONS or color not in (0, 1) or reserved:
            raise ValueError("invalid HMSH flags")
        stride = 16 if color else 12
        if len(payload) != HMSH_V2_HEADER.size + vertices * stride + indices * 4:
            raise ValueError("HMSH counts do not match payload length")
        if operation == HMSH_OPERATION_CLEAR:
            if chunk or vertices or indices or color or revision:
                raise ValueError("CLEAR must be an empty chunk-zero barrier")
        elif chunk > _MAX_CHUNK_ID or not revision:
            raise ValueError("HMSH chunk or revision outside supported range")
        elif operation == HMSH_OPERATION_DELETE:
            if vertices or indices or color:
                raise ValueError("DELETE must be header-only")
        elif not vertices or not indices or indices % 3:
            raise ValueError("incomplete HMSH triangles")
        if operation == HMSH_OPERATION_UPSERT:
            positions = np.ndarray((vertices, 3), dtype="<f4", buffer=payload,
                                   offset=HMSH_V2_HEADER.size, strides=(stride, 4))
            if not np.isfinite(positions).all():
                raise ValueError("non-finite HMSH vertex")
            index_array = np.frombuffer(payload, dtype="<u4", count=indices,
                                        offset=HMSH_V2_HEADER.size + vertices * stride)
            if int(index_array.max()) >= vertices:
                raise ValueError("HMSH index outside vertex array")
        else:
            positions = None
        return fields, positions

    def _derive(self, payload, fields, positions, *, baseline=False, color_weight=.1) -> AdaptiveMeshPage:
        _, _, chunk, x, y, z, epoch, revision, _, color, _, vertices, indices = fields
        stride = 16 if color else 12
        vertex_data = np.frombuffer(payload, dtype=np.uint8, count=vertices * stride,
                                    offset=HMSH_V2_HEADER.size)
        index_data = np.frombuffer(payload, dtype="<u4", count=indices,
                                   offset=HMSH_V2_HEADER.size + vertices * stride)
        output_vertices = np.empty(vertices * stride, dtype=np.uint8)
        output_indices = np.empty(indices, dtype=np.uint32)
        counts = (ctypes.c_size_t * 4)()
        diagnostics = (ctypes.c_size_t * 16)()
        error = ctypes.c_float()
        simplify = (self._native.horus_adaptive_mesh_baseline_diagnostics if baseline else
                    self._native.horus_adaptive_mesh_diagnostics)
        result = simplify(
            vertex_data.ctypes.data, vertices, stride, index_data.ctypes.data, indices,
            output_vertices.ctypes.data, output_indices.ctypes.data, counts,
            ctypes.c_float(1 / 9), ctypes.c_float(0.015), ctypes.c_float(color_weight), ctypes.byref(error), diagnostics)
        if result:
            raise RuntimeError(f"Adaptive mesh native simplification failed ({result})")
        coarse_vertices, coarse_indices, welded, boundary = counts
        if not (0 < coarse_vertices <= vertices and 0 < coarse_indices <= indices and coarse_indices % 3 == 0):
            raise RuntimeError("Adaptive mesh native kernel returned invalid counts")
        header = HMSH_V2_HEADER.pack(HMSH_MAGIC, HMSH_VERSION_LIVE, chunk, x, y, z,
                                     epoch, revision, HMSH_OPERATION_UPSERT, color, 0,
                                     coarse_vertices, coarse_indices)
        coarse = (header + output_vertices[:coarse_vertices * stride].tobytes() +
                  output_indices[:coarse_indices].tobytes())
        if len(coarse) >= len(payload):
            coarse, coarse_vertices, coarse_indices = payload, vertices, indices
        low, high = positions.min(axis=0), positions.max(axis=0)
        return AdaptiveMeshPage(chunk, (x, y, z), epoch, revision, payload, coarse,
            (tuple(map(float, low)), tuple(map(float, high))), vertices, coarse_vertices,
            indices // 3, coarse_indices // 3, float(error.value), welded, boundary,
            int(diagnostics[4]), int(diagnostics[2]),
            {0: "", 1: "empty_result", 2: "invalid_indices", 3: "nonfinite_error",
             4: "error_limit", 5: "boundary_changed", 6: "component_lost",
             7: "normalization_range", 8: "degenerate_only", 9: "unsafe_components_restored"}[diagnostics[3]],
            struct.unpack("<f", struct.pack("<I", diagnostics[5]))[0],
            int(diagnostics[8]), int(diagnostics[9]), int(diagnostics[10]), int(diagnostics[7]))

    def ingest(self, payload: bytes) -> bool:
        try:
            fields, positions = self._decode(payload)
        except (ValueError, OverflowError, struct.error) as error:
            self.last_rejection = str(error)
            return False
        _, _, chunk, _, _, _, epoch, revision, operation, *_ = fields
        if self.source_epoch is not None and epoch < self.source_epoch:
            self.last_rejection = "stale source epoch"
            return False
        reset = self.source_epoch != epoch
        if not reset and (operation == HMSH_OPERATION_CLEAR or
                          revision <= self._source_revisions.get(chunk, -1)):
            self.last_rejection = "stale source revision or delayed CLEAR"
            return False
        pages = {} if reset else self._pages.copy()
        tombstones = {} if reset else self._tombstones.copy()
        revisions = {} if reset else self._source_revisions.copy()
        if operation == HMSH_OPERATION_UPSERT:
            pages[chunk] = self._derive(payload, fields, positions)
            tombstones.pop(chunk, None)
            revisions[chunk] = revision
        elif operation == HMSH_OPERATION_DELETE:
            pages.pop(chunk, None)
            tombstones[chunk] = payload
            revisions[chunk] = revision
        proposed = sum(page.cached_bytes for page in pages.values()) + sum(map(len, tombstones.values()))
        proposed += sum(len(update.payload) for client in self._clients.values()
                        for update in client.inventory.values())
        if len(pages) + len(tombstones) > self.max_pages:
            self.last_rejection = "source inventory capacity exceeded; live pages plus tombstones exceed max_pages"
            return False
        if proposed > self.max_cache_bytes:
            self.last_rejection = "source cache capacity exceeded; previous complete map retained"
            return False
        self._pages, self._tombstones, self._source_revisions = pages, tombstones, revisions
        self.source_epoch = epoch
        self.last_rejection = ""
        return True

    def expire_clients(self, now: float) -> list[str]:
        if isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(now):
            raise ValueError("now must be a finite monotonic time")
        self._last_now = max(self._last_now, now)
        expired = [key for key, client in self._clients.items()
                   if self._last_now - client.last_seen >= _CLIENT_TTL]
        for key in expired:
            del self._clients[key]
        return expired

    def _select(self, regions, triangle_budget, memory_budget):
        pages = self._pages
        context_triangles = sum(page.coarse_triangles for page in pages.values())
        selected_triangles = context_triangles
        selected_bytes = sum(len(page.coarse) for page in pages.values()) + sum(map(len, self._tombstones.values()))
        fine: set[int] = set()
        chunks = np.array(sorted(pages), dtype=np.uint32)
        bounds = np.asarray([pages[int(chunk)].bounds for chunk in chunks], dtype=np.float64).reshape((-1, 2, 3))
        activation = {region.id: _activation(region, bounds) for region in regions}
        def intersecting(region, fraction):
            if fraction <= 0:
                return set()
            return set(map(int, chunks[activation[region.id] <= fraction + 1e-12]))
        triangle_cost = {chunk: page.fine_triangles - page.coarse_triangles for chunk, page in pages.items()}
        byte_cost = {chunk: len(page.fine) - len(page.coarse) for chunk, page in pages.items()}
        def costs(added):
            return sum(triangle_cost[chunk] for chunk in added), sum(byte_cost[chunk] for chunk in added)
        fractions = {region.id: 0. for region in regions}
        feasible = selected_triangles <= triangle_budget and selected_bytes <= memory_budget
        reason = "" if feasible else "context_exceeds_budget"
        if feasible:
            cores = set().union(*(intersecting(region, region.min_fraction) for region in regions))
            triangles, size = costs(cores)
            if selected_triangles + triangles > triangle_budget or selected_bytes + size > memory_budget:
                feasible, reason = False, "minimum_cores_exceed_budget"
            else:
                fine = cores
                selected_triangles += triangles
                selected_bytes += size
                fractions = {region.id: region.min_fraction for region in regions}
        if feasible:
            blocked = set()
            while True:
                active = [region for region in regions if fractions[region.id] < 1 and region.id not in blocked]
                if not active:
                    break
                region = min(active, key=lambda value: (
                    (fractions[value.id] - value.min_fraction) / (value.priority + 1),
                    -value.priority, value.id))
                fraction = min(1., round(fractions[region.id] + .05, 12))
                added = intersecting(region, fraction) - fine
                triangles, size = costs(added)
                if selected_triangles + triangles > triangle_budget or selected_bytes + size > memory_budget:
                    blocked.add(region.id)
                    continue
                fine.update(added)
                selected_triangles += triangles
                selected_bytes += size
                fractions[region.id] = fraction
                # Another region can make a formerly blocked overlapping growth free.
                blocked.clear()
        detail = []
        for region in regions:
            candidates = intersecting(region, 1.)
            fraction = fractions[region.id]
            detail.append({"id": region.id, "pages": len(candidates),
                "fine_pages": len(candidates & fine), "fraction": fraction,
                "core_ready": feasible and fraction >= region.min_fraction,
                "ready": fraction == 1.})
        grants = [{"id": region.id, "shape": region.shape, "center": region.center.tolist(),
                   "size": (region.half_size * (2 * fractions[region.id])).tolist(),
                   "rotation": list(region.quaternion), "fraction": fractions[region.id]}
                  for region in regions if fractions[region.id] > 0]
        status = {
            "selected_triangles": selected_triangles, "context_triangles": context_triangles,
            "fine_pages": len(fine), "context_pages": len(pages) - len(fine),
            "over_budget": not feasible,
            "source_triangles": sum(page.fine_triangles for page in pages.values()),
            "selected_bytes": selected_bytes, "regions_requested": len(regions),
            "regions_ready": sum(item["ready"] for item in detail),
            "triangle_budget": triangle_budget, "memory_budget": memory_budget,
            "budget_feasible": feasible, "reason": reason,
            "granted_regions": grants,
            "degraded": not feasible or any(fractions[region.id] < 1. for region in regions),
            "coarse_target_ratio": 1 / 9,
            "coarse_actual_ratio": context_triangles / sum(page.fine_triangles for page in pages.values()) if pages else 1.,
            "max_simplification_error_m": max((page.simplification_error_m for page in pages.values()), default=0.),
            "coarse_fallback_pages": {
                reason: sum(page.native_fallback_reason == reason for page in pages.values())
                for reason in sorted({page.native_fallback_reason for page in pages.values()} - {""})},
            "coarse_zero_area_faces": sum(page.degenerate_triangles for page in pages.values()),
            "coarse_restored_components": sum(page.restored_components for page in pages.values()),
            "coarse_restored_triangles": sum(page.restored_triangles for page in pages.values()),
            "regions": detail, "memory_accounting": "HMSH payload bytes",
        }
        return fine, status

    def request_view(self, client_id, regions: list[dict], triangle_budget: int,
                     memory_budget: int, now: float):
        if not isinstance(client_id, str) or not re.fullmatch(r"[0-9a-f]{32}", client_id):
            raise ValueError("client_id must be 32 lowercase hexadecimal characters")
        triangle_budget = _integer(triangle_budget, "triangle_budget")
        memory_budget = _integer(memory_budget, "memory_budget")
        regions = _regions(regions)
        self.expire_clients(now)
        previous = self._clients.get(client_id)
        if previous is None and len(self._clients) >= _MAX_CLIENTS:
            raise ValueError("adaptive mesh supports at most four active clients")
        fine, status = self._select(regions, triangle_budget, memory_budget)
        reset = previous is None or previous.source_epoch != self.source_epoch
        # Keep last committed payloads for repair, and account for all four views.
        source_bytes = sum(page.cached_bytes for page in self._pages.values()) + sum(map(len, self._tombstones.values()))
        other_bytes = sum(len(update.payload) for key, client in self._clients.items() if key != client_id
                          for update in client.inventory.values())
        if source_bytes + other_bytes + status["selected_bytes"] > self.max_cache_bytes:
            raise MemoryError("adaptive mesh cache cannot retain the complete client view")
        epoch = self._epoch_clock + 1 if reset else previous.epoch
        if epoch > _MAX_UINT64:
            raise OverflowError("adaptive view epoch exhausted")
        needed_ids = set(self._pages) | set(self._tombstones)
        client = _Client(epoch, self.source_epoch, self._last_now,
                         {key: value for key, value in previous.revisions.items()
                          if key in needed_ids} if previous else {})
        updates = [_clear(epoch)] if reset else []
        for chunk in sorted(set(self._pages) | set(self._tombstones)):
            page = self._pages.get(chunk)
            if page is None:
                payload = self._tombstones[chunk]
                source_revision = self._source_revisions[chunk]
                lod = "delete"
            else:
                lod = "fine" if chunk in fine else "coarse"
                payload = page.fine if lod == "fine" else page.coarse
                source_revision = page.source_revision
            signature = (self.source_epoch, source_revision, lod)
            if not reset and previous.signatures.get(chunk) == signature:
                update = previous.inventory[chunk]
            else:
                revision = client.revisions.get(chunk, 0) + 1
                if revision > _MAX_UINT64:
                    raise OverflowError("adaptive view revision exhausted")
                client.revisions[chunk] = revision
                update = _update(_view_payload(payload, epoch, revision))
                updates.append(update)
            client.inventory[chunk] = update
            client.signatures[chunk] = signature
        client.status = status
        self._clients[client_id] = client
        if reset:
            self._epoch_clock = epoch
        return updates, self._manifest(client), self._status_copy(status)

    @staticmethod
    def _status_copy(status):
        return deepcopy(status)

    def _manifest(self, client):
        return {"version": 1, "epoch": str(client.epoch),
                "pages": [{"chunk_id": update.chunk_id, "revision": str(update.revision),
                           "operation": update.operation} for update in client.inventory.values()],
                "triangles": client.status["selected_triangles"],
                "adaptive": self._status_copy(client.status)}

    def snapshot(self, client_id, request) -> list[MeshPageUpdate]:
        if not isinstance(client_id, str):
            return []
        client = self._clients.get(client_id)
        if (client is None or not isinstance(request, dict) or
                not isinstance(request.get("request"), str) or
                request["request"] not in {"snapshot", "pages"}):
            return []
        inventory = client.inventory
        if request["request"] == "pages":
            ids = request.get("ids")
            if (not isinstance(ids, list) or len(ids) > self.max_pages or
                    any(type(chunk) is not int or not 0 <= chunk <= _MAX_CHUNK_ID for chunk in ids)):
                return []
        else:
            ids = list(inventory)
        requested_epoch = request.get("epoch")
        if requested_epoch is not None and str(requested_epoch) != str(client.epoch):
            return [_clear(client.epoch), *inventory.values()]
        return [inventory[chunk] for chunk in sorted(set(ids)) if chunk in inventory]


def probe_saved_pages(directory, *, limit=0, compare_baseline=True):
    """Offline per-page diagnostics using the same kernel and immutable HMSH inputs."""
    from collections import Counter
    import hashlib

    ensure_native()
    store = AdaptiveMeshStore()
    files = sorted(Path(directory).glob("*.hmsh"))
    if not files:
        raise ValueError("saved-page probe needs a directory containing .hmsh files")
    if limit:
        files = files[:limit]
    rows = []
    started = time.monotonic()
    for path in files:
        payload = path.read_bytes()
        checksum = hashlib.sha256(payload).digest()
        fields, positions = store._decode(payload)
        after = store._derive(payload, fields, positions)
        before = store._derive(payload, fields, positions, baseline=True) if compare_baseline else None
        if hashlib.sha256(after.fine).digest() != checksum or hashlib.sha256(path.read_bytes()).digest() != checksum:
            raise RuntimeError("native probe changed original source data")
        rows.append({"chunk_id": after.chunk_id, "fine": after.fine_triangles,
            "coarse": after.coarse_triangles, "welded_vertices": after.welded_vertices,
            "locked_vertices": after.boundary_vertices, "zero_area_faces": after.degenerate_triangles,
            "fallback": after.native_fallback_reason, "meshopt_triangles": after.native_simplified_triangles,
            "error_m": after.simplification_error_m,
            "source_components": after.source_components,
            "restored_components": after.restored_components,
            "restored_triangles": after.restored_triangles,
            "largest_restored_component": after.largest_restored_component,
            "before_coarse": before.coarse_triangles if before else None,
            "before_fallback": before.native_fallback_reason if before else None})
    return {"elapsed_seconds": time.monotonic() - started, "pages": len(rows),
        "source_triangles": sum(row["fine"] for row in rows),
        "coarse_triangles": sum(row["coarse"] for row in rows),
        "before_triangles": sum(row["before_coarse"] or 0 for row in rows) if compare_baseline else None,
        "zero_area_faces": sum(row["zero_area_faces"] for row in rows),
        "fallbacks": dict(Counter(row["fallback"] for row in rows if row["fallback"])),
        "before_fallbacks": dict(Counter(row["before_fallback"] for row in rows if row["before_fallback"])),
        "max_error_m": max(row["error_m"] for row in rows), "page_diagnostics": rows}


if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(description="Offline immutable saved-mesh simplification probe")
    parser.add_argument("directory", type=Path)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--no-baseline", action="store_true")
    arguments = parser.parse_args()
    print(json.dumps(probe_saved_pages(arguments.directory, limit=arguments.limit,
                                      compare_baseline=not arguments.no_baseline), sort_keys=True))
