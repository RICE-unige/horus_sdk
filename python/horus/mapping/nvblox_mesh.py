"""Full-geometry nvblox mesh-page accumulator and HMSH v2 encoder.

The ROS-facing node lives in ``python/examples/tools``. This module intentionally
has no ROS imports so its state transitions and binary contract can be tested on
any machine. HMSH stores float32 coordinates and RGBA8 colors without decimation.
"""

from __future__ import annotations

from collections import defaultdict
import copy
import heapq
import math
import time
from typing import Sequence
from types import SimpleNamespace


from .mesh_protocol import (
    HMSH_MAGIC, HMSH_VERSION_LIVE, HMSH_OPERATION_UPSERT,
    HMSH_OPERATION_DELETE, HMSH_OPERATION_CLEAR, HMSH_V2_HEADER,
    FLOAT32_MAX, BlockKey, BlockGeometry, MeshPageUpdate, LiveHeader,
    decode_live_header, encode_live_payload,
)


def _rgba8(color: object) -> tuple[int, int, int, int]:
    def channel(name: str, default: float) -> int:
        value = float(getattr(color, name, default))
        if not math.isfinite(value):
            value = default
        return max(0, min(255, int(round(value * 255.0))))

    return channel("r", 0.7), channel("g", 0.72), channel("b", 0.78), channel("a", 1.0)


def _source_colors_rgba8(colors: Sequence[object]) -> tuple[tuple[int, int, int, int], ...]:
    if len(colors) < 64:
        return tuple(_rgba8(color) for color in colors)
    try:
        import numpy as np
    except ImportError:
        return tuple(_rgba8(color) for color in colors)
    values = np.asarray([(float(getattr(c, "r", .7)), float(getattr(c, "g", .72)),
                          float(getattr(c, "b", .78)), float(getattr(c, "a", 1.0)))
                         for c in colors], dtype=np.float64)
    defaults = np.asarray([.7, .72, .78, 1.0], dtype=np.float64)
    values = np.where(np.isfinite(values), values, defaults)
    # Float64 round-to-even matches the scalar encoder, including half values.
    quantized = np.rint(np.clip(values, 0.0, 1.0) * 255.0).astype(np.uint8)
    return tuple(map(tuple, quantized.tolist()))


def _normal_color(normal: object) -> tuple[int, int, int, int]:
    components = (
        abs(float(getattr(normal, "x", 0.0))),
        abs(float(getattr(normal, "y", 0.0))),
        abs(float(getattr(normal, "z", 1.0))),
    )
    length = math.sqrt(sum(value * value for value in components))
    if not math.isfinite(length) or length < 1e-8:
        components = (0.0, 0.0, 1.0)
        length = 1.0
    rgb = tuple(max(0, min(255, int(round((0.25 + 0.75 * value / length) * 255.0)))) for value in components)
    return rgb[0], rgb[1], rgb[2], 255


def _fallback_rgba8(value: Sequence[float]) -> tuple[int, int, int, int]:
    values = list(value[:4])
    while len(values) < 4:
        values.append(1.0)
    return tuple(max(0, min(255, int(round(float(channel) * 255.0)))) for channel in values)  # type: ignore[return-value]


def decode_mesh_block(
    block: object,
    *,
    color_mode: str,
    fallback_color: Sequence[float],
) -> BlockGeometry | None:
    vertices_raw = tuple(getattr(block, "vertices", ()) or ())
    triangles_raw = tuple(int(index) for index in (getattr(block, "triangles", ()) or ()))
    if not vertices_raw and not triangles_raw:
        return None
    if not vertices_raw or not triangles_raw or len(triangles_raw) % 3:
        raise ValueError("nvblox mesh has incomplete triangle geometry")

    vertices: list[tuple[float, float, float]] = []
    for vertex in vertices_raw:
        point = (
            float(getattr(vertex, "x", 0.0)),
            float(getattr(vertex, "y", 0.0)),
            float(getattr(vertex, "z", 0.0)),
        )
        if not all(math.isfinite(component) for component in point):
            raise ValueError("nvblox mesh contains a non-finite vertex")
        if any(abs(component) > FLOAT32_MAX for component in point):
            raise ValueError("nvblox mesh vertex exceeds the HMSH float32 coordinate range")
        vertices.append(point)

    triangles: list[int] = []
    usable_indices = len(triangles_raw) - (len(triangles_raw) % 3)
    for start in range(0, usable_indices, 3):
        triangle = triangles_raw[start : start + 3]
        if min(triangle) < 0 or max(triangle) >= len(vertices):
            raise ValueError("nvblox mesh contains an out-of-range triangle index")
        if triangle[0] == triangle[1] or triangle[1] == triangle[2] or triangle[0] == triangle[2]:
            continue
        triangles.extend(triangle)
    if not triangles:
        return None

    source_colors = tuple(getattr(block, "colors", ()) or ())
    normals = tuple(getattr(block, "normals", ()) or ())
    fallback = _fallback_rgba8(fallback_color)
    normalized_mode = color_mode.strip().lower()
    if normalized_mode not in {"auto", "source", "normals", "fallback"}:
        raise ValueError(f"unsupported nvblox color mode: {color_mode}")

    if normalized_mode in {"auto", "source"} and len(source_colors) == len(vertices):
        colors = _source_colors_rgba8(source_colors)
    elif normalized_mode in {"auto", "normals"} and len(normals) == len(vertices):
        colors = tuple(_normal_color(normal) for normal in normals)
    else:
        colors = tuple(fallback for _ in vertices)

    return BlockGeometry(tuple(vertices), colors, tuple(triangles))


class PendingNvbloxMesh:
    """Bound queued source work by block identity, not a FIFO of old room maps.

    Used on the ROS executor thread. A taken batch is owned exclusively by one
    worker; newer observations replace only pending versions of the same block.
    Explicit clears are barriers and empty blocks remain deletion tombstones.
    """

    def __init__(self, max_blocks=1024 * 512, max_bytes=512 * 1024 * 1024):
        self.max_blocks, self.max_bytes = max_blocks, max_bytes
        self.frame_id = ""
        self.clear = False
        self.blocks = {}
        self.bytes = 0

    def add(self, message):
        indices, blocks = message.block_indices, message.blocks
        if len(indices) != len(blocks):
            raise ValueError("nvblox block_indices/blocks length mismatch")
        frame = str(message.header.frame_id or "").strip()
        clear = bool(message.clear)
        if frame and self.frame_id and frame != self.frame_id and not clear:
            raise ValueError("nvblox frame changed without a map clear")
        replacements = {}
        for index, block in zip(indices, blocks):
            key = (int(index.x), int(index.y), int(index.z))
            # Conservative retained-work accounting including Python source data.
            size = 256 + len(block.vertices) * 192 + len(block.triangles) * 32
            size += len(block.colors) * 192 + len(block.normals) * 160
            replacements[key] = (index, block, size)
        base = {} if clear else self.blocks
        size = (0 if clear else self.bytes) + sum(entry[2] - (base[key][2] if key in base else 0)
                                                for key, entry in replacements.items())
        count = len(base) + sum(key not in base for key in replacements)
        if count > self.max_blocks or size > self.max_bytes:
            raise ValueError("nvblox pending source work exceeds its bounded capacity; previous pending map retained")
        if clear:
            self.blocks.clear()
            self.clear = True
        self.blocks.update(replacements)
        self.bytes = size
        self.frame_id = frame or self.frame_id

    def take(self):
        if not self.blocks and not self.clear:
            return None
        entries = tuple(self.blocks.values())
        batch = SimpleNamespace(header=SimpleNamespace(frame_id=self.frame_id), clear=self.clear,
            block_indices=tuple(entry[0] for entry in entries), blocks=tuple(entry[1] for entry in entries))
        self.blocks.clear()
        self.bytes = 0
        self.clear = False
        return batch


class LiveNvbloxMeshState:
    """Retains nvblox blocks and emits spatial pages without decimating geometry."""

    def __init__(
        self,
        *,
        blocks_per_axis: int = 2,
        max_pages: int = 1024,
        max_payload_bytes: int = 16 * 1024 * 1024,
        color_mode: str = "auto",
        fallback_color: Sequence[float] = (0.70, 0.72, 0.78, 1.0),
        initial_epoch: int | None = None,
    ) -> None:
        if blocks_per_axis < 1 or blocks_per_axis > 8:
            raise ValueError("blocks_per_axis must be between 1 and 8")
        if max_pages < 1 or max_pages > 1024:
            raise ValueError("max_pages must be between 1 and 1024")
        self.blocks_per_axis = int(blocks_per_axis)
        self.max_pages = int(max_pages)
        self.max_payload_bytes = max(HMSH_V2_HEADER.size, int(max_payload_bytes))
        self.color_mode = color_mode
        self.fallback_color = tuple(float(value) for value in fallback_color)
        self.epoch = int(initial_epoch if initial_epoch is not None else time.time_ns()) & 0xFFFFFFFFFFFFFFFF
        self.frame_id = "map"
        self._blocks: dict[BlockKey, BlockGeometry] = {}
        self._height_bounds_by_block: dict[BlockKey, tuple[float, float]] = {}
        self._page_blocks: dict[BlockKey, set[BlockKey]] = defaultdict(set)
        self._chunk_by_page: dict[BlockKey, int] = {}
        self._revision_by_chunk: dict[int, int] = {}
        self._current_payload_by_page: dict[BlockKey, MeshPageUpdate] = {}
        self._deleted_payload_by_id: dict[int, MeshPageUpdate] = {}
        self._free_chunk_ids: list[int] = []
        self._next_chunk_id = 1

    @property
    def block_count(self) -> int:
        return len(self._blocks)

    @property
    def page_count(self) -> int:
        return len(self._current_payload_by_page)

    @property
    def triangle_count(self) -> int:
        return sum(update.triangle_count for update in self._current_payload_by_page.values())

    def page_key(self, block_key: BlockKey) -> BlockKey:
        size = self.blocks_per_axis
        return block_key[0] // size, block_key[1] // size, block_key[2] // size

    def clear(self) -> MeshPageUpdate:
        self.epoch = (self.epoch + 1) & 0xFFFFFFFFFFFFFFFF
        self._blocks.clear()
        self._height_bounds_by_block.clear()
        self._page_blocks.clear()
        self._chunk_by_page.clear()
        self._revision_by_chunk.clear()
        self._current_payload_by_page.clear()
        self._deleted_payload_by_id.clear()
        self._free_chunk_ids.clear()
        self._next_chunk_id = 1
        return self.snapshot_clear_update()

    def snapshot_clear_update(self) -> MeshPageUpdate:
        """Create a clear command for the current epoch without altering map state."""
        payload = encode_live_payload(
            chunk_id=0,
            page_key=(0, 0, 0),
            epoch=self.epoch,
            revision=0,
            operation=HMSH_OPERATION_CLEAR,
            geometry=None,
            max_payload_bytes=self.max_payload_bytes,
        )
        return MeshPageUpdate(
            chunk_id=0,
            page_key=(0, 0, 0),
            epoch=self.epoch,
            revision=0,
            operation=HMSH_OPERATION_CLEAR,
            payload=payload,
            vertex_count=0,
            triangle_count=0,
        )

    def apply_message(self, message: object) -> list[MeshPageUpdate]:
        indices = tuple(getattr(message, "block_indices", ()) or ())
        blocks = tuple(getattr(message, "blocks", ()) or ())
        if len(indices) != len(blocks):
            raise ValueError(
                f"nvblox block_indices/blocks length mismatch: {len(indices)} != {len(blocks)}"
            )

        clear = bool(getattr(message, "clear", False))
        replacements: dict[BlockKey, BlockGeometry | None] = {}
        for index, block in zip(indices, blocks):
            block_key = (
                int(getattr(index, "x", 0)),
                int(getattr(index, "y", 0)),
                int(getattr(index, "z", 0)),
            )
            replacements[block_key] = decode_mesh_block(
                block,
                color_mode=self.color_mode,
                fallback_color=self.fallback_color,
            )

        replacements = {
            key: geometry for key, geometry in replacements.items()
            if geometry != (None if clear else self._blocks.get(key))
        }
        header = getattr(message, "header", None)
        frame_id = str(getattr(header, "frame_id", "") or "").strip()
        if frame_id and frame_id != self.frame_id and self._blocks and not clear:
            raise ValueError("nvblox frame changed without a map clear")
        if not replacements and not clear:
            if frame_id:
                self.frame_id = frame_id
            return []

        # Geometry is immutable: stage references and copy only affected page sets.
        # Commit only after every page has passed capacity and encoding checks.
        staged = copy.copy(self)
        staged._blocks = self._blocks.copy()
        staged._height_bounds_by_block = self._height_bounds_by_block.copy()
        staged._page_blocks = self._page_blocks.copy()
        staged._chunk_by_page = self._chunk_by_page.copy()
        staged._revision_by_chunk = self._revision_by_chunk.copy()
        staged._current_payload_by_page = self._current_payload_by_page.copy()
        staged._deleted_payload_by_id = self._deleted_payload_by_id.copy()
        staged._free_chunk_ids = self._free_chunk_ids.copy()
        updates = [staged.clear()] if clear else []
        if frame_id:
            staged.frame_id = frame_id
        dirty_pages = {staged.page_key(key) for key in replacements}
        for key in dirty_pages:
            staged._page_blocks[key] = set(staged._page_blocks.get(key, ()))
        for key, geometry in replacements.items():
            page_key = staged.page_key(key)
            if geometry is None:
                staged._blocks.pop(key, None)
                staged._height_bounds_by_block.pop(key, None)
                staged._page_blocks[page_key].discard(key)
            else:
                staged._blocks[key] = geometry
                heights = [point[2] for point in geometry.vertices]
                staged._height_bounds_by_block[key] = (min(heights), max(heights))
                staged._page_blocks[page_key].add(key)
        active_pages = sum(bool(keys) for keys in staged._page_blocks.values())
        if active_pages > self.max_pages:
            raise RuntimeError(
                f"nvblox map exceeds max_pages={self.max_pages}; increase --blocks-per-axis"
            )
        # Release deleted slots first. Reused slots keep increasing revisions so
        # delayed packets cannot resurrect the previous spatial page.
        for page_key in sorted(dirty_pages, key=lambda key: (bool(staged._page_blocks[key]), key)):
            updates.append(staged._rebuild_page(page_key))
        self.__dict__.update(staged.__dict__)
        return updates

    def snapshot_updates(
        self, *, include_clear: bool = False, include_deletes: bool = False
    ) -> list[MeshPageUpdate]:
        updates = [
            self._current_payload_by_page[key]
            for key in sorted(self._current_payload_by_page)
        ]
        if include_deletes:
            updates.extend(self._deleted_payload_by_id[key] for key in sorted(self._deleted_payload_by_id))
        if include_clear:
            updates.insert(0, self.snapshot_clear_update())
        return updates

    def manifest(self) -> dict:
        """Small revision inventory; replay can repair dropped application frames."""
        return {
            "version": 1,
            "epoch": str(self.epoch),
            "frame_id": self.frame_id,
            "pages": [
                {"chunk_id": update.chunk_id, "revision": str(update.revision),
                 "operation": update.operation}
                for update in sorted(self.snapshot_updates(include_deletes=True), key=lambda item: item.chunk_id)
            ],
            "triangles": self.triangle_count,
            "height_bounds_m": ([min(value[0] for value in self._height_bounds_by_block.values()),
                                  max(value[1] for value in self._height_bounds_by_block.values())]
                                 if self._height_bounds_by_block else None),
        }

    def repair_updates(self, request: object) -> list[MeshPageUpdate]:
        """Return current revisions, never an obsolete requested revision."""
        updates = self.snapshot_updates(include_deletes=True)
        if not updates:
            return [self.snapshot_clear_update()]
        if isinstance(request, dict) and request.get("request") == "pages" and str(request.get("epoch")) == str(self.epoch):
            ids = request.get("ids")
            if (isinstance(ids, list) and len(ids) <= self.max_pages
                    and all(isinstance(value, int) and not isinstance(value, bool) for value in ids)):
                requested = set(ids)
                return [update for update in updates if update.chunk_id in requested]
        return updates

    def _chunk_id(self, page_key: BlockKey) -> int:
        chunk_id = self._chunk_by_page.get(page_key)
        if chunk_id is not None:
            return chunk_id
        if not self._free_chunk_ids and self._next_chunk_id > self.max_pages:
            raise RuntimeError(
                f"nvblox map exceeds max_pages={self.max_pages}; increase --blocks-per-axis"
            )
        if self._free_chunk_ids:
            chunk_id = heapq.heappop(self._free_chunk_ids)
        else:
            chunk_id = self._next_chunk_id
            self._next_chunk_id += 1
        self._chunk_by_page[page_key] = chunk_id
        return chunk_id

    def _next_revision(self, chunk_id: int) -> int:
        revision = self._revision_by_chunk.get(chunk_id, 0) + 1
        self._revision_by_chunk[chunk_id] = revision
        return revision

    def _rebuild_page(self, page_key: BlockKey) -> MeshPageUpdate:
        chunk_id = self._chunk_id(page_key)
        revision = self._next_revision(chunk_id)
        block_keys = sorted(self._page_blocks.get(page_key, ()))
        if not block_keys:
            self._page_blocks.pop(page_key, None)
            self._current_payload_by_page.pop(page_key, None)
            payload = encode_live_payload(
                chunk_id=chunk_id,
                page_key=page_key,
                epoch=self.epoch,
                revision=revision,
                operation=HMSH_OPERATION_DELETE,
                geometry=None,
                max_payload_bytes=self.max_payload_bytes,
            )
            update = MeshPageUpdate(
                chunk_id,
                page_key,
                self.epoch,
                revision,
                HMSH_OPERATION_DELETE,
                payload,
                0,
                0,
            )
            self._chunk_by_page.pop(page_key)
            heapq.heappush(self._free_chunk_ids, chunk_id)
            self._deleted_payload_by_id[chunk_id] = update
            return update

        vertices: list[tuple[float, float, float]] = []
        colors: list[tuple[int, int, int, int]] = []
        triangles: list[int] = []
        for block_key in block_keys:
            block = self._blocks[block_key]
            vertex_offset = len(vertices)
            vertices.extend(block.vertices)
            colors.extend(block.colors)
            triangles.extend(vertex_offset + index for index in block.triangles)
        geometry = BlockGeometry(tuple(vertices), tuple(colors), tuple(triangles))
        payload = encode_live_payload(
            chunk_id=chunk_id,
            page_key=page_key,
            epoch=self.epoch,
            revision=revision,
            operation=HMSH_OPERATION_UPSERT,
            geometry=geometry,
            max_payload_bytes=self.max_payload_bytes,
        )
        update = MeshPageUpdate(
            chunk_id,
            page_key,
            self.epoch,
            revision,
            HMSH_OPERATION_UPSERT,
            payload,
            len(vertices),
            len(triangles) // 3,
        )
        self._current_payload_by_page[page_key] = update
        self._deleted_payload_by_id.pop(chunk_id, None)
        return update
