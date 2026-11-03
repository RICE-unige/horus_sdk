"""Source-independent HMSH v2 map state. No ROS or renderer dependencies.

Each key identifies a complete replacement, not an incremental vertex patch.
Geometry uses the existing HORUS float32/RGBA8 wire representation. No triangles
are removed. State is committed only after all size/encoding checks succeed.
"""

import heapq
import math
from dataclasses import replace
import struct
import time

from .mesh_protocol import (
    BlockGeometry, MeshPageUpdate, HMSH_V2_HEADER, encode_live_payload,
    HMSH_OPERATION_UPSERT, HMSH_OPERATION_DELETE, HMSH_OPERATION_CLEAR,
)


class LiveMeshState:
    def __init__(self, *, frame_id="map", max_pages=1024,
                 max_payload_bytes=1024 * 1024, max_cache_bytes=256 * 1024 * 1024,
                 initial_epoch=None):
        if not frame_id or not 1 <= max_pages <= 1024:
            raise ValueError("a fixed frame and 1..1024 pages are required")
        if max_payload_bytes < HMSH_V2_HEADER.size or max_cache_bytes < max_payload_bytes:
            raise ValueError("invalid mesh payload/cache byte limits")
        self.frame_id = frame_id
        self.max_pages = max_pages
        self.max_payload_bytes = max_payload_bytes
        self.max_cache_bytes = max_cache_bytes
        self.epoch = time.time_ns() if initial_epoch is None else initial_epoch
        if not 0 <= self.epoch < 2**64 - 1:
            raise ValueError("epoch outside uint64 range")
        self._keys = {}
        self._current = {}
        self._revisions = {}
        self._free = []
        self._next_id = 1
        self.cached_bytes = 0
        self.unchanged_pages = 0
        self.suppressed_bytes = 0

    @property
    def page_count(self):
        return len(self._keys)

    def _encode(self, chunk_id, revision, operation, geometry=None):
        payload = encode_live_payload(
            chunk_id=chunk_id, page_key=(0, 0, 0), epoch=self.epoch,
            revision=revision, operation=operation, geometry=geometry,
            max_payload_bytes=self.max_payload_bytes)
        return MeshPageUpdate(chunk_id, (0, 0, 0), self.epoch, revision,
                              operation, payload,
                              len(geometry.vertices) if geometry else 0,
                              len(geometry.triangles) // 3 if geometry else 0)

    def replace(self, key, geometry: BlockGeometry | None):
        """Return changed page(s); timestamps/TTLs belong to the source adapter."""
        if geometry is not None:
            if (not geometry.vertices or not geometry.triangles or len(geometry.triangles) % 3
                    or len(geometry.colors) not in (0, len(geometry.vertices))):
                raise ValueError("incomplete mesh geometry/colors")
            if any(type(i) is not int or not 0 <= i < len(geometry.vertices) for i in geometry.triangles):
                raise ValueError("mesh index outside vertex array")
            if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in geometry.vertices):
                raise ValueError("invalid mesh coordinates")
            if any(len(c) != 4 or not all(type(v) is int and 0 <= v <= 255 for v in c) for c in geometry.colors):
                raise ValueError("invalid RGBA8 mesh colors")
        chunk_id = self._keys.get(key)
        if geometry is None and chunk_id is None:
            return []
        is_new = chunk_id is None
        if is_new:
            if not self._free and self._next_id > self.max_pages:
                raise ValueError("live mesh page capacity exceeded; previous map retained")
            chunk_id = self._free[0] if self._free else self._next_id
        old = self._current.get(chunk_id)
        revision = self._revisions.get(chunk_id, 0) + 1
        if revision >= 2**64:
            raise ValueError("revision exhausted; clear the map before reuse")
        operation = HMSH_OPERATION_DELETE if geometry is None else HMSH_OPERATION_UPSERT
        update = self._encode(chunk_id, old.revision if old else revision, operation, geometry)
        # Compare encoded content exactly, excluding only identity/revision fields.
        # Header counts/flags are included: equal byte lengths alone are insufficient.
        if old and old.payload == update.payload:
            self.unchanged_pages += 1
            self.suppressed_bytes += len(update.payload)
            return []
        if old:
            update = replace(update, revision=revision,
                             payload=update.payload[:32] + struct.pack("<Q", revision) + update.payload[40:])
        size = self.cached_bytes - (len(old.payload) if old else 0) + len(update.payload)
        if size > self.max_cache_bytes:
            raise ValueError("live mesh cache byte capacity exceeded; previous map retained")
        if is_new:
            if self._free:
                heapq.heappop(self._free)
            else:
                self._next_id += 1
            self._keys[key] = chunk_id
        if geometry is None:
            del self._keys[key]
            heapq.heappush(self._free, chunk_id)
        self._current[chunk_id] = update
        self._revisions[chunk_id] = revision
        self.cached_bytes = size
        return [update]

    def replace_many(self, changes):
        """Atomically replace multiple keys (for a split Marker or MarkerArray)."""
        import copy
        staged = copy.copy(self)
        for name in ("_keys", "_current", "_revisions"):
            setattr(staged, name, getattr(self, name).copy())
        staged._free = self._free.copy()
        updates = []
        for key, geometry in changes:
            updates.extend(staged.replace(key, geometry))
        self.__dict__.update(staged.__dict__)
        return updates

    def clear(self):
        if self.epoch >= 2**64 - 1:
            raise ValueError("map epoch exhausted")
        self.epoch += 1
        self._keys.clear()
        self._current.clear()
        self._revisions.clear()
        self._free.clear()
        self._next_id = 1
        self.cached_bytes = 0
        return self.snapshot_clear_update()

    def snapshot_clear_update(self):
        return self._encode(0, 0, HMSH_OPERATION_CLEAR)

    def snapshot_updates(self):
        return [self._current[key] for key in sorted(self._current)]

    def manifest(self):
        return {"version": 1, "epoch": str(self.epoch), "frame_id": self.frame_id,
                "pages": [{"chunk_id": u.chunk_id, "revision": str(u.revision),
                           "operation": u.operation} for u in self.snapshot_updates()],
                "triangles": sum(u.triangle_count for u in self._current.values())}

    def repair_updates(self, request):
        if not isinstance(request, dict) or request.get("request") not in ("snapshot", "pages"):
            return []
        if not self._current:
            return [self.snapshot_clear_update()]
        if request["request"] == "snapshot" or str(request.get("epoch")) != str(self.epoch):
            return self.snapshot_updates()
        ids = request.get("ids")
        if not isinstance(ids, list) or len(ids) > self.max_pages or any(
                type(key) is not int or not 1 <= key <= self.max_pages for key in ids):
            return []
        return [self._current[key] for key in sorted(set(ids)) if key in self._current]
