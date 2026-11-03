"""Fair, byte-paced latest-revision delivery for HMSH live map pages."""

from collections import OrderedDict
import math

from .mesh_protocol import HMSH_OPERATION_CLEAR, MeshPageUpdate


class MeshPageScheduler:
    def __init__(self, *, pages_per_second: float, bytes_per_second: float,
                 max_pages: int, max_payload_bytes: int, now: float) -> None:
        if not math.isfinite(now):
            raise ValueError("mesh scheduler clock must be finite")
        if not all(math.isfinite(value) and value > 0
                   for value in (pages_per_second, bytes_per_second)):
            raise ValueError("page and byte rates must be finite and positive")
        if max_pages < 1 or max_payload_bytes < 1:
            raise ValueError("page and payload capacities must be positive")
        self.pages_per_second = pages_per_second
        self.bytes_per_second = bytes_per_second
        self.max_pages = max_pages
        self.max_payload_bytes = max_payload_bytes
        self.epoch = -1
        self._clear: MeshPageUpdate | None = None
        self._pages: OrderedDict[int, MeshPageUpdate] = OrderedDict()
        self._page_tokens = 0.0
        self._byte_tokens = 0.0
        self._last_time = now

    @property
    def pending_count(self) -> int:
        return len(self._pages) + int(self._clear is not None)

    @property
    def pending_bytes(self) -> int:
        return sum(len(page.payload) for page in self._pages.values()) + (
            len(self._clear.payload) if self._clear is not None else 0)

    def enqueue(self, update: MeshPageUpdate) -> bool:
        if len(update.payload) > self.max_payload_bytes:
            raise ValueError("mesh page exceeds scheduler payload capacity")
        if update.epoch < self.epoch:
            return False
        new_epoch = update.epoch > self.epoch
        if new_epoch:
            self._pages.clear()
            self._clear = None
            self.epoch = update.epoch
        if update.operation == HMSH_OPERATION_CLEAR:
            # A duplicate/delayed clear must not discard that epoch's queued map.
            if new_epoch or not self._pages:
                self._clear = update
                return True
            return False
        previous = self._pages.get(update.chunk_id)
        if previous is not None and update.revision <= previous.revision:
            return False
        if previous is None and len(self._pages) >= self.max_pages:
            raise ValueError("mesh scheduler page capacity exceeded")
        # Assignment preserves FIFO position: frequently changing pages cannot
        # be starved by repeatedly moving them to the back of a replay queue.
        self._pages[update.chunk_id] = update
        return True

    def pop_ready(self, now: float) -> MeshPageUpdate | None:
        if not math.isfinite(now):
            raise ValueError("mesh scheduler clock must be finite")
        elapsed = max(0.0, now - self._last_time)
        self._last_time = max(self._last_time, now)
        update = self._clear or next(iter(self._pages.values()), None)
        self._page_tokens = min(4.0, self._page_tokens + elapsed * self.pages_per_second)
        # Idle time may bank only 100 ms of traffic. A large page is allowed to
        # accumulate its full credit while pending, so it never starves.
        capacity = max(self.bytes_per_second * 0.1, len(update.payload) if update else 0)
        self._byte_tokens = min(capacity, self._byte_tokens + elapsed * self.bytes_per_second)
        if update is None or self._page_tokens < 1 or self._byte_tokens < len(update.payload):
            return None
        self._page_tokens -= 1
        self._byte_tokens -= len(update.payload)
        if update is self._clear:
            self._clear = None
        else:
            self._pages.pop(update.chunk_id)
        return update
