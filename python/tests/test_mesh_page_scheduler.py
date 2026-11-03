from dataclasses import replace
import json
from pathlib import Path
import pytest

from horus.mapping.mesh_page_scheduler import MeshPageScheduler
from horus.mapping.nvblox_mesh import HMSH_OPERATION_CLEAR, MeshPageUpdate


def page(chunk=1, revision=1, epoch=1, size=100):
    return MeshPageUpdate(chunk, (chunk, 0, 0), epoch, revision, 0, bytes(size), 3, 1)


def scheduler():
    return MeshPageScheduler(pages_per_second=10, bytes_per_second=1000,
                             max_pages=4, max_payload_bytes=2000, now=0)


def test_large_page_accumulates_credit_without_starving_or_bursting():
    queue = scheduler()
    large = page(size=2000)
    queue.enqueue(large)
    assert queue.pop_ready(0.5) is None
    assert queue.pop_ready(1.99) is None
    assert queue.pop_ready(2.0) is large
    queue.enqueue(page(chunk=2))
    assert queue.pop_ready(2.0) is None
    assert queue.pop_ready(2.1).chunk_id == 2


def test_idle_credit_is_bounded_and_replacements_preserve_fairness():
    queue = scheduler()
    queue.pop_ready(100)
    first = page()
    queue.enqueue(first)
    queue.enqueue(page(chunk=2))
    newest = page(revision=10)
    queue.enqueue(newest)
    assert queue.pop_ready(100) is newest
    assert queue.pop_ready(100) is None
    assert queue.pop_ready(100.2).chunk_id == 2


def test_old_revisions_and_generations_cannot_replace_new_pages():
    queue = scheduler()
    current = page(revision=2, epoch=3)
    queue.enqueue(current)
    assert not queue.enqueue(page(revision=1, epoch=3))
    assert not queue.enqueue(page(revision=100, epoch=2))
    assert not queue.enqueue(replace(page(epoch=2), operation=HMSH_OPERATION_CLEAR))
    assert queue.pending_count == 1
    assert queue.pending_bytes == len(current.payload)
    assert queue.pop_ready(1) is current


def test_new_epoch_clear_precedes_pages_and_duplicate_clear_keeps_queue():
    queue = scheduler()
    queue.enqueue(page())
    clear = replace(page(chunk=0, epoch=2), operation=HMSH_OPERATION_CLEAR)
    queue.enqueue(clear)
    current = page(epoch=2)
    queue.enqueue(current)
    queue.enqueue(clear)
    assert queue.pending_count == 2
    assert queue.pop_ready(1) is clear
    assert queue.pop_ready(1.2) is current


def test_empty_generation_clear_can_be_replayed_after_delivery():
    queue = scheduler()
    clear = replace(page(chunk=0), operation=HMSH_OPERATION_CLEAR)
    queue.enqueue(clear)
    assert queue.pop_ready(1) is clear
    queue.enqueue(clear)
    assert queue.pop_ready(2) is clear


def test_shared_native_scheduler_contract():
    path = Path(__file__).parents[2] / "contracts/fixtures/mesh_scheduler.json"
    for case in json.loads(path.read_text()):
        queue = scheduler()
        for event in case["events"]:
            if "enqueue" in event:
                chunk, epoch, revision, operation, size = event["enqueue"]
                update = replace(page(chunk, revision, epoch, size), operation=operation)
                assert queue.enqueue(update) == event["accepted"]
            else:
                ready = queue.pop_ready(event["pop"])
                actual = None if ready is None else [ready.chunk_id, ready.epoch, ready.revision, ready.operation, len(ready.payload)]
                assert actual == event["ready"], case["name"]
            if "count" in event:
                assert queue.pending_count == event["count"]
            if "bytes" in event:
                assert queue.pending_bytes == event["bytes"]


@pytest.mark.parametrize("now", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_clock_rejected_without_corrupting_queue(now):
    queue = scheduler()
    update = page()
    queue.enqueue(update)
    with pytest.raises(ValueError, match="clock"):
        queue.pop_ready(now)
    assert queue.pop_ready(1) is update
    with pytest.raises(ValueError, match="clock"):
        MeshPageScheduler(pages_per_second=10, bytes_per_second=1000,
                          max_pages=4, max_payload_bytes=2000, now=now)
