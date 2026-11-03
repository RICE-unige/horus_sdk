"""Source-independent map integrity, duplicate suppression and recovery."""

import struct
from types import SimpleNamespace as NS

import pytest

from horus.mapping import LiveMeshState, MarkerMeshState
from horus.mapping.mesh_page_scheduler import MeshPageScheduler
from horus.mapping.nvblox_mesh import BlockGeometry, HMSH_V2_HEADER


def geometry(offset=0, color=(255, 0, 0, 255)):
    return BlockGeometry(((offset, 0, 0), (offset+1, 0, 0), (offset, 1, 0)),
                         (color,) * 3, (0, 1, 2))


def marker(id=0, ns="walls", offset=0):
    return NS(ns=ns, id=id, action=0, type=11, frame_locked=False,
              header=NS(frame_id="map", stamp=NS(sec=0, nanosec=0)),
              lifetime=NS(sec=0, nanosec=0),
              pose=NS(position=NS(x=offset, y=0, z=0), orientation=NS(x=0, y=0, z=0, w=1)),
              scale=NS(x=1, y=1, z=1), color=NS(r=1., g=0., b=0., a=1.), colors=[],
              points=[NS(x=x, y=y, z=z) for x, y, z in geometry().vertices])


def test_identical_content_is_silent_but_repairs_are_not_suppressed():
    state = LiveMeshState(initial_epoch=10)
    first = state.replace(("floor", 1), geometry())[0]
    for _ in range(100):
        assert state.replace(("floor", 1), geometry()) == []
    assert state.unchanged_pages == 100
    assert state.suppressed_bytes == 100 * len(first.payload)
    assert state.repair_updates({"request": "snapshot"}) == [first]
    assert state.repair_updates({"request": "pages", "epoch": "10", "ids": [1]}) == [first]
    changed = state.replace(("floor", 1), geometry(color=(0, 255, 0, 255)))[0]
    assert changed.revision == 2
    assert state.repair_updates({"request": "snapshot"}) == [changed]


def test_independent_keys_deletion_slot_reuse_and_restart():
    state = LiveMeshState(initial_epoch=10, max_pages=2)
    first = state.replace(("a", 1), geometry())[0]
    second = state.replace(("b", 1), geometry())[0]
    assert first.chunk_id != second.chunk_id
    deleted = state.replace(("a", 1), None)[0]
    assert deleted.operation == 1
    assert state.replace(("a", 1), None) == []
    replacement = state.replace(("c", 1), geometry(2))[0]
    assert replacement.chunk_id == first.chunk_id
    assert replacement.revision > deleted.revision
    assert len(state.manifest()["pages"]) == 2
    clear = state.clear()
    assert clear.epoch == 11 and clear.operation == 2
    assert state.repair_updates({"request": "pages", "epoch": "10", "ids": [1]}) == [clear]


def test_invalid_multi_update_cannot_partially_mutate_map():
    state = LiveMeshState(initial_epoch=10, max_pages=2)
    first = state.replace(1, geometry())[0]
    with pytest.raises(ValueError):
        state.replace_many([(1, geometry(3)), (2, geometry()), (3, geometry())])
    assert state.snapshot_updates() == [first]


def test_byte_limits_preserve_previous_state():
    state = LiveMeshState(initial_epoch=10, max_payload_bytes=128, max_cache_bytes=128)
    first = state.replace(1, geometry())[0]
    with pytest.raises(ValueError, match="cache byte"):
        state.replace(2, geometry())
    assert state.snapshot_updates() == [first]


@pytest.mark.parametrize("bad", [None, {}, {"request": "no"}, {"request": "pages", "epoch": "10", "ids": [True]}])
def test_invalid_requests_do_not_trigger_full_map_replays(bad):
    state = LiveMeshState(initial_epoch=10)
    state.replace(1, geometry())
    assert state.repair_updates(bad) == []


def test_marker_timestamp_only_is_silent_pose_color_and_scale_are_changes():
    state = MarkerMeshState(initial_epoch=10)
    m = marker()
    first = state.apply([m], now=0)[0]
    m.header.stamp.sec = 100
    assert state.apply([m], now=1) == []
    for mutate in (lambda: setattr(m.pose.position, "z", 3),
                   lambda: setattr(m.scale, "x", 2),
                   lambda: setattr(m.color, "g", 1)):
        mutate()
        assert state.apply([m], now=2)[0].revision > first.revision


def test_split_marker_preserves_every_triangle_and_deletes_old_tail():
    state = MarkerMeshState(initial_epoch=10, triangles_per_page=1)
    m = marker()
    m.points *= 3
    updates = state.apply([m], now=0)
    assert len(updates) == 3
    assert sum(u.triangle_count for u in updates) == 3
    assert state.apply([m], now=1) == []
    m.points = m.points[:3]
    updates = state.apply([m], now=2)
    assert len(updates) == 2 and all(u.operation == 1 for u in updates)
    assert state.state.page_count == 1
    assert len(state.state.manifest()["pages"]) == 3  # deletion tombstones retained


def test_namespace_and_incremental_omission_are_preserved():
    state = MarkerMeshState(initial_epoch=10)
    state.apply([marker(ns="a"), marker(ns="b")], now=0)
    assert state.apply([marker(ns="a")], now=1) == []
    assert state.state.page_count == 2
    delete = marker(ns="a")
    delete.action = 2
    assert state.apply([delete], now=2)[0].operation == 1
    assert state.state.page_count == 1


def test_complete_marker_array_snapshot_deduplicates_and_removes_absent_ids():
    state = MarkerMeshState(initial_epoch=10)
    clear = marker()
    clear.action = 3
    first = state.apply([clear, marker(id=1), marker(id=2)], now=0)
    assert len(first) == 2
    assert state.apply([clear, marker(id=1), marker(id=2)], now=1) == []
    removed = state.apply([clear, marker(id=2)], now=2)
    assert len(removed) == 1 and removed[0].operation == 1
    assert state.state.epoch == 10
    assert state.apply([clear], now=3)[0].epoch == 11


def test_ttl_refresh_does_not_resend_geometry_and_expiry_is_explicit():
    state = MarkerMeshState(initial_epoch=10)
    m = marker()
    m.lifetime.sec = 5
    state.apply([m], now=0)
    assert state.apply([m], now=4) == []
    assert state.expire(5) == []
    assert state.expire(9)[0].operation == 1
    assert state.expire(10) == []


def test_foreign_frame_resolves_at_stamp_before_comparison():
    state = MarkerMeshState(initial_epoch=10)
    m = marker()
    m.header.frame_id = "odom"
    seen = []
    def resolve(frame, stamp):
        seen.append((frame, stamp.sec))
        return (stamp.sec, 0, 0), (0, 0, 0, 1)
    first = state.apply([m], now=0, resolve_transform=resolve)[0]
    m.header.stamp.sec = 2
    changed = state.apply([m], now=1, resolve_transform=resolve)[0]
    assert changed.revision == 2
    assert seen == [("odom", 0), ("odom", 2)]
    assert struct.unpack_from("<fff", changed.payload, HMSH_V2_HEADER.size) == (2, 0, 0)
    assert first.payload != changed.payload


@pytest.mark.parametrize("field", ["locked", "quaternion", "infinite", "color", "incomplete", "frame"])
def test_rejected_marker_array_preserves_previous_map(field):
    state = MarkerMeshState(initial_epoch=10)
    before = state.apply([marker()], now=0)
    bad = marker(id=2)
    if field == "locked": bad.frame_locked = True
    if field == "quaternion": bad.pose.orientation.w = 0
    if field == "infinite": bad.points[0].x = float("inf")
    if field == "color": bad.colors = [bad.color]
    if field == "incomplete": bad.points.pop()
    if field == "frame": bad.header.frame_id = "unknown"
    clear = marker()
    clear.action = 3
    with pytest.raises(ValueError):
        state.apply([clear, bad], now=1)
    assert state.state.snapshot_updates() == before
    assert state.state.epoch == 10


def test_scheduler_coalesces_hot_pages_without_dropping_other_sections():
    state = LiveMeshState(initial_epoch=10)
    queue = MeshPageScheduler(pages_per_second=100, bytes_per_second=100000,
        max_pages=1024, max_payload_bytes=1024*1024, now=0)
    for key in range(10):
        queue.enqueue(state.replace(key, geometry(key))[0])
    for value in range(1, 101):
        queue.enqueue(state.replace(0, geometry(value))[0])
    assert queue.pending_count == 10
    received = {}
    for tick in range(1, 100):
        update = queue.pop_ready(tick / 100)
        if update: received[update.chunk_id] = update
    assert list(received.values()) == state.snapshot_updates()
