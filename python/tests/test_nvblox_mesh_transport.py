"""Unit tests for live nvblox block state and HMSH v2 transport."""

from types import SimpleNamespace
import pytest

from horus.mapping.nvblox_mesh import (
    HMSH_OPERATION_CLEAR,
    HMSH_OPERATION_DELETE,
    HMSH_OPERATION_UPSERT,
    HMSH_V2_HEADER,
    LiveNvbloxMeshState,
    decode_live_header,
)


def point(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


def color(r, g, b, a=1.0):
    return SimpleNamespace(r=r, g=g, b=b, a=a)


def test_repeated_vertex_indices_have_no_drawable_surface():
    from horus.mapping.nvblox_mesh import decode_mesh_block
    source = block()
    source.triangles = [0, 1, 2, 0, 0, 1, 1, 2, 1]
    result = decode_mesh_block(source, color_mode="source", fallback_color=(1, 1, 1, 1))
    assert result.triangles == (0, 1, 2)
    assert len(result.vertices) == 3
    source.triangles = [0, 0, 1]
    assert decode_mesh_block(source, color_mode="source", fallback_color=(1, 1, 1, 1)) is None


def test_vectorized_source_colors_exactly_match_scalar_rounding_and_nonfinite_defaults():
    from horus.mapping.nvblox_mesh import _rgba8, _source_colors_rgba8
    values = [-1.0, 0.0, .5 / 255, 127.5 / 255, .7, 1.0, 2.0, float("nan"), float("inf")]
    source = [color(value, .72, .78, .5) for value in values] * 40
    assert _source_colors_rgba8(source) == tuple(_rgba8(value) for value in source)


def block(offset=0.0, *, empty=False):
    if empty:
        return SimpleNamespace(vertices=[], normals=[], colors=[], triangles=[])
    return SimpleNamespace(
        vertices=[
            point(offset + 0.0, 0.0, 0.0),
            point(offset + 1.0, 0.0, 0.0),
            point(offset + 0.0, 1.0, 0.0),
        ],
        normals=[point(0.0, 0.0, 1.0)] * 3,
        colors=[color(1.0, 0.25, 0.0)] * 3,
        triangles=[0, 1, 2],
    )


def mesh_message(entries, *, clear=False, frame_id="map"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        clear=clear,
        block_indices=[point(*key) for key, _value in entries],
        blocks=[value for _key, value in entries],
    )


def test_pending_source_coalesced_latest_blocks_without_losing_unrelated_blocks_or_deletes():
    from horus.mapping.nvblox_mesh import PendingNvbloxMesh
    pending = PendingNvbloxMesh()
    pending.add(mesh_message([((0, 0, 0), block()), ((1, 0, 0), block(2))], clear=True))
    pending.add(mesh_message([((0, 0, 0), block(4)), ((2, 0, 0), block(6))]))
    pending.add(mesh_message([((1, 0, 0), block(empty=True))]))
    batch = pending.take()
    sequential, coalesced = (LiveNvbloxMeshState(initial_epoch=10) for _ in range(2))
    sequential.apply_message(mesh_message([((0, 0, 0), block()), ((1, 0, 0), block(2))], clear=True))
    sequential.apply_message(mesh_message([((0, 0, 0), block(4)), ((2, 0, 0), block(6))]))
    sequential.apply_message(mesh_message([((1, 0, 0), block(empty=True))]))
    coalesced.apply_message(batch)
    assert coalesced._blocks == sequential._blocks
    assert coalesced.epoch == sequential.epoch
    assert pending.take() is None and pending.bytes == 0


def test_pending_clear_barrier_did_not_mutate_batch_already_owned_by_worker():
    from horus.mapping.nvblox_mesh import PendingNvbloxMesh
    pending = PendingNvbloxMesh()
    pending.add(mesh_message([((0, 0, 0), block())]))
    older = pending.take()
    pending.add(mesh_message([((1, 0, 0), block(4))], clear=True, frame_id="new_map"))
    pending.add(mesh_message([((2, 0, 0), block(6))], frame_id="new_map"))
    newer = pending.take()
    assert not older.clear and older.header.frame_id == "map"
    assert len(older.blocks) == 1
    assert newer.clear and newer.header.frame_id == "new_map"
    assert len(newer.blocks) == 2


def test_pending_invalid_frame_or_capacity_kept_previous_pending_batch_intact():
    from horus.mapping.nvblox_mesh import PendingNvbloxMesh
    pending = PendingNvbloxMesh(max_blocks=1)
    pending.add(mesh_message([((0, 0, 0), block())]))
    previous_bytes = pending.bytes
    with pytest.raises(ValueError, match="capacity"):
        pending.add(mesh_message([((1, 0, 0), block())]))
    with pytest.raises(ValueError, match="frame changed"):
        pending.add(mesh_message([((0, 0, 0), block(4))], frame_id="different"))
    assert pending.bytes == previous_bytes
    batch = pending.take()
    assert batch.blocks[0].vertices[0].x == 0.0 and len(batch.blocks) == 1


def test_manifest_observed_heights_follow_replacements_deletes_and_clear():
    state = LiveNvbloxMeshState(blocks_per_axis=8)
    assert state.manifest()["height_bounds_m"] is None
    source = block()
    source.vertices[0].z = -1.6
    source.vertices[2].z = .4
    state.apply_message(mesh_message([((0, 0, 0), source)]))
    assert state.manifest()["height_bounds_m"] == [-1.6, .4]
    state.apply_message(mesh_message([((0, 0, 0), block(empty=True))]))
    assert state.manifest()["height_bounds_m"] is None
    state.apply_message(mesh_message([((0, 0, 0), source)]))
    state.clear()
    assert state.manifest()["height_bounds_m"] is None


def test_larger_dense_scene_batches_preserve_identical_blocks_colors_and_triangle_count():
    entries = [((x, y, z), block(float(x))) for x in range(-8, 8)
               for y in range(8) for z in range(2)]
    smaller, larger = (LiveNvbloxMeshState(blocks_per_axis=size) for size in (4, 8))
    for state in (smaller, larger):
        state.apply_message(mesh_message(entries))
    assert smaller._blocks == larger._blocks
    assert smaller.triangle_count == larger.triangle_count == len(entries)
    assert larger.page_count < smaller.page_count


def test_full_map_clear_and_upserts_share_one_generation():
    state = LiveNvbloxMeshState(blocks_per_axis=2, initial_epoch=100)
    updates = state.apply_message(
        mesh_message([((0, 0, 0), block()), ((1, 0, 0), block(2.0))], clear=True)
    )

    assert [update.operation for update in updates] == [
        HMSH_OPERATION_CLEAR,
        HMSH_OPERATION_UPSERT,
    ]
    assert updates[0].epoch == 101
    assert updates[1].epoch == 101
    assert updates[1].vertex_count == 6
    assert updates[1].triangle_count == 2
    assert state.frame_id == "map"
    assert state.block_count == 2
    assert state.page_count == 1

    header = decode_live_header(updates[1].payload)
    assert header.page_key == (0, 0, 0)
    assert header.operation == HMSH_OPERATION_UPSERT
    assert header.has_color is True
    assert header.vertex_count == 6
    assert header.index_count == 6
    assert len(updates[1].payload) == HMSH_V2_HEADER.size + 6 * 16 + 6 * 4


def test_neighboring_blocks_merge_losslessly_and_indices_are_offset():
    state = LiveNvbloxMeshState(blocks_per_axis=4, initial_epoch=1)
    update = state.apply_message(
        mesh_message([((0, 0, 0), block()), ((3, 3, 3), block(2.0))])
    )[0]

    assert update.vertex_count == 6
    assert update.triangle_count == 2
    payload = update.payload
    index_offset = HMSH_V2_HEADER.size + 6 * 16
    indices = [
        int.from_bytes(payload[index_offset + i : index_offset + i + 4], "little")
        for i in range(0, 24, 4)
    ]
    assert indices == [0, 1, 2, 3, 4, 5]


def test_empty_block_deletes_page_and_revision_increases():
    state = LiveNvbloxMeshState(blocks_per_axis=1, initial_epoch=10)
    upsert = state.apply_message(mesh_message([((5, -2, 1), block())]))[0]
    delete = state.apply_message(mesh_message([((5, -2, 1), block(empty=True))]))[0]

    assert upsert.operation == HMSH_OPERATION_UPSERT
    assert delete.operation == HMSH_OPERATION_DELETE
    assert delete.chunk_id == upsert.chunk_id
    assert delete.revision == upsert.revision + 1
    assert state.block_count == 0
    assert state.page_count == 0
    assert state.snapshot_updates() == []


def test_snapshot_replays_only_latest_page_revision():
    state = LiveNvbloxMeshState(blocks_per_axis=1, initial_epoch=10)
    first = state.apply_message(mesh_message([((0, 0, 0), block())]))[0]
    second = state.apply_message(mesh_message([((0, 0, 0), block(4.0))]))[0]

    snapshot = state.snapshot_updates()
    assert len(snapshot) == 1
    assert snapshot[0].chunk_id == first.chunk_id
    assert snapshot[0].revision == second.revision
    assert snapshot[0].payload == second.payload


def test_snapshot_can_clear_receiver_without_clearing_accumulated_state():
    state = LiveNvbloxMeshState(blocks_per_axis=1, initial_epoch=10)
    current = state.apply_message(mesh_message([((0, 0, 0), block())]))[0]

    snapshot = state.snapshot_updates(include_clear=True)

    assert [update.operation for update in snapshot] == [
        HMSH_OPERATION_CLEAR,
        HMSH_OPERATION_UPSERT,
    ]
    assert snapshot[0].epoch == current.epoch
    assert snapshot[1].payload == current.payload
    assert state.block_count == 1
    assert state.page_count == 1


def test_clear_payload_is_header_only_and_advances_epoch():
    state = LiveNvbloxMeshState(initial_epoch=42)
    update = state.clear()
    header = decode_live_header(update.payload)

    assert len(update.payload) == HMSH_V2_HEADER.size
    assert header.epoch == 43
    assert header.operation == HMSH_OPERATION_CLEAR
    assert header.vertex_count == 0
    assert header.index_count == 0


def test_unchanged_geometry_keeps_payload_and_revision_without_retransmit():
    state = LiveNvbloxMeshState(initial_epoch=10)
    first = state.apply_message(mesh_message([((0, 0, 0), block())]))[0]
    assert state.apply_message(mesh_message([((0, 0, 0), block())])) == []
    assert state.snapshot_updates()[0] is first
    changed = block()
    changed.colors[1] = color(0.0, 1.0, 0.0)
    assert state.apply_message(mesh_message([((0, 0, 0), changed)]))[0].revision == 2


@pytest.mark.parametrize("coordinate", [1e100, -1e100, float("nan"), float("inf")])
@pytest.mark.parametrize("clear", [False, True])
def test_unrepresentable_coordinate_preserves_map_and_generation(coordinate, clear):
    state = LiveNvbloxMeshState(initial_epoch=10)
    state.apply_message(mesh_message([((0, 0, 0), block())]))
    before = state.snapshot_updates()
    manifest = state.manifest()
    invalid = block()
    invalid.vertices[0].x = coordinate
    with pytest.raises(ValueError, match="vertex"):
        state.apply_message(mesh_message([((0, 0, 0), invalid)], clear=clear))
    assert state.snapshot_updates() == before
    assert state.manifest() == manifest


@pytest.mark.parametrize("clear", [False, True])
def test_rejected_message_preserves_all_state(clear):
    state = LiveNvbloxMeshState(blocks_per_axis=1, max_pages=2, initial_epoch=10)
    state.apply_message(mesh_message([((0, 0, 0), block())]))
    before = state.snapshot_updates()
    bad = block()
    bad.vertices[0].x = float("nan")
    with pytest.raises(ValueError, match="non-finite"):
        state.apply_message(mesh_message([((0, 0, 0), block(5)), ((1, 0, 0), bad)], clear=clear))
    assert state.snapshot_updates() == before
    assert state.epoch == 10
    assert state.block_count == 1
    malformed = mesh_message([], clear=clear, frame_id="changed")
    malformed.blocks = [block()]
    with pytest.raises(ValueError, match="length mismatch"):
        state.apply_message(malformed)
    assert state.snapshot_updates() == before
    assert state.frame_id == "map"


def test_capacity_and_encoding_failure_do_not_partially_apply():
    state = LiveNvbloxMeshState(blocks_per_axis=1, max_pages=1, initial_epoch=10)
    state.apply_message(mesh_message([((0, 0, 0), block())]))
    before = state.snapshot_updates()
    with pytest.raises(RuntimeError, match="max_pages"):
        state.apply_message(mesh_message([((0, 0, 0), block(5)), ((1, 0, 0), block())]))
    assert state.snapshot_updates() == before
    state.max_payload_bytes = HMSH_V2_HEADER.size
    with pytest.raises(ValueError, match="split the source into smaller pages"):
        state.apply_message(mesh_message([((0, 0, 0), block(5))], clear=True))
    assert state.snapshot_updates() == before
    assert state.epoch == 10


def test_page_churn_reuses_bounded_slots_with_monotonic_revisions():
    state = LiveNvbloxMeshState(blocks_per_axis=1, max_pages=1, initial_epoch=10)
    last_revision = 0
    for x in range(100):
        upsert = state.apply_message(mesh_message([((x, 0, 0), block(x))]))[0]
        assert upsert.chunk_id == 1
        assert upsert.revision > last_revision
        assert len(state.snapshot_updates(include_deletes=True)) == 1
        delete = state.apply_message(mesh_message([((x, 0, 0), block(empty=True))]))[0]
        assert delete.revision > upsert.revision
        last_revision = delete.revision
        assert state.page_count == 0
        assert state.snapshot_updates(include_deletes=True) == [delete]
    assert len(state._revision_by_chunk) == 1
    assert len(state._deleted_payload_by_id) == 1
    assert len(state._chunk_by_page) == 0


def test_full_capacity_page_replacement_releases_slots_before_allocating():
    state = LiveNvbloxMeshState(blocks_per_axis=1, max_pages=1, initial_epoch=10)
    first = state.apply_message(mesh_message([((10, 0, 0), block())]))[0]
    updates = state.apply_message(mesh_message([
        ((0, 0, 0), block(5)), ((10, 0, 0), block(empty=True))]))
    assert [update.operation for update in updates] == [HMSH_OPERATION_DELETE, HMSH_OPERATION_UPSERT]
    assert updates[-1].chunk_id == first.chunk_id
    assert updates[-1].revision > updates[0].revision
    assert state.snapshot_updates(include_deletes=True) == [updates[-1]]


def test_changed_frame_requires_explicit_generation_clear():
    state = LiveNvbloxMeshState(initial_epoch=10)
    state.apply_message(mesh_message([((0, 0, 0), block())]))
    with pytest.raises(ValueError, match="frame changed"):
        state.apply_message(mesh_message([], frame_id="odom"))
    state.apply_message(mesh_message([((0, 0, 0), block())], clear=True, frame_id="odom"))
    assert state.frame_id == "odom"


@pytest.mark.parametrize("indices", [[0, 1], [0, 1, 999]])
def test_malformed_triangles_do_not_delete_previous_page(indices):
    state = LiveNvbloxMeshState(initial_epoch=10)
    before = state.apply_message(mesh_message([((0, 0, 0), block())]))
    malformed = block()
    malformed.triangles = indices
    with pytest.raises(ValueError):
        state.apply_message(mesh_message([((0, 0, 0), malformed)]))
    assert state.snapshot_updates() == before


def test_manifest_repairs_missing_pages_and_deletes_without_global_clear():
    state = LiveNvbloxMeshState(blocks_per_axis=1, max_pages=3, initial_epoch=10)
    updates = state.apply_message(mesh_message([((x, 0, 0), block(x)) for x in range(3)]))
    # One receiver missed the first page and the last page's deletion.
    delete = state.apply_message(mesh_message([((2, 0, 0), block(empty=True))]))[0]
    manifest = state.manifest()
    assert manifest["epoch"] == "10"
    assert len(manifest["pages"]) == 3
    assert manifest["triangles"] == 2
    repaired = state.repair_updates({"request": "pages", "epoch": "10", "ids": [updates[0].chunk_id, delete.chunk_id]})
    assert repaired == [updates[0], delete]
    assert all(update.operation != HMSH_OPERATION_CLEAR for update in repaired)
    assert state.snapshot_updates() == updates[:2]


def test_repair_uses_current_revision_after_slot_reuse_and_epoch_change():
    state = LiveNvbloxMeshState(blocks_per_axis=1, max_pages=1, initial_epoch=10)
    first = state.apply_message(mesh_message([((0, 0, 0), block())]))[0]
    state.apply_message(mesh_message([((0, 0, 0), block(empty=True))]))
    current = state.apply_message(mesh_message([((1, 0, 0), block(1))]))[0]
    assert state.repair_updates({"request": "pages", "epoch": "10", "ids": [first.chunk_id]}) == [current]
    clear = state.clear()
    assert state.repair_updates({"request": "pages", "epoch": "10", "ids": [first.chunk_id]}) == [clear]
