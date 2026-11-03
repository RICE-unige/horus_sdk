"""Voxblox compatibility relay must not silently shrink or truncate source maps."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

spec = importlib.util.spec_from_file_location("voxblox_map_relay", Path(__file__).parents[1] / "examples/tools/voxblox_mesh_to_horus_marker.py")
relay = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = relay
spec.loader.exec_module(relay)


def state(limit=0):
    return relay.MeshAccumulator(fallback_color=(1, 1, 1, 1), max_triangles=limit, accumulate_blocks=True)


def block(index=(0, 0, 0)):
    return NS(index=index, x=[0, 65535, 0], y=[0, 0, 65535], z=[0, 0, 0], r=[255, 0, 0], g=[0, 255, 0], b=[0, 0, 255])


def message(*blocks, frame="map"):
    return NS(header=NS(frame_id=frame), block_edge_length=0.5, mesh_blocks=list(blocks))


def test_decode_matches_upstream_rviz_two_block_coordinate_scale():
    s = state()
    s.update_from_voxblox_mesh(message(block((2, -2, 0))))
    points, colors = s.snapshot()
    assert points == [(1, -1, 0), (2, -1, 0), (1, 0, 0)]
    assert colors == [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)]


def test_repeated_block_does_not_mutate_revision():
    s = state()
    s.update_from_voxblox_mesh(message(block()))
    s.update_from_voxblox_mesh(message(block()))
    assert s.revision == 1


@pytest.mark.parametrize("failure", ["coordinates", "color", "limit", "frame", "edge", "nan_color", "bad_index", "overflow"])
def test_rejected_delta_preserves_entire_previous_generation(failure):
    s = state(1 if failure == "limit" else 0)
    s.update_from_voxblox_mesh(message(block()))
    before = s.snapshot(), s.revision, s.last_frame_id
    incoming = message(block((1, 0, 0)), block((2, 0, 0)))
    if failure == "coordinates": incoming.mesh_blocks[1].x.pop()
    if failure == "color": incoming.mesh_blocks[1].r.pop()
    if failure == "frame": incoming.header.frame_id = "other"
    if failure == "edge": incoming.block_edge_length = float("nan")
    if failure == "nan_color": incoming.mesh_blocks[1].r[0] = float("nan")
    if failure == "bad_index": incoming.mesh_blocks[1].index = (0.5, 0, 0)
    if failure == "overflow": incoming.block_edge_length = 1e308
    with pytest.raises(ValueError): s.update_from_voxblox_mesh(incoming)
    assert (s.snapshot(), s.revision, s.last_frame_id) == before


def test_empty_block_deletes_without_lifetime_cap_or_stale_color():
    s = state(1)
    for i in range(20):
        s.update_from_voxblox_mesh(message(block((i, 0, 0))))
        s.update_from_voxblox_mesh(message(NS(index=(i, 0, 0), x=[], y=[], z=[])))
        assert s.snapshot() == ([], [])


def test_marker_transform_and_capacity_are_transactional():
    s = state(1)
    m = NS(header=NS(frame_id="map"), points=[NS(x=1, y=0, z=0)] * 3,
           pose=NS(position=NS(x=3, y=0, z=0), orientation=NS(x=0, y=0, z=1, w=0)),
           scale=NS(x=2, y=2, z=2))
    s.replace_from_marker(m)
    before = s.snapshot()
    assert before[0] == [(1, 0, 0)] * 3
    m.points *= 2
    with pytest.raises(ValueError): s.replace_from_marker(m)
    assert s.snapshot() == before
