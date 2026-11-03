"""Shared native state contract, independently executed by the Python reference."""
import json
from pathlib import Path
from horus.mapping.live_mesh import LiveMeshState
from horus.mapping.mesh_protocol import BlockGeometry


def test_retained_map_matches_native_manifest_fixture():
    state = LiveMeshState(max_pages=2, max_payload_bytes=120, max_cache_bytes=240, initial_epoch=11)
    geometry = BlockGeometry(((0,0,0),(1,0,0),(0,1,0)),
        ((255,0,0,255),(0,255,0,255),(0,0,255,255)),(0,1,2))
    assert len(state.replace("a",geometry)[0].payload)==112
    assert not state.replace("a",geometry)
    state.replace("b",geometry)
    state.replace("a",None)
    assert state.cached_bytes==164
    state.replace("c",geometry)
    fixture = Path(__file__).resolve().parents[2] / "rust/src/mapping/fixtures/live_mesh_manifest.json"
    assert state.manifest()==json.loads(fixture.read_text())
    assert state.unchanged_pages==1 and state.suppressed_bytes==112
