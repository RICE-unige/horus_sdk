import hashlib
import importlib
import json
from pathlib import Path
import sys
from dataclasses import replace

import numpy as np
import pytest

from horus.mapping.mesh_protocol import BlockGeometry, decode_live_header, encode_live_payload

sys.path.insert(0, str(Path(__file__).parents[1] / "examples"))
saved = importlib.import_module("saved_map_registration")


def fixture(tmp_path):
    geometry = BlockGeometry(((0., 0., 0.), (1., 0., 0.), (0., 1., 0.)),
        ((255, 1, 2, 255), (3, 254, 4, 255), (5, 6, 253, 255)), (0, 1, 2))
    payload = encode_live_payload(chunk_id=7, page_key=(1, 2, 3), epoch=12, revision=3,
        operation=0, geometry=geometry, max_payload_bytes=1024)
    (tmp_path / "page.hmsh").write_bytes(payload)
    metadata = dict(schema="horus.mesh_fixture.v1", source_floor_z_m=-.625, workspace_scale=.1,
        triangles=1, vertices=3, payload_bytes=len(payload),
        original_manifest=dict(epoch="12", pages=[dict(chunk_id=7, revision="3", operation=0)]),
        pages=[dict(file="page.hmsh", sha256=hashlib.sha256(payload).hexdigest(), chunk_id=7,
            revision="3", vertex_count=3, triangle_count=1)])
    (tmp_path / "manifest.json").write_text(json.dumps(metadata))
    return metadata, payload


def test_load_preserves_every_wire_byte_and_replay_changes_only_epoch(tmp_path):
    _, payload = fixture(tmp_path)
    data = saved.load_saved_map(tmp_path)
    assert data.pages[0].payload == payload
    replay = data.replay_pages(100)[0]
    assert replay.payload[:24] == payload[:24] and replay.payload[32:] == payload[32:]
    assert decode_live_header(replay.payload).epoch == 100


@pytest.mark.parametrize("profile,lod,spacewarp", [
    ("off", False, False), ("lod", True, False),
    ("spacewarp", False, True), ("combined", True, True),
])
def test_native_profiles_explicitly_select_presentation_without_geometry_limits(profile, lod, spacewarp):
    options = saved.native_render_options(profile)
    assert set(options) == {"display"}
    assert options["display"]["lod_enabled"] is lod
    assert options["display"]["spacewarp_enabled"] is spacewarp
    assert options["display"]["force_full_detail"] is False
    assert options["display"]["lod_error_pixels"] == 1.0
    options["display"]["lod_enabled"] = not lod
    assert saved.native_render_options(profile)["display"]["lod_enabled"] is lod


def test_native_profile_rejects_unknown_policy():
    with pytest.raises(ValueError, match="Unknown native mesh render profile"):
        saved.native_render_options("unknown")


def test_corrupt_page_is_rejected_before_any_ros_start(tmp_path):
    fixture(tmp_path)
    (tmp_path / "page.hmsh").write_bytes(b"invalid")
    with pytest.raises(ValueError, match="checksum"):
        saved.load_saved_map(tmp_path)


@pytest.mark.parametrize("field,value", [("triangles", 2), ("vertices", 4), ("payload_bytes", 1),
    ("source_floor_z_m", float("nan")), ("workspace_scale", True)])
def test_missing_or_inconsistent_fixture_data_is_rejected(tmp_path, field, value):
    metadata, _ = fixture(tmp_path)
    metadata[field] = value
    (tmp_path / "manifest.json").write_text(json.dumps(metadata))
    with pytest.raises(ValueError):
        saved.load_saved_map(tmp_path)


def test_missing_page_inventory_is_not_called_a_complete_map(tmp_path):
    metadata, _ = fixture(tmp_path)
    metadata["original_manifest"]["pages"].append(dict(chunk_id=8, revision="1", operation=0))
    (tmp_path / "manifest.json").write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match="totals"):
        saved.load_saved_map(tmp_path)


def test_remote_export_preserves_all_pages_rgb_winding_and_floor_alignment(tmp_path):
    fixture(tmp_path)
    original = saved.load_saved_map(tmp_path)
    # Separate pages reuse local vertex indices; exported indices must not overlap.
    data = replace(original, pages=original.pages * 2,
        metadata={**original.metadata, "vertices": 6, "triangles": 2})
    arrays = saved.remote_arrays(data)
    assert arrays["vertices"].dtype == np.float32
    assert arrays["colors"].dtype == np.uint8
    assert arrays["faces"].dtype == np.int32
    np.testing.assert_array_equal(arrays["vertices"],
        [(0, 0, .625), (1, 0, .625), (0, 1, .625)] * 2)
    np.testing.assert_array_equal(arrays["faces"], [[0, 1, 2], [3, 4, 5]])
    np.testing.assert_array_equal(arrays["colors"],
        [(255, 1, 2), (3, 254, 4), (5, 6, 253)] * 2)
    assert data.pages[0].payload == original.pages[0].payload


def test_remote_export_refuses_to_silently_remove_transparency(tmp_path):
    fixture(tmp_path)
    data = saved.load_saved_map(tmp_path)
    payload = bytearray(data.pages[0].payload)
    payload[67] = 128
    data = replace(data, pages=(replace(data.pages[0], payload=bytes(payload)),))
    with pytest.raises(ValueError, match="alpha cannot be discarded"):
        saved.remote_arrays(data)


def test_remote_export_rechecks_corrupt_cache_and_leaves_original_untouched(tmp_path):
    fixture(tmp_path)
    data = saved.load_saved_map(tmp_path)
    source_bytes = (tmp_path / "page.hmsh").read_bytes()
    cache = tmp_path / "cache"
    path = saved.prepare_remote_map(data, cache)
    timestamp = path.stat().st_mtime_ns
    assert saved.prepare_remote_map(data, cache) == path
    assert path.stat().st_mtime_ns == timestamp
    np.savez_compressed(path, vertices=np.zeros((3, 3)), colors=np.zeros((3, 3)), faces=np.zeros((1, 3)))
    assert saved.prepare_remote_map(data, cache) == path
    with np.load(path, allow_pickle=False) as arrays:
        np.testing.assert_array_equal(arrays["vertices"], saved.remote_arrays(data)["vertices"])
    assert (tmp_path / "page.hmsh").read_bytes() == source_bytes


def test_remote_registration_uses_verified_full_map_and_existing_stereo_launcher(tmp_path, monkeypatch):
    fixture(tmp_path)
    remote = importlib.import_module("remote_map_portal_registration")
    calls = []
    monkeypatch.setattr(remote, "main", lambda argv: calls.append(argv) or 0)
    assert saved.main([str(tmp_path), "--render-target", "remote", "--profile", "quality",
        "--remote-cache", str(tmp_path / "remote-cache")]) == 0
    options = dict(zip(calls[0][::2], calls[0][1::2]))
    assert options["--scene"] == "recorded_mesh"
    assert options["--workspace-scale"] == "0.1"
    assert options["--profile"] == "quality"
    assert options["--transport"] == "webrtc"
    assert options["--protocol"] == "v3"
    assert Path(options["--dataset-path"]).is_file()
