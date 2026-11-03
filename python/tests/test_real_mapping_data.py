"""Real-capture preparation must not silently replace or discard source data."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location(
    "real_mapping_data_test", Path(__file__).parents[1] / "examples/tools/real_mapping_data.py")
data = importlib.util.module_from_spec(spec)
spec.loader.exec_module(data)


def test_arrays_preserve_geometry_indices_and_per_vertex_rgb(tmp_path):
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], np.float32)
    colors = np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255]], np.uint8)
    faces = np.array([[2, 0, 1]], np.int32)
    path = tmp_path / "map.npz"
    np.savez(path, vertices=vertices, colors=colors, faces=faces)
    actual = data.arrays(path, mesh=True)
    for expected, result in zip((vertices, colors, faces), actual):
        np.testing.assert_array_equal(result, expected)


def test_no_synthetic_fallback_for_missing_real_dataset(tmp_path):
    with pytest.raises(RuntimeError, match="not prepared"):
        data.record("g1", "mesh", tmp_path)
    (tmp_path / "manifest.json").write_text(json.dumps({
        "schema": "horus.real_mapping.v1", "datasets": {}}))
    with pytest.raises(ValueError, match="no synthetic fallback"):
        data.record("g1", "mesh", tmp_path)


@pytest.mark.parametrize("binary", [False, True])
def test_octomap_preserves_original_tree_bytes_and_resolution(tmp_path, binary):
    path = tmp_path / "tree.ot"
    payload = bytes(range(256))
    path.write_bytes((b"# Octomap OcTree binary file\n" if binary else b"# Octomap OcTree file\n")
                     + b"# comment\nid ColorOcTree\nsize 100\nres 0.02\ndata\n" + payload)
    assert data.octomap_file(path) == ("ColorOcTree", .02, binary, payload)


@pytest.mark.parametrize("vertices,faces", [
    ([[float("nan"), 0, 0]], [[0, 0, 0]]),
    ([[0, 0, 0]], [[0, 0, 1]]),
    ([[0, 0, 0]], [[-1, 0, 0]]),
])
def test_invalid_source_arrays_fail_explicitly(tmp_path, vertices, faces):
    path = tmp_path / "bad.npz"
    np.savez(path, vertices=np.array(vertices), faces=np.array(faces), colors=np.zeros((1, 3), np.uint8))
    with pytest.raises(ValueError):
        data.arrays(path, mesh=True)
