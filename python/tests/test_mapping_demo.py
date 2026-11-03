"""Mapping commands isolate geometry and retain complete source detail."""

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest

EXAMPLES = Path(__file__).parents[1] / "examples"


def load(name, file):
    spec = importlib.util.spec_from_file_location(name, EXAMPLES / file)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


demo = load("mapping_demo_test", "mapping_visualization_demo.py")
source = load("mapping_source_test", "tools/mapping_demo_source.py")


@pytest.mark.parametrize("mode", [mode for mode in demo.MODES if not mode.startswith("remote-")])
def test_registration_has_exactly_one_workspace_map_and_no_robot_workloads(mode):
    from horus.bridge.robot_registry import RobotRegistryClient
    args = demo.arguments([mode])
    entity, viz = demo.registration(args)
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized, client.node = False, None
    payload = client._build_global_visualizations_payload([viz])
    assert len(payload) == 1
    assert payload[0]["scope"] == "global"
    assert not entity.sensors
    if mode.startswith("points"):
        settings = payload[0]["point_cloud"]
        assert settings["render_all_points"] and settings["base_sample_stride"] == 1
        assert settings["max_points_per_frame"] == 0
        assert not settings["enable_subpixel_culling"]
    elif mode.startswith("gaussian"):
        assert payload[0]["gaussian_splat"]["binary_chunks"]
        assert payload[0]["gaussian_splat"]["preserve_source_detail"]
        assert not payload[0]["gaussian_splat"]["pointcloud_fallback"]


def test_fixture_transform_is_finite_floor_aligned_and_does_not_remove_triangles():
    from horus.remote_rendering.synthetic_scene import build_industrial_site
    vertices, faces, colors = source.scene_geometry()
    original = build_industrial_site().arrays()
    assert faces.shape == original[1].shape
    assert vertices[:, 2].min() == 0
    assert np.isfinite(vertices).all()
    assert np.array_equal(colors, original[2].astype(np.uint8))
    assert faces.max() < len(vertices)


def test_point_fixture_is_reproducible_and_complete():
    a, c = source.surface_points(2000)
    b, d = source.surface_points(2000)
    assert a.shape == (2000, 3) and c.shape == (2000, 3)
    assert np.array_equal(a, b) and np.array_equal(c, d)
    assert a[:, 2].min() >= 0


def test_remote_commands_keep_full_source_and_use_explicit_transport(monkeypatch):
    monkeypatch.setattr(demo, "real_record", lambda args: {"path": "/capture/map.npz"})
    for mode in ("remote-mesh", "remote-gaussian", "remote-points", "remote-diagnostic"):
        command = demo.remote_command(demo.arguments([mode]))
        assert command[command.index("--source-voxel-size") + 1] == "0"
        assert command[command.index("--protocol") + 1] == "v3"
        assert command[command.index("--transport") + 1] == ("ros" if mode == "remote-diagnostic" else "webrtc")
        if mode == "remote-points":
            assert command[command.index("--scene") + 1] == "recorded_points"
        if mode != "remote-gaussian":
            assert command[command.index("--dataset-path") + 1] == "/capture/map.npz"


@pytest.mark.parametrize("mode", ["mesh", "indexed", "points", "octomap", "nvblox", "points-refresh"])
def test_demo_defaults_to_measured_data_and_never_generated_scene(mode, tmp_path):
    args = demo.arguments([mode])
    commands = demo.child_commands(args, tmp_path / "ready")
    assert not args.synthetic
    assert "--dataset" in commands[0]
    if mode in ("nvblox", "points-refresh"):
        assert any("replay_real_mapping_bag.py" in item for item in commands[0])
    assert demo.dataset_for(args) == {"points": "cow-lady", "octomap": "freiburg"}.get(mode, "g1")


def test_gaussian_does_not_prune_or_create_another_subset(tmp_path):
    commands = demo.child_commands(demo.arguments(["gaussian", "--ply", str(tmp_path / "source.ply")]), tmp_path / "ready")
    assert commands[0][commands[0].index("--splat-render-count") + 1] == "0"
    assert "--sh-prune" not in commands[0]
    assert "--no-fake-robots" in commands[0]
    assert "--binary-chunks" in commands[0]


@pytest.mark.parametrize("content", [None, "not-json", "null", "[]", '{}',
    '{"schema":"horus.bridge.build.v1","webrtc":false}',
    '{"schema":"horus.bridge.build.v1","webrtc":"true"}'])
def test_remote_preflight_rejects_missing_or_disabled_bridge(tmp_path, content):
    capabilities = tmp_path / "build_capabilities.json"
    if content is not None:
        capabilities.write_text(content)
    with pytest.raises(RuntimeError, match="ENABLE_WEBRTC=ON"):
        demo.require_webrtc_bridge(capabilities)


def test_remote_preflight_accepts_enabled_bridge(tmp_path):
    capabilities = tmp_path / "build_capabilities.json"
    capabilities.write_text('{"schema":"horus.bridge.build.v1","webrtc":true}')
    demo.require_webrtc_bridge(capabilities)
