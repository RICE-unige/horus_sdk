import importlib.util
from pathlib import Path
import sys


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "examples"
    / "remote_map_portal_registration.py"
)
SPEC = importlib.util.spec_from_file_location(
    "remote_map_portal_registration",
    SCRIPT_PATH,
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_ros_profiles_preserve_depth_fidelity_and_bound_frame_rate():
    expected_fps = {"fast": 15, "balanced": 15, "quality": 15}
    expected_depth = {"fast": 448, "balanced": 384, "quality": 448}
    expected_reprojection = {"fast": 224, "balanced": 288, "quality": 336}

    for name, profile in MODULE.PROFILES.items():
        assert profile["depth_width"] == expected_depth[name]
        assert profile["depth_height"] == expected_depth[name]
        assert profile["visibility_mask_height"] == expected_reprojection[name]
        assert profile["visibility_mask_height"] == profile["width"] // 4
        assert MODULE.stream_fps(profile, "ros") == expected_fps[name]
        assert MODULE.stream_fps(profile, "ros_debug") == expected_fps[name]
        assert MODULE.render_update_fps(profile, "ros", 60.0) == expected_fps[name]


def test_render_rate_never_exceeds_requested_rate():
    profile = MODULE.PROFILES["balanced"]

    assert MODULE.render_update_fps(profile, "ros", 7.5) == 7.5
    assert MODULE.render_update_fps(profile, "webrtc", 48.0) == 48.0


def test_cloudxr_style_v3_webrtc_is_the_launcher_default(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["remote_map_portal_registration.py"])

    args = MODULE.parse_args()

    assert args.protocol == "v3"
    assert args.transport == "webrtc"
    assert args.encoder == "nvenc"
    assert args.source_voxel_size == 0.0
