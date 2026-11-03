from __future__ import annotations
from dataclasses import replace

import cv2
import numpy as np
import pytest

from horus.remote_rendering import (
    decode_frame_marker,
    DynamicCameraPose,
    render_stereo_diagnostic,
    render_stereo_frame,
)
from horus.remote_rendering.v3_protocol import (
    EyeFov,
    EyeView,
    Quaternion,
    Vector3,
    ViewState,
    WorkspaceState,
    unpack_ros_diagnostic_frame,
)
from horus.remote_rendering.nvdiffrast_mesh_renderer import (
    NvdiffrastMeshRenderer,
)
from horus.remote_rendering.v3_renderer import _encode_robust_depth_numpy, encode_robust_depth_tensor


class _StereoRenderer:
    def __init__(self):
        self.positions = None
        self.rotations = None
        self.projections = None

    def render_views(
        self,
        positions,
        rotations,
        width,
        height,
        *,
        projections,
        **_,
    ):
        self.positions = positions
        self.rotations = rotations
        self.projections = projections
        left = np.zeros((height, width, 3), dtype=np.uint8)
        right = np.zeros_like(left)
        left[..., 0] = 241
        right[..., 1] = 223
        left_depth = np.full((height, width), 2.0, dtype=np.float32)
        right_depth = np.full((height, width), 3.0, dtype=np.float32)
        left_depth[:4, :4] = np.inf
        return [left, right], np.stack((left_depth, right_depth)), 0.25


class _VideoRenderer(_StereoRenderer):
    def __init__(self):
        super().__init__()
        self.video_render_called = False

    def render_video_views(
        self,
        positions,
        rotations,
        width,
        height,
        *,
        projections,
        visibility_mask_height,
        **_,
    ):
        self.video_render_called = True
        self.positions = positions
        self.rotations = rotations
        self.projections = projections
        left = np.zeros((height, width, 3), dtype=np.uint8)
        right = np.zeros_like(left)
        left[..., 0] = 241
        right[..., 1] = 223
        depth_codes = np.full(
            (2, visibility_mask_height, width),
            127,
            dtype=np.uint8,
        )
        depth_codes[0, 0, 0] = 0
        return np.stack((left, right)), depth_codes, 0.15


def _state() -> ViewState:
    fov = EyeFov(-0.8, 0.9, 0.85, -0.75)
    return ViewState(
        session_id="operator-fixture",
        view_sequence=12,
        sample_time_ns=100,
        predicted_display_time_ns=200,
        predicted_display_period_ns=13_888_889,
        reference_space_id="workspace-local",
        workspace=WorkspaceState(
            revision=7,
            position=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            scale=0.1,
        ),
        left_eye=EyeView(
            Vector3(-0.032, 1.6, 0.2),
            Quaternion(0.0, 0.0, 0.0, 1.0),
            fov,
        ),
        right_eye=EyeView(
            Vector3(0.032, 1.6, 0.2),
            Quaternion(0.0, 0.0, 0.0, 1.0),
            fov,
        ),
        viewport_width=64,
        viewport_height=48,
        refresh_rate_hz=72.0,
    )


def test_v3_agent_renders_independent_eyes_and_preserves_frame_identity():
    renderer = _StereoRenderer()
    poses = (
        DynamicCameraPose((-0.032, 1.6, 0.2), (0.0, 0.0, 0.0, 1.0)),
        DynamicCameraPose((0.032, 1.6, 0.2), (0.0, 0.0, 0.0, 1.0)),
    )
    projections = ((1.2, 1.3, 0.1, -0.2), (1.25, 1.35, -0.1, -0.2))
    result = render_stereo_diagnostic(
        renderer,
        poses,
        projections,
        state=replace(_state(), view_stream_id="reconnected-client"),
        sequence=5000,
        session_id="operator-fixture",
        workspace_revision=7,
        predicted_display_time_ns=200,
        width=64,
        height=48,
        vertical_fov_deg=90.0,
        near_m=0.1,
        far_m=20.0,
        point_radius=1,
        png_compression=1,
    )

    metadata, png = unpack_ros_diagnostic_frame(result.payload)
    decoded = cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_UNCHANGED)

    assert metadata.session_id == "operator-fixture"
    assert metadata.frame_id == 5000
    assert metadata.source_view_sequence == 12
    assert metadata.view_stream_id == "reconnected-client"
    assert metadata.workspace_revision == 7
    assert metadata.width_per_eye == 64
    assert metadata.height_per_eye == 48
    assert decoded.shape == (64, 128, 3)
    assert np.all(decoded[:48, :64, 2] == 241)
    assert np.all(decoded[:48, 64:, 1] == 223)
    assert decoded[48, 0, 0] == 0
    assert decoded[49, 1, 0] > 0
    assert metadata.stereo_layout == "side_by_side_with_depth"
    assert metadata.depth_representation == "linear_u16_nibbles_in_video"
    assert metadata.depth_near_m == 0.1
    assert metadata.depth_far_m == 20.0
    assert renderer.positions == [poses[0].position, poses[1].position]
    assert renderer.projections == projections


def test_v3_webrtc_frame_carries_robust_depth_in_the_same_video_atlas():
    renderer = _StereoRenderer()
    poses = (
        DynamicCameraPose((-0.032, 1.6, 0.2), (0.0, 0.0, 0.0, 1.0)),
        DynamicCameraPose((0.032, 1.6, 0.2), (0.0, 0.0, 0.0, 1.0)),
    )
    projections = ((1.2, 1.3, 0.1, -0.2), (1.25, 1.35, -0.1, -0.2))

    result = render_stereo_frame(
        renderer,
        poses,
        projections,
        state=_state(),
        sequence=12,
        session_id="operator-fixture",
        workspace_revision=7,
        predicted_display_time_ns=200,
        width=192,
        height=48,
        vertical_fov_deg=90.0,
        near_m=0.1,
        far_m=20.0,
        point_radius=1,
        codec="h264",
        visibility_mask_height=8,
        frame_marker_height=16,
    )

    assert result.video_rgb.shape == (72, 384, 3)
    assert result.metadata.stereo_layout == "side_by_side_with_depth"
    assert result.metadata.depth_representation == "linear_u16_nibbles_in_video"
    assert result.metadata.visibility_mask_height == 8
    assert result.metadata.frame_marker_height == 16
    assert result.metadata.frame_marker == 12
    assert result.metadata.video_height == 72
    assert decode_frame_marker(result.video_rgb[-16:]) == 12
    assert result.video_rgb[48, 0, 0] == 0
    assert 1 <= result.video_rgb[48, 1, 0] <= 255


def test_v3_webrtc_uses_lightweight_video_render_without_depth_readback():
    renderer = _VideoRenderer()
    poses = (
        DynamicCameraPose((-0.032, 1.6, 0.2), (0.0, 0.0, 0.0, 1.0)),
        DynamicCameraPose((0.032, 1.6, 0.2), (0.0, 0.0, 0.0, 1.0)),
    )
    result = render_stereo_frame(
        renderer,
        poses,
        ((1.2, 1.3, 0.1, -0.2), (1.25, 1.35, -0.1, -0.2)),
        state=_state(),
        sequence=12,
        session_id="operator-fixture",
        workspace_revision=7,
        predicted_display_time_ns=200,
        width=192,
        height=48,
        vertical_fov_deg=90.0,
        near_m=0.1,
        far_m=20.0,
        point_radius=1,
        codec="h264",
        visibility_mask_height=8,
        frame_marker_height=16,
    )

    assert renderer.video_render_called
    assert result.depths is None
    assert result.video_rgb.shape == (72, 384, 3)
    assert result.video_rgb[48, 0, 0] == 0
    assert result.video_rgb[48, 1, 0] == 127
    assert decode_frame_marker(result.video_rgb[-16:]) == 12
    assert result.valid_depth_fraction == (2 * 8 * 48 - 1) / (2 * 8 * 48)


def test_robust_depth_encoding_round_trips_linear_uint16_and_invalid_samples():
    depths = np.asarray(
        [
            [[0.1, 0.1, 0.1, 0.1, 20.0, 20.0, 20.0, 20.0]],
            [[np.nan, 0.05, 21.0, np.inf, 5.0, 5.0, 5.0, 5.0]],
        ],
        dtype=np.float32,
    )

    encoded = _encode_robust_depth_numpy(depths, 1, 0.1, 20.0)

    assert encoded.shape == (2, 1, 8)
    assert np.all(encoded[0, 0, 0::2] == 32)
    assert np.all(encoded[0, 0, 1::2] == 224)
    assert np.all(encoded[1, 0, 0::2] == 0)

    plane_width = encoded.shape[2] // 4
    luma = [
        encoded[:, :, offset * plane_width:(offset + 1) * plane_width]
        .astype(np.float32)
        for offset in range(4)
    ]
    nibbles = [
        np.rint(np.clip((plane - 32.0) * 15.0 / 192.0, 0.0, 15.0))
        for plane in luma
    ]
    code = nibbles[0] * 4096 + nibbles[1] * 256 + nibbles[2] * 16 + nibbles[3]
    decoded = 0.1 + (20.0 - 0.1) * code / 65535.0
    decoded[luma[0] < 24.0] = np.inf

    assert decoded[0, 0, 0] == pytest.approx(0.1, abs=0.0004)
    assert decoded[0, 0, 1] == pytest.approx(20.0, abs=0.0004)
    assert np.isinf(decoded[1, 0, 0])
    assert decoded[1, 0, 1] == pytest.approx(5.0, abs=0.0004)


def test_gpu_depth_finish_matches_robust_numpy_contract():
    torch = pytest.importorskip("torch")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    image = torch.zeros((2, 1, 8, 3), dtype=torch.float32, device=device)
    visible = torch.ones((2, 1, 8, 1), dtype=torch.bool, device=device)
    depth = torch.tensor(
        [
            [[[0.1], [0.1], [0.1], [0.1], [20.0], [20.0], [20.0], [20.0]]],
            [[[float("inf")], [0.05], [21.0], [float("nan")], [5.0], [5.0], [5.0], [5.0]]],
        ],
        dtype=torch.float32,
        device=device,
    )
    finish = NvdiffrastMeshRenderer._build_finish_fragments(torch)

    _, encoded = finish(
        image,
        visible,
        depth,
        torch.tensor([0], dtype=torch.int64, device=device),
        0.1,
        20.0,
    )
    expected = _encode_robust_depth_numpy(
        depth[..., 0].cpu().numpy(),
        1,
        0.1,
        20.0,
    )

    assert np.array_equal(encoded.cpu().numpy(), expected)


def test_tensor_depth_packing_matches_numpy_at_texel_centers():
    torch = pytest.importorskip("torch")
    depths = np.arange(2 * 8 * 16, dtype=np.float32).reshape(2, 8, 16) / 10 + 0.1
    depths[0, 2, :4] = np.inf
    expected = _encode_robust_depth_numpy(depths, 2, 0.1, 30.0)
    tensor = torch.from_numpy(depths)
    actual = encode_robust_depth_tensor(torch, tensor, 2, 0.1, 30.0)
    np.testing.assert_array_equal(actual.numpy(), expected)
    finish = NvdiffrastMeshRenderer._build_finish_fragments(torch)
    _, mesh = finish(torch.zeros((2, 8, 16, 3)), None,
                     tensor.flip(1)[..., None], torch.tensor([2, 6]), 0.1, 30.0)
    np.testing.assert_array_equal(mesh.numpy(), expected)
