import json
import math
from dataclasses import replace

import numpy as np
import pytest

from horus.remote_rendering.v3_protocol import (
    ClockAlignment,
    ClockSample,
    EyeFov,
    EyeView,
    LatestValueQueue,
    Quaternion,
    RenderedFrameMetadata,
    Vector3,
    ViewState,
    ViewStreamSequenceGate,
    WorkspaceState,
    eye_fov_from_unity_projection,
    pack_ros_diagnostic_frame,
    unpack_ros_diagnostic_frame,
)
from horus.remote_rendering.v3_renderer import eye_view_projection_matrix
from horus.remote_rendering.dynamic_stream import quaternion_to_matrix


def _view_state(sequence: int = 17) -> ViewState:
    left_fov = eye_fov_from_unity_projection(1.31, 1.47, 0.08, -0.04)
    right_fov = eye_fov_from_unity_projection(1.29, 1.46, -0.07, -0.03)
    return ViewState(
        session_id="operator-a",
        view_sequence=sequence,
        sample_time_ns=1_000_000,
        predicted_display_time_ns=1_011_111,
        predicted_display_period_ns=13_888_889,
        reference_space_id="workspace-local",
        workspace=WorkspaceState(
            4,
            Vector3(1.0, 2.0, 3.0),
            Quaternion(0.0, 0.0, 0.0, 1.0),
            0.1,
        ),
        left_eye=EyeView(
            Vector3(-0.032, 1.7, 0.1),
            Quaternion(0.0, 0.0, 0.0, 1.0),
            left_fov,
        ),
        right_eye=EyeView(
            Vector3(0.032, 1.7, 0.1),
            Quaternion(0.0, 0.0, 0.0, 1.0),
            right_fov,
        ),
        viewport_width=1832,
        viewport_height=1920,
        refresh_rate_hz=72.0,
    )


def test_view_state_round_trip_preserves_per_eye_display_time_contract():
    source = replace(_view_state(), view_stream_id="connection-a")
    decoded = ViewState.from_json(source.to_json())

    assert decoded == source
    assert decoded.left_eye != decoded.right_eye
    assert decoded.predicted_display_time_ns > decoded.sample_time_ns


def test_reconnect_restarts_pose_sequence_without_restarting_render_identity():
    gate = ViewStreamSequenceGate()
    old = replace(_view_state(4863), view_stream_id="connection-a")
    new = replace(_view_state(1), view_stream_id="connection-b", sample_time_ns=10)
    assert gate.accept(old, 100.0) is None
    assert gate.accept(new, 100.5) == "view_stream_busy"
    assert gate.accept(new, 102.1) is None  # New process/clock/pose counter.
    assert gate.revision == 2
    assert gate.last_sequence == 1
    assert gate.accept(replace(old, view_sequence=4864, sample_time_ns=2_000_000),
                       105.0) == "retired_view_stream"
    assert gate.active_stream == "connection-b"


def test_view_stream_rejects_duplicate_reordered_and_regressed_samples():
    gate = ViewStreamSequenceGate()
    state = replace(_view_state(10), view_stream_id="connection-a")
    assert gate.accept(state, 100.0) is None
    assert gate.accept(state, 101.0) == "stale_sample"
    assert gate.accept(replace(state, view_sequence=9, sample_time_ns=2_000_000),
                       105.0) == "stale_sequence"
    assert gate.accept(replace(state, view_sequence=11, sample_time_ns=999_999),
                       106.0) == "stale_sample"
    assert gate.revision == 1


def test_legacy_client_reconnect_requires_idle_and_a_newer_monotonic_sample():
    gate = ViewStreamSequenceGate()
    old = _view_state(4863)
    new = replace(_view_state(1), sample_time_ns=2_000_000)
    assert gate.accept(old, 100.0) is None
    assert gate.accept(new, 100.5) == "stale_sequence"
    assert gate.accept(new, 102.1) is None
    assert gate.accept(old, 105.0) == "stale_sample"
    assert gate.revision == 2


@pytest.mark.parametrize(
    "terms",
    ((1.31, 1.47, 0.08, -0.04), (1.29, 1.46, -0.07, 0.03)),
)
def test_unity_projection_converts_to_equivalent_renderer_ndc(terms):
    fov = eye_fov_from_unity_projection(*terms)
    scale_x, scale_y, offset_x, offset_y = fov.renderer_projection()

    for tangent, expected in ((fov.left, -1.0), (fov.right, 1.0)):
        assert np.isclose(scale_x * tangent + offset_x, expected, atol=1e-7)
    for tangent, expected in ((fov.down, -1.0), (fov.up, 1.0)):
        assert np.isclose(scale_y * tangent + offset_y, expected, atol=1e-7)
    assert np.isclose(offset_x, -terms[2])
    assert np.isclose(offset_y, -terms[3])


def test_serialized_eye_reconstructs_full_renderer_view_projection_matrix():
    decoded = ViewState.from_json(_view_state().to_json())
    near_m = 0.1
    far_m = 40.0
    matrices = []

    for eye in (decoded.left_eye, decoded.right_eye):
        matrix = eye_view_projection_matrix(eye, near_m, far_m)
        matrices.append(matrix)

        # Start with a known eye-local point, place it into world space, then
        # verify the packet-derived matrix reconstructs the same NDC point.
        camera_point = np.asarray((0.17, -0.11, 2.3), dtype=np.float64)
        camera_to_world = quaternion_to_matrix(
            (eye.rotation.x, eye.rotation.y, eye.rotation.z, eye.rotation.w)
        ).astype(np.float64)
        eye_position = np.asarray(
            (eye.position.x, eye.position.y, eye.position.z),
            dtype=np.float64,
        )
        world_point = camera_to_world @ camera_point + eye_position
        clip = matrix @ np.append(world_point, 1.0)
        ndc = clip[:3] / clip[3]
        scale_x, scale_y, offset_x, offset_y = eye.fov.renderer_projection()
        expected_xy = np.asarray(
            (
                scale_x * camera_point[0] / camera_point[2] + offset_x,
                scale_y * camera_point[1] / camera_point[2] + offset_y,
            )
        )
        assert np.allclose(ndc[:2], expected_xy, atol=1e-7)

    assert not np.allclose(matrices[0], matrices[1], atol=1e-8)


def test_invalid_workspace_revision_and_fov_fail_closed():
    payload = _view_state().to_dict()
    payload["workspace"]["revision"] = -1
    with pytest.raises(ValueError, match="revision"):
        ViewState.from_dict(payload)

    payload = _view_state().to_dict()
    payload["left_eye"]["fov"]["left"] = 1.0
    with pytest.raises(ValueError, match="ordered|vertical"):
        ViewState.from_dict(payload)


def test_latest_value_queue_never_drains_stale_views():
    queue = LatestValueQueue[ViewState]()
    queue.put(_view_state(1))
    queue.put(_view_state(2))
    queue.put(_view_state(3))

    assert queue.replaced_count == 2
    assert queue.get(timeout=0.0).view_sequence == 3
    assert queue.get(timeout=0.0) is None


def test_clock_alignment_uses_low_rtt_samples_for_offset():
    alignment = ClockAlignment(maximum_samples=9)
    # True server-minus-client offset is 2 ms. The final sample has large
    # asymmetric network delay and must not dominate the low-RTT estimator.
    for delay_ns in (500_000, 600_000, 700_000, 800_000, 900_000, 1_000_000):
        alignment.add(
            ClockSample(
                client_send_ns=10_000_000,
                server_receive_ns=12_000_000 + delay_ns,
                server_send_ns=12_100_000 + delay_ns,
                client_receive_ns=10_100_000 + delay_ns * 2,
            )
        )
    alignment.add(ClockSample(10_000_000, 32_000_000, 32_100_000, 30_100_000))

    assert math.isclose(alignment.offset_ns, 2_000_000, abs_tol=1.0)
    assert alignment.round_trip_ns > 0


def test_frame_metadata_rejects_unknown_layout():
    frame = RenderedFrameMetadata(
        session_id="operator-a",
        frame_id=7,
        source_view_sequence=5,
        predicted_display_time_ns=42,
        workspace_revision=2,
        render_start_ns=10,
        render_complete_ns=20,
        encode_start_ns=21,
        capture_time_ns=30,
        stereo_layout="side_by_side",
        width_per_eye=1024,
        height_per_eye=1024,
        codec="png",
    )
    assert RenderedFrameMetadata.from_json(frame.to_json()) == frame

    payload = frame.to_dict()
    payload["stereo_layout"] = "centre_eye"
    with pytest.raises(ValueError, match="layout"):
        RenderedFrameMetadata.from_json(json.dumps(payload))


def test_ros_diagnostic_bundle_preserves_frame_identity_and_png():
    frame = RenderedFrameMetadata(
        session_id="operator-a",
        frame_id=9,
        source_view_sequence=8,
        predicted_display_time_ns=42,
        workspace_revision=2,
        render_start_ns=10,
        render_complete_ns=20,
        encode_start_ns=21,
        capture_time_ns=30,
        stereo_layout="side_by_side",
        width_per_eye=1024,
        height_per_eye=1024,
        codec="png",
    )
    png = b"\x89PNG\r\n\x1a\nfixture"

    decoded, decoded_png = unpack_ros_diagnostic_frame(
        pack_ros_diagnostic_frame(frame, png)
    )

    assert decoded == frame
    assert decoded_png == png


def test_ros_diagnostic_bundle_rejects_trailing_or_missing_bytes():
    frame = RenderedFrameMetadata(
        "operator-a", 1, 1, 1, 1, 1, 2, 3, 4,
        "side_by_side", 64, 64, "png",
    )
    payload = pack_ros_diagnostic_frame(frame, b"png")
    with pytest.raises(ValueError, match="length"):
        unpack_ros_diagnostic_frame(payload + b"trailing")
    with pytest.raises(ValueError, match="length"):
        unpack_ros_diagnostic_frame(payload[:-1])


def test_h264_metadata_requires_exact_frame_marker_strip():
    frame = RenderedFrameMetadata(
        session_id="operator-a",
        frame_id=0x12345,
        source_view_sequence=0x12345,
        predicted_display_time_ns=42,
        workspace_revision=2,
        render_start_ns=10,
        render_complete_ns=20,
        encode_start_ns=21,
        capture_time_ns=30,
        stereo_layout="side_by_side_with_depth",
        width_per_eye=1024,
        height_per_eye=1024,
        codec="h264",
        color_range="limited",
        depth_representation="linear_u16_nibbles_in_video",
        visibility_mask_height=128,
        frame_marker_height=16,
        frame_marker=0x2345,
        video_height=1168,
        depth_near_m=0.1,
        depth_far_m=20.0,
    )

    assert RenderedFrameMetadata.from_json(frame.to_json()) == frame
    payload = frame.to_dict()
    payload["color_range"] = "full"
    with pytest.raises(ValueError, match="limited-range"):
        RenderedFrameMetadata.from_json(json.dumps(payload))
    payload = frame.to_dict()
    payload["frame_marker_height"] = 0
    payload["video_height"] = 1152
    with pytest.raises(ValueError, match="frame marker"):
        RenderedFrameMetadata.from_json(json.dumps(payload))
