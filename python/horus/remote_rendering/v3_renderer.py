"""Deterministic true-stereo rendering helpers for remote rendering v3."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Sequence

import numpy as np

from .dynamic_stream import quaternion_to_matrix
from .frame_protocol import embed_frame_marker
from .v3_protocol import RenderedFrameMetadata, ViewState, pack_ros_diagnostic_frame


DEPTH_SYMBOL_LUMA_MIN = 32
DEPTH_SYMBOL_LUMA_MAX = 224
DEPTH_SYMBOLS_PER_SAMPLE = 4
DEPTH_VALID_LUMA_MIN = 24


@dataclass(frozen=True)
class StereoDiagnosticRender:
    payload: bytes
    metadata: RenderedFrameMetadata
    stereo_rgba: np.ndarray
    depths: np.ndarray | None
    visibility_fraction: float
    cuda_ms: float

    @property
    def valid_depth_fraction(self) -> float:
        if self.depths is not None:
            return float(np.isfinite(self.depths).mean())
        return self.visibility_fraction


@dataclass(frozen=True)
class StereoFrameRender:
    metadata: RenderedFrameMetadata
    stereo_rgba: np.ndarray
    video_rgb: np.ndarray
    depths: np.ndarray | None
    visibility_fraction: float
    cuda_ms: float

    @property
    def valid_depth_fraction(self) -> float:
        if self.depths is not None:
            return float(np.isfinite(self.depths).mean())
        return self.visibility_fraction


@dataclass(frozen=True)
class CudaStereoFrameRender:
    metadata: RenderedFrameMetadata
    colors: Any
    depth_codes: Any
    visibility_fraction: float
    cuda_ms: float

    @property
    def valid_depth_fraction(self) -> float:
        return self.visibility_fraction


def eye_view_matrix(eye) -> np.ndarray:
    """Reconstruct the renderer world-to-eye matrix from a v3 eye packet."""
    position = np.asarray(
        (eye.position.x, eye.position.y, eye.position.z),
        dtype=np.float64,
    )
    camera_to_world = quaternion_to_matrix(
        (eye.rotation.x, eye.rotation.y, eye.rotation.z, eye.rotation.w)
    ).astype(np.float64)
    world_to_camera = camera_to_world.T
    view = np.eye(4, dtype=np.float64)
    view[:3, :3] = world_to_camera
    view[:3, 3] = -(world_to_camera @ position)
    return view


def eye_projection_matrix(eye, near_m: float, far_m: float) -> np.ndarray:
    """Build the positive-forward renderer projection used by GPU backends."""
    if near_m <= 0.0 or far_m <= near_m:
        raise ValueError("projection requires 0 < near_m < far_m")
    scale_x, scale_y, offset_x, offset_y = eye.fov.renderer_projection()
    depth_scale = (far_m + near_m) / (far_m - near_m)
    depth_offset = -2.0 * far_m * near_m / (far_m - near_m)
    return np.asarray(
        (
            (scale_x, 0.0, offset_x, 0.0),
            (0.0, scale_y, offset_y, 0.0),
            (0.0, 0.0, depth_scale, depth_offset),
            (0.0, 0.0, 1.0, 0.0),
        ),
        dtype=np.float64,
    )


def eye_view_projection_matrix(eye, near_m: float, far_m: float) -> np.ndarray:
    """Reconstruct a complete clip-from-world matrix for conformance tests."""
    return eye_projection_matrix(eye, near_m, far_m) @ eye_view_matrix(eye)


def render_stereo_frame(
    renderer: Any,
    poses: Sequence[Any],
    projections: Sequence[Sequence[float]],
    *,
    state: ViewState | None,
    sequence: int,
    session_id: str,
    workspace_revision: int,
    predicted_display_time_ns: int,
    width: int,
    height: int,
    vertical_fov_deg: float,
    near_m: float,
    far_m: float,
    point_radius: int,
    codec: str,
    visibility_mask_height: int = 0,
    frame_marker_height: int = 0,
) -> StereoFrameRender:
    """Render exact left/right views without coupling them to a carrier."""
    if len(poses) != 2 or len(projections) != 2:
        raise ValueError("v3 stereo rendering requires exactly two eye views")
    if width <= 0 or height <= 0:
        raise ValueError("stereo dimensions must be positive")
    if visibility_mask_height > 0 and width % DEPTH_SYMBOLS_PER_SAMPLE:
        raise ValueError("depth-enabled stereo width must be divisible by four")

    render_start_ns = time.monotonic_ns()
    video_render = getattr(renderer, "render_video_views", None)
    depth_codes = None
    if visibility_mask_height > 0 and callable(video_render):
        colors, depth_codes, cuda_ms = video_render(
            [poses[0].position, poses[1].position],
            [poses[0].rotation, poses[1].rotation],
            width,
            height,
            vertical_fov_deg=vertical_fov_deg,
            near_m=near_m,
            far_m=far_m,
            point_radius=point_radius,
            projections=projections,
            visibility_mask_height=visibility_mask_height,
        )
        depths = None
    else:
        colors, depths, cuda_ms = renderer.render_views(
            [poses[0].position, poses[1].position],
            [poses[0].rotation, poses[1].rotation],
            width,
            height,
            vertical_fov_deg=vertical_fov_deg,
            near_m=near_m,
            far_m=far_m,
            point_radius=point_radius,
            projections=projections,
        )
    render_complete_ns = time.monotonic_ns()
    if len(colors) != 2 or (depths is not None and len(depths) != 2):
        raise RuntimeError("renderer did not return two eye views")
    if depth_codes is not None and np.asarray(depth_codes).shape != (
        2,
        int(visibility_mask_height),
        int(width),
    ):
        raise RuntimeError("renderer returned an invalid depth-strip shape")

    if visibility_mask_height > 0 and depth_codes is None:
        depth_codes = _encode_robust_depth_numpy(
            depths,
            visibility_mask_height,
            near_m,
            far_m,
        )

    rgba_views = []
    for eye_index, color in enumerate(colors):
        color_array = np.ascontiguousarray(color, dtype=np.uint8)
        if color_array.shape != (height, width, 3):
            raise RuntimeError("renderer returned an invalid color shape")
        if depths is not None:
            depth_array = np.asarray(depths[eye_index], dtype=np.float32)
            if depth_array.shape != (height, width):
                raise RuntimeError("renderer returned an invalid depth shape")
            alpha = np.where(np.isfinite(depth_array), 255, 0).astype(np.uint8)
        else:
            plane_width = width // DEPTH_SYMBOLS_PER_SAMPLE
            valid_samples = (
                np.asarray(depth_codes[eye_index], dtype=np.uint8)[:, :plane_width]
                >= DEPTH_VALID_LUMA_MIN
            )
            import cv2
            alpha = cv2.resize(
                np.where(valid_samples, 255, 0).astype(np.uint8),
                (width, height),
                interpolation=cv2.INTER_NEAREST,
            )
        rgba_views.append(np.concatenate((color_array, alpha[..., None]), axis=2))
    stereo_rgba = np.ascontiguousarray(np.concatenate(rgba_views, axis=1))
    depth_array = (
        np.ascontiguousarray(np.stack(depths), dtype=np.float32)
        if depths is not None
        else None
    )
    if depth_codes is not None:
        plane_width = width // DEPTH_SYMBOLS_PER_SAMPLE
        valid_samples = np.asarray(depth_codes, dtype=np.uint8)[..., :plane_width]
        visibility_fraction = float(
            np.count_nonzero(valid_samples >= DEPTH_VALID_LUMA_MIN)
            / valid_samples.size
        )
    else:
        visibility_fraction = float(np.isfinite(depth_array).mean())

    video_rgb = stereo_rgba[..., :3]
    layout = "side_by_side"
    depth_representation = "none"
    visibility_mask_height = int(visibility_mask_height)
    if visibility_mask_height > 0:
        depth_atlas = np.concatenate(
            [np.asarray(value, dtype=np.uint8) for value in depth_codes],
            axis=1,
        )
        depth_rgb = np.repeat(depth_atlas[..., None], 3, axis=2)
        video_rgb = np.ascontiguousarray(
            np.concatenate((video_rgb, depth_rgb), axis=0),
            dtype=np.uint8,
        )
        layout = "side_by_side_with_depth"
        depth_representation = "linear_u16_nibbles_in_video"
    frame_marker_height = int(frame_marker_height)
    if frame_marker_height > 0:
        marker_strip = np.zeros(
            (frame_marker_height, video_rgb.shape[1], 3),
            dtype=np.uint8,
        )
        video_rgb = np.ascontiguousarray(
            np.concatenate((video_rgb, marker_strip), axis=0),
            dtype=np.uint8,
        )
        embed_frame_marker(video_rgb[-frame_marker_height:], sequence)

    capture_time_ns = time.monotonic_ns()

    metadata = RenderedFrameMetadata(
        session_id=(
            state.session_id
            if state is not None
            else (session_id.strip() or "preview")
        ),
        frame_id=max(0, int(sequence)),
        source_view_sequence=state.view_sequence if state else max(0, int(sequence)),
        view_stream_id=state.view_stream_id if state else "",
        predicted_display_time_ns=max(0, int(predicted_display_time_ns)),
        workspace_revision=max(0, int(workspace_revision)),
        render_start_ns=render_start_ns,
        render_complete_ns=render_complete_ns,
        encode_start_ns=capture_time_ns,
        capture_time_ns=capture_time_ns,
        stereo_layout=layout,
        width_per_eye=width,
        height_per_eye=height,
        codec=codec,
        color_space="srgb",
        color_range="limited" if codec == "h264" else "full",
        depth_representation=depth_representation,
        visibility_mask_height=visibility_mask_height,
        frame_marker_height=frame_marker_height,
        frame_marker=max(0, int(sequence)) & 0xFFFF,
        video_height=int(video_rgb.shape[0]),
        depth_near_m=float(near_m),
        depth_far_m=float(far_m),
    )
    return StereoFrameRender(
        metadata=metadata,
        stereo_rgba=stereo_rgba,
        video_rgb=video_rgb,
        depths=depth_array,
        visibility_fraction=visibility_fraction,
        cuda_ms=float(cuda_ms),
    )


def render_stereo_frame_cuda(
    renderer: Any,
    poses: Sequence[Any],
    projections: Sequence[Sequence[float]],
    *,
    state: ViewState | None,
    sequence: int,
    session_id: str,
    workspace_revision: int,
    predicted_display_time_ns: int,
    width: int,
    height: int,
    vertical_fov_deg: float,
    near_m: float,
    far_m: float,
    point_radius: int,
    visibility_mask_height: int,
    frame_marker_height: int = 0,
) -> CudaStereoFrameRender:
    """Render exact stereo while keeping every full-resolution pixel on CUDA."""
    if len(poses) != 2 or len(projections) != 2:
        raise ValueError("v3 stereo rendering requires exactly two eye views")
    if width <= 0 or height <= 0 or visibility_mask_height <= 0:
        raise ValueError("stereo and visibility dimensions must be positive")
    if width % DEPTH_SYMBOLS_PER_SAMPLE:
        raise ValueError("depth-enabled stereo width must be divisible by four")
    render_video_cuda = getattr(renderer, "render_video_views_cuda", None)
    if not callable(render_video_cuda):
        raise TypeError("renderer does not expose CUDA-resident video output")

    render_start_ns = time.monotonic_ns()
    colors, depth_codes, cuda_ms = render_video_cuda(
        [poses[0].position, poses[1].position],
        [poses[0].rotation, poses[1].rotation],
        width,
        height,
        vertical_fov_deg=vertical_fov_deg,
        near_m=near_m,
        far_m=far_m,
        point_radius=point_radius,
        projections=projections,
        visibility_mask_height=visibility_mask_height,
    )
    render_complete_ns = time.monotonic_ns()
    expected_colors = (2, int(height), int(width), 3)
    expected_depth = (2, int(visibility_mask_height), int(width))
    if tuple(colors.shape) != expected_colors:
        raise RuntimeError("renderer returned an invalid CUDA color shape")
    if tuple(depth_codes.shape) != expected_depth:
        raise RuntimeError("renderer returned an invalid CUDA depth-strip shape")

    # Visibility is diagnostic-only. Sampling the binary mask avoids a
    # full-frame reduction and CPU synchronization in the latency-critical
    # render path while retaining a representative coverage value.
    plane_width = int(width) // DEPTH_SYMBOLS_PER_SAMPLE
    sampled_visibility = depth_codes[:, ::16, :plane_width:16]
    visible = int(
        sampled_visibility.ge(DEPTH_VALID_LUMA_MIN).count_nonzero().item()
    )
    visibility_fraction = visible / float(sampled_visibility.numel())
    metadata = RenderedFrameMetadata(
        session_id=(
            state.session_id
            if state is not None
            else (session_id.strip() or "preview")
        ),
        frame_id=max(0, int(sequence)),
        source_view_sequence=state.view_sequence if state else max(0, int(sequence)),
        view_stream_id=state.view_stream_id if state else "",
        predicted_display_time_ns=max(0, int(predicted_display_time_ns)),
        workspace_revision=max(0, int(workspace_revision)),
        render_start_ns=render_start_ns,
        render_complete_ns=render_complete_ns,
        encode_start_ns=render_complete_ns,
        capture_time_ns=render_complete_ns,
        stereo_layout="side_by_side_with_depth",
        width_per_eye=int(width),
        height_per_eye=int(height),
        codec="h264",
        color_space="srgb",
        color_range="limited",
        depth_representation="linear_u16_nibbles_in_video",
        visibility_mask_height=int(visibility_mask_height),
        frame_marker_height=max(0, int(frame_marker_height)),
        frame_marker=max(0, int(sequence)) & 0xFFFF,
        video_height=int(
            height + visibility_mask_height + max(0, int(frame_marker_height))
        ),
        depth_near_m=float(near_m),
        depth_far_m=float(far_m),
    )
    return CudaStereoFrameRender(
        metadata=metadata,
        colors=colors,
        depth_codes=depth_codes,
        visibility_fraction=visibility_fraction,
        cuda_ms=float(cuda_ms),
    )


def render_stereo_diagnostic(
    renderer: Any,
    poses: Sequence[Any],
    projections: Sequence[Sequence[float]],
    *,
    state: ViewState | None,
    sequence: int,
    session_id: str,
    workspace_revision: int,
    predicted_display_time_ns: int,
    width: int,
    height: int,
    vertical_fov_deg: float,
    near_m: float,
    far_m: float,
    point_radius: int,
    png_compression: int,
) -> StereoDiagnosticRender:
    """Render exact stereo and package the bounded ROS diagnostic carrier."""
    frame = render_stereo_frame(
        renderer,
        poses,
        projections,
        state=state,
        sequence=sequence,
        session_id=session_id,
        workspace_revision=workspace_revision,
        predicted_display_time_ns=predicted_display_time_ns,
        width=width,
        height=height,
        vertical_fov_deg=vertical_fov_deg,
        near_m=near_m,
        far_m=far_m,
        point_radius=point_radius,
        codec="png",
        visibility_mask_height=max(16, min(512, height // 9)),
    )
    encode_start_ns = time.monotonic_ns()
    import cv2
    success, encoded = cv2.imencode(
        ".png",
        cv2.cvtColor(frame.video_rgb, cv2.COLOR_RGB2BGR),
        (cv2.IMWRITE_PNG_COMPRESSION, int(png_compression)),
    )
    if not success:
        raise RuntimeError("failed to encode v3 diagnostic stereo PNG")
    capture_time_ns = time.monotonic_ns()
    metadata = RenderedFrameMetadata(
        **{
            **frame.metadata.__dict__,
            "encode_start_ns": encode_start_ns,
            "capture_time_ns": capture_time_ns,
        }
    )
    return StereoDiagnosticRender(
        payload=pack_ros_diagnostic_frame(metadata, encoded.tobytes()),
        metadata=metadata,
        stereo_rgba=frame.stereo_rgba,
        depths=frame.depths,
        visibility_fraction=frame.visibility_fraction,
        cuda_ms=frame.cuda_ms,
    )


def encode_robust_depth_tensor(torch, depths, output_height, near_m, far_m):
    """Pack the existing depth contract on-device; no full-image readback."""
    if depths.ndim != 3 or depths.shape[2] % DEPTH_SYMBOLS_PER_SAMPLE:
        raise ValueError("depth must have shape (N, H, W) with W divisible by four")
    if output_height <= 0 or near_m <= 0 or far_m <= near_m:
        raise ValueError("invalid robust-depth encoding parameters")
    reduced = torch.nn.functional.interpolate(
        depths[:, None], size=(int(output_height), depths.shape[2]),
        mode="nearest-exact")[:, 0]
    groups = reduced.reshape(depths.shape[0], int(output_height), -1, 4)
    finite = torch.isfinite(groups) & (groups >= near_m) & (groups <= far_m)
    sampled = torch.where(finite, groups, torch.full_like(groups, float("inf"))).amin(dim=3)
    valid = torch.isfinite(sampled)
    normalized = torch.nan_to_num((sampled - near_m) / (far_m - near_m),
                                  nan=0.0, posinf=0.0, neginf=0.0).clamp(0.0, 1.0)
    code = normalized.mul(65535.0).round().to(torch.int32)
    planes = []
    for shift in (12, 8, 4, 0):
        nibble = (code >> shift) & 0x0F
        luma = nibble.mul(192.0 / 15.0).add(32.0).round()
        planes.append(torch.where(valid, luma, torch.zeros_like(luma)).to(torch.uint8))
    return torch.cat(planes, dim=2)


def _encode_robust_depth_numpy(
    depths: np.ndarray,
    output_height: int,
    near_m: float,
    far_m: float,
) -> np.ndarray:
    """Encode 16-bit metric depth as four H.264-resistant luma planes."""
    source = np.asarray(depths, dtype=np.float32)
    if source.ndim != 3 or source.shape[0] != 2:
        raise ValueError("stereo depth must have shape (2, H, W)")
    if source.shape[2] % DEPTH_SYMBOLS_PER_SAMPLE:
        raise ValueError("stereo depth width must be divisible by four")
    if output_height <= 0 or near_m <= 0.0 or far_m <= near_m:
        raise ValueError("invalid robust-depth encoding parameters")
    encoded = []
    for eye_depth in source:
        rows = np.floor((np.arange(int(output_height)) + 0.5)
                        * eye_depth.shape[0] / int(output_height)).astype(np.int64)
        reduced = eye_depth[rows]
        groups = reduced.reshape(
            int(output_height),
            reduced.shape[1] // DEPTH_SYMBOLS_PER_SAMPLE,
            DEPTH_SYMBOLS_PER_SAMPLE,
        )
        finite = np.isfinite(groups) & (groups >= near_m) & (groups <= far_m)
        sampled = np.where(finite, groups, np.inf).min(axis=2)
        valid = np.isfinite(sampled)
        normalized = np.nan_to_num(
            (sampled - near_m) / (far_m - near_m),
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )
        code = np.rint(
            np.clip(normalized, 0.0, 1.0) * 65535.0
        ).astype(np.uint16)
        planes = []
        for shift in (12, 8, 4, 0):
            nibble = ((code >> shift) & 0x0F).astype(np.float32)
            luma = np.rint(
                DEPTH_SYMBOL_LUMA_MIN
                + nibble
                * (DEPTH_SYMBOL_LUMA_MAX - DEPTH_SYMBOL_LUMA_MIN)
                / 15.0
            ).astype(np.uint8)
            luma[~valid] = 0
            planes.append(luma)
        encoded.append(np.concatenate(planes, axis=1))
    return np.ascontiguousarray(np.stack(encoded), dtype=np.uint8)
