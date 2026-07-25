"""Binary frame contract for pose-adaptive HORUS remote rendering.

H.264 transports color only. This side-channel payload carries the exact
render metadata and a lossless, linearly quantized 16-bit depth image.
"""

from __future__ import annotations

from dataclasses import dataclass
import gzip
import struct
import time
import zlib

import numpy as np


REMOTE_FRAME_FORMAT_VERSION = "remote_stereo_rgbd_webrtc_v2"
REMOTE_FRAME_DATA_TOPIC = "/horus/remote_render/frame_data"
ROS_DEBUG_FRAME_FORMAT_VERSION = "remote_stereo_rgbd_ros_v2"
ROS_DEBUG_FRAME_TOPIC = "/horus/remote_render/rgbd"
FRAME_MARKER_MAGIC = 0xA5
FRAME_MARKER_BLOCK_SIZE = 8
FRAME_MARKER_BITS = 24
FRAME_MARKER_WIDTH = FRAME_MARKER_BITS * 2 * FRAME_MARKER_BLOCK_SIZE
FRAME_MARKER_HEIGHT = FRAME_MARKER_BLOCK_SIZE

_MAGIC = b"HRRF"
_VERSION = 1
_STEREO_VERSION = 2
_STEREO_VIEW_COUNT = 2
_DEPTH_ENCODING_UINT16_LINEAR = 1
_DEPTH_ENCODING_UINT16_LINEAR_GZIP = 2
# Header flag bit 0: the colour and depth atlases hold ONE view that both eyes
# sample, instead of two side-by-side copies. Per-eye metadata is unaffected.
_FLAG_MONO_ATLAS = 1
_MAXIMUM_DEPTH_PAYLOAD_BYTES = 4 * 1024 * 1024
_HEADER = struct.Struct("<4sBBHIQHHHHffffff3f4fB3xII")
_STEREO_HEADER = struct.Struct("<4sBBHIQB3xHHHHff22fB3xII")
_ROS_DEBUG_HEADER = struct.Struct("<4sB3xII")


@dataclass(frozen=True)
class RemoteFrame:
    sequence: int
    capture_time_ns: int
    color_width: int
    color_height: int
    depth_width: int
    depth_height: int
    near_m: float
    far_m: float
    projection_x: float
    projection_y: float
    projection_offset_x: float
    projection_offset_y: float
    position: tuple[float, float, float]
    rotation: tuple[float, float, float, float]
    depth_codes: np.ndarray


@dataclass(frozen=True)
class StereoRemoteFrame:
    sequence: int
    capture_time_ns: int
    color_width: int
    color_height: int
    depth_width: int
    depth_height: int
    near_m: float
    far_m: float
    projections: tuple[tuple[float, float, float, float], ...]
    positions: tuple[tuple[float, float, float], ...]
    rotations: tuple[tuple[float, float, float, float], ...]
    depth_codes: np.ndarray


def quantize_depth(
    depth: np.ndarray,
    *,
    near_m: float,
    far_m: float,
) -> np.ndarray:
    """Quantize metric depth to uint16, reserving zero for invalid pixels."""
    if depth.ndim != 2:
        raise ValueError("depth must have shape (height, width)")
    if not np.isfinite((near_m, far_m)).all() or near_m <= 0.0 or far_m <= near_m:
        raise ValueError("depth range must be finite and satisfy 0 < near_m < far_m")

    valid = np.isfinite(depth) & (depth >= near_m) & (depth <= far_m)
    normalized = np.nan_to_num(
        (depth.astype(np.float32, copy=False) - near_m) / (far_m - near_m),
        nan=0.0,
        posinf=0.0,
        neginf=0.0,
    )
    codes = np.zeros(depth.shape, dtype="<u2")
    codes[valid] = (
        np.rint(np.clip(normalized[valid], 0.0, 1.0) * 65534.0).astype(np.uint16)
        + np.uint16(1)
    )
    return codes


def dequantize_depth(
    codes: np.ndarray,
    *,
    near_m: float,
    far_m: float,
) -> np.ndarray:
    """Restore metric depth; invalid code zero becomes positive infinity."""
    codes = np.asarray(codes, dtype=np.uint16)
    depth = np.full(codes.shape, np.inf, dtype=np.float32)
    valid = codes > 0
    depth[valid] = near_m + (far_m - near_m) * (
        (codes[valid].astype(np.float32) - 1.0) / 65534.0
    )
    return depth


def downsample_depth_min(depth: np.ndarray, width: int, height: int) -> np.ndarray:
    """Downsample depth with finite minimum pooling to preserve foreground edges."""
    source_height, source_width = depth.shape
    if width <= 0 or height <= 0 or width > source_width or height > source_height:
        raise ValueError("depth output dimensions must be positive and no larger than the source")
    if source_width % width or source_height % height:
        raise ValueError("depth output dimensions must divide the source dimensions exactly")

    row_factor = source_height // height
    column_factor = source_width // width
    blocks = depth.reshape(height, row_factor, width, column_factor)
    finite = np.where(np.isfinite(blocks), blocks, np.inf)
    return np.min(finite, axis=(1, 3)).astype(np.float32, copy=False)


def downsample_depth_for_reprojection(
    depth: np.ndarray,
    width: int,
    height: int,
    *,
    edge_absolute_m: float = 0.08,
    edge_relative: float = 0.035,
) -> np.ndarray:
    """Prepare a stable low-resolution depth proxy for client reprojection.

    Sampling the centre of each output footprint avoids foreground bleeding
    from minimum pooling. The farther endpoint of every large depth jump is
    then invalidated so the client's connected grid cannot bridge unrelated
    surfaces into long triangles.
    """
    depth = np.asarray(depth, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError("depth must have shape (height, width)")
    source_height, source_width = depth.shape
    if width <= 0 or height <= 0 or width > source_width or height > source_height:
        raise ValueError("depth output dimensions must be positive and no larger than the source")
    if (
        not np.isfinite((edge_absolute_m, edge_relative)).all()
        or edge_absolute_m < 0.0
        or edge_relative < 0.0
    ):
        raise ValueError("depth edge thresholds must be finite and non-negative")

    source_rows = np.minimum(
        ((np.arange(height, dtype=np.float64) + 0.5) * source_height / height).astype(
            np.int64
        ),
        source_height - 1,
    )
    source_columns = np.minimum(
        ((np.arange(width, dtype=np.float64) + 0.5) * source_width / width).astype(
            np.int64
        ),
        source_width - 1,
    )
    reduced = np.ascontiguousarray(
        depth[np.ix_(source_rows, source_columns)],
        dtype=np.float32,
    )
    invalid = ~np.isfinite(reduced)

    def mark_farther(
        first: np.ndarray,
        second: np.ndarray,
        first_invalid: np.ndarray,
        second_invalid: np.ndarray,
    ) -> None:
        valid_pair = np.isfinite(first) & np.isfinite(second)
        nearer = np.minimum(first, second)
        tolerance = np.maximum(edge_absolute_m, nearer * edge_relative)
        difference = np.zeros_like(first)
        np.subtract(first, second, out=difference, where=valid_pair)
        discontinuity = valid_pair & (np.abs(difference) > tolerance)
        first_invalid |= discontinuity & (first > second)
        second_invalid |= discontinuity & (second > first)

    mark_farther(reduced[:, :-1], reduced[:, 1:], invalid[:, :-1], invalid[:, 1:])
    mark_farther(reduced[:-1, :], reduced[1:, :], invalid[:-1, :], invalid[1:, :])
    mark_farther(
        reduced[:-1, :-1],
        reduced[1:, 1:],
        invalid[:-1, :-1],
        invalid[1:, 1:],
    )
    mark_farther(
        reduced[1:, :-1],
        reduced[:-1, 1:],
        invalid[1:, :-1],
        invalid[:-1, 1:],
    )
    reduced[invalid] = np.inf
    return reduced


def downsample_depth_for_surfels(
    depth: np.ndarray,
    width: int,
    height: int,
) -> np.ndarray:
    """Sample one source depth for every independent client-side surfel.

    Center sampling avoids the foreground expansion caused by minimum pooling.
    Unlike a connected depth grid, surfels do not need destructive edge
    invalidation because neighboring samples never share a triangle.
    """
    depth = np.asarray(depth, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError("depth must have shape (height, width)")
    source_height, source_width = depth.shape
    if width <= 0 or height <= 0 or width > source_width or height > source_height:
        raise ValueError("depth output dimensions must be positive and no larger than the source")

    source_rows = np.minimum(
        ((np.arange(height, dtype=np.float64) + 0.5) * source_height / height).astype(
            np.int64
        ),
        source_height - 1,
    )
    source_columns = np.minimum(
        ((np.arange(width, dtype=np.float64) + 0.5) * source_width / width).astype(
            np.int64
        ),
        source_width - 1,
    )
    return np.ascontiguousarray(
        depth[np.ix_(source_rows, source_columns)],
        dtype=np.float32,
    )


def projection_from_vertical_fov(
    vertical_fov_deg: float,
    aspect: float,
) -> tuple[float, float, float, float]:
    if not 1.0 < vertical_fov_deg < 179.0 or aspect <= 0.0:
        raise ValueError("invalid projection parameters")
    tangent = np.tan(np.deg2rad(vertical_fov_deg) * 0.5)
    return 1.0 / (tangent * aspect), 1.0 / tangent, 0.0, 0.0


# A projection here is (scale_x, scale_y, offset_x, offset_y) applied as
#   ndc = scale * (camera_xy / camera_z) + offset
# with camera_z positive forward. This is the convention the CUDA renderer and
# the Unity reprojection shader both use.
#
# Unity hands us its own projection matrix terms (m00, m11, m02, m12) with a
# right-handed view space looking down -Z, where ndc_x = m00 * x / z_forward - m02.
# The offsets therefore change sign on the way in.
def projection_from_unity_matrix_terms(
    terms,
) -> tuple[float, float, float, float]:
    """Convert Unity (m00, m11, m02, m12) into renderer projection terms."""
    values = tuple(float(value) for value in terms)
    if len(values) != 4 or not np.isfinite(values).all():
        raise ValueError("projection must contain four finite values")
    scale_x, scale_y, offset_x, offset_y = values
    if scale_x <= 0.0 or scale_y <= 0.0:
        raise ValueError("projection scales must be positive")
    return scale_x, scale_y, -offset_x, -offset_y


def projection_tangent_bounds(
    projection,
) -> tuple[float, float, float, float]:
    """Return (left, right, bottom, top) tangents of a projection's frustum."""
    scale_x, scale_y, offset_x, offset_y = (float(value) for value in projection)
    if scale_x <= 0.0 or scale_y <= 0.0:
        raise ValueError("projection scales must be positive")
    return (
        (-1.0 - offset_x) / scale_x,
        (1.0 - offset_x) / scale_x,
        (-1.0 - offset_y) / scale_y,
        (1.0 - offset_y) / scale_y,
    )


def projection_from_tangent_bounds(
    left: float,
    right: float,
    bottom: float,
    top: float,
) -> tuple[float, float, float, float]:
    """Rebuild projection terms from frustum tangents."""
    if not (right > left and top > bottom):
        raise ValueError("frustum tangent bounds must be ordered")
    scale_x = 2.0 / (right - left)
    scale_y = 2.0 / (top - bottom)
    return scale_x, scale_y, 1.0 - scale_x * right, 1.0 - scale_y * top


def projection_aspect(projection) -> float:
    """Width-to-height ratio of a projection's frustum."""
    left, right, bottom, top = projection_tangent_bounds(projection)
    return (right - left) / (top - bottom)


def expand_projection(
    projection,
    *,
    guard_band_degrees: float,
    minimum_vertical_fov_deg: float,
) -> tuple[float, float, float, float]:
    """Expand an asymmetric eye frustum without moving its optical center."""
    projection_x, projection_y, offset_x, offset_y = (
        float(value) for value in projection
    )
    if (
        not np.isfinite(
            (
                projection_x,
                projection_y,
                offset_x,
                offset_y,
                guard_band_degrees,
                minimum_vertical_fov_deg,
            )
        ).all()
        or abs(projection_x) < 1e-5
        or abs(projection_y) < 1e-5
        or not 0.0 <= guard_band_degrees <= 25.0
        or not 1.0 < minimum_vertical_fov_deg < 179.0
    ):
        raise ValueError("invalid projection expansion parameters")

    left = (offset_x - 1.0) / projection_x
    right = (offset_x + 1.0) / projection_x
    bottom = (offset_y - 1.0) / projection_y
    top = (offset_y + 1.0) / projection_y

    guard = np.deg2rad(guard_band_degrees)
    vertical_fov = np.arctan(top) - np.arctan(bottom)
    minimum_vertical_fov = np.deg2rad(minimum_vertical_fov_deg)
    extra_vertical = max(0.0, minimum_vertical_fov - vertical_fov) * 0.5
    left = np.tan(max(-1.535, np.arctan(left) - guard))
    right = np.tan(min(1.535, np.arctan(right) + guard))
    bottom = np.tan(max(-1.535, np.arctan(bottom) - guard - extra_vertical))
    top = np.tan(min(1.535, np.arctan(top) + guard + extra_vertical))

    horizontal_span = right - left
    vertical_span = top - bottom
    if horizontal_span <= 1e-5 or vertical_span <= 1e-5:
        raise ValueError("expanded projection is degenerate")
    return (
        float(2.0 / horizontal_span),
        float(2.0 / vertical_span),
        float((right + left) / horizontal_span),
        float((top + bottom) / vertical_span),
    )


def pack_remote_frame(
    depth: np.ndarray,
    *,
    sequence: int,
    color_width: int,
    color_height: int,
    near_m: float,
    far_m: float,
    projection: tuple[float, float, float, float],
    position,
    rotation,
    capture_time_ns: int | None = None,
) -> bytes:
    depth_codes = quantize_depth(depth, near_m=near_m, far_m=far_m)
    depth_height, depth_width = depth_codes.shape
    if not 1 <= color_width <= 65535 or not 1 <= color_height <= 65535:
        raise ValueError("color dimensions exceed the frame contract")
    if depth_width > 65535 or depth_height > 65535:
        raise ValueError("depth dimensions exceed the frame contract")

    position = tuple(float(value) for value in position)
    rotation = tuple(float(value) for value in rotation)
    projection = tuple(float(value) for value in projection)
    if len(position) != 3 or len(rotation) != 4 or len(projection) != 4:
        raise ValueError("position, rotation, and projection have invalid dimensions")
    numeric = np.asarray((*position, *rotation, *projection), dtype=np.float64)
    if not np.isfinite(numeric).all():
        raise ValueError("frame metadata must be finite")

    raw_payload = depth_codes.tobytes(order="C")
    payload = gzip.compress(raw_payload, compresslevel=1, mtime=0)
    checksum = zlib.crc32(payload) & 0xFFFFFFFF
    timestamp = time.time_ns() if capture_time_ns is None else int(capture_time_ns)
    header = _HEADER.pack(
        _MAGIC,
        _VERSION,
        _DEPTH_ENCODING_UINT16_LINEAR,
        _HEADER.size,
        int(sequence) & 0xFFFFFFFF,
        timestamp & 0xFFFFFFFFFFFFFFFF,
        depth_width,
        depth_height,
        int(color_width),
        int(color_height),
        float(near_m),
        float(far_m),
        *projection,
        *position,
        *rotation,
        _DEPTH_ENCODING_UINT16_LINEAR_GZIP,
        len(payload),
        checksum,
    )
    return header + payload


def pack_stereo_remote_frame(
    depth: np.ndarray,
    *,
    sequence: int,
    color_width: int,
    color_height: int,
    near_m: float,
    far_m: float,
    projections,
    positions,
    rotations,
    capture_time_ns: int | None = None,
    mono_atlas: bool = False,
) -> bytes:
    """Pack two eye views with exact workspace camera poses and projections.

    ``depth`` has shape ``(2, height, width)``. Rows are flipped before
    serialization so Unity's bottom-left raw-texture convention matches the
    decoded color texture without a second full-frame copy on Quest.

    ``positions`` and ``rotations`` are the Quest workspace-local camera poses
    that requested the render, not the renderer's private scene-space camera
    poses. Depth is camera-local, so this lets Quest reconstruct directly into
    its stable workspace without recovering the server's scene alignment.

    With ``mono_atlas`` the two views must be identical and only one is written;
    the client is told through the header flag to sample that single view for
    both eyes. Per-eye poses and projections are still carried, so the metadata
    stays the same shape whether or not the atlas is shared.
    """
    depth = np.asarray(depth, dtype=np.float32)
    if depth.ndim != 3 or depth.shape[0] != _STEREO_VIEW_COUNT:
        raise ValueError("stereo depth must have shape (2, height, width)")
    depth_height, depth_width = depth.shape[1:]
    if not 1 <= depth_width <= 65535 or not 1 <= depth_height <= 65535:
        raise ValueError("stereo depth dimensions exceed the frame contract")
    if not 1 <= color_width <= 65535 or not 1 <= color_height <= 65535:
        raise ValueError("stereo color dimensions exceed the frame contract")

    projections = tuple(tuple(float(value) for value in item) for item in projections)
    positions = tuple(tuple(float(value) for value in item) for item in positions)
    rotations = tuple(tuple(float(value) for value in item) for item in rotations)
    if (
        len(projections) != _STEREO_VIEW_COUNT
        or any(len(item) != 4 for item in projections)
        or len(positions) != _STEREO_VIEW_COUNT
        or any(len(item) != 3 for item in positions)
        or len(rotations) != _STEREO_VIEW_COUNT
        or any(len(item) != 4 for item in rotations)
    ):
        raise ValueError("stereo frame metadata must contain exactly two eye views")
    metadata = tuple(
        value
        for view_index in range(_STEREO_VIEW_COUNT)
        for values in (
            projections[view_index],
            positions[view_index],
            rotations[view_index],
        )
        for value in values
    )
    if not np.isfinite(np.asarray(metadata, dtype=np.float64)).all():
        raise ValueError("stereo frame metadata must be finite")

    # NumPy/OpenCV images are top-left origin. Unity raw textures are
    # bottom-left origin, unlike LoadImage/WebRTC decoded color textures.
    depth_codes = np.stack(
        [
            quantize_depth(
                depth[eye_index, ::-1, :],
                near_m=near_m,
                far_m=far_m,
            )
            for eye_index in range(_STEREO_VIEW_COUNT)
        ],
        axis=0,
    )
    if mono_atlas:
        # Both eyes reproject the same server view, so the second copy in the
        # atlas is byte-identical to the first. H.264 has no intra-frame block
        # copy, so shipping it costs a full second image for nothing - half the
        # video bitrate and half the depth payload. Send one view and let the
        # client sample it for both eyes.
        if not np.array_equal(depth_codes[0], depth_codes[1]):
            raise ValueError("mono atlas requires both depth views to be identical")
        packed_depth_codes = depth_codes[0]
    else:
        packed_depth_codes = np.concatenate(
            (depth_codes[0], depth_codes[1]),
            axis=1,
        )
    raw_payload = packed_depth_codes.tobytes(order="C")
    payload = gzip.compress(raw_payload, compresslevel=1, mtime=0)
    if len(payload) > _MAXIMUM_DEPTH_PAYLOAD_BYTES:
        raise ValueError("stereo depth payload exceeds the frame contract")
    checksum = zlib.crc32(payload) & 0xFFFFFFFF
    timestamp = time.time_ns() if capture_time_ns is None else int(capture_time_ns)
    header = _STEREO_HEADER.pack(
        _MAGIC,
        _STEREO_VERSION,
        _FLAG_MONO_ATLAS if mono_atlas else 0,
        _STEREO_HEADER.size,
        int(sequence) & 0xFFFFFFFF,
        timestamp & 0xFFFFFFFFFFFFFFFF,
        _STEREO_VIEW_COUNT,
        depth_width,
        depth_height,
        int(color_width),
        int(color_height),
        float(near_m),
        float(far_m),
        *metadata,
        _DEPTH_ENCODING_UINT16_LINEAR_GZIP,
        len(payload),
        checksum,
    )
    return header + payload


def unpack_stereo_remote_frame(
    data: bytes | bytearray | memoryview,
) -> StereoRemoteFrame:
    view = memoryview(data)
    if len(view) < _STEREO_HEADER.size:
        raise ValueError("stereo remote frame is shorter than its header")
    values = _STEREO_HEADER.unpack_from(view)
    (
        magic,
        version,
        _flags,
        header_size,
        sequence,
        capture_time_ns,
        view_count,
        depth_width,
        depth_height,
        color_width,
        color_height,
        near_m,
        far_m,
        *tail,
    ) = values
    metadata = tail[:22]
    depth_encoding, payload_size, checksum = tail[22:]
    if (
        magic != _MAGIC
        or version != _STEREO_VERSION
        or header_size != _STEREO_HEADER.size
        or view_count != _STEREO_VIEW_COUNT
    ):
        raise ValueError("unsupported stereo remote frame header")
    if depth_encoding != _DEPTH_ENCODING_UINT16_LINEAR_GZIP:
        raise ValueError("unsupported stereo remote depth encoding")
    mono_atlas = bool(int(_flags) & _FLAG_MONO_ATLAS)
    atlas_views = 1 if mono_atlas else int(view_count)
    expected_size = atlas_views * int(depth_width) * int(depth_height) * 2
    if (
        expected_size <= 0
        or expected_size > _MAXIMUM_DEPTH_PAYLOAD_BYTES
        or payload_size <= 0
        or payload_size > _MAXIMUM_DEPTH_PAYLOAD_BYTES
        or len(view) != header_size + payload_size
    ):
        raise ValueError("stereo remote depth payload has invalid dimensions")
    payload = view[header_size:]
    if zlib.crc32(payload) & 0xFFFFFFFF != checksum:
        raise ValueError("stereo remote depth payload checksum mismatch")
    try:
        raw_payload = gzip.decompress(payload)
    except (EOFError, OSError) as exception:
        raise ValueError("stereo remote depth payload is not valid gzip data") from exception
    if len(raw_payload) != expected_size:
        raise ValueError("stereo remote depth payload size does not match its dimensions")

    projections = []
    positions = []
    rotations = []
    cursor = 0
    for _ in range(_STEREO_VIEW_COUNT):
        projections.append(tuple(metadata[cursor:cursor + 4]))
        cursor += 4
        positions.append(tuple(metadata[cursor:cursor + 3]))
        cursor += 3
        rotations.append(tuple(metadata[cursor:cursor + 4]))
        cursor += 4
    packed_depth_codes = np.frombuffer(raw_payload, dtype="<u2").reshape(
        depth_height,
        depth_width * atlas_views,
    )
    if mono_atlas:
        # One stored view, presented to callers as both eyes.
        depth_codes = np.stack(
            (packed_depth_codes, packed_depth_codes),
            axis=0,
        )[:, ::-1, :]
    else:
        depth_codes = np.stack(
            (
                packed_depth_codes[:, :depth_width],
                packed_depth_codes[:, depth_width:],
            ),
            axis=0,
        )[:, ::-1, :]
    return StereoRemoteFrame(
        sequence=sequence,
        capture_time_ns=capture_time_ns,
        color_width=color_width,
        color_height=color_height,
        depth_width=depth_width,
        depth_height=depth_height,
        near_m=near_m,
        far_m=far_m,
        projections=tuple(projections),
        positions=tuple(positions),
        rotations=tuple(rotations),
        depth_codes=depth_codes,
    )


def unpack_remote_frame(data: bytes | bytearray | memoryview) -> RemoteFrame:
    view = memoryview(data)
    if len(view) < _HEADER.size:
        raise ValueError("remote frame is shorter than its header")
    values = _HEADER.unpack_from(view)
    (
        magic,
        version,
        _flags,
        header_size,
        sequence,
        capture_time_ns,
        depth_width,
        depth_height,
        color_width,
        color_height,
        near_m,
        far_m,
        projection_x,
        projection_y,
        projection_offset_x,
        projection_offset_y,
        position_x,
        position_y,
        position_z,
        rotation_x,
        rotation_y,
        rotation_z,
        rotation_w,
        depth_encoding,
        payload_size,
        checksum,
    ) = values
    if magic != _MAGIC or version != _VERSION or header_size != _HEADER.size:
        raise ValueError("unsupported remote frame header")
    if depth_encoding not in (
        _DEPTH_ENCODING_UINT16_LINEAR,
        _DEPTH_ENCODING_UINT16_LINEAR_GZIP,
    ):
        raise ValueError("unsupported remote depth encoding")
    expected_size = int(depth_width) * int(depth_height) * 2
    if (
        expected_size <= 0
        or expected_size > _MAXIMUM_DEPTH_PAYLOAD_BYTES
        or payload_size <= 0
        or payload_size > _MAXIMUM_DEPTH_PAYLOAD_BYTES
        or len(view) != header_size + payload_size
    ):
        raise ValueError("remote depth payload size does not match its dimensions")
    payload = view[header_size:]
    if zlib.crc32(payload) & 0xFFFFFFFF != checksum:
        raise ValueError("remote depth payload checksum mismatch")
    if depth_encoding == _DEPTH_ENCODING_UINT16_LINEAR_GZIP:
        try:
            raw_payload = gzip.decompress(payload)
        except (EOFError, OSError) as exception:
            raise ValueError("remote depth payload is not valid gzip data") from exception
    else:
        raw_payload = payload
    if len(raw_payload) != expected_size:
        raise ValueError("remote depth payload size does not match its dimensions")

    depth_codes = np.frombuffer(raw_payload, dtype="<u2").reshape(
        depth_height,
        depth_width,
    )
    return RemoteFrame(
        sequence=sequence,
        capture_time_ns=capture_time_ns,
        color_width=color_width,
        color_height=color_height,
        depth_width=depth_width,
        depth_height=depth_height,
        near_m=near_m,
        far_m=far_m,
        projection_x=projection_x,
        projection_y=projection_y,
        projection_offset_x=projection_offset_x,
        projection_offset_y=projection_offset_y,
        position=(position_x, position_y, position_z),
        rotation=(rotation_x, rotation_y, rotation_z, rotation_w),
        depth_codes=depth_codes,
    )


def embed_frame_marker(color: np.ndarray, sequence: int) -> None:
    """Embed a codec-resistant 16-bit sequence marker at both vertical edges."""
    if color.ndim != 3 or color.shape[2] != 3 or color.dtype != np.uint8:
        raise ValueError("color must be an RGB uint8 image")
    if color.shape[0] < FRAME_MARKER_HEIGHT or color.shape[1] < FRAME_MARKER_WIDTH:
        raise ValueError(
            f"color image must be at least {FRAME_MARKER_WIDTH}x{FRAME_MARKER_HEIGHT}"
        )

    word = (FRAME_MARKER_MAGIC << 16) | (int(sequence) & 0xFFFF)
    for bit_index in range(FRAME_MARKER_BITS):
        bit = (word >> (FRAME_MARKER_BITS - 1 - bit_index)) & 1
        for complement in range(2):
            encoded_bit = bit ^ complement
            x0 = (bit_index * 2 + complement) * FRAME_MARKER_BLOCK_SIZE
            level = 224 if encoded_bit else 32
            color[
                :FRAME_MARKER_HEIGHT,
                x0:x0 + FRAME_MARKER_BLOCK_SIZE,
                :,
            ] = level
            color[
                -FRAME_MARKER_HEIGHT:,
                x0:x0 + FRAME_MARKER_BLOCK_SIZE,
                :,
            ] = level


def decode_frame_marker(color: np.ndarray) -> int:
    """Decode the marker from an RGB image; primarily used by tests/tools."""
    if color.shape[0] < FRAME_MARKER_HEIGHT or color.shape[1] < FRAME_MARKER_WIDTH:
        raise ValueError("color image is too small for a frame marker")
    word = 0
    for bit_index in range(FRAME_MARKER_BITS):
        x0 = (bit_index * 2) * FRAME_MARKER_BLOCK_SIZE
        first = float(
            np.mean(
                color[
                    :FRAME_MARKER_HEIGHT,
                    x0:x0 + FRAME_MARKER_BLOCK_SIZE,
                    :,
                ]
            )
        )
        second_x0 = x0 + FRAME_MARKER_BLOCK_SIZE
        second = float(
            np.mean(
                color[
                    :FRAME_MARKER_HEIGHT,
                    second_x0:second_x0 + FRAME_MARKER_BLOCK_SIZE,
                    :,
                ]
            )
        )
        word = (word << 1) | int(first > second)
    if word >> 16 != FRAME_MARKER_MAGIC:
        raise ValueError("remote frame marker magic does not match")
    return word & 0xFFFF


def pack_ros_debug_frame(jpeg: bytes, frame_data: bytes) -> bytes:
    """Bundle matching JPEG color and HRRF depth metadata for ROS diagnostics."""
    jpeg = bytes(jpeg)
    frame_data = bytes(frame_data)
    if not jpeg or not frame_data:
        raise ValueError("ROS diagnostic color and frame data must be non-empty")
    if len(jpeg) > 4 * 1024 * 1024 or len(frame_data) > 4 * 1024 * 1024:
        raise ValueError("ROS diagnostic frame exceeds the 4 MiB component limit")
    return (
        _ROS_DEBUG_HEADER.pack(
            b"HRRD",
            1,
            len(frame_data),
            len(jpeg),
        )
        + frame_data
        + jpeg
    )


def unpack_ros_debug_frame(payload: bytes) -> tuple[bytes, bytes]:
    """Return ``(jpeg, frame_data)`` from a ROS diagnostic payload."""
    payload = bytes(payload)
    if len(payload) < _ROS_DEBUG_HEADER.size:
        raise ValueError("ROS diagnostic frame is shorter than its header")
    magic, version, frame_data_size, jpeg_size = _ROS_DEBUG_HEADER.unpack_from(payload)
    expected = _ROS_DEBUG_HEADER.size + frame_data_size + jpeg_size
    if (
        magic != b"HRRD"
        or version != 1
        or frame_data_size <= 0
        or jpeg_size <= 0
        or frame_data_size > 4 * 1024 * 1024
        or jpeg_size > 4 * 1024 * 1024
        or len(payload) != expected
    ):
        raise ValueError("invalid ROS diagnostic frame")
    frame_start = _ROS_DEBUG_HEADER.size
    jpeg_start = frame_start + frame_data_size
    return payload[jpeg_start:], payload[frame_start:jpeg_start]
