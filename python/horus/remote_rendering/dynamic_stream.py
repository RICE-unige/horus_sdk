"""Pose-tagged RGB-D contract for depth-aware remote-map reprojection."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .synthetic_scene import DEPTH_LUMA_MAX, DEPTH_LUMA_MIN, encode_luma_depth


DYNAMIC_RGBD_FORMAT_VERSION = "rgbd_pose_timewarp_luma_nibbles_v5"
LEGACY_DYNAMIC_RGBD_FORMAT_VERSION = "rgbd_pose_timewarp_luma_nibbles_v4"
VIEWER_POSE_VERSION = "remote_render_stereo_pose.v2"
DEFAULT_VIEWER_POSE_TOPIC = "/horus/remote_render/viewer_pose"
DYNAMIC_VIEW_COUNT = 1
DYNAMIC_DEPTH_BITS = 16
DYNAMIC_DEPTH_PIXELS_PER_SAMPLE = 4
DYNAMIC_DEPTH_VALID_LUMA_MIN = int(DEPTH_LUMA_MIN * 0.75)
METADATA_MAGIC = 0xA35C
METADATA_BLOCK_PIXELS = 8
METADATA_HEADER_VALUES = 2
METADATA_VALUES_PER_VIEW = 7
METADATA_VALUE_COUNT = METADATA_HEADER_VALUES + DYNAMIC_VIEW_COUNT * METADATA_VALUES_PER_VIEW
METADATA_NIBBLE_COUNT = METADATA_VALUE_COUNT * 4
_DYNAMIC_DEPTH_NIBBLE_TO_LUMA = np.rint(
    np.linspace(DEPTH_LUMA_MIN, DEPTH_LUMA_MAX, 16, dtype=np.float32)
).astype(np.uint8)


@dataclass(frozen=True)
class DynamicCameraPose:
    position: tuple[float, float, float]
    rotation: tuple[float, float, float, float]


def normalize_quaternion(rotation) -> np.ndarray:
    value = np.asarray(rotation, dtype=np.float32)
    if value.shape != (4,):
        raise ValueError("rotation must contain x, y, z and w")
    norm = float(np.linalg.norm(value))
    if norm < 1e-8:
        return np.array((0.0, 0.0, 0.0, 1.0), dtype=np.float32)
    return value / norm


def quaternion_to_matrix(rotation) -> np.ndarray:
    x, y, z, w = normalize_quaternion(rotation)
    return np.array(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float32,
    )


def quaternion_multiply(left, right) -> np.ndarray:
    """Compose two x/y/z/w quaternions."""
    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def quaternion_inverse(rotation) -> np.ndarray:
    value = normalize_quaternion(rotation)
    return np.array((-value[0], -value[1], -value[2], value[3]), dtype=np.float32)


def build_dynamic_camera_poses(
    position,
    rotation,
    *,
    baseline_m: float,
) -> tuple[DynamicCameraPose, ...]:
    """Build the single server camera used for pose-driven remote rendering.

    ``baseline_m`` remains in the signature so older launchers fail softly, but
    the v5 contract deliberately has no offset views. Rendering multiple opaque
    reconstructions of the same surface caused depth fighting and multiplied
    the Quest fill cost.
    """
    center = np.asarray(position, dtype=np.float32)
    if center.shape != (3,):
        raise ValueError("position must contain x, y and z")
    quaternion = normalize_quaternion(rotation)
    _ = baseline_m
    return (
        DynamicCameraPose(
            tuple(float(value) for value in center),
            tuple(float(value) for value in quaternion),
        ),
    )


def metadata_rows(width: int) -> int:
    blocks_per_row = max(1, int(width) // METADATA_BLOCK_PIXELS)
    block_rows = math.ceil(METADATA_NIBBLE_COUNT / blocks_per_row)
    return block_rows * METADATA_BLOCK_PIXELS


def _quantize_unit(value: float) -> int:
    return int(round(float(np.clip(value, 0.0, 1.0)) * 65535.0))


def _encode_pose_values(
    poses: tuple[DynamicCameraPose, ...],
    sequence: int,
    position_range_m: float,
) -> list[int]:
    if len(poses) != DYNAMIC_VIEW_COUNT:
        raise ValueError(f"expected {DYNAMIC_VIEW_COUNT} dynamic camera poses")
    position_range = max(1.0, float(position_range_m))
    values = [METADATA_MAGIC, int(sequence) & 0xFFFF]
    for pose in poses:
        position = np.asarray(pose.position, dtype=np.float32)
        rotation = normalize_quaternion(pose.rotation)
        if position.shape != (3,):
            raise ValueError("dynamic pose position must contain three values")
        values.extend(
            _quantize_unit(float(component) / (2.0 * position_range) + 0.5)
            for component in position
        )
        values.extend(_quantize_unit(float(component) * 0.5 + 0.5) for component in rotation)
    return values


def encode_dynamic_pose_metadata(
    encoded_depth: np.ndarray,
    poses: tuple[DynamicCameraPose, ...],
    *,
    sequence: int,
    position_range_m: float,
) -> int:
    """Write block-coded camera metadata into the top of an encoded depth atlas."""
    if encoded_depth.ndim != 3 or encoded_depth.shape[2] != 3:
        raise ValueError("encoded depth atlas must have shape (H, W, 3)")
    height, width, _ = encoded_depth.shape
    required_rows = metadata_rows(width)
    if required_rows > height:
        raise ValueError("depth atlas is too small for dynamic pose metadata")
    blocks_per_row = max(1, width // METADATA_BLOCK_PIXELS)
    level_span = DEPTH_LUMA_MAX - DEPTH_LUMA_MIN
    values = _encode_pose_values(poses, sequence, position_range_m)
    nibble_index = 0
    for value in values:
        for shift in (12, 8, 4, 0):
            nibble = (value >> shift) & 0x0F
            luma = DEPTH_LUMA_MIN + int(round(nibble * level_span / 15.0))
            block_x = nibble_index % blocks_per_row
            block_y = nibble_index // blocks_per_row
            x0 = block_x * METADATA_BLOCK_PIXELS
            y0 = block_y * METADATA_BLOCK_PIXELS
            encoded_depth[
                y0:y0 + METADATA_BLOCK_PIXELS,
                x0:x0 + METADATA_BLOCK_PIXELS,
            ] = luma
            nibble_index += 1
    return required_rows


def decode_dynamic_pose_metadata(
    encoded_depth: np.ndarray,
    *,
    position_range_m: float,
) -> tuple[int, tuple[DynamicCameraPose, ...]]:
    """Decode the metadata header for tests and transport diagnostics."""
    if encoded_depth.ndim != 3 or encoded_depth.shape[2] != 3:
        raise ValueError("encoded depth atlas must have shape (H, W, 3)")
    height, width, _ = encoded_depth.shape
    if metadata_rows(width) > height:
        raise ValueError("depth atlas is too small for dynamic pose metadata")
    blocks_per_row = max(1, width // METADATA_BLOCK_PIXELS)
    level_span = DEPTH_LUMA_MAX - DEPTH_LUMA_MIN
    nibbles = []
    for nibble_index in range(METADATA_NIBBLE_COUNT):
        block_x = nibble_index % blocks_per_row
        block_y = nibble_index // blocks_per_row
        x = block_x * METADATA_BLOCK_PIXELS + METADATA_BLOCK_PIXELS // 2
        y = block_y * METADATA_BLOCK_PIXELS + METADATA_BLOCK_PIXELS // 2
        luma = float(encoded_depth[y, x].astype(np.float32).mean())
        nibbles.append(int(round(np.clip((luma - DEPTH_LUMA_MIN) * 15.0 / level_span, 0.0, 15.0))))
    values = []
    for index in range(0, len(nibbles), 4):
        values.append(
            (nibbles[index] << 12)
            | (nibbles[index + 1] << 8)
            | (nibbles[index + 2] << 4)
            | nibbles[index + 3]
        )
    if values[0] != METADATA_MAGIC:
        raise ValueError(f"invalid dynamic metadata marker 0x{values[0]:04x}")
    position_range = max(1.0, float(position_range_m))
    poses = []
    cursor = METADATA_HEADER_VALUES
    for _ in range(DYNAMIC_VIEW_COUNT):
        position = tuple(
            ((values[cursor + component] / 65535.0) - 0.5) * 2.0 * position_range
            for component in range(3)
        )
        cursor += 3
        rotation = tuple(
            (values[cursor + component] / 65535.0) * 2.0 - 1.0
            for component in range(4)
        )
        cursor += 4
        poses.append(
            DynamicCameraPose(
                tuple(float(value) for value in position),
                tuple(float(value) for value in normalize_quaternion(rotation)),
            )
        )
    return values[1], tuple(poses)


def encode_dynamic_depth(
    depth: np.ndarray,
    *,
    near_m: float,
    far_m: float,
) -> np.ndarray:
    """Encode 16-bit depth as four contiguous H.264-resistant luma planes."""
    if depth.ndim != 2 or depth.shape[1] % DYNAMIC_DEPTH_PIXELS_PER_SAMPLE:
        raise ValueError("dynamic depth width must be divisible by four")
    if far_m <= near_m:
        raise ValueError("far_m must be greater than near_m")

    height, width = depth.shape
    groups = depth.reshape(height, width // DYNAMIC_DEPTH_PIXELS_PER_SAMPLE, 4)
    finite = np.isfinite(groups) & (groups >= near_m) & (groups <= far_m)
    sampled = np.where(finite, groups, np.inf).min(axis=2)
    valid = np.isfinite(sampled)
    normalized = np.nan_to_num(
        (sampled - near_m) / (far_m - near_m),
        nan=0.0,
        posinf=0.0,
        neginf=0.0,
    )
    code = np.rint(np.clip(normalized, 0.0, 1.0) * 65535.0).astype(np.uint16)

    encoded = np.zeros((height, width, 3), dtype=np.uint8)
    plane_width = width // DYNAMIC_DEPTH_PIXELS_PER_SAMPLE
    for offset, shift in enumerate((12, 8, 4, 0)):
        luma = _DYNAMIC_DEPTH_NIBBLE_TO_LUMA[(code >> shift) & 0x0F]
        luma[~valid] = 0
        x0 = offset * plane_width
        encoded[:, x0:x0 + plane_width, :] = luma[..., None]
    return encoded


def decode_dynamic_depth(
    encoded: np.ndarray,
    *,
    near_m: float,
    far_m: float,
) -> np.ndarray:
    """Decode the v5 four-plane depth payload for tests and diagnostics."""
    if encoded.ndim != 3 or encoded.shape[1] % DYNAMIC_DEPTH_PIXELS_PER_SAMPLE:
        raise ValueError("encoded dynamic depth width must be divisible by four")
    level_span = DEPTH_LUMA_MAX - DEPTH_LUMA_MIN
    plane_width = encoded.shape[1] // DYNAMIC_DEPTH_PIXELS_PER_SAMPLE
    luma_planes = [
        encoded[:, offset * plane_width:(offset + 1) * plane_width]
        .astype(np.float32)
        .mean(axis=-1)
        for offset in range(DYNAMIC_DEPTH_PIXELS_PER_SAMPLE)
    ]
    nibbles = [
        np.rint(np.clip((luma - DEPTH_LUMA_MIN) * 15.0 / level_span, 0.0, 15.0))
        for luma in luma_planes
    ]
    code = nibbles[0] * 4096.0 + nibbles[1] * 256.0 + nibbles[2] * 16.0 + nibbles[3]
    sampled = near_m + (far_m - near_m) * code / 65535.0
    sampled[luma_planes[0] < DYNAMIC_DEPTH_VALID_LUMA_MIN] = np.inf
    return np.repeat(sampled, DYNAMIC_DEPTH_PIXELS_PER_SAMPLE, axis=1).astype(np.float32)


def pack_dynamic_rgbd(
    color_atlas: np.ndarray,
    depth_atlas: np.ndarray,
    poses: tuple[DynamicCameraPose, ...],
    *,
    sequence: int,
    near_m: float,
    far_m: float,
    position_range_m: float,
) -> np.ndarray:
    encoded_depth = encode_dynamic_depth(depth_atlas, near_m=near_m, far_m=far_m)
    encode_dynamic_pose_metadata(
        encoded_depth,
        poses,
        sequence=sequence,
        position_range_m=position_range_m,
    )
    return np.ascontiguousarray(np.concatenate((color_atlas, encoded_depth), axis=1))
