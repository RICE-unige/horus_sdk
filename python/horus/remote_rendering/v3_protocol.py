"""Versioned contracts for the HORUS remote-rendering v3 session.

The v3 protocol carries OpenXR display-time views rather than a predicted
centre-eye camera.  The data model is transport-neutral: the ROS diagnostic
path serializes it as JSON while the production session can use the same
fields in a binary control channel.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
import statistics
import struct
import threading
from typing import Any, Generic, Iterable, TypeVar


PROTOCOL_VERSION = "horus.remote_render.v3"
VIEW_STATE_VERSION = "horus.remote_render.view_state.v3"
FRAME_METADATA_VERSION = "horus.remote_render.frame.v3"
SESSION_CAPABILITIES_VERSION = "horus.remote_render.capabilities.v3"
ROS_DIAGNOSTIC_FORMAT_VERSION = "horus.remote_render.v3.ros"
WEBRTC_FORMAT_VERSION = "horus.remote_render.v3.webrtc"

DEFAULT_VIEW_STATE_TOPIC = "/horus/remote_render/v3/view_state"
DEFAULT_ROS_DIAGNOSTIC_TOPIC = "/horus/remote_render/v3/stereo"
DEFAULT_STATUS_TOPIC = "/horus/remote_render/v3/status"

_ROS_DIAGNOSTIC_MAGIC = b"HRV3"
_ROS_DIAGNOSTIC_VERSION = 1
_ROS_DIAGNOSTIC_HEADER = struct.Struct("<4sBBHII")
_MAXIMUM_METADATA_BYTES = 64 * 1024
_MAXIMUM_IMAGE_BYTES = 32 * 1024 * 1024

_T = TypeVar("_T")


def _finite_float(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _positive_int(value: Any, name: str) -> int:
    result = int(value)
    if result <= 0:
        raise ValueError(f"{name} must be positive")
    return result


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float

    @classmethod
    def from_value(cls, value: Any, name: str = "vector") -> "Vector3":
        if isinstance(value, dict):
            values = (value.get("x"), value.get("y"), value.get("z"))
        else:
            values = tuple(value) if value is not None else ()
        if len(values) != 3:
            raise ValueError(f"{name} must contain x, y and z")
        return cls(*(_finite_float(item, name) for item in values))

    def to_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z}


@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float

    @classmethod
    def from_value(cls, value: Any, name: str = "rotation") -> "Quaternion":
        if isinstance(value, dict):
            values = (
                value.get("x"),
                value.get("y"),
                value.get("z"),
                value.get("w"),
            )
        else:
            values = tuple(value) if value is not None else ()
        if len(values) != 4:
            raise ValueError(f"{name} must contain x, y, z and w")
        components = tuple(_finite_float(item, name) for item in values)
        norm = math.sqrt(sum(item * item for item in components))
        if norm < 1e-8:
            raise ValueError(f"{name} cannot be a zero quaternion")
        return cls(*(item / norm for item in components))

    def to_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z, "w": self.w}


@dataclass(frozen=True)
class EyeFov:
    """OpenXR-style signed tangent bounds for one eye.

    ``left`` and ``down`` are normally negative. ``right`` and ``up`` are
    normally positive. Keeping the four bounds explicit avoids projection
    offset sign ambiguity between Unity, OpenXR and the renderer.
    """

    left: float
    right: float
    up: float
    down: float

    @classmethod
    def from_value(cls, value: Any, name: str = "fov") -> "EyeFov":
        if not isinstance(value, dict):
            raise ValueError(f"{name} must be an object")
        result = cls(
            _finite_float(value.get("left"), f"{name}.left"),
            _finite_float(value.get("right"), f"{name}.right"),
            _finite_float(value.get("up"), f"{name}.up"),
            _finite_float(value.get("down"), f"{name}.down"),
        )
        if result.left >= result.right or result.down >= result.up:
            raise ValueError(f"{name} bounds are not ordered")
        if result.left >= 0.0 or result.right <= 0.0:
            raise ValueError(f"{name} must cross the vertical axis")
        if result.down >= 0.0 or result.up <= 0.0:
            raise ValueError(f"{name} must cross the horizontal axis")
        return result

    def to_dict(self) -> dict[str, float]:
        return {
            "left": self.left,
            "right": self.right,
            "up": self.up,
            "down": self.down,
        }

    def renderer_projection(self) -> tuple[float, float, float, float]:
        scale_x = 2.0 / (self.right - self.left)
        scale_y = 2.0 / (self.up - self.down)
        offset_x = -(self.right + self.left) / (self.right - self.left)
        offset_y = -(self.up + self.down) / (self.up - self.down)
        return scale_x, scale_y, offset_x, offset_y


def eye_fov_from_unity_projection(
    m00: float,
    m11: float,
    m02: float,
    m12: float,
) -> EyeFov:
    """Convert Unity projection terms to explicit positive-forward tangents."""
    scale_x = _finite_float(m00, "m00")
    scale_y = _finite_float(m11, "m11")
    if scale_x <= 0.0 or scale_y <= 0.0:
        raise ValueError("projection scales must be positive")
    # Unity looks down -Z. In the renderer's +Z convention the NDC offsets
    # are the negated m02/m12 terms.
    offset_x = -_finite_float(m02, "m02")
    offset_y = -_finite_float(m12, "m12")
    return EyeFov(
        left=(-1.0 - offset_x) / scale_x,
        right=(1.0 - offset_x) / scale_x,
        up=(1.0 - offset_y) / scale_y,
        down=(-1.0 - offset_y) / scale_y,
    )


@dataclass(frozen=True)
class EyeView:
    position: Vector3
    rotation: Quaternion
    fov: EyeFov

    @classmethod
    def from_value(cls, value: Any, name: str) -> "EyeView":
        if not isinstance(value, dict):
            raise ValueError(f"{name} must be an object")
        return cls(
            Vector3.from_value(value.get("position"), f"{name}.position"),
            Quaternion.from_value(value.get("rotation"), f"{name}.rotation"),
            EyeFov.from_value(value.get("fov"), f"{name}.fov"),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "position": self.position.to_dict(),
            "rotation": self.rotation.to_dict(),
            "fov": self.fov.to_dict(),
        }


@dataclass(frozen=True)
class WorkspaceState:
    revision: int
    position: Vector3
    rotation: Quaternion
    scale: float

    @classmethod
    def from_value(cls, value: Any) -> "WorkspaceState":
        if not isinstance(value, dict):
            raise ValueError("workspace must be an object")
        revision = int(value.get("revision", -1))
        if revision < 0:
            raise ValueError("workspace.revision cannot be negative")
        scale = _finite_float(value.get("scale"), "workspace.scale")
        if scale <= 0.0:
            raise ValueError("workspace.scale must be positive")
        return cls(
            revision=revision,
            position=Vector3.from_value(value.get("position"), "workspace.position"),
            rotation=Quaternion.from_value(value.get("rotation"), "workspace.rotation"),
            scale=scale,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "revision": self.revision,
            "position": self.position.to_dict(),
            "rotation": self.rotation.to_dict(),
            "scale": self.scale,
        }


@dataclass(frozen=True)
class ViewState:
    session_id: str
    view_sequence: int
    sample_time_ns: int
    predicted_display_time_ns: int
    predicted_display_period_ns: int
    reference_space_id: str
    workspace: WorkspaceState
    left_eye: EyeView
    right_eye: EyeView
    viewport_width: int
    viewport_height: int
    refresh_rate_hz: float
    view_stream_id: str = ""

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "ViewState":
        if payload.get("version") != VIEW_STATE_VERSION:
            raise ValueError("unsupported view-state version")
        session_id = str(payload.get("session_id") or "").strip()
        reference_space_id = str(payload.get("reference_space_id") or "").strip()
        if not session_id or not reference_space_id:
            raise ValueError("session_id and reference_space_id are required")
        sequence = int(payload.get("view_sequence", -1))
        sample_time_ns = int(payload.get("sample_time_ns", -1))
        display_time_ns = int(payload.get("predicted_display_time_ns", -1))
        display_period_ns = int(payload.get("predicted_display_period_ns", -1))
        if min(sequence, sample_time_ns, display_time_ns) < 0:
            raise ValueError("view sequence and timestamps cannot be negative")
        if display_period_ns <= 0:
            raise ValueError("predicted display period must be positive")
        refresh_rate = _finite_float(payload.get("refresh_rate_hz"), "refresh_rate_hz")
        if not 30.0 <= refresh_rate <= 240.0:
            raise ValueError("refresh_rate_hz is outside the supported range")
        return cls(
            session_id=session_id,
            view_sequence=sequence,
            sample_time_ns=sample_time_ns,
            predicted_display_time_ns=display_time_ns,
            predicted_display_period_ns=display_period_ns,
            reference_space_id=reference_space_id,
            workspace=WorkspaceState.from_value(payload.get("workspace")),
            left_eye=EyeView.from_value(payload.get("left_eye"), "left_eye"),
            right_eye=EyeView.from_value(payload.get("right_eye"), "right_eye"),
            viewport_width=_positive_int(payload.get("viewport_width"), "viewport_width"),
            viewport_height=_positive_int(payload.get("viewport_height"), "viewport_height"),
            refresh_rate_hz=refresh_rate,
            view_stream_id=str(payload.get("view_stream_id") or "").strip(),
        )

    @classmethod
    def from_json(cls, value: str | bytes) -> "ViewState":
        if isinstance(value, bytes):
            value = value.decode("utf-8")
        payload = json.loads(value)
        if not isinstance(payload, dict):
            raise ValueError("view state must be a JSON object")
        return cls.from_dict(payload)

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": VIEW_STATE_VERSION,
            "session_id": self.session_id,
            "view_stream_id": self.view_stream_id,
            "view_sequence": self.view_sequence,
            "sample_time_ns": self.sample_time_ns,
            "predicted_display_time_ns": self.predicted_display_time_ns,
            "predicted_display_period_ns": self.predicted_display_period_ns,
            "reference_space_id": self.reference_space_id,
            "workspace": self.workspace.to_dict(),
            "left_eye": self.left_eye.to_dict(),
            "right_eye": self.right_eye.to_dict(),
            "viewport_width": self.viewport_width,
            "viewport_height": self.viewport_height,
            "refresh_rate_hz": self.refresh_rate_hz,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"), sort_keys=True)


@dataclass(frozen=True)
class RenderedFrameMetadata:
    session_id: str
    frame_id: int
    source_view_sequence: int
    predicted_display_time_ns: int
    workspace_revision: int
    render_start_ns: int
    render_complete_ns: int
    encode_start_ns: int
    capture_time_ns: int
    stereo_layout: str
    width_per_eye: int
    height_per_eye: int
    codec: str
    color_space: str = "srgb"
    color_range: str = "full"
    depth_representation: str = "none"
    visibility_mask_height: int = 0
    frame_marker_height: int = 0
    frame_marker: int = 0
    video_height: int = 0
    depth_near_m: float = 0.05
    depth_far_m: float = 100.0
    view_stream_id: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {"version": FRAME_METADATA_VERSION, **self.__dict__}

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"), sort_keys=True)

    @classmethod
    def from_json(cls, value: str | bytes) -> "RenderedFrameMetadata":
        if isinstance(value, bytes):
            value = value.decode("utf-8")
        payload = json.loads(value)
        if not isinstance(payload, dict) or payload.pop("version", None) != FRAME_METADATA_VERSION:
            raise ValueError("unsupported rendered-frame metadata")
        result = cls(**payload)
        if not result.session_id or result.frame_id < 0 or result.source_view_sequence < 0:
            raise ValueError("invalid rendered-frame identity")
        if result.stereo_layout not in {
            "side_by_side",
            "side_by_side_with_visibility",
            "side_by_side_with_depth",
            "independent_tracks",
            "multiview",
        }:
            raise ValueError("unsupported stereo layout")
        if result.width_per_eye <= 0 or result.height_per_eye <= 0:
            raise ValueError("invalid rendered-frame dimensions")
        if result.codec not in {"png", "h264"}:
            raise ValueError("unsupported rendered-frame codec")
        expected_color_range = "limited" if result.codec == "h264" else "full"
        if result.color_range != expected_color_range:
            raise ValueError(
                f"{result.codec} frames require {expected_color_range}-range color"
            )
        if (
            result.visibility_mask_height < 0
            or result.frame_marker_height < 0
            or result.video_height < 0
            or not 0 <= result.frame_marker <= 0xFFFF
        ):
            raise ValueError("invalid rendered-frame auxiliary dimensions")
        if result.stereo_layout in {
            "side_by_side_with_visibility",
            "side_by_side_with_depth",
        }:
            if result.visibility_mask_height <= 0:
                raise ValueError("visibility-mask stereo layout requires a mask")
            if result.video_height != (
                result.height_per_eye
                + result.visibility_mask_height
                + result.frame_marker_height
            ):
                raise ValueError("video height does not match the visibility layout")
        if result.stereo_layout == "side_by_side_with_depth":
            if result.depth_representation != "linear_u16_nibbles_in_video":
                raise ValueError("depth layout requires robust 16-bit depth")
            if result.width_per_eye % 4:
                raise ValueError("depth layout width must be divisible by four")
            if result.depth_near_m <= 0.0 or result.depth_far_m <= result.depth_near_m:
                raise ValueError("invalid depth range")
        if result.codec == "h264" and result.frame_marker_height < 8:
            raise ValueError("h264 remote frames require an in-band frame marker")
        return result


class ViewStreamSequenceGate:
    """Order poses within a viewer incarnation, not across reconnects.

    A live stream owns this single-view renderer until it goes idle. Retired
    incarnations cannot reclaim it with delayed packets. The empty-token path
    supports older clients using their monotonic sample timestamp after idle.
    Call under the renderer's pose lock.
    """

    def __init__(self, idle_timeout: float = 2.0) -> None:
        if idle_timeout <= 0:
            raise ValueError("idle_timeout must be positive")
        self.idle_timeout = idle_timeout
        self.active_stream: str | None = None
        self.last_sequence = -1
        self.last_sample_ns = -1
        self.last_received = 0.0
        self.revision = 0
        self._retired: deque[str] = deque(maxlen=128)

    def accept(self, state: ViewState, now: float) -> str | None:
        stream = state.view_stream_id
        idle = now - self.last_received >= self.idle_timeout
        if self.active_stream is not None:
            if stream != self.active_stream:
                if stream in self._retired:
                    return "retired_view_stream"
                if not idle:
                    return "view_stream_busy"
                self._retired.append(self.active_stream)
            else:
                if state.sample_time_ns <= self.last_sample_ns:
                    return "stale_sample"
                if state.view_sequence <= self.last_sequence:
                    # New clients have explicit incarnation IDs. For old
                    # clients only a fresh timestamp after idle may restart.
                    if stream or not idle:
                        return "stale_sequence"
        self.active_stream = stream
        self.last_sequence = state.view_sequence
        self.last_sample_ns = state.sample_time_ns
        self.last_received = now
        self.revision += 1
        return None


@dataclass(frozen=True)
class ClockSample:
    client_send_ns: int
    server_receive_ns: int
    server_send_ns: int
    client_receive_ns: int

    @property
    def round_trip_ns(self) -> int:
        return max(
            0,
            (self.client_receive_ns - self.client_send_ns)
            - (self.server_send_ns - self.server_receive_ns),
        )

    @property
    def server_minus_client_ns(self) -> float:
        return (
            (self.server_receive_ns - self.client_send_ns)
            + (self.server_send_ns - self.client_receive_ns)
        ) / 2.0


class ClockAlignment:
    """Small bounded NTP-style estimator for renderer/client timestamp mapping."""

    def __init__(self, maximum_samples: int = 32) -> None:
        if maximum_samples < 3:
            raise ValueError("maximum_samples must be at least three")
        self._samples: deque[ClockSample] = deque(maxlen=maximum_samples)

    def add(self, sample: ClockSample) -> None:
        if min(
            sample.client_send_ns,
            sample.server_receive_ns,
            sample.server_send_ns,
            sample.client_receive_ns,
        ) < 0:
            raise ValueError("clock timestamps cannot be negative")
        if sample.server_send_ns < sample.server_receive_ns:
            raise ValueError("server send precedes server receive")
        if sample.client_receive_ns < sample.client_send_ns:
            raise ValueError("client receive precedes client send")
        self._samples.append(sample)

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    @property
    def offset_ns(self) -> float:
        if not self._samples:
            raise RuntimeError("clock alignment has no samples")
        ranked = sorted(self._samples, key=lambda item: item.round_trip_ns)
        best = ranked[: max(1, len(ranked) // 3)]
        return statistics.median(item.server_minus_client_ns for item in best)

    @property
    def round_trip_ns(self) -> float:
        if not self._samples:
            raise RuntimeError("clock alignment has no samples")
        return statistics.median(item.round_trip_ns for item in self._samples)


class LatestValueQueue(Generic[_T]):
    """Thread-safe, newest-only handoff with observable replacement counts."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._value: _T | None = None
        self._closed = False
        self.replaced_count = 0

    def put(self, value: _T) -> None:
        with self._condition:
            if self._closed:
                raise RuntimeError("queue is closed")
            if self._value is not None:
                self.replaced_count += 1
            self._value = value
            self._condition.notify()

    def get(self, timeout: float | None = None) -> _T | None:
        with self._condition:
            if self._value is None and not self._closed:
                self._condition.wait(timeout=timeout)
            value = self._value
            self._value = None
            return value

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()


def select_newest_view(states: Iterable[ViewState]) -> ViewState:
    candidates = list(states)
    if not candidates:
        raise ValueError("at least one view state is required")
    session_id = candidates[0].session_id
    if any(item.session_id != session_id for item in candidates):
        raise ValueError("view states from different sessions cannot share a queue")
    return max(candidates, key=lambda item: item.view_sequence)


def pack_ros_diagnostic_frame(
    metadata: RenderedFrameMetadata,
    stereo_image: bytes,
) -> bytes:
    """Bundle v3 metadata and a side-by-side PNG for diagnostic transport."""
    metadata_bytes = metadata.to_json().encode("utf-8")
    image_bytes = bytes(stereo_image)
    if not metadata_bytes or len(metadata_bytes) > _MAXIMUM_METADATA_BYTES:
        raise ValueError("render metadata exceeds the diagnostic limit")
    if not image_bytes or len(image_bytes) > _MAXIMUM_IMAGE_BYTES:
        raise ValueError("stereo image exceeds the diagnostic limit")
    return (
        _ROS_DIAGNOSTIC_HEADER.pack(
            _ROS_DIAGNOSTIC_MAGIC,
            _ROS_DIAGNOSTIC_VERSION,
            0,
            0,
            len(metadata_bytes),
            len(image_bytes),
        )
        + metadata_bytes
        + image_bytes
    )


def unpack_ros_diagnostic_frame(
    payload: bytes,
) -> tuple[RenderedFrameMetadata, bytes]:
    data = bytes(payload)
    if len(data) < _ROS_DIAGNOSTIC_HEADER.size:
        raise ValueError("diagnostic frame is truncated")
    magic, version, flags, reserved, metadata_size, image_size = (
        _ROS_DIAGNOSTIC_HEADER.unpack_from(data)
    )
    if magic != _ROS_DIAGNOSTIC_MAGIC or version != _ROS_DIAGNOSTIC_VERSION:
        raise ValueError("unsupported diagnostic frame header")
    if flags != 0 or reserved != 0:
        raise ValueError("unsupported diagnostic frame flags")
    if not 0 < metadata_size <= _MAXIMUM_METADATA_BYTES:
        raise ValueError("invalid diagnostic metadata size")
    if not 0 < image_size <= _MAXIMUM_IMAGE_BYTES:
        raise ValueError("invalid diagnostic image size")
    expected_size = _ROS_DIAGNOSTIC_HEADER.size + metadata_size + image_size
    if len(data) != expected_size:
        raise ValueError("diagnostic frame length does not match its header")
    metadata_start = _ROS_DIAGNOSTIC_HEADER.size
    image_start = metadata_start + metadata_size
    metadata = RenderedFrameMetadata.from_json(data[metadata_start:image_start])
    return metadata, data[image_start:]
