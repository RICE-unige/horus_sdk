"""Typed camera metadata config helpers for HORUS MR payload compatibility."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional, Sequence


class _PayloadEnum(str, Enum):
    def __str__(self) -> str:
        return self.value


class CameraTransport(_PayloadEnum):
    ROS = "ros"
    WEBRTC = "webrtc"


class CameraImageType(_PayloadEnum):
    RAW = "raw"
    COMPRESSED = "compressed"


class CameraStartupMode(_PayloadEnum):
    MINIMAP = "minimap"
    TELEOP = "teleop"


class StereoLayout(_PayloadEnum):
    MONO = "mono"
    SIDE_BY_SIDE = "side_by_side"
    DUAL_TOPIC = "dual_topic"


def _payload_value(value: Any) -> Any:
    return value.value if isinstance(value, Enum) else value


def _coerce_vec3(value: Sequence[float], field_name: str) -> list[float]:
    if value is None or len(value) != 3:
        raise ValueError(f"{field_name} must contain exactly 3 values")
    return [float(value[0]), float(value[1]), float(value[2])]


def _put_if_set(payload: Dict[str, Any], key: str, value: Any) -> None:
    if value is not None:
        payload[key] = _payload_value(value)


@dataclass(frozen=True)
class CameraProjectedViewConfig:
    position_offset: Optional[Sequence[float]] = None
    rotation_offset: Optional[Sequence[float]] = None
    scale_multiplier: Optional[Sequence[float]] = None
    image_scale: Optional[float] = None
    focal_length_scale: Optional[float] = None
    projection_target_frame: Optional[str] = None
    show_frustum: Optional[bool] = None
    frustum_color: Optional[str] = None

    @classmethod
    def from_values(cls, **kwargs: Any) -> "CameraProjectedViewConfig":
        return cls(**kwargs)

    def to_payload(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {"display_mode": "projected"}
        if self.position_offset is not None:
            payload["projected_position_offset"] = _coerce_vec3(
                self.position_offset,
                "position_offset",
            )
        if self.rotation_offset is not None:
            payload["view_rotation_offset"] = _coerce_vec3(
                self.rotation_offset,
                "rotation_offset",
            )
        if self.scale_multiplier is not None:
            payload["projected_scale_multiplier"] = _coerce_vec3(
                self.scale_multiplier,
                "scale_multiplier",
            )
        _put_if_set(payload, "image_scale", None if self.image_scale is None else float(self.image_scale))
        _put_if_set(payload, "focal_length_scale", None if self.focal_length_scale is None else float(self.focal_length_scale))
        _put_if_set(payload, "projection_target_frame", self.projection_target_frame)
        _put_if_set(payload, "show_frustum", None if self.show_frustum is None else bool(self.show_frustum))
        _put_if_set(payload, "frustum_color", self.frustum_color)
        return payload


@dataclass(frozen=True)
class CameraMinimapViewConfig:
    size: Optional[float] = None
    position_offset: Optional[Sequence[float]] = None
    face_camera: Optional[bool] = None
    rotation_offset: Optional[Sequence[float]] = None

    @classmethod
    def from_values(cls, **kwargs: Any) -> "CameraMinimapViewConfig":
        return cls(**kwargs)

    def to_payload(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {}
        _put_if_set(payload, "overhead_size", None if self.size is None else float(self.size))
        if self.position_offset is not None:
            payload["overhead_position_offset"] = _coerce_vec3(
                self.position_offset,
                "position_offset",
            )
        _put_if_set(payload, "overhead_face_camera", None if self.face_camera is None else bool(self.face_camera))
        if self.rotation_offset is not None:
            payload["overhead_rotation_offset"] = _coerce_vec3(
                self.rotation_offset,
                "rotation_offset",
            )
        return payload


@dataclass(frozen=True)
class CameraImmersiveViewConfig:
    ros_flip_x: Optional[bool] = None
    ros_flip_y: Optional[bool] = None

    @classmethod
    def from_values(cls, **kwargs: Any) -> "CameraImmersiveViewConfig":
        return cls(**kwargs)

    def to_payload(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {}
        _put_if_set(payload, "immersive_ros_flip_x", None if self.ros_flip_x is None else bool(self.ros_flip_x))
        _put_if_set(payload, "immersive_ros_flip_y", None if self.ros_flip_y is None else bool(self.ros_flip_y))
        return payload


@dataclass(frozen=True)
class CameraWebRtcTransportConfig:
    client_signal_topic: str = "/horus/webrtc/client_signal"
    server_signal_topic: str = "/horus/webrtc/server_signal"
    bitrate_kbps: Optional[int] = None
    framerate: Optional[int] = None
    stun_server_url: Optional[str] = None
    turn_server_url: Optional[str] = None
    turn_username: Optional[str] = None
    turn_credential: Optional[str] = None

    @classmethod
    def from_values(cls, **kwargs: Any) -> "CameraWebRtcTransportConfig":
        return cls(**kwargs)

    def to_payload(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "webrtc_client_signal_topic": str(self.client_signal_topic),
            "webrtc_server_signal_topic": str(self.server_signal_topic),
        }
        _put_if_set(payload, "webrtc_bitrate_kbps", None if self.bitrate_kbps is None else int(self.bitrate_kbps))
        _put_if_set(payload, "webrtc_framerate", None if self.framerate is None else int(self.framerate))
        _put_if_set(payload, "webrtc_stun_server_url", self.stun_server_url)
        _put_if_set(payload, "webrtc_turn_server_url", self.turn_server_url)
        _put_if_set(payload, "webrtc_turn_username", self.turn_username)
        _put_if_set(payload, "webrtc_turn_credential", self.turn_credential)
        return payload
