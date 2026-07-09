"""Typed robot metadata config helpers for HORUS MR payload compatibility."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import re
from typing import Any, Dict, Optional


def normalize_binding_mode(value: Any, default: str) -> str:
    normalized = str(value or "").strip().lower()
    return normalized if normalized in {"prefixed", "flat"} else default


def normalize_frame_token(value: Any, default: str = "") -> str:
    normalized = str(value or "").strip().strip("/")
    return normalized or default


def normalize_topic_prefix(value: Any) -> str:
    raw = str(value or "").strip()
    if not raw:
        return ""
    return "/" + raw.strip("/")


def normalize_topic_leaf(value: Any, default: str = "") -> str:
    normalized = str(value or "").strip().strip("/")
    normalized = re.sub(r"[^A-Za-z0-9_]+", "_", normalized)
    normalized = re.sub(r"_+", "_", normalized).strip("_")
    return normalized or default


def coerce_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


class _PayloadEnum(str, Enum):
    def __str__(self) -> str:
        return self.value


class TeleopProfile(_PayloadEnum):
    WHEELED = "wheeled"
    LEGGED = "legged"
    AERIAL = "aerial"
    DRONE = "drone"
    CUSTOM = "custom"


class TeleopResponseMode(_PayloadEnum):
    ANALOG = "analog"
    DISCRETE = "discrete"


class DeadmanPolicy(_PayloadEnum):
    EITHER_INDEX_TRIGGER = "either_index_trigger"
    LEFT_INDEX_TRIGGER = "left_index_trigger"
    RIGHT_INDEX_TRIGGER = "right_index_trigger"
    EITHER_GRIP_TRIGGER = "either_grip_trigger"


def _payload_value(value: Any) -> Any:
    return value.value if isinstance(value, Enum) else value


def _put_if_set(payload: Dict[str, Any], key: str, value: Any) -> None:
    if value is not None:
        payload[key] = _payload_value(value)


@dataclass(frozen=True)
class RobotManagerConfig:
    enabled: bool = True
    status: bool = True
    data_viz: bool = True
    teleop: bool = True
    tasks: bool = True
    prefab_asset_path: str = "Assets/Prefabs/UI/RobotManager.prefab"
    prefab_resource_path: str = ""

    @classmethod
    def from_values(
        cls,
        enabled: Any = True,
        status: Any = True,
        data_viz: Any = True,
        teleop: Any = True,
        tasks: Any = True,
        prefab_asset_path: Any = "Assets/Prefabs/UI/RobotManager.prefab",
        prefab_resource_path: Any = "",
    ) -> "RobotManagerConfig":
        return cls(
            enabled=bool(enabled),
            status=bool(status),
            data_viz=bool(data_viz),
            teleop=bool(teleop),
            tasks=bool(tasks),
            prefab_asset_path=str(prefab_asset_path or "Assets/Prefabs/UI/RobotManager.prefab"),
            prefab_resource_path=str(prefab_resource_path or ""),
        )

    def to_payload(self) -> Dict[str, Any]:
        return {
            "enabled": self.enabled,
            "prefab_asset_path": self.prefab_asset_path,
            "prefab_resource_path": self.prefab_resource_path,
            "sections": {
                "status": self.status,
                "data_viz": self.data_viz,
                "teleop": self.teleop,
                "tasks": self.tasks,
            },
        }


@dataclass(frozen=True)
class TeleopConfig:
    enabled: bool = True
    command_topic: Optional[Any] = None
    raw_input_topic: Optional[Any] = None
    head_pose_topic: Optional[Any] = None
    robot_profile: Optional[Any] = None
    response_mode: Optional[Any] = None
    publish_rate_hz: Optional[Any] = None
    custom_passthrough_only: Optional[Any] = None
    deadman_policy: Optional[Any] = None
    deadman_timeout_ms: Optional[Any] = None
    deadzone: Optional[Any] = None
    expo: Optional[Any] = None
    linear_xy_max_mps: Optional[Any] = None
    linear_z_max_mps: Optional[Any] = None
    angular_z_max_rps: Optional[Any] = None
    invert_linear_x: Optional[Any] = None
    invert_linear_y: Optional[Any] = None
    invert_linear_z: Optional[Any] = None
    invert_angular_z: Optional[Any] = None
    discrete_threshold: Optional[Any] = None
    linear_xy_step_mps: Optional[Any] = None
    linear_z_step_mps: Optional[Any] = None
    angular_z_step_rps: Optional[Any] = None

    @classmethod
    def from_values(cls, **kwargs: Any) -> "TeleopConfig":
        return cls(**kwargs)

    def to_payload(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {"enabled": bool(self.enabled)}
        for key in (
            "command_topic",
            "raw_input_topic",
            "head_pose_topic",
            "robot_profile",
            "response_mode",
            "publish_rate_hz",
            "custom_passthrough_only",
        ):
            _put_if_set(payload, key, getattr(self, key))

        deadman: Dict[str, Any] = {}
        _put_if_set(deadman, "policy", self.deadman_policy)
        _put_if_set(deadman, "timeout_ms", self.deadman_timeout_ms)
        if deadman:
            payload["deadman"] = deadman

        axes: Dict[str, Any] = {}
        for key in (
            "deadzone",
            "expo",
            "linear_xy_max_mps",
            "linear_z_max_mps",
            "angular_z_max_rps",
            "invert_linear_x",
            "invert_linear_y",
            "invert_linear_z",
            "invert_angular_z",
        ):
            _put_if_set(axes, key, getattr(self, key))
        if axes:
            payload["axes"] = axes

        discrete: Dict[str, Any] = {}
        discrete_keys = {
            "discrete_threshold": "threshold",
            "linear_xy_step_mps": "linear_xy_step_mps",
            "linear_z_step_mps": "linear_z_step_mps",
            "angular_z_step_rps": "angular_z_step_rps",
        }
        for attr, key in discrete_keys.items():
            _put_if_set(discrete, key, getattr(self, attr))
        if discrete:
            payload["discrete"] = discrete

        return payload


@dataclass(frozen=True)
class GoToPointTaskConfig:
    enabled: bool = True
    goal_topic: Optional[Any] = None
    cancel_topic: Optional[Any] = None
    status_topic: Optional[Any] = None
    frame_id: Optional[Any] = None
    position_tolerance_m: Optional[Any] = None
    yaw_tolerance_deg: Optional[Any] = None
    min_altitude_m: Optional[Any] = None
    max_altitude_m: Optional[Any] = None

    @classmethod
    def from_values(cls, **kwargs: Any) -> "GoToPointTaskConfig":
        return cls(**kwargs)

    def to_payload(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {"enabled": bool(self.enabled)}
        for key in (
            "goal_topic",
            "cancel_topic",
            "status_topic",
            "frame_id",
            "position_tolerance_m",
            "yaw_tolerance_deg",
            "min_altitude_m",
            "max_altitude_m",
        ):
            _put_if_set(payload, key, getattr(self, key))
        return payload


@dataclass(frozen=True)
class WaypointTaskConfig:
    enabled: bool = True
    path_topic: Optional[Any] = None
    status_topic: Optional[Any] = None
    frame_id: Optional[Any] = None
    position_tolerance_m: Optional[Any] = None
    yaw_tolerance_deg: Optional[Any] = None

    @classmethod
    def from_values(cls, **kwargs: Any) -> "WaypointTaskConfig":
        return cls(**kwargs)

    def to_payload(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {"enabled": bool(self.enabled)}
        for key in (
            "path_topic",
            "status_topic",
            "frame_id",
            "position_tolerance_m",
            "yaw_tolerance_deg",
        ):
            _put_if_set(payload, key, getattr(self, key))
        return payload


@dataclass(frozen=True)
class NavigationTasksConfig:
    go_to_point: GoToPointTaskConfig
    waypoint: WaypointTaskConfig

    @classmethod
    def from_values(
        cls,
        go_to_point_enabled: Any = True,
        waypoint_enabled: Any = True,
        goal_topic: Optional[Any] = None,
        cancel_topic: Optional[Any] = None,
        goal_status_topic: Optional[Any] = None,
        waypoint_path_topic: Optional[Any] = None,
        waypoint_status_topic: Optional[Any] = None,
        frame_id: Optional[Any] = "map",
        position_tolerance_m: Optional[Any] = None,
        yaw_tolerance_deg: Optional[Any] = None,
        min_altitude_m: Optional[Any] = None,
        max_altitude_m: Optional[Any] = None,
    ) -> "NavigationTasksConfig":
        return cls(
            go_to_point=GoToPointTaskConfig.from_values(
                enabled=go_to_point_enabled,
                goal_topic=goal_topic,
                cancel_topic=cancel_topic,
                status_topic=goal_status_topic,
                frame_id=frame_id,
                position_tolerance_m=position_tolerance_m,
                yaw_tolerance_deg=yaw_tolerance_deg,
                min_altitude_m=min_altitude_m,
                max_altitude_m=max_altitude_m,
            ),
            waypoint=WaypointTaskConfig.from_values(
                enabled=waypoint_enabled,
                path_topic=waypoint_path_topic,
                status_topic=waypoint_status_topic,
                frame_id=frame_id,
                position_tolerance_m=position_tolerance_m,
                yaw_tolerance_deg=yaw_tolerance_deg,
            ),
        )

    def to_payload(self) -> Dict[str, Any]:
        return {
            "go_to_point": self.go_to_point.to_payload(),
            "waypoint": self.waypoint.to_payload(),
        }


@dataclass(frozen=True)
class RosBindingConfig:
    logical_name: str
    tf_mode: str = "prefixed"
    topic_mode: str = "prefixed"
    base_frame: str = "base_link"
    tf_prefix: str = ""
    topic_prefix: str = ""

    @classmethod
    def from_values(
        cls,
        logical_name: str,
        tf_mode: Any = "prefixed",
        topic_mode: Any = "prefixed",
        base_frame: Any = "base_link",
        tf_prefix: Any = "",
        topic_prefix: Any = "",
    ) -> "RosBindingConfig":
        name = str(logical_name or "").strip() or "robot"
        resolved_tf_mode = normalize_binding_mode(tf_mode, "prefixed")
        resolved_topic_mode = normalize_binding_mode(topic_mode, "prefixed")
        resolved_base_frame = normalize_frame_token(base_frame, "base_link")
        resolved_tf_prefix = (
            normalize_frame_token(tf_prefix, name)
            if resolved_tf_mode == "prefixed"
            else normalize_frame_token(tf_prefix)
        )
        resolved_topic_prefix = (
            normalize_topic_prefix(topic_prefix or f"/{name}")
            if resolved_topic_mode == "prefixed"
            else normalize_topic_prefix(topic_prefix)
        )
        return cls(
            logical_name=name,
            tf_mode=resolved_tf_mode,
            topic_mode=resolved_topic_mode,
            base_frame=resolved_base_frame,
            tf_prefix=resolved_tf_prefix,
            topic_prefix=resolved_topic_prefix,
        )

    @classmethod
    def from_metadata(
        cls,
        logical_name: str,
        raw: Any,
        description_base_frame: str = "",
    ) -> "RosBindingConfig":
        metadata = raw if isinstance(raw, dict) else {}
        name = str(logical_name or "").strip() or "robot"
        return cls.from_values(
            logical_name=name,
            tf_mode=metadata.get("tf_mode"),
            topic_mode=metadata.get("topic_mode"),
            base_frame=metadata.get("base_frame") or description_base_frame or "base_link",
            tf_prefix=metadata.get("tf_prefix"),
            topic_prefix=metadata.get("topic_prefix"),
        )

    def to_payload(self) -> Dict[str, str]:
        return {
            "logical_name": self.logical_name,
            "tf_mode": self.tf_mode,
            "topic_mode": self.topic_mode,
            "base_frame": self.base_frame,
            "tf_prefix": self.tf_prefix,
            "topic_prefix": self.topic_prefix,
        }


@dataclass(frozen=True)
class WorkspaceTutorialConfig:
    preset_id: str
    enabled: bool = True

    @classmethod
    def from_values(cls, preset_id: Any, enabled: Any = True) -> "WorkspaceTutorialConfig":
        normalized_preset_id = str(preset_id or "").strip()
        if not normalized_preset_id:
            raise ValueError("preset_id must be a non-empty string")
        return cls(preset_id=normalized_preset_id, enabled=bool(enabled))

    def to_payload(self) -> Dict[str, Any]:
        return {"enabled": self.enabled, "preset_id": self.preset_id}


@dataclass(frozen=True)
class WorkspaceCompassConfig:
    enabled: bool = False
    gateway_host: str = ""
    gateway_http_base_url: str = ""
    gateway_ws_url: str = ""
    gateway_port: int = 8088
    voice_mode: str = "auto"
    autonomy: str = "approve_actions"
    contract_version: str = "compass.v1"

    @classmethod
    def from_values(
        cls,
        enabled: Any = False,
        gateway_host: Any = "",
        gateway_http_base_url: Any = "",
        gateway_ws_url: Any = "",
        gateway_port: Any = 8088,
        voice_mode: Any = "auto",
        autonomy: Any = "approve_actions",
        contract_version: Any = "compass.v1",
    ) -> "WorkspaceCompassConfig":
        normalized_voice_mode = str(voice_mode or "auto").strip().lower()
        if normalized_voice_mode not in {"auto", "batch", "realtime"}:
            normalized_voice_mode = "auto"

        normalized_contract = str(contract_version or "compass.v1").strip()
        if not normalized_contract:
            normalized_contract = "compass.v1"

        resolved_port = coerce_int(gateway_port, 8088)
        if resolved_port <= 0 or resolved_port > 65535:
            resolved_port = 8088

        return cls(
            enabled=bool(enabled),
            gateway_host=str(gateway_host or "").strip(),
            gateway_http_base_url=str(gateway_http_base_url or "").strip().rstrip("/"),
            gateway_ws_url=str(gateway_ws_url or "").strip(),
            gateway_port=resolved_port,
            voice_mode=normalized_voice_mode,
            autonomy="approve_actions",
            contract_version=normalized_contract,
        )

    def to_payload(self) -> Dict[str, Any]:
        payload = {
            "enabled": self.enabled,
            "gateway_port": self.gateway_port,
            "voice_mode": self.voice_mode,
            "autonomy": self.autonomy,
            "contract_version": self.contract_version,
        }
        _put_if_set(payload, "gateway_host", self.gateway_host or None)
        _put_if_set(payload, "gateway_http_base_url", self.gateway_http_base_url or None)
        _put_if_set(payload, "gateway_ws_url", self.gateway_ws_url or None)
        return payload


@dataclass(frozen=True)
class WorkspaceExperimentConfig:
    enabled: bool = False
    contract_version: str = "experiment.v1"

    @classmethod
    def from_values(
        cls,
        enabled: Any = False,
        contract_version: Any = "experiment.v1",
    ) -> "WorkspaceExperimentConfig":
        normalized_contract = str(contract_version or "experiment.v1").strip()
        if not normalized_contract:
            normalized_contract = "experiment.v1"

        return cls(
            enabled=bool(enabled),
            contract_version=normalized_contract,
        )

    def to_payload(self) -> Dict[str, Any]:
        return {
            "enabled": self.enabled,
            "contract_version": self.contract_version,
        }


@dataclass(frozen=True)
class RobotDescriptionConfig:
    urdf_path: str = ""
    base_frame: str = "base_link"
    source: str = "ros"
    ros_param_node: str = ""
    ros_param_name: str = "robot_description"
    robot_description_topic: str = "/robot_description"
    urdf_package: str = ""
    mesh_root: str = ""
    chunk_size_bytes: int = 12000
    is_transparent: bool = False
    include_visual_meshes: bool = True
    visual_mesh_triangle_budget: int = 90000
    body_mesh_mode: str = "preview_mesh"
    visual_link_pose_source: str = "static"
    enabled: bool = True

    @classmethod
    def from_values(
        cls,
        urdf_path: Any = "",
        base_frame: Any = "base_link",
        source: Any = "ros",
        ros_param_node: Any = "",
        ros_param_name: Any = "robot_description",
        robot_description_topic: Any = "/robot_description",
        urdf_package: Any = "",
        mesh_root: Any = "",
        chunk_size_bytes: Any = 12000,
        is_transparent: Any = False,
        include_visual_meshes: Any = True,
        visual_mesh_triangle_budget: Any = 90000,
        body_mesh_mode: Any = "preview_mesh",
        visual_link_pose_source: Any = "static",
        enabled: Any = True,
    ) -> "RobotDescriptionConfig":
        normalized_body_mesh_mode = str(body_mesh_mode or "preview_mesh").strip().lower()
        if normalized_body_mesh_mode == "max_quality_mesh":
            normalized_body_mesh_mode = "runtime_high_mesh"
        if normalized_body_mesh_mode not in {"collision_only", "preview_mesh", "runtime_high_mesh"}:
            normalized_body_mesh_mode = "preview_mesh"

        resolved_include_visual_meshes = bool(include_visual_meshes)
        if normalized_body_mesh_mode == "collision_only":
            resolved_include_visual_meshes = False

        normalized_source = str(source or "ros").strip().lower()
        if normalized_source not in {"local", "ros", "topic"}:
            normalized_source = "ros"

        normalized_pose_source = str(visual_link_pose_source or "static").strip().lower()
        if normalized_pose_source not in {"static", "tf"}:
            normalized_pose_source = "static"

        return cls(
            urdf_path=str(urdf_path or ""),
            base_frame=str(base_frame or "base_link"),
            source=normalized_source,
            ros_param_node=str(ros_param_node or ""),
            ros_param_name=str(ros_param_name or "robot_description"),
            robot_description_topic=str(robot_description_topic or "/robot_description"),
            urdf_package=str(urdf_package or ""),
            mesh_root=str(mesh_root or ""),
            chunk_size_bytes=int(max(1024, min(64000, coerce_int(chunk_size_bytes, 12000)))),
            is_transparent=bool(is_transparent),
            include_visual_meshes=resolved_include_visual_meshes,
            visual_mesh_triangle_budget=int(max(2000, min(500000, coerce_int(visual_mesh_triangle_budget, 90000)))),
            body_mesh_mode=normalized_body_mesh_mode,
            visual_link_pose_source=normalized_pose_source,
            enabled=bool(enabled),
        )

    def to_payload(self) -> Dict[str, Any]:
        return {
            "enabled": self.enabled,
            "source": self.source,
            "urdf_path": self.urdf_path,
            "base_frame": self.base_frame,
            "ros_param_node": self.ros_param_node,
            "ros_param_name": self.ros_param_name,
            "robot_description_topic": self.robot_description_topic,
            "urdf_package": self.urdf_package,
            "mesh_root": self.mesh_root,
            "chunk_size_bytes": self.chunk_size_bytes,
            "is_transparent": self.is_transparent,
            "include_visual_meshes": self.include_visual_meshes,
            "visual_mesh_triangle_budget": self.visual_mesh_triangle_budget,
            "body_mesh_mode": self.body_mesh_mode,
            "visual_link_pose_source": self.visual_link_pose_source,
        }


@dataclass(frozen=True)
class LocalBodyModelConfig:
    robot_model_id: str
    enabled: bool = True

    @classmethod
    def from_values(cls, robot_model_id: Any, enabled: Any = True) -> "LocalBodyModelConfig":
        normalized_model_id = str(robot_model_id or "").strip().lower()
        return cls(
            robot_model_id=normalized_model_id,
            enabled=bool(enabled) and bool(normalized_model_id),
        )

    def to_payload(self) -> Dict[str, Any]:
        return {"enabled": self.enabled, "robot_model_id": self.robot_model_id}


# ---------------------------------------------------------------------------
# Entity capability contract (capability-driven, default-deny safety)
# ---------------------------------------------------------------------------

ENTITY_KIND_ROBOT = "robot"
ENTITY_KIND_FIELD_TEAMMATE = "field_teammate"
_VALID_ENTITY_KINDS = {ENTITY_KIND_ROBOT, ENTITY_KIND_FIELD_TEAMMATE}


def normalize_entity_kind(value: Any, default: str = ENTITY_KIND_ROBOT) -> str:
    """Normalize an entity-kind token to a known value (default-safe)."""
    normalized = str(value or "").strip().lower()
    return normalized if normalized in _VALID_ENTITY_KINDS else default


@dataclass(frozen=True)
class EntityCapabilities:
    """What an entity is permitted to do inside HORUS MR.

    Safety is **capability-driven and default-deny**: consumers must gate every
    command path on these flags rather than inferring permission from
    ``robot_type``. A robot is controllable; a human field teammate is not —
    they are *guidable*. The flags travel in the registration payload so the MR
    runtime never has to guess.
    """

    controllable: bool = True
    teleoperable: bool = True
    taskable: bool = True
    guidable: bool = False
    observable: bool = True
    communicative: bool = False

    @classmethod
    def for_robot(cls) -> "EntityCapabilities":
        """Capabilities for a controllable robot (the SDK default)."""
        return cls()

    @classmethod
    def for_field_teammate(cls) -> "EntityCapabilities":
        """Default-deny capabilities for a human field teammate.

        Robot-control affordances are denied; guidance/observation/communication
        are granted. These are the *only* safe defaults for a human entity.
        """
        return cls(
            controllable=False,
            teleoperable=False,
            taskable=False,
            guidable=True,
            observable=True,
            communicative=True,
        )

    @classmethod
    def from_values(
        cls,
        base: Optional["EntityCapabilities"] = None,
        **overrides: Any,
    ) -> "EntityCapabilities":
        """Build capabilities from a base set plus explicit boolean overrides."""
        resolved = base if isinstance(base, cls) else cls()
        merged = asdict(resolved)
        for key, value in overrides.items():
            if key in merged and value is not None:
                merged[key] = bool(value)
        return cls(**merged)

    def to_payload(self) -> Dict[str, bool]:
        return {
            "controllable": self.controllable,
            "teleoperable": self.teleoperable,
            "taskable": self.taskable,
            "guidable": self.guidable,
            "observable": self.observable,
            "communicative": self.communicative,
        }


def _field_teammate_topic_prefix(name: Any) -> str:
    leaf = normalize_topic_leaf(name, "field_teammate")
    return "/" + leaf


@dataclass(frozen=True)
class FieldTeammateConfig:
    """Typed contract for a HoloLens-class field teammate represented in HORUS.

    Mirrors the existing ``*Config`` style: ``from_values`` normalizes inputs and
    derives sensible per-entity topic defaults, ``to_payload`` emits the
    versioned dict embedded under ``field_teammate_config`` in the registration
    payload. The HoloLens is the first wearable; the same contract accepts other
    glasses later.
    """

    wearable_type: str = "hololens2"
    base_frame: str = "field_teammate/base"
    camera_frame: str = "field_teammate/camera"
    first_person_video_topic: str = ""
    localization_confidence_topic: str = ""
    guidance_request_topic: str = ""
    guidance_response_topic: str = ""
    guidance_state_topic: str = ""
    guidance_annotation_topic: str = ""
    guidance_route_topic: str = ""
    guidance_warning_topic: str = ""
    status_topic: str = ""
    audio_topic: str = ""
    can_acknowledge: bool = True
    can_clarify: bool = True
    can_reject: bool = True
    can_complete: bool = True
    profile_height_m: float = 1.75
    profile_sex: str = "unspecified"
    body_model: str = "meta_avatar"
    contract_version: str = "field_teammate.v1"

    _SUPPORTED_WEARABLES = ("hololens2", "aria", "quest_pro", "generic")

    @classmethod
    def from_values(
        cls,
        name: Any = "field_teammate",
        *,
        wearable_type: Any = "hololens2",
        base_frame: Any = None,
        camera_frame: Any = None,
        first_person_video_topic: Any = None,
        localization_confidence_topic: Any = None,
        guidance_request_topic: Any = None,
        guidance_response_topic: Any = None,
        guidance_state_topic: Any = None,
        guidance_annotation_topic: Any = None,
        guidance_route_topic: Any = None,
        guidance_warning_topic: Any = None,
        status_topic: Any = None,
        audio_topic: Any = None,
        can_acknowledge: Any = True,
        can_clarify: Any = True,
        can_reject: Any = True,
        can_complete: Any = True,
        profile_height_m: Any = 1.75,
        profile_sex: Any = "unspecified",
        body_model: Any = "meta_avatar",
        contract_version: Any = "field_teammate.v1",
    ) -> "FieldTeammateConfig":
        prefix = _field_teammate_topic_prefix(name)
        leaf = prefix.lstrip("/")

        normalized_wearable = str(wearable_type or "hololens2").strip().lower()
        if normalized_wearable not in cls._SUPPORTED_WEARABLES:
            normalized_wearable = "hololens2"

        normalized_contract = str(contract_version or "field_teammate.v1").strip()
        if not normalized_contract:
            normalized_contract = "field_teammate.v1"

        try:
            normalized_height = float(profile_height_m)
        except (TypeError, ValueError):
            normalized_height = 1.75
        normalized_height = max(1.35, min(2.10, normalized_height))

        normalized_sex = str(profile_sex or "unspecified").strip().lower()
        if normalized_sex not in {"female", "male", "unspecified"}:
            normalized_sex = "unspecified"

        normalized_body_model = str(body_model or "meta_avatar").strip().lower()
        if not normalized_body_model:
            normalized_body_model = "meta_avatar"

        def _topic(value: Any, default: str) -> str:
            text = str(value or "").strip()
            return text if text else default

        return cls(
            wearable_type=normalized_wearable,
            base_frame=normalize_frame_token(base_frame, f"{leaf}/base"),
            camera_frame=normalize_frame_token(camera_frame, f"{leaf}/camera"),
            first_person_video_topic=_topic(
                first_person_video_topic, f"{prefix}/fpv/image_raw/compressed"
            ),
            localization_confidence_topic=_topic(
                localization_confidence_topic, f"{prefix}/localization_confidence"
            ),
            guidance_request_topic=_topic(
                guidance_request_topic, f"{prefix}/guidance/request"
            ),
            guidance_response_topic=_topic(
                guidance_response_topic, f"{prefix}/guidance/response"
            ),
            guidance_state_topic=_topic(
                guidance_state_topic, f"{prefix}/guidance/state"
            ),
            guidance_annotation_topic=_topic(
                guidance_annotation_topic, f"{prefix}/guidance/annotation"
            ),
            guidance_route_topic=_topic(
                guidance_route_topic, f"{prefix}/guidance/route"
            ),
            guidance_warning_topic=_topic(
                guidance_warning_topic, f"{prefix}/guidance/warning"
            ),
            status_topic=_topic(status_topic, f"{prefix}/status"),
            audio_topic=_topic(audio_topic, f"{prefix}/audio/message"),
            can_acknowledge=bool(can_acknowledge),
            can_clarify=bool(can_clarify),
            can_reject=bool(can_reject),
            can_complete=bool(can_complete),
            profile_height_m=normalized_height,
            profile_sex=normalized_sex,
            body_model=normalized_body_model,
            contract_version=normalized_contract,
        )

    def to_payload(self) -> Dict[str, Any]:
        return {
            "wearable_type": self.wearable_type,
            "base_frame": self.base_frame,
            "camera_frame": self.camera_frame,
            "topics": {
                "first_person_video": self.first_person_video_topic,
                "localization_confidence": self.localization_confidence_topic,
                "guidance_request": self.guidance_request_topic,
                "guidance_response": self.guidance_response_topic,
                "guidance_state": self.guidance_state_topic,
                "guidance_annotation": self.guidance_annotation_topic,
                "guidance_route": self.guidance_route_topic,
                "guidance_warning": self.guidance_warning_topic,
                "status": self.status_topic,
                "audio": self.audio_topic,
            },
            "interactions": {
                "can_acknowledge": self.can_acknowledge,
                "can_clarify": self.can_clarify,
                "can_reject": self.can_reject,
                "can_complete": self.can_complete,
            },
            "profile": {
                "human_height_m": self.profile_height_m,
                "sex": self.profile_sex,
                "body_model": self.body_model,
            },
            "contract_version": self.contract_version,
        }
