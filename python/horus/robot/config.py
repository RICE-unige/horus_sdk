"""Typed robot metadata config helpers for HORUS MR payload compatibility."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
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
class RobotDescriptionConfig:
    urdf_path: str
    base_frame: str = "base_link"
    source: str = "ros"
    ros_param_node: str = ""
    ros_param_name: str = "robot_description"
    chunk_size_bytes: int = 12000
    is_transparent: bool = False
    include_visual_meshes: bool = True
    visual_mesh_triangle_budget: int = 90000
    body_mesh_mode: str = "preview_mesh"
    enabled: bool = True

    @classmethod
    def from_values(
        cls,
        urdf_path: Any,
        base_frame: Any = "base_link",
        source: Any = "ros",
        ros_param_node: Any = "",
        ros_param_name: Any = "robot_description",
        chunk_size_bytes: Any = 12000,
        is_transparent: Any = False,
        include_visual_meshes: Any = True,
        visual_mesh_triangle_budget: Any = 90000,
        body_mesh_mode: Any = "preview_mesh",
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

        return cls(
            urdf_path=str(urdf_path or ""),
            base_frame=str(base_frame or "base_link"),
            source=str(source or "ros"),
            ros_param_node=str(ros_param_node or ""),
            ros_param_name=str(ros_param_name or "robot_description"),
            chunk_size_bytes=int(max(1024, min(64000, coerce_int(chunk_size_bytes, 12000)))),
            is_transparent=bool(is_transparent),
            include_visual_meshes=resolved_include_visual_meshes,
            visual_mesh_triangle_budget=int(max(2000, min(500000, coerce_int(visual_mesh_triangle_budget, 90000)))),
            body_mesh_mode=normalized_body_mesh_mode,
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
            "chunk_size_bytes": self.chunk_size_bytes,
            "is_transparent": self.is_transparent,
            "include_visual_meshes": self.include_visual_meshes,
            "visual_mesh_triangle_budget": self.visual_mesh_triangle_budget,
            "body_mesh_mode": self.body_mesh_mode,
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
