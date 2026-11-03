"""Versioned, robot-independent teaching configuration."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import hashlib
import json
import math
from pathlib import Path
import re

INPUT_FEATURES = ("head_pose", "left_stick", "right_stick", "left_buttons", "right_buttons")


def canonical(value) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def identifier(value: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]{0,95}", value):
        raise ValueError("Expected a nonempty identifier containing letters, digits, '_' or '-'")
    return value


@dataclass(frozen=True)
class TeachStream:
    stream_id: str
    topic: str
    message_type: str
    role: str = "observation"
    required: bool = True
    max_gap_seconds: float = 1.0
    clock_id: str = "ros"
    frame_id: str = ""
    source_identity: str = "unknown"
    stamp_semantics: str = "unknown"
    calibration: dict = field(default_factory=dict)
    reliability: str = "best_effort"
    durability: str = "volatile"
    queue_depth: int = 5

    @property
    def retained_calibration(self):
        return self.message_type == "sensor_msgs/msg/CameraInfo" or (
            self.message_type == "tf2_msgs/msg/TFMessage" and self.durability == "transient_local")

    def __post_init__(self):
        identifier(self.stream_id)
        if not re.fullmatch(r"/[A-Za-z0-9_/]+", self.topic):
            raise ValueError("Teach streams require absolute ROS topic names")
        if not re.fullmatch(r"[A-Za-z0-9_]+/msg/[A-Za-z0-9_]+", self.message_type):
            raise ValueError("Expected package/msg/Type")
        if self.role not in {"observation", "camera", "measured_state", "accepted_command", "feedback", "tf"}:
            raise ValueError("Unknown teaching stream role")
        if not math.isfinite(self.max_gap_seconds) or self.max_gap_seconds <= 0:
            raise ValueError("Stream maximum gap must be positive and finite")
        if not self.clock_id:
            raise ValueError("Every stream must declare its source clock")
        if self.stamp_semantics not in {"unknown", "acquisition", "receipt", "simulation"}:
            raise ValueError("Unknown source timestamp semantics")
        if self.reliability not in {"best_effort", "reliable"} or self.durability not in {"volatile", "transient_local"}:
            raise ValueError("Invalid source QoS")
        if type(self.queue_depth) is not int or not 1 <= self.queue_depth <= 100:
            raise ValueError("Source queue depth must be 1-100")


@dataclass(frozen=True)
class TeachPortal:
    """Optional presentation; estimated stereo is never metric depth."""

    mode: str = "stereo"
    edge_fade: float = 0.06
    corner_radius: float = 0.08
    crop: float = 0.02
    swap_eyes: bool = False
    max_skew_ms: float = 50.0

    def __post_init__(self):
        if self.mode not in {"mono", "stereo", "estimated"}:
            raise ValueError("Portal mode must be mono, stereo or estimated")
        for name, upper in (("edge_fade", .25), ("corner_radius", .25), ("crop", .1)):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= upper:
                raise ValueError(f"Invalid portal {name}")
        if type(self.swap_eyes) is not bool:
            raise ValueError("swap_eyes must be boolean")
        if type(self.max_skew_ms) not in (int, float) or not math.isfinite(self.max_skew_ms) or not 0 <= self.max_skew_ms <= 100:
            raise ValueError("Stereo maximum skew must be 0-100 milliseconds")


@dataclass(frozen=True)
class TeachCamera:
    sensor_id: str
    role: str = "front"
    position: tuple[float, float, float] = (0.0, 1.35, 1.2)
    width_m: float = 0.7
    stream_id: str = ""
    required_for_control: bool = False
    rotation_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    portal: TeachPortal | None = None

    def __post_init__(self):
        if self.portal is not None and not isinstance(self.portal, TeachPortal):
            raise ValueError("portal must be TeachPortal")
        if not isinstance(self.sensor_id, str) or not self.sensor_id.strip() or len(self.sensor_id) > 128:
            raise ValueError("Camera sensor_id must reference a registered sensor name")
        if len(self.position) != 3 or not all(math.isfinite(v) for v in self.position):
            raise ValueError("Camera position must contain three finite metres in calibrated Unity space")
        if not math.isfinite(self.width_m) or not 0.1 <= self.width_m <= 3:
            raise ValueError("Camera width must be between 0.1 and 3 metres")
        if not self.role:
            raise ValueError("Camera role is required")
        if len(self.rotation_deg) != 3 or not all(math.isfinite(v) for v in self.rotation_deg):
            raise ValueError("Camera rotation must contain three finite Euler angles")


@dataclass(frozen=True)
class TeachPointCloud:
    """Measured XYZ/RGB in the robot's TF tree, shown at metric Teach scale."""

    stream_id: str
    point_size_m: float = 0.004
    max_points: int = 1_000_000
    max_age_seconds: float = 1.0

    def __post_init__(self):
        identifier(self.stream_id)
        for name, low, high in (("point_size_m", .001, .03), ("max_age_seconds", .1, 5.0)):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"Invalid spatial view {name}")
        if type(self.max_points) is not int or not 1000 <= self.max_points <= 2_000_000:
            raise ValueError("Spatial view max_points must be 1000-2000000")


@dataclass(frozen=True)
class TeachAction:
    action_id: str
    label: str
    description: str = ""

    def __post_init__(self):
        identifier(self.action_id)
        if not isinstance(self.label, str) or not self.label.strip() or len(self.label) > 64:
            raise ValueError("Teach action label must contain 1-64 characters")
        if not isinstance(self.description, str) or len(self.description) > 256:
            raise ValueError("Teach action description exceeds 256 characters")


@dataclass(frozen=True)
class TeachConfig:
    """Configuration owned by the recording host; no implicit device discovery.

    The bundled ``observe`` adapter cannot actuate hardware. Robot integrations
    explicitly provide a qualified adapter to TeachRuntime, never import a class
    named in a message sent by the headset.
    """

    recording_root: str
    streams: tuple[TeachStream, ...] = ()
    cameras: tuple[TeachCamera, ...] = ()
    task: str = ""
    adapter_id: str = "observe"
    workflow: str = "observe_record"
    joint_names: tuple[str, ...] = ()
    joint_groups: dict[str, tuple[str, ...]] = field(default_factory=dict)
    command_frame: str = ""
    calibration_id: str = ""
    provenance: str = "real"
    publish_rate_hz: float = 30.0
    input_timeout_ms: int = 200
    controlled_hands: tuple[str, ...] = ("left", "right")
    clutch_binding: str = "both_grips"
    hold_binding: str = "right_secondary"
    grasp_bindings: dict[str, str] = field(default_factory=dict)
    task_presets: tuple[str, ...] = ()
    queue_budget_bytes: int = 64 * 1024 * 1024
    minimum_free_bytes: int = 512 * 1024 * 1024
    schema_version: int = 1
    input_schema_version: int = 1
    required_input_features: tuple[str, ...] = ()
    engagement_mode: str = "hold_to_run"
    engage_countdown_seconds: float = 3.0
    disengage_binding: str = "both_primary"
    tracking_recovery: str = "manual"
    spatial_views: tuple[TeachPointCloud, ...] = ()
    task_management: bool = False
    episode_counts: bool = False
    actions: tuple[TeachAction, ...] = ()

    def __post_init__(self):
        if type(self.task_management) is not bool or type(self.episode_counts) is not bool:
            raise ValueError("Teach task/count options must be boolean")
        if len(self.actions) > 8 or any(not isinstance(a, TeachAction) for a in self.actions):
            raise ValueError("Choose at most eight TeachAction entries")
        if len({a.action_id for a in self.actions}) != len(self.actions):
            raise ValueError("Teach action IDs must be unique")
        if self.schema_version != 1:
            raise ValueError("Unsupported Teach schema version")
        if type(self.input_schema_version) is not int or self.input_schema_version not in (1, 2):
            raise ValueError("Unsupported Teach input schema version")
        if (not isinstance(self.required_input_features, (list, tuple)) or
            any(not isinstance(v, str) or v not in INPUT_FEATURES for v in self.required_input_features) or
            len(set(self.required_input_features)) != len(self.required_input_features)):
            raise ValueError("Required input features must be unique supported features")
        if self.input_schema_version == 1 and self.required_input_features:
            raise ValueError("Required input features need input schema version 2")
        if self.engagement_mode not in {"hold_to_run", "continuous"}:
            raise ValueError("Unsupported Teach engagement mode")
        if self.engagement_mode == "continuous" and self.input_schema_version != 2:
            raise ValueError("Continuous engagement requires input schema version 2")
        if (type(self.engage_countdown_seconds) not in (int, float) or
            not math.isfinite(self.engage_countdown_seconds) or not 3 <= self.engage_countdown_seconds <= 10):
            raise ValueError("Engagement countdown must be 3-10 seconds")
        if self.disengage_binding not in {"both_primary", "left_primary", "right_primary"}:
            raise ValueError("Unsupported disengage binding")
        if self.tracking_recovery not in {"manual", "auto_resume"}:
            raise ValueError("Unsupported tracking recovery policy")
        if self.tracking_recovery == "auto_resume" and (
                self.engagement_mode != "continuous" or self.input_schema_version != 2):
            raise ValueError("Automatic tracking recovery requires a continuous v2 profile")
        if not Path(self.recording_root).expanduser().is_absolute():
            raise ValueError("recording_root must be an absolute path on the recording host")
        if self.workflow not in {"practice", "collection", "observe_record"}:
            raise ValueError("Unknown Teach workflow")
        if self.provenance not in {"real", "simulation", "fixture"}:
            raise ValueError("Unknown data provenance")
        identifier(self.adapter_id)
        if not math.isfinite(self.publish_rate_hz) or not 5 <= self.publish_rate_hz <= 90:
            raise ValueError("Teach input rate must be 5-90 Hz")
        if not 50 <= self.input_timeout_ms <= 500:
            raise ValueError("Input timeout must be 50-500 ms")
        if not self.controlled_hands or len(set(self.controlled_hands)) != len(self.controlled_hands) or not set(self.controlled_hands) <= {"left", "right"}:
            raise ValueError("Choose left, right or both controlled hands")
        bindings = {"both_grips", "left_grip", "right_grip", "both_primary", "left_primary", "right_primary"}
        if self.clutch_binding not in bindings or self.hold_binding not in {"left_secondary", "right_secondary"}:
            raise ValueError("Unsupported clutch/hold binding")
        for name, binding in self.grasp_bindings.items():
            identifier(name)
            if binding not in {"left_trigger", "right_trigger", "left_grip", "right_grip"}:
                raise ValueError("Grasp must reference a measured controller axis")
            if self.engagement_mode == "hold_to_run" and (binding == self.clutch_binding or (self.clutch_binding == "both_grips" and binding.endswith("_grip"))):
                raise ValueError("Clutch and grasp cannot share an input")
        if any(not isinstance(task, str) or not task.strip() for task in self.task_presets):
            raise ValueError("Task presets must be nonempty instructions")
        if not 1024 <= self.queue_budget_bytes <= 1024**3 or self.minimum_free_bytes < 1024:
            raise ValueError("Invalid recorder memory/disk budget")
        ids = [s.stream_id for s in self.streams]
        if len(ids) != len(set(ids)) or len({s.topic for s in self.streams}) != len(ids):
            raise ValueError("Stream IDs and source topics must be unique")
        if len({c.sensor_id for c in self.cameras}) != len(self.cameras):
            raise ValueError("A camera sensor may only be bound once")
        for camera in self.cameras:
            if camera.stream_id and camera.stream_id not in ids:
                raise ValueError("Camera recording stream is not declared")
            if camera.required_for_control and not camera.stream_id:
                raise ValueError("Control-critical camera requires a freshness-monitored stream")
        if any(not isinstance(v, TeachPointCloud) for v in self.spatial_views):
            raise ValueError("Spatial views must be TeachPointCloud entries")
        if len(self.spatial_views) > 2 or len({v.stream_id for v in self.spatial_views}) != len(self.spatial_views):
            raise ValueError("Choose at most two unique spatial views")
        for view in self.spatial_views:
            if not isinstance(view, TeachPointCloud) or not any(
                    s.stream_id == view.stream_id and s.message_type == "sensor_msgs/msg/PointCloud2"
                    for s in self.streams):
                raise ValueError("Spatial views require a declared PointCloud2 recording stream")
        identities = [s.source_identity for s in self.streams if s.role == "camera" and s.source_identity != "unknown"]
        if len(identities) != len(set(identities)):
            raise ValueError("Two camera streams declare the same physical source")
        grouped = [j for values in self.joint_groups.values() for j in values]
        if len(grouped) != len(set(grouped)) or len(self.joint_names) != len(set(self.joint_names)):
            raise ValueError("Joint groups/order must not contain duplicate or overlapping joints")
        if set(grouped) != set(self.joint_names):
            raise ValueError("Each named joint must belong to exactly one owned joint group")
        if self.adapter_id != "observe" and (not self.joint_names or not self.command_frame or not self.calibration_id):
            raise ValueError("Motion profiles require named joints, command frame and calibration identity")

    def to_dict(self) -> dict:
        value = asdict(self)
        for key in ("task_management", "episode_counts", "actions"):
            if not value[key]: value.pop(key)
        if not self.spatial_views:
            value.pop("spatial_views")
        if self.tracking_recovery == "manual":
            value.pop("tracking_recovery")
        for camera in value["cameras"]:
            if camera["portal"] is None:
                camera.pop("portal")
        # Preserve existing profile hashes; continuous engagement is explicit.
        if self.engagement_mode == "hold_to_run":
            for key in ("engagement_mode", "engage_countdown_seconds", "disengage_binding"):
                value.pop(key)
        else:
            value["engage_countdown_seconds"] = float(self.engage_countdown_seconds)
        # Keep existing profile hashes and recorded v1 manifests byte-compatible.
        if self.input_schema_version == 1:
            value.pop("input_schema_version")
            value.pop("required_input_features")
        return json.loads(canonical(value))

    @property
    def digest(self) -> str:
        return hashlib.sha256(canonical(self.to_dict())).hexdigest()

    def registration(self, robot_name: str) -> dict:
        identifier(robot_name)
        prefix = f"/horus/teach/{robot_name}"
        result = {
            "schema_version": 1, "enabled": True, "config_hash": self.digest,
            "request_topic": prefix + "/request", "input_topic": prefix + "/input",
            "state_topic": prefix + "/state", "workflow": self.workflow,
            "task": self.task, "publish_rate_hz": self.publish_rate_hz,
            "input_timeout_ms": self.input_timeout_ms,
            "controlled_hands": self.controlled_hands, "clutch_binding": self.clutch_binding,
            "hold_binding": self.hold_binding, "task_presets": self.task_presets,
            "cameras": [dict(asdict(c), position=dict(zip(("x", "y", "z"), c.position)),
                rotation_deg=dict(zip(("x", "y", "z"), c.rotation_deg))) for c in self.cameras],
        }
        if self.tracking_recovery != "manual":
            result["tracking_recovery"] = self.tracking_recovery
        if self.grasp_bindings:
            result["grasp_hints"] = [dict(control=control, action_id=action,
                label=action.removeprefix("left_").removeprefix("right_").replace("_", " ").capitalize()[:24])
                for action, control in sorted(self.grasp_bindings.items())]
        if self.task_management or self.episode_counts or self.actions:
            result.update(extensions_version=1, task_management=self.task_management,
                          episode_counts=self.episode_counts, actions=[asdict(a) for a in self.actions])
        if self.spatial_views:
            result["spatial_views"] = [dict(asdict(v), topic=next(s.topic for s in self.streams
                if s.stream_id == v.stream_id)) for v in self.spatial_views]
        for camera in result["cameras"]:
            if camera["portal"] is None:
                camera.pop("portal")
        if self.input_schema_version == 2:
            result.update(input_schema_version=2, required_input_features=self.required_input_features)
        if self.engagement_mode == "continuous":
            result.update(engagement_mode=self.engagement_mode,
                          engage_countdown_seconds=float(self.engage_countdown_seconds),
                          disengage_binding=self.disengage_binding)
        return result

    @classmethod
    def from_dict(cls, value: dict) -> "TeachConfig":
        data = dict(value)
        data["streams"] = tuple(TeachStream(**v) for v in data.get("streams", ()))
        data["cameras"] = tuple(TeachCamera(**dict(v, portal=TeachPortal(**v["portal"]) if v.get("portal") is not None else None))
                                for v in data.get("cameras", ()))
        data["spatial_views"] = tuple(TeachPointCloud(**v) for v in data.get("spatial_views", ()))
        data["actions"] = tuple(TeachAction(**v) for v in data.get("actions", ()))
        data["joint_names"] = tuple(data.get("joint_names", ()))
        data["controlled_hands"] = tuple(data.get("controlled_hands", ("left", "right")))
        data["task_presets"] = tuple(data.get("task_presets", ()))
        data["required_input_features"] = tuple(data.get("required_input_features", ()))
        data["joint_groups"] = {k: tuple(v) for k, v in data.get("joint_groups", {}).items()}
        return cls(**data)
