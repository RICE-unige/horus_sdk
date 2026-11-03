"""Display-only controller descriptions. These declarations never bind commands."""
from dataclasses import dataclass
from enum import Enum
import hashlib
import json
import math
import re
import uuid
import unicodedata


class ControllerControl(str, Enum):
    LEFT_PRIMARY = "left_primary"
    LEFT_SECONDARY = "left_secondary"
    LEFT_TRIGGER = "left_trigger"
    LEFT_GRIP = "left_grip"
    LEFT_STICK_X = "left_stick_x"
    LEFT_STICK_Y = "left_stick_y"
    LEFT_STICK_CLICK = "left_stick_click"
    RIGHT_PRIMARY = "right_primary"
    RIGHT_SECONDARY = "right_secondary"
    RIGHT_TRIGGER = "right_trigger"
    RIGHT_GRIP = "right_grip"
    RIGHT_STICK_X = "right_stick_x"
    RIGHT_STICK_Y = "right_stick_y"
    RIGHT_STICK_CLICK = "right_stick_click"


class TeleopHintMode(str, Enum):
    MINIMAP = "minimap"
    IMMERSIVE = "immersive"
    FOLLOW_LEADER = "follow_leader"
    TEACH = "teach"


def _identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r"[a-z][a-z0-9_]{0,63}", value):
        raise ValueError("Hint action/profile IDs must be lowercase identifiers, at most 64 characters")
    return value


def _label(value):
    if not isinstance(value, str) or any(unicodedata.category(c) == "Cc" or c in "\u2028\u2029" for c in value):
        raise ValueError("Controller labels must be single-line strings of at most 24 characters")
    try:
        if len(value.encode("utf-16-le")) > 48:
            raise ValueError("Controller labels must fit 24 UTF-16 code units")
    except UnicodeEncodeError as exc:
        raise ValueError("Controller labels must be valid Unicode") from exc
    return value.strip()


def _integer(value, field):
    if type(value) is not int or not 0 <= value <= 2_147_483_647:
        raise ValueError(field + " must be a nonnegative 32-bit integer")
    return value


@dataclass(frozen=True)
class ControllerHint:
    control: ControllerControl
    action_id: str
    label: str

    def __post_init__(self):
        object.__setattr__(self, "control", ControllerControl(self.control))
        _identifier(self.action_id)
        object.__setattr__(self, "label", _label(self.label))

    def to_dict(self):
        return dict(control=self.control.value, action_id=self.action_id, label=self.label)


@dataclass(frozen=True)
class ControllerHintProfile:
    mode: TeleopHintMode
    bindings: tuple[ControllerHint, ...]
    profile_id: str = "default"

    def __post_init__(self):
        object.__setattr__(self, "mode", TeleopHintMode(self.mode))
        object.__setattr__(self, "bindings", tuple(self.bindings))
        _identifier(self.profile_id)
        if len(self.bindings) > 14 or any(not isinstance(b, ControllerHint) for b in self.bindings):
            raise ValueError("A hint profile accepts at most 14 typed bindings")
        if len({b.control for b in self.bindings}) != len(self.bindings):
            raise ValueError("Each controller input may be described only once per profile")

    def to_dict(self):
        return dict(mode=self.mode.value, profile_id=self.profile_id, bindings=[b.to_dict() for b in self.bindings])


@dataclass(frozen=True)
class ControllerHints:
    profiles: tuple[ControllerHintProfile, ...] = ()
    state_topic: str = ""
    state_timeout_seconds: float = 2.0

    def __post_init__(self):
        object.__setattr__(self, "profiles", tuple(self.profiles))
        if len(self.profiles) > 4 or any(not isinstance(p, ControllerHintProfile) for p in self.profiles):
            raise ValueError("Choose at most four typed teleop hint profiles")
        if len({p.mode for p in self.profiles}) != len(self.profiles):
            raise ValueError("A teleop mode may have only one active hint profile")
        if not isinstance(self.state_topic, str) or len(self.state_topic) > 240 or (self.state_topic and
                not re.fullmatch(r"/(?:[a-zA-Z_][a-zA-Z0-9_]*)(?:/[a-zA-Z_][a-zA-Z0-9_]*)*", self.state_topic)):
            raise ValueError("Hint state_topic must be an absolute ROS topic or empty")
        if type(self.state_timeout_seconds) not in (int, float) or not math.isfinite(self.state_timeout_seconds) or not .5 <= self.state_timeout_seconds <= 10:
            raise ValueError("Hint state timeout must be between 0.5 and 10 seconds")
        object.__setattr__(self, "state_timeout_seconds", float(self.state_timeout_seconds))

    @property
    def context_topic(self):
        return self.state_topic + "/context" if self.state_topic else ""

    def _content(self):
        value = dict(schema_version=1, profiles=[p.to_dict() for p in self.profiles],
                     state_timeout_seconds=self.state_timeout_seconds)
        if self.state_topic:
            value["state_topic"] = self.state_topic
        return value

    @property
    def digest(self):
        return hashlib.sha256(json.dumps(self._content(), sort_keys=True, separators=(",", ":"),
                                         ensure_ascii=True).encode()).hexdigest()

    def to_dict(self):
        return dict(self._content(), config_hash=self.digest)

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or type(value.get("schema_version")) is not int or value["schema_version"] != 1:
            raise ValueError("Controller hints require schema_version 1")
        if set(value) - {"schema_version", "profiles", "state_topic", "state_timeout_seconds", "config_hash"}:
            raise ValueError("Unknown controller hint configuration field")
        try:
            result = cls(tuple(ControllerHintProfile(p["mode"], tuple(ControllerHint(**b) for b in p["bindings"]),
                p.get("profile_id", "default")) for p in value["profiles"]),
                value.get("state_topic", ""), value.get("state_timeout_seconds", 2.0))
        except (KeyError, TypeError) as exc:
            raise ValueError("Invalid controller hint configuration") from exc
        if value.get("config_hash", result.digest) != result.digest:
            raise ValueError("Controller hint configuration hash mismatch")
        return result

    def reply(self, context, *, robot_name, labels, revision):
        """Build a scoped display update; caller publishes it on state_topic."""
        if not isinstance(context, ControllerHintContext) or context.robot_name != robot_name or context.config_hash != self.digest:
            raise ValueError("Controller hint context does not match this robot/configuration")
        profile = next((p for p in self.profiles if p.mode == context.mode and p.profile_id == context.profile_id), None)
        if profile is None or not isinstance(labels, dict) or set(labels) - {b.action_id for b in profile.bindings}:
            raise ValueError("Hint updates may only describe actions declared in the active profile")
        return ControllerHintUpdate(context, _integer(revision, "revision"), tuple(
            ControllerHintLabel(_identifier(key), _label(value)) for key, value in labels.items()))


@dataclass(frozen=True)
class ControllerHintContext:
    robot_name: str
    config_hash: str
    profile_id: str
    mode: TeleopHintMode
    context_id: str
    context_sequence: int

    def __post_init__(self):
        if not isinstance(self.robot_name, str) or not self.robot_name or len(self.robot_name) > 128 or any(ord(c) < 32 for c in self.robot_name):
            raise ValueError("Invalid hint robot name")
        if not isinstance(self.config_hash, str) or not re.fullmatch("[0-9a-f]{64}", self.config_hash):
            raise ValueError("Invalid hint configuration hash")
        _identifier(self.profile_id)
        object.__setattr__(self, "mode", TeleopHintMode(self.mode))
        if not isinstance(self.context_id, str) or str(uuid.UUID(self.context_id)) != self.context_id.lower():
            raise ValueError("A canonical context UUID is required")
        _integer(self.context_sequence, "context_sequence")

    @classmethod
    def from_json(cls, payload):
        if not isinstance(payload, str) or len(payload.encode()) > 16384:
            raise ValueError("Hint context exceeds the message budget")
        value = json.loads(payload)
        if not isinstance(value, dict) or type(value.get("schema_version")) is not int or value.pop("schema_version") != 1:
            raise ValueError("Hint context requires schema_version 1")
        try:
            return cls(**value)
        except TypeError as exc:
            raise ValueError("Invalid controller hint context") from exc


@dataclass(frozen=True)
class ControllerHintLabel:
    action_id: str
    label: str

    def __post_init__(self):
        _identifier(self.action_id)
        object.__setattr__(self, "label", _label(self.label))


@dataclass(frozen=True)
class ControllerHintUpdate:
    context: ControllerHintContext
    revision: int
    labels: tuple[ControllerHintLabel, ...]

    def __post_init__(self):
        if not isinstance(self.context, ControllerHintContext):
            raise ValueError("An active typed hint context is required")
        _integer(self.revision, "revision")
        object.__setattr__(self, "labels", tuple(self.labels))
        if len(self.labels) > 14 or any(not isinstance(x, ControllerHintLabel) for x in self.labels) or len({x.action_id for x in self.labels}) != len(self.labels):
            raise ValueError("Hint updates accept at most 14 unique action labels")

    def to_dict(self):
        c = self.context
        return dict(schema_version=1, robot_name=c.robot_name, config_hash=c.config_hash,
                    profile_id=c.profile_id, mode=c.mode.value, context_id=c.context_id,
                    context_sequence=c.context_sequence, revision=self.revision,
                    labels=[dict(action_id=x.action_id, label=x.label) for x in self.labels])

    def to_json(self):
        return json.dumps(self.to_dict(), separators=(",", ":"), ensure_ascii=False)
