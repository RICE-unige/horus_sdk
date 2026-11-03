"""Relative Cartesian teaching for controllers with a declared target interface.

HORUS handles anchoring, frame rotation and reach/slew checks. The robot driver
still owns IK, collision constraints, balance and an independent watchdog.
Callbacks are supplied explicitly by the local SDK application, never by a
network configuration. No robot-specific topics or joint counts live here.
"""

from copy import deepcopy
from dataclasses import dataclass
import math
import time

from .adapter import TeachAdapter, validate_pose


@dataclass(frozen=True)
class CartesianLimits:
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]
    max_speed_m_s: float
    max_rotation_rad_s: float
    feedback_timeout_s: float = 0.2
    translation_scale: float = 1.0
    floor_to_command_rotation: tuple[float, float, float, float] = (0, 0, 0, 1)

    def __post_init__(self):
        validate_pose({"position": self.minimum, "rotation": self.floor_to_command_rotation})
        validate_pose({"position": self.maximum, "rotation": self.floor_to_command_rotation})
        if any(a >= b for a, b in zip(self.minimum, self.maximum)):
            raise ValueError("Invalid Cartesian workspace bounds")
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.max_speed_m_s, self.max_rotation_rad_s, self.feedback_timeout_s, self.translation_scale)):
            raise ValueError("Cartesian limits must be finite and positive")


class CartesianAdapter(TeachAdapter):
    """``feedback`` returns poses + host-monotonic receipt time + ready flag.

    ``publish`` receives one atomic target bundle and returns a receipt with
    ``boundary`` equal to the declared published/accepted/scheduled boundary.
    It must reject failure rather than claim execution. ``hold(context)`` returns
    True only for controller-confirmed hold scoped to the supplied authority.
    ``publish`` receives that authority with each target bundle. Both callbacks
    must enforce ownership and be bounded/nonblocking; a late hold must not
    override another owner's control.
    """

    def __init__(self, capabilities, limits: CartesianLimits, *, feedback, publish, hold,
                 hands=("left", "right"), tool_offsets=None):
        import numpy as np
        from scipy.spatial.transform import Rotation

        if capabilities.action_representation != "cartesian_pose_absolute":
            raise ValueError("Cartesian adapter requires an explicit absolute-pose action contract")
        if capabilities.command_boundary not in {"published", "accepted", "scheduled"}:
            raise ValueError("Declare the controller boundary actually observed")
        if capabilities.authority_contract == "unavailable":
            raise ValueError("Declare how the controller enforces authority-scoped commands and hold")
        if not hands or len(set(hands)) != len(hands) or not set(hands) <= {"left", "right"}:
            raise ValueError("Declare one or both controlled hands")
        self.capabilities, self.limits = capabilities, limits
        self.feedback, self.publish, self.hold_callback = feedback, publish, hold
        self.np, self.Rotation, self.hands = np, Rotation, tuple(hands)
        self.rotation = Rotation.from_quat(limits.floor_to_command_rotation)
        self.offsets = tool_offsets or {}
        for offset in self.offsets.values(): validate_pose(offset)
        if not set(self.offsets) <= set(hands): raise ValueError("Offset for an uncontrolled hand")
        self.anchors = None
        self.authority = None

    def _feedback(self):
        value = self.feedback()
        age = time.monotonic() - value["received_monotonic"]
        if value.get("ready") is not True or not 0 <= age <= self.limits.feedback_timeout_s:
            raise RuntimeError("Robot feedback is stale or controller is not ready")
        for hand in self.hands: validate_pose(value["poses"][hand])
        return value

    def ready(self):
        try:
            self._feedback()
            return True, ""
        except (KeyError, ValueError, TypeError, RuntimeError) as exc:
            return False, str(exc)

    def _tool(self, pose, hand):
        validate_pose(pose)
        position = self.np.asarray(pose["position"], dtype=float)
        rotation = self.Rotation.from_quat(pose["rotation"])
        offset = self.offsets.get(hand)
        if offset is not None:
            position = position + rotation.apply(offset["position"])
            rotation = rotation * self.Rotation.from_quat(offset["rotation"])
        return position, rotation

    def anchor(self, sample):
        authority = sample.get("authority")
        if not isinstance(authority, dict) or not all(isinstance(authority.get(key), str) and authority[key]
                for key in ("robot_name", "owner", "session_id", "authority_generation")):
            raise ValueError("Coordinator-assigned motion authority is required")
        measured = self._feedback()["poses"]
        self.authority = deepcopy(authority)
        self.anchors = {hand: (self._tool(sample[hand], hand), deepcopy(measured[hand])) for hand in self.hands}
        self.previous = deepcopy(measured)
        self.previous_time = time.monotonic()

    def apply(self, sample):
        self._feedback()
        if self.anchors is None: raise RuntimeError("Anchor before sending targets")
        if sample.get("authority") != self.authority:
            raise RuntimeError("Motion authority changed; hold and re-anchor")
        now = time.monotonic()
        dt = min(now - self.previous_time, self.limits.feedback_timeout_s)
        if dt <= 0: raise RuntimeError("Invalid control interval")
        targets = {}
        for hand in self.hands:
            (anchor_position, anchor_rotation), measured = self.anchors[hand]
            position, rotation = self._tool(sample[hand], hand)
            target_position = self.np.asarray(measured["position"]) + self.rotation.apply(
                position - anchor_position) * self.limits.translation_scale
            target_rotation = (self.rotation * rotation * anchor_rotation.inv() * self.rotation.inv() *
                               self.Rotation.from_quat(measured["rotation"]))
            if self.np.any(target_position < self.limits.minimum) or self.np.any(target_position > self.limits.maximum):
                raise RuntimeError("Cartesian workspace limit reached")
            delta = self.np.linalg.norm(target_position - self.np.asarray(self.previous[hand]["position"]))
            angle = (target_rotation * self.Rotation.from_quat(self.previous[hand]["rotation"]).inv()).magnitude()
            if delta > self.limits.max_speed_m_s * dt or angle > self.limits.max_rotation_rad_s * dt:
                raise RuntimeError("Cartesian speed limit reached; hold and re-anchor")
            targets[hand] = dict(position=target_position.tolist(), rotation=target_rotation.as_quat().tolist())
        bundle = dict(sequence=sample["sequence"], frame_id=self.capabilities.command_frame, targets=targets,
                      tools=sample.get("tools", {}), representation="cartesian_pose_absolute",
                      authority=deepcopy(self.authority))
        receipt = self.publish(bundle)
        if not isinstance(receipt, dict) or receipt.get("ok") is not True or receipt.get("boundary") != self.capabilities.command_boundary:
            raise RuntimeError("Controller receipt does not match the declared action boundary")
        self.previous, self.previous_time = targets, now
        return dict(bundle, receipt=receipt, boundary=self.capabilities.command_boundary)

    def hold(self, context):
        self.anchors = None
        self.authority = None
        return self.hold_callback(context) is True
