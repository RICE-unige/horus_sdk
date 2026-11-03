"""Installed robot adapter boundary. No robot connection occurs at import time."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
import math
from numbers import Real


def validate_pose(value: dict) -> None:
    if not isinstance(value, dict):
        raise ValueError("Missing controller pose")
    for key, length in (("position", 3), ("rotation", 4)):
        values = value.get(key, [])
        if not isinstance(values, (list, tuple)) or len(values) != length or not all(isinstance(v, Real) and not isinstance(v, bool) and math.isfinite(v) for v in values):
            raise ValueError("Controller pose must contain finite position and quaternion")
    norm = sum(v * v for v in value["rotation"])
    if abs(norm - 1.0) > 0.02:
        raise ValueError("Controller quaternion is not normalized")


@dataclass(frozen=True)
class AdapterCapabilities:
    adapter_id: str
    qualification: str = "observation"
    joint_names: tuple[str, ...] = ()
    command_frame: str = ""
    calibration_id: str = ""
    action_representation: str = "none"
    controller_watchdog: str = "unavailable"
    profile_revision: str = "unspecified"
    joint_units: dict[str, str] = field(default_factory=dict)
    command_boundary: str = "unavailable"
    hold_description: str = "unavailable"
    command_period_seconds: float | None = None
    authority_contract: str = "unavailable"
    supports_tracking_pause: bool = False
    supports_physical_tracking_recovery: bool = False


class TeachAdapter(ABC):
    """Robot-local adapter methods must be bounded and never perform disk IO.

    ``anchor`` anchors grip poses to fresh measured robot end effectors, without
    motion. ``apply`` retargets in the declared command frame, enforces the robot
    profile's limits and returns accepted (not presumed executed) command data.
    Samples carry a coordinator-assigned ``authority`` object. ``hold`` receives
    that same authority plus a reason, and returns True only on controller
    confirmation. Adapters must scope both targets and holds to this identity;
    an expired session must never cancel a newer controller owner's commands.
    A hardware adapter must have an independent controller-local input watchdog.

    V2 samples may include ``input_edges``, a bounded ordered list of validated
    button transitions since the previous adapter delivery. Each entry contains
    ``button``, ``pressed``, ``sequence`` and its original
    ``source_monotonic_seconds``. The containing poses and timestamp are always
    the latest accepted capture; an edge must not be treated as another motion
    sample. Adapters using discrete button actions should consume these entries
    rather than infer every transition from the latest button level. The list
    is cleared on hold/tracking pause and never crosses control authority.
    """

    capabilities: AdapterCapabilities

    @abstractmethod
    def ready(self) -> tuple[bool, str]: ...

    @abstractmethod
    def anchor(self, sample: dict) -> None: ...

    @abstractmethod
    def apply(self, sample: dict) -> dict: ...

    @abstractmethod
    def hold(self, context: dict) -> bool: ...

    def pause(self, context: dict) -> bool:
        """Confirm a non-actuating hold while retaining this authority's anchor.

        Explicitly opted-in adapters only. Clear pending targets; return True only
        after the controller confirms holding. A subsequent fresh apply resumes.
        Explicit hold must still revoke the anchor and cannot auto-resume.

        Hardware adapters must also opt into supports_physical_tracking_recovery.
        They retain their independent watchdog and scoped authority, confirm hold
        for every controlled channel, and bound resumed motion from measured hold
        state using robot-specific limits. A backend fault is not tracking loss.
        """
        raise NotImplementedError("Adapter does not support tracking pause")


class ObserveAdapter(TeachAdapter):
    capabilities = AdapterCapabilities("observe")

    def ready(self):
        return False, "Observation-only adapter; physical control is unavailable"

    def anchor(self, sample):
        raise RuntimeError("Observation-only adapter cannot engage")

    def apply(self, sample):
        raise RuntimeError("Observation-only adapter cannot send commands")

    def hold(self, context):
        return False
