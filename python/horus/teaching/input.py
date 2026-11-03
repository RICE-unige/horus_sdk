"""Versioned XR input validation; robot-specific retargeting belongs to adapters."""

import math

from .adapter import validate_pose
from .config import INPUT_FEATURES


class TrackingUnavailable(ValueError):
    """Well-formed measurements with explicitly unavailable required inputs."""


def negotiate_input(config, request):
    if request.get("tracking_recovery", "manual") != config.tracking_recovery:
        raise ValueError("Teach tracking recovery mismatch; update the client or profile")
    if request.get("engagement_mode", "hold_to_run") != config.engagement_mode:
        raise ValueError("Teach engagement mode mismatch; update the client or profile")
    version = request.get("input_schema_version", 1)
    if type(version) is not int or version != config.input_schema_version:
        raise ValueError("Teach input schema mismatch; update the client or select a compatible profile")
    if version == 1:
        return []
    features = request.get("input_features")
    if (not isinstance(features, list) or
        any(not isinstance(f, str) or f not in INPUT_FEATURES for f in features) or
        len(set(features)) != len(features)):
        raise ValueError("Client must declare unique supported input_features")
    if not set(config.required_input_features) <= set(features):
        raise ValueError("Client lacks required Teach input features")
    return sorted(features)


def validate_input_v2(config, sample):
    if sample.get("tracking_recovery", "manual") != config.tracking_recovery:
        raise ValueError("Teach input tracking recovery mismatch")
    if sample.get("engagement_mode", "hold_to_run") != config.engagement_mode:
        raise ValueError("Teach input engagement mode mismatch")
    if type(sample.get("input_schema_version")) is not int or sample["input_schema_version"] != 2:
        raise ValueError("Expected Teach input schema version 2")
    if (sample.get("frame_id") != "teach_calibrated_floor_flu" or
        sample.get("clock_id") != "quest_monotonic" or
        sample.get("pose_time_semantics") != "unity_xr_latest_available"):
        raise ValueError("Unexpected Teach pose frame, clock or time semantics")
    timestamp = sample.get("source_monotonic_seconds")
    if type(timestamp) not in (int, float) or not math.isfinite(timestamp) or timestamp < 0:
        raise ValueError("Invalid XR sample timestamp")

    def flag(name):
        value = sample.get(name)
        if type(value) is not bool:
            raise ValueError("Missing boolean input field: " + name)
        return value

    def number(name, low, high):
        value = sample.get(name)
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError("Invalid measured axis: " + name)

    for name in ("tracking_valid", "clutch", "ui_focus"):
        flag(name)
    available, unavailable = {}, []
    for device in ("head", "left", "right"):
        valid = flag(device + "_tracking_valid")
        if valid:
            validate_pose(sample.get(device))
        if device in config.controlled_hands and not valid:
            unavailable.append("Controlled hand tracking unavailable: " + device)
        if device == "head":
            available["head_pose"] = valid
    for hand in ("left", "right"):
        stick = hand + "_stick"
        available[stick] = flag(stick + "_valid")
        values = sample.get(stick)
        if (not isinstance(values, (list, tuple)) or len(values) != 2 or
            any(type(v) not in (int, float) or not math.isfinite(v) or not -1 <= v <= 1 for v in values)):
            raise ValueError("Stick must contain [x-right, y-forward] within [-1, 1]")
        available[hand + "_buttons"] = flag(hand + "_buttons_valid")
        for button in ("primary", "secondary", "stick_click"):
            flag(hand + "_" + button)
        for axis in ("trigger", "grip"):
            flag(hand + "_" + axis + "_valid")
            number(hand + "_" + axis, 0, 1)
    for feature in config.required_input_features:
        if not available[feature]:
            unavailable.append("Required input feature unavailable: " + feature)
    # Hold must remain measurable even for a single-hand profile.
    hold_hand = config.hold_binding.split("_")[0]
    if sample[config.hold_binding]:
        raise ValueError("Hold button pressed")
    if not available[hold_hand + "_buttons"]:
        unavailable.append("Hold button unavailable")
    if config.engagement_mode == "continuous":
        hands = ("left", "right") if config.disengage_binding == "both_primary" else (config.disengage_binding.split("_")[0],)
        if not all(available[hand + "_buttons"] for hand in hands):
            unavailable.append("Disengage buttons unavailable")
        if all(sample[hand + "_primary"] for hand in hands):
            raise ValueError("Disengage pressed")
    else:
        clutch_hands = ("left", "right") if config.clutch_binding.startswith("both_") else (config.clutch_binding.split("_")[0],)
        for hand in clutch_hands:
            valid = sample[hand + "_grip_valid"] if config.clutch_binding.endswith("grips") or config.clutch_binding.endswith("_grip") else available[hand + "_buttons"]
            if not valid:
                raise ValueError("Clutch input unavailable")
            held = sample[hand + "_grip"] >= 0.5 if "grip" in config.clutch_binding else sample[hand + "_primary"]
            if sample.get("clutch") is True and not held:
                raise ValueError("Clutch disagrees with measured controls")
    for binding in config.grasp_bindings.values():
        if not sample[binding + "_valid"]:
            unavailable.append("Configured grasp axis unavailable: " + binding)
    if unavailable:
        raise TrackingUnavailable("; ".join(unavailable))
