#!/usr/bin/env python3
"""Regenerate the native Teach golden contract from the public Python SDK."""
import json
from pathlib import Path
from horus.teaching import TeachConfig
from horus.teaching.config import INPUT_FEATURES


def input_v2():
    pose = {"position": [0.25, 0.1, 1.2], "rotation": [0, 0, 0, 1]}
    sample = dict(input_schema_version=2, source_monotonic_seconds=12.5,
                  frame_id="teach_calibrated_floor_flu", clock_id="quest_monotonic",
                  pose_time_semantics="unity_xr_latest_available", tracking_valid=True,
                  clutch=True, ui_focus=False, head=pose, head_tracking_valid=True)
    for hand in ("left", "right"):
        sample.update({hand: pose, hand + "_tracking_valid": True,
                       hand + "_stick": [.2, -.3], hand + "_stick_valid": True,
                       hand + "_buttons_valid": True, hand + "_primary": True,
                       hand + "_secondary": False, hand + "_stick_click": False,
                       hand + "_trigger": .4, hand + "_grip": .6,
                       hand + "_trigger_valid": True, hand + "_grip_valid": True})
    faults = [
        {"input_schema_version": 1}, {"input_schema_version": True},
        {"frame_id": "world"}, {"clock_id": "unix"}, {"pose_time_semantics": "capture"},
        {"source_monotonic_seconds": -1}, {"source_monotonic_seconds": True},
        {"head_tracking_valid": False}, {"left_tracking_valid": False},
        {"right_tracking_valid": False}, {"head_tracking_valid": "true"},
        {"head": {"position": [0, 0, 0], "rotation": [0, 0, 0, 0]}},
        {"head": {"position": [True, 0, 0], "rotation": [0, 0, 0, 1]}},
        {"left_stick": [1.01, 0]}, {"right_stick": [0]}, {"left_stick": [True, 0]},
        {"left_stick_valid": False}, {"right_stick_valid": False},
        {"left_buttons_valid": False}, {"right_buttons_valid": False},
        {"right_secondary": True}, {"left_primary": False}, {"right_primary": False},
        {"left_stick_click": 1}, {"left_grip_valid": False}, {"right_trigger_valid": False},
        {"left_grip": 2}, {"right_trigger": -.1}, {"clutch": False},
        {"tracking_valid": False}, {"ui_focus": True},
    ]
    return {"features": list(INPUT_FEATURES), "sample": sample,
            "invalid": [{"patch": fault} for fault in faults] +
            [{"remove": key} for key in sample]}


def build():
    profiles = [
        {"recording_root": "/tmp/horus-teach-contract"},
        {"recording_root": "/tmp/horus-teach-contract", "task": "Move the object",
         "adapter_id": "fixture", "workflow": "collection", "provenance": "fixture",
         "joint_names": ["left_joint", "right_joint"], "joint_groups": {"arms": ["left_joint", "right_joint"]},
         "command_frame": "base", "calibration_id": "calibration_v1", "publish_rate_hz": 45.0,
         "grasp_bindings": {"left_hand": "left_trigger", "right_hand": "right_trigger"},
         "streams": [{"stream_id": "front", "topic": "/front/image", "message_type": "sensor_msgs/msg/Image", "role": "camera", "source_identity": "front_device"}],
         "cameras": [{"sensor_id": "front_camera", "stream_id": "front", "required_for_control": True}],
         "task_presets": ["Pick", "Place"]},
        {"recording_root": "/tmp/horus-teach-contract", "task": "Unicode \u00e9 \U0001f680",
         "streams": [{"stream_id": "state", "topic": "/state", "message_type": "std_msgs/msg/String",
                      "calibration": {"small": 1e-7, "large": 1e16, "negative_zero": -0.0, "decimal": 1.2345678901234567}}]},
    ]
    profiles.append({"recording_root": "/tmp/horus-teach-contract", "input_schema_version": 2,
                     "required_input_features": list(INPUT_FEATURES), "clutch_binding": "both_primary",
                     "grasp_bindings": {"left_grasp": "left_grip", "right_grasp": "right_grip",
                                        "left_pinch": "left_trigger", "right_pinch": "right_trigger"}})
    return {"cases": [{"input": value, "normalized": (c := TeachConfig.from_dict(value)).to_dict(),
                       "digest": c.digest, "registration": c.registration("teach_robot")} for value in profiles],
            "input_v2": input_v2()}


if __name__ == "__main__":
    path = Path(__file__).resolve().parents[2] / "contracts/fixtures/teaching.json"
    path.write_text(json.dumps(build(), indent=2) + "\n")
