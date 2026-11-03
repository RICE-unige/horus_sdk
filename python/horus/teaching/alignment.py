"""Metric XR alignment, applied once by the session before robot adapters."""

from copy import deepcopy
import math
import uuid

from .adapter import validate_pose

TRACKING_FRAME = "quest_tracking_flu"
CALIBRATED_FRAME = "teach_calibrated_floor_flu"


def validate_alignment(value):
    if not isinstance(value, dict) or type(value.get("schema_version")) is not int or value["schema_version"] != 1:
        raise ValueError("Teach alignment schema 1 is required")
    for key in ("alignment_id", "tracking_origin_id"):
        if not isinstance(value.get(key), str):
            raise ValueError("Alignment and tracking-origin UUIDs are required")
        if str(uuid.UUID(value[key])) != value[key].lower():
            raise ValueError("Alignment and tracking-origin UUIDs must be canonical")
    if value.get("frame_id") != TRACKING_FRAME or value.get("child_frame_id") != CALIBRATED_FRAME:
        raise ValueError("Unexpected alignment frames")
    validate_pose(value)
    p, q = value["position"], value["rotation"]
    if any(abs(v) > 100 for v in p) or abs(sum(v*v for v in q) - 1) > 0.001:
        raise ValueError("Alignment must be a bounded metric rigid transform")
    # Both frames are gravity-aligned FLU; pitch/roll here would tilt control axes.
    if q[0]*q[0] + q[1]*q[1] > math.sin(math.radians(1)/2)**2:
        raise ValueError("Floor alignment must preserve gravity (yaw only)")
    return {key: deepcopy(value[key]) for key in (
        "schema_version", "alignment_id", "tracking_origin_id", "frame_id", "child_frame_id", "position", "rotation")}


def _multiply(a, b):
    x, y, z, w = a
    u, v, t, s = b
    return [w*u + x*s + y*t - z*v, w*v - x*t + y*s + z*u,
            w*t + x*v - y*u + z*s, w*s - x*u - y*v - z*t]


def map_input(sample, alignment):
    if alignment is None:
        raise ValueError("Complete alignment before engagement")
    if (sample.get("alignment_id") != alignment["alignment_id"] or
            sample.get("tracking_origin_id") != alignment["tracking_origin_id"] or
            sample.get("frame_id") != TRACKING_FRAME):
        raise ValueError("Alignment or tracking origin changed; align again")
    result = dict(sample)
    q = alignment["rotation"]
    norm = math.sqrt(sum(v*v for v in q))
    q = [v/norm for v in q]
    inv = [-q[0], -q[1], -q[2], q[3]]
    for device in ("head", "left", "right"):
        # Unity may serialize an unavailable pose as an object with empty arrays.
        # Preserve the validity flag, not an invented measurement or lost alignment.
        if sample.get("input_schema_version") == 2 and sample.get(device + "_tracking_valid") is False:
            result[device] = None
            continue
        pose = sample.get(device)
        if pose is None:
            continue
        validate_pose(pose)
        if any(abs(v) > 100 for v in pose["position"]):
            raise ValueError("XR pose exceeds metric tracking bounds")
        offset = [p-o for p, o in zip(pose["position"], alignment["position"])]
        result[device] = {"position": _multiply(_multiply(inv, offset + [0]), q)[:3],
                          "rotation": _multiply(inv, pose["rotation"])}
    result["frame_id"] = CALIBRATED_FRAME
    result["source_frame_id"] = TRACKING_FRAME
    return result
