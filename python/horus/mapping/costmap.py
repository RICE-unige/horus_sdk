"""Bounded costmap reconstruction. No ROS imports or geometry downsampling."""

from dataclasses import dataclass
import math
import operator

import numpy as np


# Match Nav2's published OccupancyGrid visualization encoding, not occupancy probability.
RAW_TO_OCCUPANCY = np.empty(256, dtype=np.int8)
RAW_TO_OCCUPANCY[0] = 0
RAW_TO_OCCUPANCY[1:253] = 1 + (97 * np.arange(252) // 251)
RAW_TO_OCCUPANCY[253:] = (99, 100, -1)


def integer(value, name, minimum=0):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be an integer")
    try:
        result = operator.index(value)
    except TypeError as exc:
        raise ValueError(f"{name} must be an integer") from exc
    if result < minimum:
        raise ValueError(f"{name} must be >= {minimum}")
    return result


def cells(data, count, raw):
    if len(data) != count:
        raise ValueError("costmap payload length does not match its dimensions")
    values = np.asarray(data)
    if values.ndim != 1 or values.dtype.kind not in "iu":
        raise ValueError("costmap cells must be a one-dimensional integer array")
    low, high = (0, 255) if raw else (-1, 100)
    if np.any(values < low) or np.any(values > high):
        raise ValueError(f"costmap cells must be in {low}..{high}")
    return RAW_TO_OCCUPANCY[values] if raw else values.astype(np.int8, copy=True)


@dataclass(frozen=True)
class GridGeometry:
    frame: str
    width: int
    height: int
    resolution: float
    # ROS xyz, quaternion xyzw.
    origin: tuple[float, float, float, float, float, float, float]

    def validate(self):
        if not isinstance(self.frame, str) or not self.frame.strip().lstrip("/"):
            raise ValueError("costmap frame is empty")
        width = integer(self.width, "width", 1)
        height = integer(self.height, "height", 1)
        if width > 4096 or height > 4096 or width * height > 4 * 1024 * 1024:
            raise ValueError("costmap exceeds HORUS capacity: 4096 per side, 4M cells")
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError("costmap resolution must be finite and positive")
        if len(self.origin) != 7 or not all(math.isfinite(v) for v in self.origin):
            raise ValueError("costmap origin must be a finite position and quaternion")
        if abs(math.hypot(*self.origin[3:]) - 1.0) > 1e-3:
            raise ValueError("costmap origin quaternion must have unit length")
        float_limit = float(np.finfo(np.float32).max)
        extent = max(width, height) * self.resolution
        if extent + max(abs(v) for v in self.origin[:3]) > float_limit:
            raise ValueError("costmap geometry exceeds the headset coordinate range")


class CostmapState:
    """Apply full maps and ordered patches atomically; preserve the original grid."""

    def __init__(self):
        self.geometry = None
        self.data = None
        self.stamp_ns = 0
        self.revision = 0

    def full(self, geometry, data, stamp_ns, *, raw=False):
        geometry.validate()
        stamp_ns = integer(stamp_ns, "stamp")
        decoded = cells(data, geometry.width * geometry.height, raw)
        if self.geometry is not None and stamp_ns < self.stamp_ns:
            return False
        self.geometry = geometry
        self.data = decoded.reshape(geometry.height, geometry.width)
        self.stamp_ns = stamp_ns
        self.revision += 1
        return True

    def patch(self, frame, x, y, width, height, data, stamp_ns, *, raw=False):
        if self.geometry is None:
            raise ValueError("a complete costmap snapshot is required before patches")
        stamp_ns = integer(stamp_ns, "stamp")
        if stamp_ns < self.stamp_ns:
            return False
        if frame.strip().lstrip("/") != self.geometry.frame.strip().lstrip("/"):
            raise ValueError("costmap patch frame differs from the snapshot")
        x, y = integer(x, "x"), integer(y, "y")
        width, height = integer(width, "width", 1), integer(height, "height", 1)
        if x + width > self.geometry.width or y + height > self.geometry.height:
            raise ValueError("costmap patch lies outside the snapshot")
        decoded = cells(data, width * height, raw).reshape(height, width)
        self.data[y:y + height, x:x + width] = decoded
        self.stamp_ns = stamp_ns
        self.revision += 1
        return True
