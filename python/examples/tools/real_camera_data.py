"""Recorded TUM RGB-D timing and bounded measurement helpers."""

from bisect import bisect_left
from collections import deque
from pathlib import Path

import numpy as np

DATASET_URL = "https://webshare.cvg.cit.tum.de/g/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz"
DATASET_PAGE = "https://cvg.cit.tum.de/data/datasets/rgbd-dataset"
INTRINSICS = (517.3, 516.5, 318.6, 255.3)


def read_index(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        if line.strip() and not line.lstrip().startswith("#"):
            values = line.split()
            rows.append((float(values[0]), *values[1:]))
    if len(rows) < 2 or any(a[0] >= b[0] for a, b in zip(rows, rows[1:])):
        raise ValueError(f"Expected strictly increasing recorded timestamps: {path}")
    return rows


def nearest_index(times, value, tolerance=.035):
    index = bisect_left(times, value)
    choices = [i for i in (index - 1, index) if 0 <= i < len(times)]
    result = min(choices, key=lambda i: abs(times[i] - value))
    return result if abs(times[result] - value) <= tolerance else None


def loop_period(times):
    values = np.asarray(times, dtype=np.float64)
    return float(values[-1] - values[0] + np.median(np.diff(values)))


def pointcloud(depth, rgb):
    if depth.shape != rgb.shape[:2] or rgb.shape[2:] != (3,):
        raise ValueError("Registered depth and RGB dimensions do not match")
    fx, fy, cx, cy = INTRINSICS
    v, u = np.nonzero(depth)
    z = depth[v, u].astype(np.float32) / 5000
    output = np.empty(len(z), dtype=[("x", "<f4"), ("y", "<f4"),
                                    ("z", "<f4"), ("rgb", "<u4")])
    output["x"], output["y"], output["z"] = (u-cx)*z/fx, (v-cy)*z/fy, z
    colors = rgb[v, u].astype(np.uint32)
    output["rgb"] = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
    return output


def decode_depth_png(payload):
    import cv2
    image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if image is None or image.shape != (480, 640) or image.dtype != np.uint16:
        raise ValueError("Invalid or corrupted full-resolution depth PNG")
    return image


class Rate:
    def __init__(self, seconds=5):
        self.seconds = seconds
        self.times = deque()
        self.count = 0

    def tick(self, now):
        self.count += 1
        self.times.append(now)
        self.trim(now)

    def trim(self, now):
        while self.times and now-self.times[0] > self.seconds:
            self.times.popleft()

    def hz(self, now):
        self.trim(now)
        return ((len(self.times)-1)/(self.times[-1]-self.times[0])
                if len(self.times) > 1 else 0.)


class ClockEstimate:
    """NTP-style offset bounds, not a claim of synchronized hardware clocks."""

    def __init__(self):
        self.samples = deque(maxlen=32)

    def add(self, sent_ns, received_ns, source_received_ns, source_sent_ns):
        rtt = received_ns-sent_ns-(source_sent_ns-source_received_ns)
        if rtt < 0 or received_ns < sent_ns or source_sent_ns < source_received_ns:
            return False
        offset = ((source_received_ns-sent_ns)+(source_sent_ns-received_ns))/2
        self.samples.append((received_ns, offset, rtt/2))
        return True

    def best(self, now_ns):
        recent = [x for x in self.samples if 0 <= now_ns-x[0] <= 15_000_000_000]
        return min(recent, key=lambda x: x[2]) if recent else None


def summary(times):
    values = np.asarray(times, dtype=float)
    gaps = np.diff(values)
    return dict(count=len(values), fps=(len(values)-1)/(values[-1]-values[0])
                if len(values) > 1 and values[-1] > values[0] else 0.,
                p95_gap_ms=float(np.percentile(gaps, 95)*1000) if len(gaps) else None,
                max_gap_ms=float(gaps.max()*1000) if len(gaps) else None)


def window_summary(times, start, end):
    if end <= start:
        raise ValueError("Measurement window must be positive")
    values = sorted(t for t in times if start <= t <= end)
    result = summary(values)
    result["window_fps"] = len(values)/(end-start)
    gaps = np.diff([start, *values, end])
    result["max_silence_ms"] = float(gaps.max()*1000)
    result["gaps_over_250ms"] = int(np.count_nonzero(gaps > .250))
    result["gaps_over_1s"] = int(np.count_nonzero(gaps > 1.))
    result["p99_gap_ms"] = float(np.percentile(np.diff(values), 99)*1000) if len(values) > 1 else None
    return result


def payload_integrity(source, received):
    expected = {(e["kind"], e["stream"], e["publish_wall_ns"]): e.get("sha256")
                for e in source if e.get("sha256")}
    checked, mismatches = 0, []
    for e in received:
        key = (e["kind"], e["stream"].removeprefix("cam"), e["source_publish_wall_ns"])
        if e.get("sha256") and key in expected:
            checked += 1
            if e["sha256"] != expected[key]:
                mismatches.append(dict(kind=key[0], stream=key[1], stamp=key[2]))
    return dict(checked=checked, mismatches=mismatches)
