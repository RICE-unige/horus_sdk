"""
Collision risk analyzer utilities for navigation safety DataViz.
"""

from __future__ import annotations

import json
import math
import time
from typing import Iterable, Optional, Tuple

try:
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan, PointCloud2
    from std_msgs.msg import String
except Exception:
    QoSProfile = None
    DurabilityPolicy = None
    ReliabilityPolicy = None
    LaserScan = None
    PointCloud2 = None
    String = None

try:
    from sensor_msgs_py import point_cloud2
except Exception:
    point_cloud2 = None


def clamp01(value: float) -> float:
    if value <= 0.0:
        return 0.0
    if value >= 1.0:
        return 1.0
    return float(value)


def normalize_planar(x: float, y: float) -> Tuple[float, float, float]:
    norm = math.hypot(x, y)
    if norm <= 1e-6:
        return (0.0, 0.0, 0.0)
    return (x / norm, y / norm, 0.0)


def risk_from_distance(min_distance_m: Optional[float], threshold_m: float) -> float:
    if min_distance_m is None or not math.isfinite(min_distance_m):
        return 0.0
    threshold = max(float(threshold_m), 1e-6)
    return clamp01((threshold - float(min_distance_m)) / threshold)


def analyze_laser_scan(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    min_valid_range: float,
    max_valid_range: float,
) -> Tuple[Optional[float], Tuple[float, float, float]]:
    best_distance = math.inf
    best_angle = 0.0
    angle = float(angle_min)
    for value in ranges:
        distance = float(value)
        if not math.isfinite(distance):
            angle += angle_increment
            continue
        if distance < min_valid_range or distance > max_valid_range:
            angle += angle_increment
            continue
        if distance < best_distance:
            best_distance = distance
            best_angle = angle
        angle += angle_increment

    if not math.isfinite(best_distance):
        return None, (0.0, 0.0, 0.0)

    # Direction points toward nearest observed obstacle in the sensor frame.
    direction_x = math.cos(best_angle)
    direction_y = math.sin(best_angle)
    return best_distance, normalize_planar(direction_x, direction_y)


def analyze_point_cloud(
    points_xyz: Iterable[Tuple[float, float, float]],
    z_window_m: float,
) -> Tuple[Optional[float], Tuple[float, float, float]]:
    z_window = max(float(z_window_m), 0.0)
    best_distance = math.inf
    best_direction = (0.0, 0.0, 0.0)

    for x, y, z in points_xyz:
        xf = float(x)
        yf = float(y)
        zf = float(z)
        if not (math.isfinite(xf) and math.isfinite(yf) and math.isfinite(zf)):
            continue
        if abs(zf) > z_window:
            continue
        distance = math.hypot(xf, yf)
        if distance < best_distance:
            best_distance = distance
            best_direction = normalize_planar(xf, yf)

    if not math.isfinite(best_distance):
        return None, (0.0, 0.0, 0.0)

    return best_distance, best_direction


class CollisionRiskAnalyzer:
    """
    Compute collision risk from LaserScan or PointCloud2 and publish compact JSON.

    Output topic payload:
    {
      "robot": "...",
      "frame": "...",
      "stamp_ms": ...,
      "source": "laser_scan|point_cloud",
      "threshold_m": ...,
      "min_distance_m": ...,
      "risk": ...,
      "direction": {"x":..., "y":..., "z":...}
    }
    """

    SOURCE_LASER_SCAN = "laser_scan"
    SOURCE_POINT_CLOUD = "point_cloud"

    def __init__(
        self,
        node,
        robot_name: str,
        source_type: str,
        source_topic: str,
        output_topic: Optional[str] = None,
        threshold_m: float = 1.2,
        publish_hz: float = 10.0,
        smoothing_alpha: float = 0.35,
        z_window_m: float = 0.35,
    ):
        if QoSProfile is None or String is None:
            raise RuntimeError("ROS 2 Python dependencies are required for CollisionRiskAnalyzer.")

        self.node = node
        self.robot_name = str(robot_name).strip()
        self.source_type = str(source_type).strip().lower()
        self.source_topic = str(source_topic).strip()
        self.output_topic = (
            str(output_topic).strip() if output_topic else f"/{self.robot_name}/collision_risk"
        )
        self.threshold_m = max(float(threshold_m), 1e-3)
        self.publish_hz = max(float(publish_hz), 1.0)
        self.smoothing_alpha = clamp01(float(smoothing_alpha))
        self.z_window_m = max(float(z_window_m), 0.0)

        if self.source_type not in (self.SOURCE_LASER_SCAN, self.SOURCE_POINT_CLOUD):
            raise ValueError("source_type must be 'laser_scan' or 'point_cloud'")
        if not self.source_topic:
            raise ValueError("source_topic cannot be empty")

        self._last_min_distance: Optional[float] = None
        self._last_direction = (0.0, 0.0, 0.0)
        self._last_frame = f"{self.robot_name}/base_link"
        self._smoothed_risk = 0.0

        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.node.create_publisher(String, self.output_topic, qos)

        if self.source_type == self.SOURCE_LASER_SCAN:
            if LaserScan is None:
                raise RuntimeError("sensor_msgs/LaserScan not available.")
            self._subscription = self.node.create_subscription(
                LaserScan,
                self.source_topic,
                self._on_laser_scan,
                qos,
            )
        else:
            if PointCloud2 is None:
                raise RuntimeError("sensor_msgs/PointCloud2 not available.")
            self._subscription = self.node.create_subscription(
                PointCloud2,
                self.source_topic,
                self._on_point_cloud,
                qos,
            )

        self._timer = self.node.create_timer(1.0 / self.publish_hz, self._publish_risk)

    def _on_laser_scan(self, msg: LaserScan):
        min_distance, direction = analyze_laser_scan(
            ranges=msg.ranges,
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            min_valid_range=max(float(msg.range_min), 0.0),
            max_valid_range=max(float(msg.range_max), float(msg.range_min)),
        )
        self._last_min_distance = min_distance
        self._last_direction = direction
        frame = str(msg.header.frame_id).strip()
        if frame:
            self._last_frame = frame

    def _on_point_cloud(self, msg: PointCloud2):
        if point_cloud2 is None:
            return
        points = point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        min_distance, direction = analyze_point_cloud(points, self.z_window_m)
        self._last_min_distance = min_distance
        self._last_direction = direction
        frame = str(msg.header.frame_id).strip()
        if frame:
            self._last_frame = frame

    def _publish_risk(self):
        raw_risk = risk_from_distance(self._last_min_distance, self.threshold_m)
        self._smoothed_risk = (
            (self.smoothing_alpha * raw_risk) +
            ((1.0 - self.smoothing_alpha) * self._smoothed_risk)
        )
        risk_value = clamp01(self._smoothed_risk)
        frame_id = str(self._last_frame).strip() or f"{self.robot_name}/base_link"
        dir_x = float(self._last_direction[0]) if len(self._last_direction) > 0 else 0.0
        dir_y = float(self._last_direction[1]) if len(self._last_direction) > 1 else 0.0
        if not (math.isfinite(dir_x) and math.isfinite(dir_y)):
            dir_x, dir_y = 0.0, 0.0
        direction = normalize_planar(dir_x, dir_y)
        if risk_value <= 1e-4:
            direction = (0.0, 0.0, 0.0)

        payload = {
            "robot": self.robot_name,
            "frame": frame_id,
            "stamp_ms": int(time.time() * 1000.0),
            "source": self.source_type,
            "threshold_m": round(self.threshold_m, 4),
            "min_distance_m": (
                round(float(self._last_min_distance), 4)
                if self._last_min_distance is not None and math.isfinite(self._last_min_distance)
                else None
            ),
            "risk": round(risk_value, 4),
            "direction": {
                "x": round(float(direction[0]), 4),
                "y": round(float(direction[1]), 4),
                "z": round(float(direction[2]), 4),
            },
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._publisher.publish(msg)
