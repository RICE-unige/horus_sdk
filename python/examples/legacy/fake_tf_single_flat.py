#!/usr/bin/env python3
"""Publish a flat single-robot ops example for HORUS."""

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.join(SCRIPT_DIR, "..", "..")
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half_yaw = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


@dataclass
class GoalState:
    x: float
    y: float
    yaw: float


class FlatSingleRobotPublisher(Node):
    def __init__(self, args):
        super().__init__("horus_fake_tf_single_flat")
        self.map_frame = args.map_frame
        self.base_frame = args.base_frame
        self.camera_frame = args.camera_frame
        self.lidar_frame = args.lidar_frame
        self.imu_frame = args.imu_frame
        self.rate_hz = max(1.0, float(args.rate))
        self.image_rate_hz = max(1.0, float(args.image_rate))
        self.scale = float(args.scale)
        self.teleop_timeout_s = max(0.1, float(args.teleop_timeout))
        self.goal_tolerance_m = max(0.02, float(args.goal_tolerance))
        self.goal_yaw_tolerance_rad = math.radians(max(1.0, float(args.goal_yaw_tolerance_deg)))
        self.max_linear_speed = max(0.1, float(args.max_linear_speed))
        self.max_angular_speed = max(0.1, float(args.max_angular_speed))
        self.status_frame_id = args.status_frame

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_x_cmd = 0.0
        self.angular_z_cmd = 0.0
        self.last_cmd_time = 0.0
        self.active_goal: Optional[GoalState] = None
        self.active_waypoints: List[GoalState] = []
        self.current_waypoint_index = 0

        tf_static_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        tf_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.tf_pub = self.create_publisher(TFMessage, "/tf", tf_qos)
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", tf_static_qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", status_qos)
        self.goal_status_pub = self.create_publisher(String, "/goal_status", status_qos)
        self.waypoint_status_pub = self.create_publisher(String, "/waypoint_status", status_qos)
        self.global_path_pub = self.create_publisher(Path, "/global_path", path_qos)
        self.local_path_pub = self.create_publisher(Path, "/local_path", path_qos)
        self.collision_risk_pub = self.create_publisher(String, "/collision_risk", status_qos)
        self.camera_pub = self.create_publisher(Image, "/camera/image_raw", image_qos)

        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, status_qos)
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self._on_goal_pose, status_qos)
        self.goal_cancel_sub = self.create_subscription(String, "/goal_cancel", self._on_goal_cancel, status_qos)
        self.waypoint_sub = self.create_subscription(Path, "/waypoint_path", self._on_waypoint_path, status_qos)

        self.timer = self.create_timer(1.0 / self.rate_hz, self._on_timer)
        self.image_timer = self.create_timer(1.0 / self.image_rate_hz, self._publish_camera_image)

        self._publish_static_frames()
        self.get_logger().info(
            "Flat single-robot ops example ready: "
            "teleop=/cmd_vel, goal=/goal_pose, cancel=/goal_cancel, "
            "waypoints=/waypoint_path, odom=/odom, paths=/global_path|/local_path, "
            "collision=/collision_risk, camera=/camera/image_raw"
        )

    def _publish_static_frames(self):
        now = self.get_clock().now().to_msg()
        transforms = []
        for child, offset in (
            (self.camera_frame, (0.25, 0.0, 0.20)),
            (self.lidar_frame, (0.18, 0.0, 0.16)),
            (self.imu_frame, (0.0, 0.0, 0.14)),
        ):
            msg = TransformStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.base_frame
            msg.child_frame_id = child
            msg.transform.translation.x = offset[0] * self.scale
            msg.transform.translation.y = offset[1] * self.scale
            msg.transform.translation.z = offset[2] * self.scale
            qx, qy, qz, qw = quaternion_from_yaw(0.0)
            msg.transform.rotation.x = qx
            msg.transform.rotation.y = qy
            msg.transform.rotation.z = qz
            msg.transform.rotation.w = qw
            transforms.append(msg)

        self.tf_static_pub.publish(TFMessage(transforms=transforms))

    def _on_cmd_vel(self, msg: Twist):
        self.linear_x_cmd = clamp(float(msg.linear.x), -self.max_linear_speed, self.max_linear_speed)
        self.angular_z_cmd = clamp(float(msg.angular.z), -self.max_angular_speed, self.max_angular_speed)
        self.last_cmd_time = time.time()

    def _on_goal_pose(self, msg: PoseStamped):
        yaw = self._yaw_from_pose(msg)
        self.active_goal = GoalState(
            x=float(msg.pose.position.x) / self.scale,
            y=float(msg.pose.position.y) / self.scale,
            yaw=yaw,
        )
        self.active_waypoints = []
        self.current_waypoint_index = 0
        self._publish_goal_status("goal_sent", self.active_goal)
        self._publish_paths()

    def _on_goal_cancel(self, msg: String):
        payload = str(msg.data or "").strip().lower()
        if payload and payload not in ("cancel", "goal_cancel", "stop"):
            return

        if self.active_goal is not None:
            cancelled_goal = self.active_goal
            self.active_goal = None
            self._publish_goal_status("goal_cancelled", cancelled_goal)
        if self.active_waypoints:
            self.active_waypoints = []
            self.current_waypoint_index = 0
            self._publish_waypoint_status("path_cancelled")
        self._publish_paths()

    def _on_waypoint_path(self, msg: Path):
        waypoints: List[GoalState] = []
        for pose_stamped in msg.poses:
            waypoints.append(
                GoalState(
                    x=float(pose_stamped.pose.position.x) / self.scale,
                    y=float(pose_stamped.pose.position.y) / self.scale,
                    yaw=self._yaw_from_pose(pose_stamped),
                )
            )

        self.active_goal = None
        self.active_waypoints = waypoints
        self.current_waypoint_index = 0
        if self.active_waypoints:
            self._publish_waypoint_status("path_sent")
        else:
            self._publish_waypoint_status("path_failed")
        self._publish_paths()

    def _yaw_from_pose(self, msg: PoseStamped) -> float:
        z = float(msg.pose.orientation.z)
        w = float(msg.pose.orientation.w)
        return math.atan2(2.0 * z * w, 1.0 - 2.0 * z * z)

    def _is_teleop_active(self) -> bool:
        return (time.time() - self.last_cmd_time) <= self.teleop_timeout_s

    def _step_towards_target(self, target: GoalState, dt: float) -> Tuple[bool, float, float]:
        dx = target.x - self.x
        dy = target.y - self.y
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx) if distance > 1e-6 else target.yaw
        heading_error = self._normalize_angle(target_heading - self.yaw)

        linear_speed = clamp(distance * 1.2, 0.0, self.max_linear_speed)
        angular_speed = clamp(heading_error * 2.0, -self.max_angular_speed, self.max_angular_speed)

        if abs(heading_error) > 1.2:
            linear_speed *= 0.2
        elif abs(heading_error) > 0.6:
            linear_speed *= 0.5

        self.yaw = self._normalize_angle(self.yaw + angular_speed * dt)
        self.x += math.cos(self.yaw) * linear_speed * dt
        self.y += math.sin(self.yaw) * linear_speed * dt

        yaw_error = abs(self._normalize_angle(target.yaw - self.yaw))
        reached = distance <= self.goal_tolerance_m and yaw_error <= self.goal_yaw_tolerance_rad
        return reached, linear_speed, angular_speed

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _publish_goal_status(self, state: str, goal: Optional[GoalState]):
        payload = {
            "robot_name": "robot",
            "state": state,
            "status": state,
            "frame_id": self.status_frame_id,
            "ts_unix_ms": int(time.time() * 1000.0),
        }
        if goal is not None:
            payload["goal_pose"] = {
                "x": round(goal.x * self.scale, 4),
                "y": round(goal.y * self.scale, 4),
                "yaw": round(goal.yaw, 4),
            }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.goal_status_pub.publish(msg)

    def _publish_waypoint_status(self, state: str):
        payload = {
            "robot_name": "robot",
            "state": state,
            "status": state,
            "current_index": int(self.current_waypoint_index),
            "total": int(len(self.active_waypoints)),
            "frame_id": self.status_frame_id,
            "ts_unix_ms": int(time.time() * 1000.0),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.waypoint_status_pub.publish(msg)

    def _publish_paths(self):
        now = self.get_clock().now().to_msg()

        def build_path(points: List[GoalState]) -> Path:
            path = Path()
            path.header.stamp = now
            path.header.frame_id = self.map_frame
            for point in points:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = point.x * self.scale
                pose.pose.position.y = point.y * self.scale
                qx, qy, qz, qw = quaternion_from_yaw(point.yaw)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                path.poses.append(pose)
            return path

        global_points: List[GoalState] = []
        if self.active_goal is not None:
            global_points = [GoalState(self.x, self.y, self.yaw), self.active_goal]
        elif self.active_waypoints:
            global_points = [GoalState(self.x, self.y, self.yaw)] + self.active_waypoints[self.current_waypoint_index :]

        local_points = global_points[: min(2, len(global_points))]
        self.global_path_pub.publish(build_path(global_points))
        self.local_path_pub.publish(build_path(local_points))

    def _publish_camera_image(self):
        msg = Image()
        now = self.get_clock().now().to_msg()
        width = 96
        height = 54
        data = bytearray(width * height * 3)
        for y in range(height):
            for x in range(width):
                idx = (y * width + x) * 3
                data[idx] = int((x / max(1, width - 1)) * 255)
                data[idx + 1] = int((y / max(1, height - 1)) * 255)
                data[idx + 2] = 160
        msg.header.stamp = now
        msg.header.frame_id = self.camera_frame
        msg.height = height
        msg.width = width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = bytes(data)
        self.camera_pub.publish(msg)

    def _publish_tf(self, stamp_msg):
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        tf = TransformStamped()
        tf.header.stamp = stamp_msg
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x * self.scale
        tf.transform.translation.y = self.y * self.scale
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_pub.publish(TFMessage(transforms=[tf]))

    def _publish_odometry(self, stamp_msg, linear_speed: float, angular_speed: float):
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        msg = Odometry()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x * self.scale
        msg.pose.pose.position.y = self.y * self.scale
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = linear_speed * self.scale
        msg.twist.twist.angular.z = angular_speed
        self.odom_pub.publish(msg)

    def _publish_collision_risk(self, linear_speed: float):
        speed_ratio = abs(linear_speed) / max(0.001, self.max_linear_speed)
        risk = round(clamp(0.15 + (0.65 * speed_ratio), 0.0, 1.0), 4)
        payload = {
            "robot": "robot",
            "frame": self.base_frame,
            "stamp_ms": int(time.time() * 1000.0),
            "source": "fake_single_flat",
            "threshold_m": 1.0,
            "min_distance_m": round(max(0.2, 2.0 - speed_ratio), 4),
            "risk": risk,
            "direction": {"x": 1.0, "y": 0.0, "z": 0.0},
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.collision_risk_pub.publish(msg)

    def _on_timer(self):
        dt = 1.0 / self.rate_hz
        linear_speed = 0.0
        angular_speed = 0.0

        if self._is_teleop_active():
            linear_speed = self.linear_x_cmd
            angular_speed = self.angular_z_cmd
            self.yaw = self._normalize_angle(self.yaw + angular_speed * dt)
            self.x += math.cos(self.yaw) * linear_speed * dt
            self.y += math.sin(self.yaw) * linear_speed * dt
        elif self.active_waypoints:
            target = self.active_waypoints[self.current_waypoint_index]
            reached, linear_speed, angular_speed = self._step_towards_target(target, dt)
            if reached:
                self.current_waypoint_index += 1
                self._publish_waypoint_status("waypoint_reached")
                if self.current_waypoint_index >= len(self.active_waypoints):
                    self.active_waypoints = []
                    self.current_waypoint_index = 0
                    self._publish_waypoint_status("path_completed")
                self._publish_paths()
        elif self.active_goal is not None:
            reached, linear_speed, angular_speed = self._step_towards_target(self.active_goal, dt)
            if reached:
                goal = self.active_goal
                self.active_goal = None
                self._publish_goal_status("goal_reached", goal)
                self._publish_paths()

        stamp_msg = self.get_clock().now().to_msg()
        self._publish_tf(stamp_msg)
        self._publish_odometry(stamp_msg, linear_speed, angular_speed)
        self._publish_collision_risk(linear_speed)


def build_parser():
    parser = argparse.ArgumentParser(description="Publish a flat single-robot ops example for HORUS.")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--camera-frame", default="camera_link")
    parser.add_argument("--lidar-frame", default="lidar_link")
    parser.add_argument("--imu-frame", default="imu_link")
    parser.add_argument("--status-frame", default="map")
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--image-rate", type=float, default=6.0)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--teleop-timeout", type=float, default=0.35)
    parser.add_argument("--goal-tolerance", type=float, default=0.20)
    parser.add_argument("--goal-yaw-tolerance-deg", type=float, default=12.0)
    parser.add_argument("--max-linear-speed", type=float, default=0.8)
    parser.add_argument("--max-angular-speed", type=float, default=1.2)
    return parser


def main():
    args = build_parser().parse_args()
    rclpy.init()
    node = FlatSingleRobotPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
