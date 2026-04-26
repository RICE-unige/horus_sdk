#!/usr/bin/env python3
"""Unified fake TF runtime for teleop + go-to-point + waypoint validation."""

import json
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

# Ensure local example imports resolve when script is run from arbitrary CWD.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry, Path
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    sys.exit(1)

from fake_tf_publisher import parse_resolution_list, parse_robot_base_frames
from fake_tf_teleop_common import TeleopDrivenFakeTFPublisher, build_teleop_parser


DEFAULT_ROBOT_NAMES = [
    "atlas",
    "nova",
    "orion",
    "luna",
    "phoenix",
    "rover",
    "zephyr",
    "aurora",
    "titan",
    "vega",
]


def _normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


def _is_planar_yaw_quaternion(x: float, y: float, tol: float = 1e-3) -> bool:
    return abs(x) <= tol and abs(y) <= tol


@dataclass
class GoalState:
    target_x: float
    target_y: float
    target_yaw: float


@dataclass
class WaypointGoal:
    target_x: float
    target_y: float
    target_yaw: float


class OpsSuiteFakeTFPublisher(TeleopDrivenFakeTFPublisher):
    """Fake TF publisher where robots move only from teleop or task commands."""

    def __init__(
        self,
        command_timeout_s: float,
        goal_position_tolerance_m: float,
        goal_yaw_tolerance_deg: float,
        waypoint_position_tolerance_m: float,
        waypoint_yaw_tolerance_deg: float,
        max_turn_rate_rps: float,
        task_path_publish_rate_hz: float,
        publish_collision_risk: bool,
        collision_threshold_m: float,
        collision_risk_rate_hz: float,
        status_frame_id: str,
        *args,
        **kwargs,
    ):
        self.strict_single_tf_publisher = bool(kwargs.pop("strict_single_tf_publisher", False))
        self._tf_conflict_warned = False
        super().__init__(command_timeout_s=command_timeout_s, *args, **kwargs)
        self.mode = "ops_suite"
        self.goal_position_tolerance_m = max(0.01, float(goal_position_tolerance_m))
        self.goal_yaw_tolerance_rad = math.radians(max(0.1, float(goal_yaw_tolerance_deg)))
        self.waypoint_position_tolerance_m = max(0.01, float(waypoint_position_tolerance_m))
        self.waypoint_yaw_tolerance_rad = math.radians(max(0.1, float(waypoint_yaw_tolerance_deg)))
        self.max_turn_rate_rps = max(0.1, float(max_turn_rate_rps))
        self.task_path_publish_rate_hz = max(0.2, float(task_path_publish_rate_hz))
        self._task_path_publish_period_s = 1.0 / self.task_path_publish_rate_hz
        self.publish_collision_risk = bool(publish_collision_risk)
        self.collision_threshold_m = max(0.1, float(collision_threshold_m))
        self.collision_risk_rate_hz = max(0.2, float(collision_risk_rate_hz))
        self._collision_risk_publish_period_s = 1.0 / self.collision_risk_rate_hz
        self.status_frame_id = status_frame_id.strip() if status_frame_id else "map"
        self._map_scale = self.scale if abs(self.scale) > 1e-6 else 1.0
        # Approximate robot footprint radius in published map meters.
        self._collision_body_radius_m = 0.18 * abs(self._map_scale)

        self._goals: Dict[str, Optional[GoalState]] = {robot.name: None for robot in self.robots}
        self._path_queues: Dict[str, List[WaypointGoal]] = {robot.name: [] for robot in self.robots}
        self._active_path_index: Dict[str, int] = {robot.name: -1 for robot in self.robots}
        self._goal_status_publishers = {}
        self._waypoint_status_publishers = {}
        self._global_path_publishers = {}
        self._local_path_publishers = {}
        self._odom_publishers = {}
        self._collision_risk_publishers = {}
        self._last_task_path_publish_time: Dict[str, float] = {robot.name: 0.0 for robot in self.robots}
        self._last_collision_risk_publish_time: Dict[str, float] = {robot.name: 0.0 for robot in self.robots}
        self._goal_subscriptions = []
        self._cancel_subscriptions = []
        self._path_subscriptions = []
        self._non_planar_goal_warned = set()
        self._non_planar_waypoint_warned = set()

        command_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        odom_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        for robot in self.robots:
            name = robot.name
            self._goal_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    f"/{name}/goal_pose",
                    self._make_goal_callback(name),
                    command_qos,
                )
            )
            self._cancel_subscriptions.append(
                self.create_subscription(
                    String,
                    f"/{name}/goal_cancel",
                    self._make_goal_cancel_callback(name),
                    command_qos,
                )
            )
            self._path_subscriptions.append(
                self.create_subscription(
                    Path,
                    f"/{name}/waypoint_path",
                    self._make_waypoint_path_callback(name),
                    command_qos,
                )
            )
            self._goal_status_publishers[name] = self.create_publisher(
                String, f"/{name}/goal_status", command_qos
            )
            self._waypoint_status_publishers[name] = self.create_publisher(
                String, f"/{name}/waypoint_status", command_qos
            )
            self._global_path_publishers[name] = self.create_publisher(
                Path, f"/{name}/global_path", path_qos
            )
            self._local_path_publishers[name] = self.create_publisher(
                Path, f"/{name}/local_path", path_qos
            )
            self._odom_publishers[name] = self.create_publisher(
                Odometry, f"/{name}/odom", odom_qos
            )
            if self.publish_collision_risk:
                self._collision_risk_publishers[name] = self.create_publisher(
                    String, f"/{name}/collision_risk", command_qos
                )
            robot.vx = 0.0
            robot.vy = 0.0
            robot.target_x = robot.x
            robot.target_y = robot.y

        self.get_logger().info(
            "Ops suite mode active. Robots stay static until cmd_vel, goal_pose, or waypoint_path is received. "
            "Teleop cmd_vel overrides and cancels active tasks. "
            "Fake nav paths are published on /<robot>/global_path and /<robot>/local_path. "
            "Odometry is published on /<robot>/odom."
        )
        if self.publish_collision_risk:
            self.get_logger().info(
                f"Fake collision risk is published on /<robot>/collision_risk at "
                f"{self.collision_risk_rate_hz:.1f}Hz (threshold={self.collision_threshold_m:.2f}m)."
            )
        if self.strict_single_tf_publisher:
            self.get_logger().info(
                "Strict TF source mode enabled: this node will stop if multiple /tf publishers are detected."
            )

    def _enforce_single_tf_publisher(self) -> bool:
        if not self.strict_single_tf_publisher:
            return True

        infos = self.get_publishers_info_by_topic("/tf")
        if len(infos) <= 1:
            return True

        if not self._tf_conflict_warned:
            publishers = []
            for info in infos:
                namespace = str(getattr(info, "node_namespace", "") or "")
                node_name = str(getattr(info, "node_name", "") or "unknown")
                publishers.append(f"{namespace}/{node_name}".replace("//", "/"))
            self.get_logger().error(
                "Detected multiple /tf publishers; this causes frame jumping. "
                f"Publishers={publishers}. "
                "Stop other TF publishers and restart this demo."
            )
            self._tf_conflict_warned = True

        # Stop this demo node so the operator can resolve the conflicting source.
        try:
            self.destroy_timer(self.timer)
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        return False

    def _make_goal_callback(self, robot_name):
        def _callback(msg: PoseStamped):
            target_x = float(msg.pose.position.x) / self._map_scale
            target_y = float(msg.pose.position.y) / self._map_scale
            qx = float(msg.pose.orientation.x)
            qy = float(msg.pose.orientation.y)
            qz = float(msg.pose.orientation.z)
            qw = float(msg.pose.orientation.w)

            if robot_name not in self._non_planar_goal_warned and not _is_planar_yaw_quaternion(qx, qy):
                self.get_logger().warning(
                    f"Non-planar goal orientation for {robot_name}: qx={qx:.4f}, qy={qy:.4f}, qz={qz:.4f}, qw={qw:.4f}. "
                    "Yaw extraction will ignore roll/pitch terms."
                )
                self._non_planar_goal_warned.add(robot_name)

            target_yaw = _yaw_from_quaternion(
                qx,
                qy,
                qz,
                qw,
            )

            target_x, target_y = self._nearest_free_world_position(target_x, target_y)
            goal = GoalState(
                target_x=target_x,
                target_y=target_y,
                target_yaw=_normalize_angle(target_yaw),
            )

            self._cancel_waypoint_path(robot_name, reason="path_cancelled")
            self._goals[robot_name] = goal
            robot = self._robot_by_name(robot_name)
            if robot is not None:
                robot.target_x = goal.target_x
                robot.target_y = goal.target_y
            self._publish_goal_status(robot_name, "goal_sent", goal)
            self._publish_task_nav_paths(robot_name)

        return _callback

    def _make_goal_cancel_callback(self, robot_name):
        def _callback(_msg: String):
            self._cancel_goal(robot_name)

        return _callback

    def _make_waypoint_path_callback(self, robot_name):
        def _callback(msg: Path):
            if msg is None or not msg.poses:
                self._path_queues[robot_name] = []
                self._active_path_index[robot_name] = -1
                self._publish_waypoint_status(robot_name, "path_failed", -1, 0)
                self._publish_empty_nav_paths(robot_name)
                return

            queue: List[WaypointGoal] = []
            for pose in msg.poses:
                target_x = float(pose.pose.position.x) / self._map_scale
                target_y = float(pose.pose.position.y) / self._map_scale
                qx = float(pose.pose.orientation.x)
                qy = float(pose.pose.orientation.y)
                qz = float(pose.pose.orientation.z)
                qw = float(pose.pose.orientation.w)

                if robot_name not in self._non_planar_waypoint_warned and not _is_planar_yaw_quaternion(qx, qy):
                    self.get_logger().warning(
                        f"Non-planar waypoint orientation for {robot_name}: qx={qx:.4f}, qy={qy:.4f}, qz={qz:.4f}, qw={qw:.4f}. "
                        "Yaw extraction will ignore roll/pitch terms."
                    )
                    self._non_planar_waypoint_warned.add(robot_name)

                target_yaw = _yaw_from_quaternion(
                    qx,
                    qy,
                    qz,
                    qw,
                )
                target_x, target_y = self._nearest_free_world_position(target_x, target_y)
                queue.append(
                    WaypointGoal(
                        target_x=target_x,
                        target_y=target_y,
                        target_yaw=_normalize_angle(target_yaw),
                    )
                )

            self._cancel_goal(robot_name, publish_status=True)
            self._path_queues[robot_name] = queue
            self._active_path_index[robot_name] = 0
            self._publish_waypoint_status(robot_name, "path_sent", 0, len(queue))
            self._publish_task_nav_paths(robot_name)

        return _callback

    def _robot_by_name(self, robot_name: str):
        for robot in self.robots:
            if robot.name == robot_name:
                return robot
        return None

    def _cancel_goal(self, robot_name: str, publish_status: bool = True):
        goal = self._goals.get(robot_name)
        self._goals[robot_name] = None
        robot = self._robot_by_name(robot_name)
        if robot is not None:
            robot.vx = 0.0
            robot.vy = 0.0
            robot.target_x = robot.x
            robot.target_y = robot.y
        if publish_status and goal is not None:
            self._publish_goal_status(robot_name, "goal_cancelled", goal)
        if goal is not None and not self._path_queues.get(robot_name):
            self._publish_empty_nav_paths(robot_name)

    def _cancel_waypoint_path(self, robot_name: str, reason: str = "path_cancelled"):
        queue = self._path_queues.get(robot_name, [])
        active_index = self._active_path_index.get(robot_name, -1)
        if queue and active_index >= 0:
            self._publish_waypoint_status(
                robot_name,
                reason,
                active_index,
                len(queue),
            )
        self._path_queues[robot_name] = []
        self._active_path_index[robot_name] = -1
        if queue and self._goals.get(robot_name) is None:
            self._publish_empty_nav_paths(robot_name)

    def _cancel_tasks_for_teleop(self, robot_name: str):
        self._cancel_goal(robot_name, publish_status=True)
        self._cancel_waypoint_path(robot_name, reason="path_cancelled")

    def _publish_goal_status(self, robot_name: str, state: str, goal: Optional[GoalState]):
        publisher = self._goal_status_publishers.get(robot_name)
        if publisher is None:
            return

        payload = {
            "robot_name": robot_name,
            "status": state,
            "state": state,
            "frame_id": self.status_frame_id,
            "ts_unix_ms": int(time.time() * 1000.0),
        }
        if goal is not None:
            payload["goal_pose"] = {
                "x": round(goal.target_x * self._map_scale, 4),
                "y": round(goal.target_y * self._map_scale, 4),
                "yaw": round(goal.target_yaw, 4),
            }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        publisher.publish(msg)

    def _publish_waypoint_status(self, robot_name: str, state: str, current_index: int, total: int):
        publisher = self._waypoint_status_publishers.get(robot_name)
        if publisher is None:
            return

        payload = {
            "robot_name": robot_name,
            "state": state,
            "status": state,
            "current_index": int(current_index),
            "total": int(max(0, total)),
            "frame_id": self.status_frame_id,
            "ts_unix_ms": int(time.time() * 1000.0),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        publisher.publish(msg)

    def _build_pose_stamped(self, x_world: float, y_world: float, yaw: float, stamp_msg) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = stamp_msg
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x_world * self._map_scale
        pose.pose.position.y = y_world * self._map_scale
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = self._quat_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _publish_path_points(self, publisher, points: List[Tuple[float, float, float]], stamp_msg):
        if publisher is None:
            return
        path = Path()
        path.header.stamp = stamp_msg
        path.header.frame_id = self.map_frame
        for x, y, yaw in points:
            path.poses.append(self._build_pose_stamped(x, y, yaw, stamp_msg))
        publisher.publish(path)

    def _publish_empty_nav_paths(self, robot_name: str):
        stamp_msg = self.get_clock().now().to_msg()
        self._publish_path_points(self._global_path_publishers.get(robot_name), [], stamp_msg)
        self._publish_path_points(self._local_path_publishers.get(robot_name), [], stamp_msg)
        self._last_task_path_publish_time[robot_name] = time.time()

    @staticmethod
    def _append_line_points(
        points: List[Tuple[float, float, float]],
        start_x: float,
        start_y: float,
        end_x: float,
        end_y: float,
        end_yaw: float,
        include_start: bool = False,
        max_step: float = 0.35,
    ):
        dx = end_x - start_x
        dy = end_y - start_y
        distance = math.hypot(dx, dy)
        heading = end_yaw if distance <= 1e-6 else math.atan2(dy, dx)
        steps = max(1, int(math.ceil(distance / max(0.05, max_step))))

        if include_start and not points:
            points.append((start_x, start_y, heading))

        for i in range(1, steps + 1):
            alpha = float(i) / float(steps)
            x = start_x + (dx * alpha)
            y = start_y + (dy * alpha)
            yaw = end_yaw if i == steps else heading
            points.append((x, y, yaw))

    def _build_goal_global_path_points(self, robot, goal: GoalState) -> List[Tuple[float, float, float]]:
        points: List[Tuple[float, float, float]] = [(robot.x, robot.y, robot.yaw)]
        self._append_line_points(
            points,
            robot.x,
            robot.y,
            goal.target_x,
            goal.target_y,
            goal.target_yaw,
            include_start=False,
        )
        return points

    def _build_goal_local_path_points(self, robot, goal: GoalState) -> List[Tuple[float, float, float]]:
        dx = goal.target_x - robot.x
        dy = goal.target_y - robot.y
        distance = math.hypot(dx, dy)
        if distance <= 1e-6:
            return [(robot.x, robot.y, robot.yaw), (goal.target_x, goal.target_y, goal.target_yaw)]

        lookahead = min(distance, 1.5)
        ux = dx / distance
        uy = dy / distance
        mid_x = robot.x + (ux * lookahead)
        mid_y = robot.y + (uy * lookahead)
        heading = math.atan2(dy, dx)

        points: List[Tuple[float, float, float]] = [(robot.x, robot.y, robot.yaw)]
        self._append_line_points(points, robot.x, robot.y, mid_x, mid_y, heading)
        if lookahead < distance:
            # Add a short continuation hint toward the final target as a fake local planner preview.
            tail_x = robot.x + (ux * min(distance, lookahead + 0.8))
            tail_y = robot.y + (uy * min(distance, lookahead + 0.8))
            self._append_line_points(points, mid_x, mid_y, tail_x, tail_y, heading)
        return points

    def _build_waypoint_global_path_points(
        self,
        robot,
        queue: List[WaypointGoal],
        active_index: int,
    ) -> List[Tuple[float, float, float]]:
        points: List[Tuple[float, float, float]] = [(robot.x, robot.y, robot.yaw)]
        prev_x = robot.x
        prev_y = robot.y
        for idx in range(max(0, active_index), len(queue)):
            wp = queue[idx]
            self._append_line_points(points, prev_x, prev_y, wp.target_x, wp.target_y, wp.target_yaw)
            prev_x = wp.target_x
            prev_y = wp.target_y
        return points

    def _build_waypoint_local_path_points(
        self,
        robot,
        queue: List[WaypointGoal],
        active_index: int,
    ) -> List[Tuple[float, float, float]]:
        points: List[Tuple[float, float, float]] = [(robot.x, robot.y, robot.yaw)]
        if not queue or active_index < 0 or active_index >= len(queue):
            return points

        remaining_budget = 2.0
        prev_x = robot.x
        prev_y = robot.y
        for idx in range(active_index, len(queue)):
            wp = queue[idx]
            dx = wp.target_x - prev_x
            dy = wp.target_y - prev_y
            segment_len = math.hypot(dx, dy)
            if segment_len <= 1e-6:
                prev_x = wp.target_x
                prev_y = wp.target_y
                continue

            if segment_len <= remaining_budget + 1e-6:
                self._append_line_points(points, prev_x, prev_y, wp.target_x, wp.target_y, wp.target_yaw)
                remaining_budget -= segment_len
                prev_x = wp.target_x
                prev_y = wp.target_y
                if remaining_budget <= 1e-3:
                    break
                continue

            ux = dx / segment_len
            uy = dy / segment_len
            end_x = prev_x + (ux * remaining_budget)
            end_y = prev_y + (uy * remaining_budget)
            heading = math.atan2(dy, dx)
            self._append_line_points(points, prev_x, prev_y, end_x, end_y, heading)
            break

        return points

    def _publish_task_nav_paths(self, robot_name: str):
        robot = self._robot_by_name(robot_name)
        if robot is None:
            return

        goal = self._goals.get(robot_name)
        queue = self._path_queues.get(robot_name, [])
        active_index = self._active_path_index.get(robot_name, -1)
        stamp_msg = self.get_clock().now().to_msg()

        if goal is not None:
            global_points = self._build_goal_global_path_points(robot, goal)
            local_points = self._build_goal_local_path_points(robot, goal)
            self._publish_path_points(self._global_path_publishers.get(robot_name), global_points, stamp_msg)
            self._publish_path_points(self._local_path_publishers.get(robot_name), local_points, stamp_msg)
            self._last_task_path_publish_time[robot_name] = time.time()
            return

        if queue and 0 <= active_index < len(queue):
            global_points = self._build_waypoint_global_path_points(robot, queue, active_index)
            local_points = self._build_waypoint_local_path_points(robot, queue, active_index)
            self._publish_path_points(self._global_path_publishers.get(robot_name), global_points, stamp_msg)
            self._publish_path_points(self._local_path_publishers.get(robot_name), local_points, stamp_msg)
            self._last_task_path_publish_time[robot_name] = time.time()
            return

        self._publish_empty_nav_paths(robot_name)

    def _maybe_publish_task_nav_paths(self, robot_name: str, now_sec: float):
        last = float(self._last_task_path_publish_time.get(robot_name, 0.0))
        if (now_sec - last) < self._task_path_publish_period_s:
            return
        if self._goals.get(robot_name) is None:
            queue = self._path_queues.get(robot_name, [])
            active_index = self._active_path_index.get(robot_name, -1)
            if not queue or active_index < 0 or active_index >= len(queue):
                return
        self._publish_task_nav_paths(robot_name)

    def _is_teleop_active(self, robot_name: str, now_sec: float) -> bool:
        state = self._cmd_state.get(robot_name)
        if not state:
            return False
        stamp = float(state.get("timestamp", 0.0))
        if stamp <= 0.0:
            return False
        return (now_sec - stamp) <= self.command_timeout_s

    def _apply_teleop_to_robot(self, robot, dt: float):
        state = self._cmd_state.get(robot.name, {})

        linear_x = float(state.get("linear_x", 0.0))
        linear_y = float(state.get("linear_y", 0.0))
        linear_z = float(state.get("linear_z", 0.0))
        angular_z = float(state.get("angular_z", 0.0))

        if abs(linear_x) < self.linear_deadband_mps:
            linear_x = 0.0
        if abs(linear_y) < self.linear_deadband_mps:
            linear_y = 0.0
        if abs(linear_z) < self.linear_deadband_mps:
            linear_z = 0.0
        if abs(angular_z) < self.angular_deadband_rps:
            angular_z = 0.0

        planar_speed = math.hypot(linear_x, linear_y)
        if planar_speed > self.max_speed and planar_speed > 1e-6:
            ratio = self.max_speed / planar_speed
            linear_x *= ratio
            linear_y *= ratio

        robot.yaw = _normalize_angle(robot.yaw + (angular_z * dt))
        world_vx = (math.cos(robot.yaw) * linear_x) - (math.sin(robot.yaw) * linear_y)
        world_vy = (math.sin(robot.yaw) * linear_x) + (math.cos(robot.yaw) * linear_y)

        robot.vx = world_vx
        robot.vy = world_vy
        robot.x += world_vx * dt
        robot.y += world_vy * dt
        robot.target_x = robot.x
        robot.target_y = robot.y

        altitude = self._robot_altitudes.get(robot.name, self.height)
        altitude += linear_z * dt
        altitude = max(-2.0, min(4.0, altitude))
        self._robot_altitudes[robot.name] = altitude

    def _set_robot_idle(self, robot):
        robot.vx = 0.0
        robot.vy = 0.0
        robot.target_x = robot.x
        robot.target_y = robot.y

    def _advance_goal(self, robot, goal: GoalState, dt: float) -> bool:
        dx = goal.target_x - robot.x
        dy = goal.target_y - robot.y
        distance = math.hypot(dx, dy)

        desired_heading = goal.target_yaw
        if distance > self.goal_position_tolerance_m:
            desired_heading = math.atan2(dy, dx)

        yaw_error = _normalize_angle(desired_heading - robot.yaw)
        max_turn = self.max_turn_rate_rps * dt
        yaw_step = max(-max_turn, min(max_turn, yaw_error))
        robot.yaw = _normalize_angle(robot.yaw + yaw_step)

        if distance > self.goal_position_tolerance_m:
            target_speed = min(self.max_speed, max(0.15, distance * 1.5))
            robot.vx = math.cos(robot.yaw) * target_speed
            robot.vy = math.sin(robot.yaw) * target_speed
            robot.x += robot.vx * dt
            robot.y += robot.vy * dt
        else:
            snap = min(1.0, dt * 8.0)
            robot.x += dx * snap
            robot.y += dy * snap
            robot.vx = 0.0
            robot.vy = 0.0

        remaining_distance = math.hypot(goal.target_x - robot.x, goal.target_y - robot.y)
        remaining_yaw = abs(_normalize_angle(goal.target_yaw - robot.yaw))
        if remaining_distance <= self.goal_position_tolerance_m and remaining_yaw <= self.goal_yaw_tolerance_rad:
            robot.x = goal.target_x
            robot.y = goal.target_y
            robot.yaw = goal.target_yaw
            robot.vx = 0.0
            robot.vy = 0.0
            return True

        return False

    def _advance_waypoint(
        self,
        robot,
        goal: WaypointGoal,
        dt: float,
    ) -> bool:
        dx = goal.target_x - robot.x
        dy = goal.target_y - robot.y
        distance = math.hypot(dx, dy)

        desired_heading = goal.target_yaw
        if distance > self.waypoint_position_tolerance_m:
            desired_heading = math.atan2(dy, dx)

        yaw_error = _normalize_angle(desired_heading - robot.yaw)
        max_turn = self.max_turn_rate_rps * dt
        yaw_step = max(-max_turn, min(max_turn, yaw_error))
        robot.yaw = _normalize_angle(robot.yaw + yaw_step)

        if distance > self.waypoint_position_tolerance_m:
            target_speed = min(self.max_speed, max(0.15, distance * 1.5))
            robot.vx = math.cos(robot.yaw) * target_speed
            robot.vy = math.sin(robot.yaw) * target_speed
            robot.x += robot.vx * dt
            robot.y += robot.vy * dt
        else:
            snap = min(1.0, dt * 8.0)
            robot.x += dx * snap
            robot.y += dy * snap
            robot.vx = 0.0
            robot.vy = 0.0

        remaining_distance = math.hypot(goal.target_x - robot.x, goal.target_y - robot.y)
        remaining_yaw = abs(_normalize_angle(goal.target_yaw - robot.yaw))
        if remaining_distance <= self.waypoint_position_tolerance_m and remaining_yaw <= self.waypoint_yaw_tolerance_rad:
            robot.x = goal.target_x
            robot.y = goal.target_y
            robot.yaw = goal.target_yaw
            robot.vx = 0.0
            robot.vy = 0.0
            return True

        return False

    def _update_robot_motion(self, dt: float, now_sec: float):
        for robot in self.robots:
            name = robot.name

            if self._is_teleop_active(name, now_sec):
                self._cancel_tasks_for_teleop(name)
                self._apply_teleop_to_robot(robot, dt)
                continue

            queue = self._path_queues.get(name, [])
            active_index = self._active_path_index.get(name, -1)
            if queue and 0 <= active_index < len(queue):
                reached = self._advance_waypoint(robot, queue[active_index], dt)
                if reached:
                    self._publish_waypoint_status(name, "waypoint_reached", active_index, len(queue))
                    next_index = active_index + 1
                    if next_index >= len(queue):
                        self._publish_waypoint_status(name, "path_completed", active_index, len(queue))
                        self._path_queues[name] = []
                        self._active_path_index[name] = -1
                        self._publish_empty_nav_paths(name)
                    else:
                        self._active_path_index[name] = next_index
                        self._publish_task_nav_paths(name)
                else:
                    self._maybe_publish_task_nav_paths(name, now_sec)
                continue

            goal = self._goals.get(name)
            if goal is not None:
                if self._advance_goal(robot, goal, dt):
                    self._goals[name] = None
                    self._publish_goal_status(name, "goal_reached", goal)
                    self._publish_empty_nav_paths(name)
                else:
                    self._maybe_publish_task_nav_paths(name, now_sec)
                continue

            self._set_robot_idle(robot)

        self._resolve_collisions(dt)
        self._apply_bounds()

    def _publish_robot_odometry(self, robot, stamp_msg):
        publisher = self._odom_publishers.get(robot.name)
        if publisher is None:
            return

        msg = Odometry()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = f"{robot.name}/{self._base_frame_for(robot.name)}"
        msg.pose.pose.position.x = robot.x * self.scale
        msg.pose.pose.position.y = robot.y * self.scale
        msg.pose.pose.position.z = self._robot_altitudes.get(robot.name, self.height) * self.scale
        qx, qy, qz, qw = self._quat_from_yaw(robot.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = robot.vx * self.scale
        msg.twist.twist.linear.y = robot.vy * self.scale
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0
        publisher.publish(msg)

    def _publish_all_odometry(self, stamp_msg):
        for robot in self.robots:
            self._publish_robot_odometry(robot, stamp_msg)

    @staticmethod
    def _normalize_direction(dx: float, dy: float) -> Tuple[float, float, float]:
        norm = math.hypot(dx, dy)
        if norm <= 1e-6:
            return (0.0, 0.0, 0.0)
        return (dx / norm, dy / norm, 0.0)

    @staticmethod
    def _world_direction_to_base_link(
        direction_world: Tuple[float, float, float],
        robot_yaw: float,
    ) -> Tuple[float, float, float]:
        dx_world = float(direction_world[0])
        dy_world = float(direction_world[1])
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        # Rotate world direction into robot base_link (x forward, y left).
        local_x = (cos_yaw * dx_world) + (sin_yaw * dy_world)
        local_y = (-sin_yaw * dx_world) + (cos_yaw * dy_world)
        return OpsSuiteFakeTFPublisher._normalize_direction(local_x, local_y)

    def _nearest_boundary_obstacle(self, robot) -> Tuple[float, Tuple[float, float, float]]:
        candidates = [
            (self.area_size - robot.x, (1.0, 0.0, 0.0)),
            (self.area_size + robot.x, (-1.0, 0.0, 0.0)),
            (self.area_size - robot.y, (0.0, 1.0, 0.0)),
            (self.area_size + robot.y, (0.0, -1.0, 0.0)),
        ]
        best_distance = math.inf
        best_direction = (0.0, 0.0, 0.0)
        for distance, direction in candidates:
            if distance < best_distance:
                best_distance = float(distance)
                best_direction = direction
        return best_distance, best_direction

    def _nearest_robot_obstacle(self, robot) -> Tuple[float, Tuple[float, float, float]]:
        best_distance = math.inf
        best_direction = (0.0, 0.0, 0.0)
        for other in self.robots:
            if other is robot:
                continue
            dx = other.x - robot.x
            dy = other.y - robot.y
            distance = math.hypot(dx, dy)
            if distance < best_distance:
                best_distance = distance
                best_direction = self._normalize_direction(dx, dy)
        return best_distance, best_direction

    def _nearest_occupancy_obstacle(self, robot) -> Tuple[float, Tuple[float, float, float]]:
        if self.occupancy_data is None:
            return math.inf, (0.0, 0.0, 0.0)

        origin = self._world_to_grid(robot.x, robot.y)
        if origin is None:
            return math.inf, (0.0, 0.0, 0.0)

        gx, gy = origin
        cell_size_world = max(self._occupancy_cell_size_world(), 1e-6)
        max_search_radius = max(1, int(math.ceil((self.collision_threshold_m * 1.5) / cell_size_world)))

        best_distance = math.inf
        best_direction = (0.0, 0.0, 0.0)

        for radius in range(1, max_search_radius + 1):
            min_x = max(0, gx - radius)
            max_x = min(self.occupancy_width - 1, gx + radius)
            min_y = max(0, gy - radius)
            max_y = min(self.occupancy_height - 1, gy + radius)

            for sx in range(min_x, max_x + 1):
                for sy in (min_y, max_y):
                    if not self._is_grid_blocked(sx, sy):
                        continue
                    wx, wy = self._grid_to_world(sx, sy)
                    dx = wx - robot.x
                    dy = wy - robot.y
                    distance = math.hypot(dx, dy)
                    if distance < best_distance:
                        best_distance = distance
                        best_direction = self._normalize_direction(dx, dy)

            for sy in range(min_y + 1, max_y):
                for sx in (min_x, max_x):
                    if not self._is_grid_blocked(sx, sy):
                        continue
                    wx, wy = self._grid_to_world(sx, sy)
                    dx = wx - robot.x
                    dy = wy - robot.y
                    distance = math.hypot(dx, dy)
                    if distance < best_distance:
                        best_distance = distance
                        best_direction = self._normalize_direction(dx, dy)

            if math.isfinite(best_distance):
                break

        return best_distance, best_direction

    def _compute_collision_risk(self, robot) -> Tuple[float, float, Tuple[float, float, float]]:
        boundary_distance, boundary_direction = self._nearest_boundary_obstacle(robot)
        robot_distance, robot_direction = self._nearest_robot_obstacle(robot)
        occupancy_distance, occupancy_direction = self._nearest_occupancy_obstacle(robot)

        best_distance = boundary_distance
        best_direction = boundary_direction
        if robot_distance < best_distance:
            best_distance = robot_distance
            best_direction = robot_direction
        if occupancy_distance < best_distance:
            best_distance = occupancy_distance
            best_direction = occupancy_direction

        # Distances are tracked in simulator world units; convert to published map meters.
        distance_map = best_distance * abs(self._map_scale)
        # Convert center-to-center distance to approximate free-space clearance around the robot body.
        clearance_map = max(0.0, distance_map - (2.0 * self._collision_body_radius_m))

        threshold = max(self.collision_threshold_m, 1e-6)
        if clearance_map >= threshold:
            return 0.0, clearance_map, best_direction

        normalized = max(0.0, min(1.0, (threshold - clearance_map) / threshold))
        # Slight easing for clearer visual progression in MR.
        risk = normalized ** 0.85
        return risk, clearance_map, best_direction

    def _publish_collision_risk_if_due(self, now_sec: float):
        if not self.publish_collision_risk:
            return

        for robot in self.robots:
            robot_name = robot.name
            publisher = self._collision_risk_publishers.get(robot_name)
            if publisher is None:
                continue

            last_time = float(self._last_collision_risk_publish_time.get(robot_name, 0.0))
            if (now_sec - last_time) < self._collision_risk_publish_period_s:
                continue

            risk, min_distance, direction = self._compute_collision_risk(robot)
            base_link_direction = self._world_direction_to_base_link(direction, robot.yaw)
            payload = {
                "robot": robot_name,
                "frame": f"{robot_name}/{self._base_frame_for(robot_name)}",
                "stamp_ms": int(now_sec * 1000.0),
                "source": "fake_sim",
                "threshold_m": round(self.collision_threshold_m, 4),
                "min_distance_m": round(float(min_distance), 4) if math.isfinite(min_distance) else None,
                "risk": round(float(risk), 4),
                "direction": {
                    "x": round(float(base_link_direction[0]), 4),
                    "y": round(float(base_link_direction[1]), 4),
                    "z": round(float(base_link_direction[2]), 4),
                },
            }
            msg = String()
            msg.data = json.dumps(payload, separators=(",", ":"))
            publisher.publish(msg)
            self._last_collision_risk_publish_time[robot_name] = now_sec

    def _on_timer(self):
        if not self._enforce_single_tf_publisher():
            return

        now = self.get_clock().now().to_msg()
        current_time = time.time()
        dt = current_time - self.last_update_time
        if dt <= 0.0:
            dt = 1.0 / self.rate_hz
        dt = min(dt, 0.2)
        self.last_update_time = current_time

        self._update_robot_motion(dt, current_time)
        self._enforce_free_space()
        self._publish_all_odometry(now)
        self._publish_collision_risk_if_due(current_time)

        transforms = []
        for robot in self.robots:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = f"{robot.name}/{self._base_frame_for(robot.name)}"
            tf.transform.translation.x = robot.x * self.scale
            tf.transform.translation.y = robot.y * self.scale
            tf.transform.translation.z = self._robot_altitudes.get(robot.name, self.height) * self.scale
            qx, qy, qz, qw = self._quat_from_yaw(robot.yaw)
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            transforms.append(tf)

        if transforms:
            self.tf_pub.publish(TFMessage(transforms=transforms))

    @staticmethod
    def _quat_from_yaw(yaw_rad: float) -> Tuple[float, float, float, float]:
        half = yaw_rad * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)


def build_ops_suite_parser():
    parser = build_teleop_parser(default_robot_count=10)
    parser.description = (
        "Publish fake TF for a multi-robot operations suite where robots move only "
        "from teleop cmd_vel, go-to-point, or waypoint tasks."
    )
    parser.set_defaults(
        robot_name="",
        robot_count=10,
        mode="ops_suite",
        max_speed=0.8,
        rate=30.0,
        static_camera=True,
        publish_compressed_images=True,
        publish_images=False,
        publish_occupancy_grid=False,
    )
    parser.add_argument(
        "--goal-position-tolerance",
        type=float,
        default=0.20,
        help="Go-to-point completion position tolerance in map meters.",
    )
    parser.add_argument(
        "--goal-yaw-tolerance-deg",
        type=float,
        default=12.0,
        help="Go-to-point completion yaw tolerance in degrees.",
    )
    parser.add_argument(
        "--waypoint-position-tolerance",
        type=float,
        default=0.20,
        help="Waypoint completion position tolerance in map meters.",
    )
    parser.add_argument(
        "--waypoint-yaw-tolerance-deg",
        type=float,
        default=12.0,
        help="Waypoint completion yaw tolerance in degrees.",
    )
    parser.add_argument(
        "--max-turn-rate",
        type=float,
        default=1.6,
        help="Maximum heading change rate in rad/s for task execution.",
    )
    parser.add_argument(
        "--task-path-publish-rate",
        type=float,
        default=5.0,
        help="Publish rate (Hz) for fake /<robot>/global_path and /<robot>/local_path while tasks are active.",
    )
    parser.add_argument(
        "--publish-collision-risk",
        dest="publish_collision_risk",
        action="store_true",
        default=True,
        help="Publish fake collision risk JSON on /<robot>/collision_risk (default: on).",
    )
    parser.add_argument(
        "--no-publish-collision-risk",
        dest="publish_collision_risk",
        action="store_false",
        help="Disable fake collision risk publishing.",
    )
    parser.add_argument(
        "--collision-threshold-m",
        type=float,
        default=0.45,
        help="Near-obstacle clearance threshold in meters for fake risk output.",
    )
    parser.add_argument(
        "--collision-risk-rate",
        type=float,
        default=10.0,
        help="Publish rate (Hz) for fake /<robot>/collision_risk.",
    )
    parser.add_argument(
        "--status-frame-id",
        default="map",
        help="Frame id used in goal_status and waypoint_status payload metadata.",
    )
    parser.add_argument(
        "--strict-single-tf-publisher",
        dest="strict_single_tf_publisher",
        action="store_true",
        default=False,
        help="Stop this node if multiple /tf publishers are detected.",
    )
    parser.add_argument(
        "--allow-multiple-tf-publishers",
        dest="strict_single_tf_publisher",
        action="store_false",
        help="Disable strict /tf publisher conflict guard.",
    )
    return parser


def resolve_ops_robot_names(args) -> List[str]:
    if args.robot_names:
        names = [name.strip() for name in args.robot_names.split(",") if name.strip()]
        if names:
            return names

    count = max(1, int(args.robot_count))
    prefix = (args.robot_name or "").strip()
    if prefix:
        if count <= 1:
            return [prefix]
        return [f"{prefix}_{idx + 1}" for idx in range(count)]

    if count <= len(DEFAULT_ROBOT_NAMES):
        return DEFAULT_ROBOT_NAMES[:count]

    names = list(DEFAULT_ROBOT_NAMES)
    for idx in range(len(DEFAULT_ROBOT_NAMES), count):
        names.append(f"unit_{idx + 1}")
    return names


def run_from_args(args):
    rclpy.init()
    robot_names = resolve_ops_robot_names(args)
    parsed_resolutions = parse_resolution_list(args.image_resolutions)
    if not parsed_resolutions:
        parsed_resolutions = [(args.image_width, args.image_height)]

    node = OpsSuiteFakeTFPublisher(
        command_timeout_s=args.command_timeout,
        goal_position_tolerance_m=args.goal_position_tolerance,
        goal_yaw_tolerance_deg=args.goal_yaw_tolerance_deg,
        waypoint_position_tolerance_m=args.waypoint_position_tolerance,
        waypoint_yaw_tolerance_deg=args.waypoint_yaw_tolerance_deg,
        max_turn_rate_rps=args.max_turn_rate,
        task_path_publish_rate_hz=args.task_path_publish_rate,
        publish_collision_risk=args.publish_collision_risk,
        collision_threshold_m=args.collision_threshold_m,
        collision_risk_rate_hz=args.collision_risk_rate,
        status_frame_id=args.status_frame_id,
        strict_single_tf_publisher=args.strict_single_tf_publisher,
        robot_names=robot_names,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
        robot_base_frames=parse_robot_base_frames(args.robot_base_frames),
        rate_hz=args.rate,
        height=args.height,
        scale=args.scale,
        mode="ops_suite",
        radius=args.radius,
        omega=args.omega,
        area_size=args.area_size,
        min_distance=args.min_distance,
        max_speed=args.max_speed,
        jitter=0.0,
        seed=args.seed,
        publish_static_frames=not args.no_static_frames,
        publish_camera=args.static_camera,
        publish_images=args.publish_images,
        publish_compressed_images=args.publish_compressed_images,
        image_rate_hz=args.image_rate,
        image_width=args.image_width,
        image_height=args.image_height,
        image_reliable=not args.image_best_effort,
        jpeg_quality=args.jpeg_quality,
        vary_image_resolution=args.vary_image_resolution,
        image_resolutions=parsed_resolutions,
        publish_occupancy_grid=args.publish_occupancy_grid,
        occupancy_topic=args.occupancy_topic,
        occupancy_rate_hz=args.occupancy_rate,
        occupancy_resolution=args.occupancy_resolution,
        occupancy_width=args.occupancy_width,
        occupancy_height=args.occupancy_height,
        occupancy_unknown_ratio=args.occupancy_unknown_ratio,
        occupancy_obstacle_count=args.occupancy_obstacle_count,
    )

    executor = None
    try:
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.remove_node(node)
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # Ctrl+C paths may already shutdown context in some environments.
            pass


def main():
    parser = build_ops_suite_parser()
    args = parser.parse_args()
    run_from_args(args)


if __name__ == "__main__":
    main()
