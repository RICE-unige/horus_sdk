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
    from nav_msgs.msg import Path
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    sys.exit(1)

from fake_tf_publisher import parse_resolution_list
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
        status_frame_id: str,
        *args,
        **kwargs,
    ):
        super().__init__(command_timeout_s=command_timeout_s, *args, **kwargs)
        self.mode = "ops_suite"
        self.goal_position_tolerance_m = max(0.01, float(goal_position_tolerance_m))
        self.goal_yaw_tolerance_rad = math.radians(max(0.1, float(goal_yaw_tolerance_deg)))
        self.waypoint_position_tolerance_m = max(0.01, float(waypoint_position_tolerance_m))
        self.waypoint_yaw_tolerance_rad = math.radians(max(0.1, float(waypoint_yaw_tolerance_deg)))
        self.max_turn_rate_rps = max(0.1, float(max_turn_rate_rps))
        self.status_frame_id = status_frame_id.strip() if status_frame_id else "map"
        self._map_scale = self.scale if abs(self.scale) > 1e-6 else 1.0

        self._goals: Dict[str, Optional[GoalState]] = {robot.name: None for robot in self.robots}
        self._path_queues: Dict[str, List[WaypointGoal]] = {robot.name: [] for robot in self.robots}
        self._active_path_index: Dict[str, int] = {robot.name: -1 for robot in self.robots}
        self._goal_status_publishers = {}
        self._waypoint_status_publishers = {}
        self._goal_subscriptions = []
        self._cancel_subscriptions = []
        self._path_subscriptions = []

        command_qos = QoSProfile(
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
            robot.vx = 0.0
            robot.vy = 0.0
            robot.target_x = robot.x
            robot.target_y = robot.y

        self.get_logger().info(
            "Ops suite mode active. Robots stay static until cmd_vel, goal_pose, or waypoint_path is received. "
            "Teleop cmd_vel overrides and cancels active tasks."
        )

    def _make_goal_callback(self, robot_name):
        def _callback(msg: PoseStamped):
            target_x = float(msg.pose.position.x) / self._map_scale
            target_y = float(msg.pose.position.y) / self._map_scale
            target_yaw = _yaw_from_quaternion(
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
                float(msg.pose.orientation.w),
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
                return

            queue: List[WaypointGoal] = []
            for pose in msg.poses:
                target_x = float(pose.pose.position.x) / self._map_scale
                target_y = float(pose.pose.position.y) / self._map_scale
                target_yaw = _yaw_from_quaternion(
                    float(pose.pose.orientation.x),
                    float(pose.pose.orientation.y),
                    float(pose.pose.orientation.z),
                    float(pose.pose.orientation.w),
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
                    else:
                        self._active_path_index[name] = next_index
                continue

            goal = self._goals.get(name)
            if goal is not None:
                if self._advance_goal(robot, goal, dt):
                    self._goals[name] = None
                    self._publish_goal_status(name, "goal_reached", goal)
                continue

            self._set_robot_idle(robot)

        self._resolve_collisions(dt)
        self._apply_bounds()

    def _on_timer(self):
        now = self.get_clock().now().to_msg()
        current_time = time.time()
        dt = current_time - self.last_update_time
        if dt <= 0.0:
            dt = 1.0 / self.rate_hz
        dt = min(dt, 0.2)
        self.last_update_time = current_time

        self._update_robot_motion(dt, current_time)
        self._enforce_free_space()

        transforms = []
        for robot in self.robots:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = f"{robot.name}/{self.base_frame}"
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
        "--status-frame-id",
        default="map",
        help="Frame id used in goal_status and waypoint_status payload metadata.",
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
        status_frame_id=args.status_frame_id,
        robot_names=robot_names,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
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
        rclpy.shutdown()


def main():
    parser = build_ops_suite_parser()
    args = parser.parse_args()
    run_from_args(args)


if __name__ == "__main__":
    main()
