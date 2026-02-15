#!/usr/bin/env python3
"""Fake TF publisher that executes go-to-point tasks from ROS topics."""

import json
import math
import os
import sys
import time
from dataclasses import dataclass

# Ensure local example imports resolve when script is run from arbitrary CWD.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    sys.exit(1)

from fake_tf_publisher import (  # noqa: E402
    FakeTFPublisher,
    build_parser as build_base_parser,
    parse_resolution_list,
    quaternion_from_yaw,
    resolve_robot_names,
)


def _normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    # Standard ROS yaw extraction for Z-up quaternions.
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class GoalState:
    target_x: float
    target_y: float
    target_yaw: float


class GoToPointFakeTFPublisher(FakeTFPublisher):
    """Fake TF publisher where robots navigate to goal poses sent on ROS topics."""

    def __init__(
        self,
        position_tolerance_m: float,
        yaw_tolerance_deg: float,
        max_turn_rate_rps: float,
        status_frame_id: str,
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.mode = "go_to_point"
        self.position_tolerance_m = max(0.01, float(position_tolerance_m))
        self.yaw_tolerance_rad = math.radians(max(0.1, float(yaw_tolerance_deg)))
        self.max_turn_rate_rps = max(0.1, float(max_turn_rate_rps))
        self.status_frame_id = status_frame_id.strip() if status_frame_id else "map"
        self._map_scale = self.scale if abs(self.scale) > 1e-6 else 1.0

        self._goals = {robot.name: None for robot in self.robots}
        self._status_publishers = {}
        self._goal_subscriptions = []
        self._cancel_subscriptions = []
        self._pending_cancel_status_bursts = []

        command_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        cancel_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        for robot in self.robots:
            goal_topic = f"/{robot.name}/goal_pose"
            cancel_topic = f"/{robot.name}/goal_cancel"
            status_topic = f"/{robot.name}/goal_status"

            goal_sub = self.create_subscription(
                PoseStamped,
                goal_topic,
                self._make_goal_callback(robot.name),
                command_qos,
            )
            cancel_sub = self.create_subscription(
                String,
                cancel_topic,
                self._make_cancel_callback(robot.name),
                cancel_qos,
            )
            status_pub = self.create_publisher(String, status_topic, command_qos)

            self._goal_subscriptions.append(goal_sub)
            self._cancel_subscriptions.append(cancel_sub)
            self._status_publishers[robot.name] = status_pub

            robot.vx = 0.0
            robot.vy = 0.0
            robot.target_x = robot.x
            robot.target_y = robot.y

        self.get_logger().info(
            "Go-to-point mode active. Send PoseStamped on /<robot>/goal_pose and String('cancel') on /<robot>/goal_cancel."
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
            target_yaw = _normalize_angle(target_yaw)
            self._goals[robot_name] = GoalState(target_x=target_x, target_y=target_y, target_yaw=target_yaw)

            robot = self._robot_by_name(robot_name)
            if robot is not None:
                robot.target_x = target_x
                robot.target_y = target_y

            self._publish_status(robot_name, "goal_sent", self._goals[robot_name])

        return _callback

    def _make_cancel_callback(self, robot_name):
        def _callback(_msg: String):
            self.get_logger().info(f"Received goal cancel for {robot_name}")
            goal = self._goals.get(robot_name)
            self._goals[robot_name] = None
            robot = self._robot_by_name(robot_name)
            if robot is not None:
                robot.vx = 0.0
                robot.vy = 0.0
                robot.target_x = robot.x
                robot.target_y = robot.y
            self._publish_status(robot_name, "goal_cancelled", goal)
            self._schedule_cancel_status_burst(robot_name, goal)

        return _callback

    def _schedule_cancel_status_burst(self, robot_name: str, goal: GoalState):
        now = time.monotonic()
        for delay in (0.1, 0.3):
            self._pending_cancel_status_bursts.append((now + delay, robot_name, goal))

    def _flush_cancel_status_burst_queue(self):
        if not self._pending_cancel_status_bursts:
            return

        now = time.monotonic()
        remaining = []
        for due_time, robot_name, goal in self._pending_cancel_status_bursts:
            if due_time <= now:
                self._publish_status(robot_name, "goal_cancelled", goal)
            else:
                remaining.append((due_time, robot_name, goal))
        self._pending_cancel_status_bursts = remaining

    def _robot_by_name(self, robot_name):
        for robot in self.robots:
            if robot.name == robot_name:
                return robot
        return None

    def _publish_status(self, robot_name: str, state: str, goal: GoalState):
        publisher = self._status_publishers.get(robot_name)
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

    def _on_timer(self):
        self._flush_cancel_status_burst_queue()

        now = self.get_clock().now().to_msg()
        current_time = time.time()
        dt = current_time - self.last_update_time
        if dt <= 0.0:
            dt = 1.0 / self.rate_hz
        dt = min(dt, 0.2)
        self.last_update_time = current_time

        self._update_goal_navigation(dt)
        self._enforce_free_space()

        transforms = []
        for robot in self.robots:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = f"{robot.name}/{self.base_frame}"
            tf.transform.translation.x = robot.x * self.scale
            tf.transform.translation.y = robot.y * self.scale
            tf.transform.translation.z = self.height * self.scale
            qx, qy, qz, qw = quaternion_from_yaw(robot.yaw)
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            transforms.append(tf)

        if transforms:
            self.tf_pub.publish(TFMessage(transforms=transforms))

    def _update_goal_navigation(self, dt: float):
        for robot in self.robots:
            goal = self._goals.get(robot.name)
            if goal is None:
                robot.vx = 0.0
                robot.vy = 0.0
                continue

            dx = goal.target_x - robot.x
            dy = goal.target_y - robot.y
            distance = math.hypot(dx, dy)

            desired_heading = goal.target_yaw
            if distance > self.position_tolerance_m:
                desired_heading = math.atan2(dy, dx)

            yaw_error = _normalize_angle(desired_heading - robot.yaw)
            max_turn = self.max_turn_rate_rps * dt
            yaw_step = max(-max_turn, min(max_turn, yaw_error))
            robot.yaw = _normalize_angle(robot.yaw + yaw_step)

            if distance > self.position_tolerance_m:
                target_speed = min(self.max_speed, max(0.15, distance * 1.5))
                robot.vx = math.cos(robot.yaw) * target_speed
                robot.vy = math.sin(robot.yaw) * target_speed
                robot.x += robot.vx * dt
                robot.y += robot.vy * dt
            else:
                # Snap gently near goal while allowing orientation settle.
                snap = min(1.0, dt * 8.0)
                robot.x += dx * snap
                robot.y += dy * snap
                robot.vx = 0.0
                robot.vy = 0.0

            remaining_distance = math.hypot(goal.target_x - robot.x, goal.target_y - robot.y)
            remaining_yaw = abs(_normalize_angle(goal.target_yaw - robot.yaw))
            if remaining_distance <= self.position_tolerance_m and remaining_yaw <= self.yaw_tolerance_rad:
                robot.x = goal.target_x
                robot.y = goal.target_y
                robot.yaw = goal.target_yaw
                robot.vx = 0.0
                robot.vy = 0.0
                self._goals[robot.name] = None
                self._publish_status(robot.name, "goal_reached", goal)

        self._resolve_collisions(dt)
        self._apply_bounds()


def build_go_to_point_parser():
    parser = build_base_parser()
    parser.description = "Publish fake TF where robots execute /<robot>/goal_pose tasks."
    parser.set_defaults(
        robot_count=3,
        robot_name="test_bot",
        mode="go_to_point",
        max_speed=0.8,
        rate=30.0,
        static_camera=True,
        publish_compressed_images=True,
        publish_images=False,
        publish_occupancy_grid=False,
    )
    parser.add_argument(
        "--position-tolerance",
        type=float,
        default=0.20,
        help="Goal completion position tolerance in map meters.",
    )
    parser.add_argument(
        "--yaw-tolerance-deg",
        type=float,
        default=12.0,
        help="Goal completion yaw tolerance in degrees.",
    )
    parser.add_argument(
        "--max-turn-rate",
        type=float,
        default=1.6,
        help="Maximum heading change rate in rad/s.",
    )
    parser.add_argument(
        "--status-frame-id",
        default="map",
        help="Frame id used in goal_status payload metadata.",
    )
    return parser


def run_from_args(args):
    rclpy.init()
    robot_names = resolve_robot_names(args)
    parsed_resolutions = parse_resolution_list(args.image_resolutions)
    if not parsed_resolutions:
        parsed_resolutions = [(args.image_width, args.image_height)]

    node = GoToPointFakeTFPublisher(
        position_tolerance_m=args.position_tolerance,
        yaw_tolerance_deg=args.yaw_tolerance_deg,
        max_turn_rate_rps=args.max_turn_rate,
        status_frame_id=args.status_frame_id,
        robot_names=robot_names,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
        rate_hz=args.rate,
        height=args.height,
        scale=args.scale,
        mode="go_to_point",
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
    parser = build_go_to_point_parser()
    args = parser.parse_args()
    run_from_args(args)


if __name__ == "__main__":
    main()
