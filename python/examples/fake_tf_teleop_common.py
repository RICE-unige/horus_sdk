#!/usr/bin/env python3
"""Shared teleop-driven fake TF publisher utilities."""

import math
import os
import sys
import time

# Ensure local example imports resolve when script is run from arbitrary CWD.
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import TransformStamped, Twist
    from tf2_msgs.msg import TFMessage
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    sys.exit(1)

from fake_tf_publisher import (
    FakeTFPublisher,
    build_parser as build_base_parser,
    parse_resolution_list,
    quaternion_from_yaw,
    resolve_robot_names,
)


class TeleopDrivenFakeTFPublisher(FakeTFPublisher):
    """Fake TF publisher where robots only move from cmd_vel teleop commands."""

    def __init__(self, command_timeout_s: float, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.command_timeout_s = max(0.05, float(command_timeout_s))
        self.mode = "teleop"
        self._robot_altitudes = {robot.name: float(self.height) for robot in self.robots}
        self._cmd_state = {
            robot.name: {
                "linear_x": 0.0,
                "linear_y": 0.0,
                "linear_z": 0.0,
                "angular_z": 0.0,
                "timestamp": 0.0,
            }
            for robot in self.robots
        }

        cmd_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._cmd_subscriptions = []
        for robot in self.robots:
            topic = f"/{robot.name}/cmd_vel"
            subscription = self.create_subscription(
                Twist,
                topic,
                self._make_cmd_callback(robot.name),
                cmd_qos,
            )
            self._cmd_subscriptions.append(subscription)
            robot.vx = 0.0
            robot.vy = 0.0
            robot.target_x = robot.x
            robot.target_y = robot.y

        self.get_logger().info(
            f"Teleop command mode active. Robots stay static until Twist arrives on /<robot>/cmd_vel (timeout={self.command_timeout_s:.2f}s)."
        )

    def _make_cmd_callback(self, robot_name):
        def _callback(msg: Twist):
            state = self._cmd_state.get(robot_name)
            if state is None:
                return
            state["linear_x"] = float(msg.linear.x)
            state["linear_y"] = float(msg.linear.y)
            state["linear_z"] = float(msg.linear.z)
            state["angular_z"] = float(msg.angular.z)
            state["timestamp"] = time.time()

        return _callback

    def _on_timer(self):
        now = self.get_clock().now().to_msg()
        current_time = time.time()
        dt = current_time - self.last_update_time
        if dt <= 0.0:
            dt = 1.0 / self.rate_hz
        dt = min(dt, 0.2)
        self.last_update_time = current_time

        self._update_from_teleop_commands(dt, current_time)
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
            qx, qy, qz, qw = quaternion_from_yaw(robot.yaw)
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            transforms.append(tf)

        if transforms:
            self.tf_pub.publish(TFMessage(transforms=transforms))

    def _update_from_teleop_commands(self, dt, now_sec):
        for robot in self.robots:
            state = self._cmd_state.get(robot.name, {})
            age_s = now_sec - float(state.get("timestamp", 0.0))
            if age_s > self.command_timeout_s:
                linear_x = 0.0
                linear_y = 0.0
                linear_z = 0.0
                angular_z = 0.0
            else:
                linear_x = float(state.get("linear_x", 0.0))
                linear_y = float(state.get("linear_y", 0.0))
                linear_z = float(state.get("linear_z", 0.0))
                angular_z = float(state.get("angular_z", 0.0))

            planar_speed = math.hypot(linear_x, linear_y)
            if planar_speed > self.max_speed and planar_speed > 1e-6:
                ratio = self.max_speed / planar_speed
                linear_x *= ratio
                linear_y *= ratio

            robot.yaw += angular_z * dt
            robot.yaw = math.atan2(math.sin(robot.yaw), math.cos(robot.yaw))

            world_vx = (math.cos(robot.yaw) * linear_x) - (math.sin(robot.yaw) * linear_y)
            world_vy = (math.sin(robot.yaw) * linear_x) + (math.cos(robot.yaw) * linear_y)

            robot.vx = world_vx
            robot.vy = world_vy
            robot.x += world_vx * dt
            robot.y += world_vy * dt

            altitude = self._robot_altitudes.get(robot.name, self.height)
            altitude += linear_z * dt
            altitude = max(-2.0, min(4.0, altitude))
            self._robot_altitudes[robot.name] = altitude

        self._resolve_collisions(dt)
        self._apply_bounds()


def build_teleop_parser(default_robot_count: int):
    parser = build_base_parser()
    parser.description = "Publish fake TF where robots move only from per-robot cmd_vel teleop commands."
    parser.set_defaults(
        robot_count=max(1, int(default_robot_count)),
        mode="teleop",
        max_speed=1.5,
        rate=30.0,
        static_camera=True,
        publish_compressed_images=True,
        publish_images=False,
        publish_occupancy_grid=False,
    )
    parser.add_argument(
        "--command-timeout",
        type=float,
        default=0.35,
        help="Stop robot if no cmd_vel is received within timeout seconds.",
    )
    return parser


def run_from_args(args):
    rclpy.init()
    robot_names = resolve_robot_names(args)
    parsed_resolutions = parse_resolution_list(args.image_resolutions)
    if not parsed_resolutions:
        parsed_resolutions = [(args.image_width, args.image_height)]

    node = TeleopDrivenFakeTFPublisher(
        command_timeout_s=args.command_timeout,
        robot_names=robot_names,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
        rate_hz=args.rate,
        height=args.height,
        scale=args.scale,
        mode="teleop",
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
