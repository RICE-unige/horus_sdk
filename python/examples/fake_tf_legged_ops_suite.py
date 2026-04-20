#!/usr/bin/env python3
"""Legged-focused fake TF runtime for teleop + 3D go-to-point + 3D waypoint validation."""

import json
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path
    from std_msgs.msg import Empty, String
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    raise SystemExit(1)

from fake_tf_ops_suite import (
    OpsSuiteFakeTFPublisher,
    _is_planar_yaw_quaternion,
    _normalize_angle,
    _yaw_from_quaternion,
    build_ops_suite_parser,
    parse_resolution_list,
    parse_robot_base_frames,
    resolve_ops_robot_names,
)


@dataclass
class LeggedGoalState:
    target_x: float
    target_y: float
    target_z: float
    target_yaw: float


@dataclass
class LeggedWaypointGoal:
    target_x: float
    target_y: float
    target_z: float
    target_yaw: float


class LeggedOpsSuiteFakeTFPublisher(OpsSuiteFakeTFPublisher):
    """Ops suite publisher specialized for legged robots with 3D task execution."""

    def __init__(
        self,
        min_altitude_m: float,
        max_altitude_m: float,
        goal_altitude_tolerance_m: float,
        waypoint_altitude_tolerance_m: float,
        max_vertical_speed_mps: float,
        takeoff_altitude_m: float,
        land_altitude_m: float,
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.min_altitude_m = float(min_altitude_m)
        self.max_altitude_m = max(float(max_altitude_m), self.min_altitude_m + 0.1)
        self.goal_altitude_tolerance_m = max(0.02, float(goal_altitude_tolerance_m))
        self.waypoint_altitude_tolerance_m = max(0.02, float(waypoint_altitude_tolerance_m))
        self.max_vertical_speed_mps = max(0.05, float(max_vertical_speed_mps))
        self.takeoff_altitude_m = max(
            self.min_altitude_m + self.goal_altitude_tolerance_m,
            self._clamp_altitude(float(takeoff_altitude_m)),
        )
        self.land_altitude_m = self._clamp_altitude(float(land_altitude_m))
        self.collision_vertical_band_m = max(
            0.25,
            min(
                self.max_altitude_m - self.min_altitude_m,
                max(0.35, self.collision_threshold_m * 0.5),
            ),
        )
        self._robot_vertical_speed: Dict[str, float] = {robot.name: 0.0 for robot in self.robots}
        self._flight_command: Dict[str, Optional[str]] = {robot.name: None for robot in self.robots}
        self._flight_target_altitude: Dict[str, float] = {
            robot.name: self._clamp_altitude(self._robot_altitudes.get(robot.name, self.height))
            for robot in self.robots
        }
        self._takeoff_subscriptions = []
        self._land_subscriptions = []

        # Clamp initial altitudes so all robots start in valid stance bounds.
        for robot in self.robots:
            self._robot_altitudes[robot.name] = self._clamp_altitude(
                self._robot_altitudes.get(robot.name, self.height)
            )
            self._flight_target_altitude[robot.name] = self._robot_altitudes[robot.name]

            self._takeoff_subscriptions.append(
                self.create_subscription(
                    Empty,
                    f"/{robot.name}/stand_up",
                    self._make_takeoff_callback(robot.name),
                    10,
                )
            )
            self._land_subscriptions.append(
                self.create_subscription(
                    Empty,
                    f"/{robot.name}/sit_down",
                    self._make_land_callback(robot.name),
                    10,
                )
            )

        self.get_logger().info(
            f"Legged task mode enabled: 3D goals + 3D waypoint paths + teleop cmd_vel z "
            f"+ stand_up/sit_down commands (height bounds {self.min_altitude_m:.2f}..{self.max_altitude_m:.2f}m)."
        )
        self.get_logger().info(
            f"Legged command topics: /<robot>/stand_up and /<robot>/sit_down "
            f"(stand_up target {self.takeoff_altitude_m:.2f}m, sit_down target {self.land_altitude_m:.2f}m)."
        )
        self.get_logger().info(
            f"Legged collision gating: robot-robot checks only within {self.collision_vertical_band_m:.2f}m height band."
        )

    def _clamp_altitude(self, value: float) -> float:
        return max(self.min_altitude_m, min(self.max_altitude_m, float(value)))

    def _altitude_for(self, robot_name: str) -> float:
        return self._clamp_altitude(self._robot_altitudes.get(robot_name, self.height))

    @staticmethod
    def _normalize_direction_3d(dx: float, dy: float, dz: float) -> Tuple[float, float, float]:
        norm = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
        if norm <= 1e-6:
            return (0.0, 0.0, 0.0)
        return (dx / norm, dy / norm, dz / norm)

    @staticmethod
    def _world_direction_to_base_link(
        direction_world: Tuple[float, float, float],
        robot_yaw: float,
    ) -> Tuple[float, float, float]:
        dx_world = float(direction_world[0])
        dy_world = float(direction_world[1])
        dz_world = float(direction_world[2]) if len(direction_world) > 2 else 0.0
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        # Rotate world direction into robot base_link (x forward, y left), preserve vertical axis.
        local_x = (cos_yaw * dx_world) + (sin_yaw * dy_world)
        local_y = (-sin_yaw * dx_world) + (cos_yaw * dy_world)
        local_z = dz_world
        return LeggedOpsSuiteFakeTFPublisher._normalize_direction_3d(local_x, local_y, local_z)

    def _nearest_robot_obstacle(self, robot) -> Tuple[float, Tuple[float, float, float]]:
        best_distance = math.inf
        best_direction = (0.0, 0.0, 0.0)
        robot_altitude = self._altitude_for(robot.name)
        for other in self.robots:
            if other is robot:
                continue
            other_altitude = self._altitude_for(other.name)
            dz = other_altitude - robot_altitude
            if abs(dz) > self.collision_vertical_band_m:
                continue
            dx = other.x - robot.x
            dy = other.y - robot.y
            distance = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
            if distance < best_distance:
                best_distance = distance
                best_direction = self._normalize_direction_3d(dx, dy, dz)
        return best_distance, best_direction

    def _resolve_collisions(self, dt: float):
        count = len(self.robots)
        max_push = max(0.02, self.max_speed * max(dt, 1.0 / self.rate_hz) * 0.6)
        max_vertical_push = max(0.01, self.max_vertical_speed_mps * max(dt, 1.0 / self.rate_hz) * 0.4)
        for i in range(count):
            for j in range(i + 1, count):
                a = self.robots[i]
                b = self.robots[j]
                az = self._altitude_for(a.name)
                bz = self._altitude_for(b.name)
                dz = az - bz
                if abs(dz) > self.collision_vertical_band_m:
                    continue

                dx = a.x - b.x
                dy = a.y - b.y
                planar_dist = math.hypot(dx, dy)
                if planar_dist < 1e-4:
                    angle = self.random.uniform(0.0, math.tau)
                    dx = math.cos(angle)
                    dy = math.sin(angle)
                    planar_dist = 1.0

                distance_3d = math.sqrt((planar_dist * planar_dist) + (dz * dz))
                if distance_3d >= self.min_distance:
                    continue

                overlap = self.min_distance - distance_3d
                planar_push = min(overlap * 0.5, max_push)
                nx = dx / planar_dist
                ny = dy / planar_dist

                a.x += nx * planar_push
                a.y += ny * planar_push
                b.x -= nx * planar_push
                b.y -= ny * planar_push

                # Keep height separation stable instead of forcing horizontal roots to "bounce" on near-overlap.
                if abs(dz) <= 1e-4:
                    nz = 1.0 if (i % 2 == 0) else -1.0
                else:
                    nz = dz / abs(dz)
                vertical_push = min(overlap * 0.25, max_vertical_push)
                self._robot_altitudes[a.name] = self._clamp_altitude(az + (nz * vertical_push))
                self._robot_altitudes[b.name] = self._clamp_altitude(bz - (nz * vertical_push))

                rel_vx = a.vx - b.vx
                rel_vy = a.vy - b.vy
                rel_n = (rel_vx * nx) + (rel_vy * ny)
                if rel_n < 0.0:
                    impulse = -0.5 * rel_n
                    a.vx += nx * impulse
                    a.vy += ny * impulse
                    b.vx -= nx * impulse
                    b.vy -= ny * impulse

    def _set_flight_command(self, robot_name: str, command: Optional[str], target_altitude: Optional[float] = None):
        self._flight_command[robot_name] = command
        if target_altitude is not None:
            self._flight_target_altitude[robot_name] = self._clamp_altitude(target_altitude)

    def _make_takeoff_callback(self, robot_name):
        def _callback(_msg: Empty):
            self._cancel_goal(robot_name, publish_status=True)
            self._cancel_waypoint_path(robot_name, reason="path_cancelled")
            self._set_flight_command(robot_name, "standing_up", self.takeoff_altitude_m)
            self.get_logger().info(
                f"StandUp command received for {robot_name} -> target height {self.takeoff_altitude_m:.2f}m."
            )

        return _callback

    def _make_land_callback(self, robot_name):
        def _callback(_msg: Empty):
            self._cancel_goal(robot_name, publish_status=True)
            self._cancel_waypoint_path(robot_name, reason="path_cancelled")
            self._set_flight_command(robot_name, "sitting_down", self.land_altitude_m)
            self.get_logger().info(
                f"SitDown command received for {robot_name} -> target height {self.land_altitude_m:.2f}m."
            )

        return _callback

    def _publish_goal_status(self, robot_name: str, state: str, goal: Optional[LeggedGoalState]):
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
                "z": round(goal.target_z * self._map_scale, 4),
                "yaw": round(goal.target_yaw, 4),
            }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        publisher.publish(msg)

    def _make_goal_callback(self, robot_name):
        def _callback(msg: PoseStamped):
            target_x = float(msg.pose.position.x) / self._map_scale
            target_y = float(msg.pose.position.y) / self._map_scale
            target_z = self._clamp_altitude(float(msg.pose.position.z) / self._map_scale)
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

            target_yaw = _yaw_from_quaternion(qx, qy, qz, qw)
            target_x, target_y = self._nearest_free_world_position(target_x, target_y)

            goal = LeggedGoalState(
                target_x=target_x,
                target_y=target_y,
                target_z=target_z,
                target_yaw=_normalize_angle(target_yaw),
            )

            self._cancel_waypoint_path(robot_name, reason="path_cancelled")
            self._set_flight_command(robot_name, None)
            self._goals[robot_name] = goal
            robot = self._robot_by_name(robot_name)
            if robot is not None:
                robot.target_x = goal.target_x
                robot.target_y = goal.target_y
            self._publish_goal_status(robot_name, "goal_sent", goal)
            self._publish_task_nav_paths(robot_name)

        return _callback

    def _make_waypoint_path_callback(self, robot_name):
        def _callback(msg: Path):
            if msg is None or not msg.poses:
                self._path_queues[robot_name] = []
                self._active_path_index[robot_name] = -1
                self._publish_waypoint_status(robot_name, "path_failed", -1, 0)
                self._publish_empty_nav_paths(robot_name)
                return

            queue: List[LeggedWaypointGoal] = []
            for pose in msg.poses:
                target_x = float(pose.pose.position.x) / self._map_scale
                target_y = float(pose.pose.position.y) / self._map_scale
                target_z = self._clamp_altitude(float(pose.pose.position.z) / self._map_scale)
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

                target_yaw = _yaw_from_quaternion(qx, qy, qz, qw)
                target_x, target_y = self._nearest_free_world_position(target_x, target_y)
                queue.append(
                    LeggedWaypointGoal(
                        target_x=target_x,
                        target_y=target_y,
                        target_z=target_z,
                        target_yaw=_normalize_angle(target_yaw),
                    )
                )

            self._cancel_goal(robot_name, publish_status=True)
            self._set_flight_command(robot_name, None)
            self._path_queues[robot_name] = queue
            self._active_path_index[robot_name] = 0
            self._publish_waypoint_status(robot_name, "path_sent", 0, len(queue))
            self._publish_task_nav_paths(robot_name)

        return _callback

    @staticmethod
    def _append_line_points_3d(
        points: List[Tuple[float, float, float, float]],
        start_x: float,
        start_y: float,
        start_z: float,
        end_x: float,
        end_y: float,
        end_z: float,
        end_yaw: float,
        include_start: bool = False,
        max_step: float = 0.35,
    ):
        dx = end_x - start_x
        dy = end_y - start_y
        dz = end_z - start_z
        distance = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
        heading = end_yaw if (abs(dx) < 1e-6 and abs(dy) < 1e-6) else math.atan2(dy, dx)
        steps = max(1, int(math.ceil(distance / max(0.05, max_step))))

        if include_start and not points:
            points.append((start_x, start_y, start_z, heading))

        for idx in range(1, steps + 1):
            alpha = float(idx) / float(steps)
            x = start_x + (dx * alpha)
            y = start_y + (dy * alpha)
            z = start_z + (dz * alpha)
            yaw = end_yaw if idx == steps else heading
            points.append((x, y, z, yaw))

    def _build_pose_stamped_3d(self, x_world: float, y_world: float, z_world: float, yaw: float, stamp_msg) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = stamp_msg
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x_world * self._map_scale
        pose.pose.position.y = y_world * self._map_scale
        pose.pose.position.z = z_world * self._map_scale
        qx, qy, qz, qw = self._quat_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _publish_path_points(self, publisher, points: List[Tuple[float, float, float, float]], stamp_msg):
        if publisher is None:
            return
        path = Path()
        path.header.stamp = stamp_msg
        path.header.frame_id = self.map_frame
        for x, y, z, yaw in points:
            path.poses.append(self._build_pose_stamped_3d(x, y, z, yaw, stamp_msg))
        publisher.publish(path)

    def _publish_empty_nav_paths(self, robot_name: str):
        stamp_msg = self.get_clock().now().to_msg()
        self._publish_path_points(self._global_path_publishers.get(robot_name), [], stamp_msg)
        self._publish_path_points(self._local_path_publishers.get(robot_name), [], stamp_msg)
        self._last_task_path_publish_time[robot_name] = time.time()

    def _build_goal_global_path_points(
        self,
        robot,
        goal: LeggedGoalState,
    ) -> List[Tuple[float, float, float, float]]:
        start_z = self._robot_altitudes.get(robot.name, self.height)
        points: List[Tuple[float, float, float, float]] = [(robot.x, robot.y, start_z, robot.yaw)]
        self._append_line_points_3d(
            points,
            robot.x,
            robot.y,
            start_z,
            goal.target_x,
            goal.target_y,
            goal.target_z,
            goal.target_yaw,
            include_start=False,
        )
        return points

    def _build_goal_local_path_points(
        self,
        robot,
        goal: LeggedGoalState,
    ) -> List[Tuple[float, float, float, float]]:
        current_z = self._robot_altitudes.get(robot.name, self.height)
        dx = goal.target_x - robot.x
        dy = goal.target_y - robot.y
        dz = goal.target_z - current_z
        distance = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
        if distance <= 1e-6:
            return [(robot.x, robot.y, current_z, robot.yaw), (goal.target_x, goal.target_y, goal.target_z, goal.target_yaw)]

        lookahead = min(distance, 1.8)
        inv = 1.0 / distance
        ux = dx * inv
        uy = dy * inv
        uz = dz * inv
        mid_x = robot.x + (ux * lookahead)
        mid_y = robot.y + (uy * lookahead)
        mid_z = current_z + (uz * lookahead)
        heading = math.atan2(dy, dx) if (abs(dx) > 1e-6 or abs(dy) > 1e-6) else goal.target_yaw

        points: List[Tuple[float, float, float, float]] = [(robot.x, robot.y, current_z, robot.yaw)]
        self._append_line_points_3d(points, robot.x, robot.y, current_z, mid_x, mid_y, mid_z, heading)
        if lookahead < distance:
            tail = min(distance, lookahead + 0.9)
            tail_x = robot.x + (ux * tail)
            tail_y = robot.y + (uy * tail)
            tail_z = current_z + (uz * tail)
            self._append_line_points_3d(points, mid_x, mid_y, mid_z, tail_x, tail_y, tail_z, heading)
        return points

    def _build_waypoint_global_path_points(
        self,
        robot,
        queue: List[LeggedWaypointGoal],
        active_index: int,
    ) -> List[Tuple[float, float, float, float]]:
        current_z = self._robot_altitudes.get(robot.name, self.height)
        points: List[Tuple[float, float, float, float]] = [(robot.x, robot.y, current_z, robot.yaw)]
        prev_x = robot.x
        prev_y = robot.y
        prev_z = current_z
        for idx in range(max(0, active_index), len(queue)):
            wp = queue[idx]
            self._append_line_points_3d(points, prev_x, prev_y, prev_z, wp.target_x, wp.target_y, wp.target_z, wp.target_yaw)
            prev_x = wp.target_x
            prev_y = wp.target_y
            prev_z = wp.target_z
        return points

    def _build_waypoint_local_path_points(
        self,
        robot,
        queue: List[LeggedWaypointGoal],
        active_index: int,
    ) -> List[Tuple[float, float, float, float]]:
        current_z = self._robot_altitudes.get(robot.name, self.height)
        points: List[Tuple[float, float, float, float]] = [(robot.x, robot.y, current_z, robot.yaw)]
        if not queue or active_index < 0 or active_index >= len(queue):
            return points

        remaining_budget = 2.4
        prev_x = robot.x
        prev_y = robot.y
        prev_z = current_z
        for idx in range(active_index, len(queue)):
            wp = queue[idx]
            dx = wp.target_x - prev_x
            dy = wp.target_y - prev_y
            dz = wp.target_z - prev_z
            segment_len = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
            if segment_len <= 1e-6:
                prev_x = wp.target_x
                prev_y = wp.target_y
                prev_z = wp.target_z
                continue

            heading = math.atan2(dy, dx) if (abs(dx) > 1e-6 or abs(dy) > 1e-6) else wp.target_yaw
            if segment_len <= remaining_budget + 1e-6:
                self._append_line_points_3d(points, prev_x, prev_y, prev_z, wp.target_x, wp.target_y, wp.target_z, wp.target_yaw)
                remaining_budget -= segment_len
                prev_x = wp.target_x
                prev_y = wp.target_y
                prev_z = wp.target_z
                if remaining_budget <= 1e-3:
                    break
                continue

            alpha = remaining_budget / segment_len
            end_x = prev_x + (dx * alpha)
            end_y = prev_y + (dy * alpha)
            end_z = prev_z + (dz * alpha)
            self._append_line_points_3d(points, prev_x, prev_y, prev_z, end_x, end_y, end_z, heading)
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

    def _set_robot_idle(self, robot):
        robot.vx = 0.0
        robot.vy = 0.0
        robot.target_x = robot.x
        robot.target_y = robot.y
        self._robot_vertical_speed[robot.name] = 0.0

    def _cancel_goal(self, robot_name: str, publish_status: bool = True):
        goal = self._goals.get(robot_name)
        self._goals[robot_name] = None
        robot = self._robot_by_name(robot_name)
        if robot is not None:
            robot.vx = 0.0
            robot.vy = 0.0
            robot.target_x = robot.x
            robot.target_y = robot.y
        self._robot_vertical_speed[robot_name] = 0.0
        if publish_status and goal is not None:
            self._publish_goal_status(robot_name, "goal_cancelled", goal)
        if goal is not None and not self._path_queues.get(robot_name):
            self._publish_empty_nav_paths(robot_name)

    def _apply_teleop_to_robot(self, robot, dt: float):
        self._set_flight_command(robot.name, None)
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

        if abs(linear_z) > self.max_vertical_speed_mps:
            linear_z = math.copysign(self.max_vertical_speed_mps, linear_z)

        robot.yaw = _normalize_angle(robot.yaw + (angular_z * dt))
        world_vx = (math.cos(robot.yaw) * linear_x) - (math.sin(robot.yaw) * linear_y)
        world_vy = (math.sin(robot.yaw) * linear_x) + (math.cos(robot.yaw) * linear_y)

        robot.vx = world_vx
        robot.vy = world_vy
        robot.x += world_vx * dt
        robot.y += world_vy * dt
        robot.target_x = robot.x
        robot.target_y = robot.y

        height = self._robot_altitudes.get(robot.name, self.height)
        height = self._clamp_altitude(height + (linear_z * dt))
        self._robot_altitudes[robot.name] = height
        self._robot_vertical_speed[robot.name] = linear_z

    def _apply_flight_command(self, robot, dt: float) -> bool:
        robot_name = robot.name
        mode = self._flight_command.get(robot_name)
        if not mode:
            return False

        target_altitude = self._flight_target_altitude.get(
            robot_name,
            self.takeoff_altitude_m if mode == "standing_up" else self.land_altitude_m,
        )
        target_altitude = self._clamp_altitude(target_altitude)
        current_altitude = self._robot_altitudes.get(robot_name, self.height)
        dz = target_altitude - current_altitude
        if mode == "standing_up":
            tolerance = self.goal_altitude_tolerance_m
        else:
            tolerance = min(self.goal_altitude_tolerance_m, self.waypoint_altitude_tolerance_m, 0.03)

        # During stand_up/sit_down command execution we keep horizontal motion idle.
        robot.vx = 0.0
        robot.vy = 0.0
        robot.target_x = robot.x
        robot.target_y = robot.y

        if abs(dz) <= tolerance:
            self._robot_altitudes[robot_name] = target_altitude
            self._robot_vertical_speed[robot_name] = 0.0
            self._set_flight_command(robot_name, None, target_altitude)
            return True

        vertical_speed = max(-self.max_vertical_speed_mps, min(self.max_vertical_speed_mps, dz * 2.0))
        next_altitude = self._clamp_altitude(current_altitude + (vertical_speed * dt))
        self._robot_altitudes[robot_name] = next_altitude
        self._robot_vertical_speed[robot_name] = vertical_speed
        return True

    def _advance_goal(self, robot, goal: LeggedGoalState, dt: float) -> bool:
        current_z = self._robot_altitudes.get(robot.name, self.height)
        dx = goal.target_x - robot.x
        dy = goal.target_y - robot.y
        dz = goal.target_z - current_z
        planar_distance = math.hypot(dx, dy)

        desired_heading = goal.target_yaw
        if planar_distance > self.goal_position_tolerance_m:
            desired_heading = math.atan2(dy, dx)

        yaw_error = _normalize_angle(desired_heading - robot.yaw)
        max_turn = self.max_turn_rate_rps * dt
        yaw_step = max(-max_turn, min(max_turn, yaw_error))
        robot.yaw = _normalize_angle(robot.yaw + yaw_step)

        if planar_distance > self.goal_position_tolerance_m:
            target_speed = min(self.max_speed, max(0.15, planar_distance * 1.5))
            robot.vx = math.cos(robot.yaw) * target_speed
            robot.vy = math.sin(robot.yaw) * target_speed
            robot.x += robot.vx * dt
            robot.y += robot.vy * dt
        else:
            snap_xy = min(1.0, dt * 8.0)
            robot.x += dx * snap_xy
            robot.y += dy * snap_xy
            robot.vx = 0.0
            robot.vy = 0.0

        if abs(dz) > self.goal_altitude_tolerance_m:
            target_vz = max(-self.max_vertical_speed_mps, min(self.max_vertical_speed_mps, dz * 2.0))
            current_z = self._clamp_altitude(current_z + (target_vz * dt))
            self._robot_vertical_speed[robot.name] = target_vz
            self._robot_altitudes[robot.name] = current_z
        else:
            current_z += dz * min(1.0, dt * 8.0)
            current_z = self._clamp_altitude(current_z)
            self._robot_altitudes[robot.name] = current_z
            self._robot_vertical_speed[robot.name] = 0.0

        remaining_distance = math.hypot(goal.target_x - robot.x, goal.target_y - robot.y)
        remaining_altitude = abs(goal.target_z - current_z)
        remaining_yaw = abs(_normalize_angle(goal.target_yaw - robot.yaw))

        if (
            remaining_distance <= self.goal_position_tolerance_m
            and remaining_altitude <= self.goal_altitude_tolerance_m
            and remaining_yaw <= self.goal_yaw_tolerance_rad
        ):
            robot.x = goal.target_x
            robot.y = goal.target_y
            robot.yaw = goal.target_yaw
            self._robot_altitudes[robot.name] = self._clamp_altitude(goal.target_z)
            robot.vx = 0.0
            robot.vy = 0.0
            self._robot_vertical_speed[robot.name] = 0.0
            return True

        return False

    def _advance_waypoint(self, robot, goal: LeggedWaypointGoal, dt: float) -> bool:
        current_z = self._robot_altitudes.get(robot.name, self.height)
        dx = goal.target_x - robot.x
        dy = goal.target_y - robot.y
        dz = goal.target_z - current_z
        planar_distance = math.hypot(dx, dy)

        desired_heading = goal.target_yaw
        if planar_distance > self.waypoint_position_tolerance_m:
            desired_heading = math.atan2(dy, dx)

        yaw_error = _normalize_angle(desired_heading - robot.yaw)
        max_turn = self.max_turn_rate_rps * dt
        yaw_step = max(-max_turn, min(max_turn, yaw_error))
        robot.yaw = _normalize_angle(robot.yaw + yaw_step)

        if planar_distance > self.waypoint_position_tolerance_m:
            target_speed = min(self.max_speed, max(0.15, planar_distance * 1.5))
            robot.vx = math.cos(robot.yaw) * target_speed
            robot.vy = math.sin(robot.yaw) * target_speed
            robot.x += robot.vx * dt
            robot.y += robot.vy * dt
        else:
            snap_xy = min(1.0, dt * 8.0)
            robot.x += dx * snap_xy
            robot.y += dy * snap_xy
            robot.vx = 0.0
            robot.vy = 0.0

        if abs(dz) > self.waypoint_altitude_tolerance_m:
            target_vz = max(-self.max_vertical_speed_mps, min(self.max_vertical_speed_mps, dz * 2.0))
            current_z = self._clamp_altitude(current_z + (target_vz * dt))
            self._robot_vertical_speed[robot.name] = target_vz
            self._robot_altitudes[robot.name] = current_z
        else:
            current_z += dz * min(1.0, dt * 8.0)
            current_z = self._clamp_altitude(current_z)
            self._robot_altitudes[robot.name] = current_z
            self._robot_vertical_speed[robot.name] = 0.0

        remaining_distance = math.hypot(goal.target_x - robot.x, goal.target_y - robot.y)
        remaining_altitude = abs(goal.target_z - current_z)
        remaining_yaw = abs(_normalize_angle(goal.target_yaw - robot.yaw))

        if (
            remaining_distance <= self.waypoint_position_tolerance_m
            and remaining_altitude <= self.waypoint_altitude_tolerance_m
            and remaining_yaw <= self.waypoint_yaw_tolerance_rad
        ):
            robot.x = goal.target_x
            robot.y = goal.target_y
            robot.yaw = goal.target_yaw
            self._robot_altitudes[robot.name] = self._clamp_altitude(goal.target_z)
            robot.vx = 0.0
            robot.vy = 0.0
            self._robot_vertical_speed[robot.name] = 0.0
            return True

        return False

    def _update_robot_motion(self, dt: float, now_sec: float):
        for robot in self.robots:
            name = robot.name

            if self._is_teleop_active(name, now_sec):
                self._cancel_tasks_for_teleop(name)
                self._apply_teleop_to_robot(robot, dt)
                continue

            if self._apply_flight_command(robot, dt):
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
        msg.twist.twist.linear.z = self._robot_vertical_speed.get(robot.name, 0.0) * self.scale
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0
        publisher.publish(msg)


def build_legged_ops_parser():
    parser = build_ops_suite_parser()
    parser.description = (
        "Publish fake TF for legged robots with 3D task execution: teleop cmd_vel, "
        "3D go-to goal_pose, and 3D waypoint_path."
    )
    parser.set_defaults(
        robot_count=3,
        robot_name="legged",
        robot_names="legged_1,legged_2,legged_3",
        robot_base_frames="legged_1:base_link,legged_2:base_link,legged_3:base_link",
        max_speed=1.6,
        max_turn_rate=1.8,
        height=0.3,
        static_camera=True,
        publish_compressed_images=True,
        publish_images=False,
        publish_occupancy_grid=False,
        strict_single_tf_publisher=True,
    )
    parser.add_argument(
        "--min-height",
        type=float,
        default=0.0,
        help="Minimum height in map meters for legged motion and setpoints.",
    )
    parser.add_argument(
        "--max-height",
        type=float,
        default=10.0,
        help="Maximum height in map meters for legged motion and setpoints.",
    )
    parser.add_argument(
        "--goal-height-tolerance",
        type=float,
        default=0.18,
        help="Go-to-point completion height tolerance in map meters.",
    )
    parser.add_argument(
        "--waypoint-height-tolerance",
        type=float,
        default=0.18,
        help="Waypoint completion height tolerance in map meters.",
    )
    parser.add_argument(
        "--max-vertical-speed",
        type=float,
        default=1.2,
        help="Maximum vertical speed in m/s used for teleop and task execution.",
    )
    parser.add_argument(
        "--stand_up-height",
        type=float,
        default=1.2,
        help="Target height in map meters when /<robot>/stand_up is received.",
    )
    parser.add_argument(
        "--sit_down-height",
        type=float,
        default=0.0,
        help="Target height in map meters when /<robot>/sit_down is received.",
    )
    return parser


def run_from_args(args):
    rclpy.init()
    robot_names = resolve_ops_robot_names(args)
    parsed_resolutions = parse_resolution_list(args.image_resolutions)
    if not parsed_resolutions:
        parsed_resolutions = [(args.image_width, args.image_height)]

    node = LeggedOpsSuiteFakeTFPublisher(
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
        min_altitude_m=args.min_height,
        max_altitude_m=args.max_height,
        goal_altitude_tolerance_m=args.goal_height_tolerance,
        waypoint_altitude_tolerance_m=args.waypoint_height_tolerance,
        max_vertical_speed_mps=args.max_vertical_speed,
        takeoff_altitude_m=args.stand_up_height,
        land_altitude_m=args.sit_down_height,
        robot_names=robot_names,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
        robot_base_frames=parse_robot_base_frames(args.robot_base_frames),
        rate_hz=args.rate,
        height=args.height,
        scale=args.scale,
        mode="legged_ops_suite",
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
    parser = build_legged_ops_parser()
    args = parser.parse_args()
    run_from_args(args)


if __name__ == "__main__":
    main()


