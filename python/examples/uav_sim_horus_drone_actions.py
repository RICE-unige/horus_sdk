#!/usr/bin/env python3
"""Relay HORUS UAV demo topics into the simulator topics HORUS can drive."""

from __future__ import annotations

import argparse
import math

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Empty, String


def _multiply_quaternions(a: Quaternion, b: Quaternion) -> Quaternion:
    q = Quaternion()
    q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
    q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
    q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm > 1e-9:
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
    return q


class DroneActionRelay(Node):
    def __init__(self, robot_name: str, command_topic: str) -> None:
        super().__init__("uav_sim_horus_drone_action_relay")
        self._publisher = self.create_publisher(String, command_topic, 10)
        self._command_topic = command_topic

        empty_actions = {
            "takeoff": "takeoff",
            "land": "land",
            "hold": "hold",
            "arm": "arm",
            "disarm": "disarm",
        }
        for action, command in empty_actions.items():
            topic = f"/{robot_name}/{action}"
            self.create_subscription(Empty, topic, self._callback(command), 10)
            self.get_logger().info(f"listening on {topic} -> {command_topic}:{command}")

        cancel_topic = f"/{robot_name}/goal_cancel"
        self.create_subscription(String, cancel_topic, self._callback("cancel"), 10)
        self.get_logger().info(f"listening on {cancel_topic} -> {command_topic}:cancel")

    def _callback(self, command: str):
        def publish_command(_msg: Empty) -> None:
            msg = String()
            msg.data = command
            self._publisher.publish(msg)
            self.get_logger().info(f"sent {command!r} to {self._command_topic}")

        return publish_command


class GoalFrameRelay(Node):
    """Relay HORUS ENU map goals to the UAV sim goal topic."""

    _MAP_NED_TO_MAP = Quaternion(x=-0.7071068, y=-0.7071068, z=0.0, w=0.0)
    _MAP_TO_MAP_NED = Quaternion(x=0.7071068, y=0.7071068, z=0.0, w=0.0)

    def __init__(
        self,
        input_topic: str,
        output_topic: str,
        transform_mode: str = "map_to_ned",
    ) -> None:
        super().__init__("uav_sim_horus_goal_frame_relay")
        self._publisher = self.create_publisher(PoseStamped, output_topic, 10)
        self._output_topic = output_topic
        self._transform_mode = transform_mode.strip().lower()
        self.create_subscription(PoseStamped, input_topic, self._on_goal, 10)
        self.get_logger().info(
            f"listening on {input_topic} -> {output_topic} mode={self._transform_mode}"
        )

    def _on_goal(self, msg: PoseStamped) -> None:
        out = PoseStamped()
        out.header = msg.header
        out.pose.position.x = msg.pose.position.x
        out.pose.position.y = msg.pose.position.y
        out.pose.position.z = msg.pose.position.z
        out.pose.orientation.x = msg.pose.orientation.x
        out.pose.orientation.y = msg.pose.orientation.y
        out.pose.orientation.z = msg.pose.orientation.z
        out.pose.orientation.w = msg.pose.orientation.w

        if self._transform_mode in ("map_to_ned", "ned"):
            out.header.frame_id = "map_ned"
            out.pose.position.x = msg.pose.position.y
            out.pose.position.y = msg.pose.position.x
            out.pose.position.z = -msg.pose.position.z
            out.pose.orientation = _multiply_quaternions(
                self._MAP_TO_MAP_NED,
                msg.pose.orientation,
            )
        elif self._transform_mode not in ("enu", "map", "pass_through"):
            self.get_logger().warning(
                f"unknown goal transform mode {self._transform_mode!r}; passing through"
            )

        self._publisher.publish(out)
        self.get_logger().info(
            "relayed goal "
            f"in=({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}) "
            f"out=({out.pose.position.x:.2f}, {out.pose.position.y:.2f}, {out.pose.position.z:.2f}) "
            f"frame={out.header.frame_id}"
        )


class OccupancyGridMeshRelay(Node):
    """Publish a HORUS-compatible mesh marker from the projected map.

    HORUS' current octomap visualizer consumes visualization_msgs/Marker triangle
    meshes. The UAV sim currently exposes a raw octomap_msgs/Octomap topic, so
    this lightweight mesh keeps the 3D map path visible until a native Octomap
    renderer/converter is added.
    """

    def __init__(
        self,
        grid_topic: str,
        mesh_topic: str,
        *,
        occupied_threshold: int = 50,
        z_height_m: float = 0.08,
        max_triangles: int = 100000,
        republish_period_sec: float = 2.0,
    ) -> None:
        super().__init__("uav_sim_horus_projected_map_mesh_relay")
        self._grid_topic = grid_topic
        self._mesh_topic = mesh_topic
        self._occupied_threshold = max(0, min(100, int(occupied_threshold)))
        self._z_height_m = max(0.0, float(z_height_m))
        self._max_triangles = max(2, int(max_triangles))
        self._latest_marker: Marker | None = None
        qos = _transient_local_qos()
        self._publisher = self.create_publisher(Marker, mesh_topic, qos)
        self.create_subscription(OccupancyGrid, grid_topic, self._on_grid, qos)
        self.create_timer(max(0.5, republish_period_sec), self._republish)
        self.get_logger().info(
            f"listening on {grid_topic} -> {mesh_topic} "
            f"threshold={self._occupied_threshold} max_triangles={self._max_triangles}"
        )

    def _on_grid(self, msg: OccupancyGrid) -> None:
        marker = self._build_marker(msg)
        self._latest_marker = marker
        self._publisher.publish(marker)

    def _republish(self) -> None:
        if self._latest_marker is not None:
            self._latest_marker.header.stamp = self.get_clock().now().to_msg()
            self._publisher.publish(self._latest_marker)

    def _build_marker(self, msg: OccupancyGrid) -> Marker:
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id or "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "uav_sim_projected_map_mesh"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.70
        marker.color.g = 0.74
        marker.color.b = 0.78
        marker.color.a = 0.95

        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)
        max_cells = max(1, self._max_triangles // 2)
        occupied = [
            idx
            for idx, value in enumerate(msg.data)
            if value >= self._occupied_threshold
        ]
        stride = max(1, math.ceil(len(occupied) / max_cells))

        for idx in occupied[::stride]:
            y, x = divmod(idx, width)
            if y >= height:
                continue
            x0 = origin_x + x * resolution
            y0 = origin_y + y * resolution
            x1 = x0 + resolution
            y1 = y0 + resolution
            z = self._z_height_m
            marker.points.extend(
                [
                    Point(x=x0, y=y0, z=z),
                    Point(x=x1, y=y0, z=z),
                    Point(x=x1, y=y1, z=z),
                    Point(x=x0, y=y0, z=z),
                    Point(x=x1, y=y1, z=z),
                    Point(x=x0, y=y1, z=z),
                ]
            )

        if not marker.points:
            x0 = origin_x
            y0 = origin_y
            x1 = origin_x + width * resolution
            y1 = origin_y + height * resolution
            z = self._z_height_m
            marker.points.extend(
                [
                    Point(x=x0, y=y0, z=z),
                    Point(x=x1, y=y0, z=z),
                    Point(x=x1, y=y1, z=z),
                    Point(x=x0, y=y0, z=z),
                    Point(x=x1, y=y1, z=z),
                    Point(x=x0, y=y1, z=z),
                ]
            )
            self.get_logger().warning(
                "projected map has no occupied cells; publishing map extent plane "
                f"as fallback mesh on {self._mesh_topic}"
            )

        self.get_logger().info(
            f"published mesh marker cells={len(occupied)} sampled={len(occupied[::stride])} "
            f"triangles={len(marker.points) // 3} topic={self._mesh_topic}"
        )
        return marker


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-name", default="arancino_uav")
    parser.add_argument("--command-topic", default="/uav_sim/command")
    parser.add_argument("--goal-input-topic", default="/arancino_uav/goal")
    parser.add_argument("--goal-output-topic", default="/uav_sim/goal")
    parser.add_argument("--goal-transform", default="enu")
    parser.add_argument("--no-goal-relay", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    nodes = [DroneActionRelay(args.robot_name, args.command_topic)]
    if not args.no_goal_relay:
        nodes.append(
            GoalFrameRelay(
                args.goal_input_topic,
                args.goal_output_topic,
                args.goal_transform,
            )
        )
    try:
        executor = SingleThreadedExecutor()
        for node in nodes:
            executor.add_node(node)
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
