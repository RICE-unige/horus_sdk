#!/usr/bin/env python3
"""Register the Arancino UAV sim with HORUS MR from WSL."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import subprocess
import sys
import threading
from pathlib import Path
from typing import Iterator, Optional

SDK_PYTHON = Path(__file__).resolve().parents[1]
if (SDK_PYTHON / "horus").is_dir():
    sys.path.insert(0, str(SDK_PYTHON))

from horus.robot import Robot, RobotDimensions, RobotType, register_robots


DEFAULT_COMMAND_TOPIC = "/uav_sim/command"
DEFAULT_GOAL_RELAY_TOPIC = "/arancino_uav/goal"
DEFAULT_SIM_GOAL_TOPIC = "/uav_sim/goal"
DEFAULT_NATIVE_OCTOMAP_TOPIC = "/uav_sim/octomap_binary"
DEFAULT_OCTOMAP_MESH_TOPIC = "/map_3d_octomap_mesh"


class _UavSimRelayRuntime:
    def __init__(
        self,
        robot_name: str,
        *,
        command_topic: str,
        goal_input_topic: str,
        goal_output_topic: str,
        goal_transform: str,
        octomap_topic: str,
        mesh_topic: str,
        enable_actions: bool,
        enable_goal: bool,
        enable_octomap: bool,
        max_octomap_triangles: int,
        octomap_voxel_scale: float,
        octomap_marker_style: str,
    ) -> None:
        self.robot_name = robot_name
        self.command_topic = command_topic
        self.goal_input_topic = goal_input_topic
        self.goal_output_topic = goal_output_topic
        self.goal_transform = goal_transform
        self.octomap_topic = octomap_topic
        self.mesh_topic = mesh_topic
        self.enable_actions = enable_actions
        self.enable_goal = enable_goal
        self.enable_octomap = enable_octomap
        self.max_octomap_triangles = max_octomap_triangles
        self.octomap_voxel_scale = octomap_voxel_scale
        self.octomap_marker_style = octomap_marker_style
        self._executor = None
        self._nodes = []
        self._octomap_process: Optional[subprocess.Popen] = None
        self._thread: Optional[threading.Thread] = None
        self._owns_rclpy = False
        self._rclpy = None

    def start(self) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from uav_sim_horus_drone_actions import (
            DroneActionRelay,
            GoalFrameRelay,
        )

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True

        if self.enable_actions:
            self._nodes.append(DroneActionRelay(self.robot_name, self.command_topic))
        if self.enable_goal:
            self._nodes.append(
                GoalFrameRelay(
                    self.goal_input_topic,
                    self.goal_output_topic,
                    self.goal_transform,
                )
            )
        if self.enable_octomap:
            tool = Path(__file__).resolve().parent / "tools" / "uav_sim_octomap_marker_relay.py"
            self._octomap_process = subprocess.Popen(
                [
                    sys.executable,
                    str(tool),
                    "--input-topic",
                    self.octomap_topic,
                    "--output-topic",
                    self.mesh_topic,
                    "--max-triangles",
                    str(self.max_octomap_triangles),
                    "--voxel-scale",
                    str(self.octomap_voxel_scale),
                    "--style",
                    self.octomap_marker_style,
                ]
            )

        self._executor = SingleThreadedExecutor()
        for node in self._nodes:
            self._executor.add_node(node)
        self._thread = threading.Thread(
            target=self._executor.spin,
            name="uav-sim-horus-action-relay",
            daemon=True,
        )
        self._thread.start()
        print(
            "[uav_sim] Runtime relays active: "
            f"actions={self.enable_actions}, goal={self.enable_goal} "
            f"({self.goal_input_topic}->{self.goal_output_topic}, mode={self.goal_transform}), "
            f"octomap={self.enable_octomap} ({self.octomap_topic}->{self.mesh_topic})"
        )

    def stop(self) -> None:
        if self._octomap_process is not None and self._octomap_process.poll() is None:
            self._octomap_process.terminate()
            try:
                self._octomap_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._octomap_process.kill()
                self._octomap_process.wait(timeout=2.0)
        if self._executor is not None:
            self._executor.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for node in self._nodes:
            node.destroy_node()
        if self._owns_rclpy and self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()


@contextmanager
def _maybe_action_relay(
    enabled: bool,
    robot_name: str,
    *,
    command_topic: str,
    goal_input_topic: str,
    goal_output_topic: str,
    goal_transform: str,
    octomap_topic: str,
    mesh_topic: str,
    enable_actions: bool,
    enable_goal: bool,
    enable_octomap: bool,
    max_octomap_triangles: int,
    octomap_voxel_scale: float,
    octomap_marker_style: str,
) -> Iterator[None]:
    relay: Optional[_UavSimRelayRuntime] = None
    if enabled:
        relay = _UavSimRelayRuntime(
            robot_name,
            command_topic=command_topic,
            goal_input_topic=goal_input_topic,
            goal_output_topic=goal_output_topic,
            goal_transform=goal_transform,
            octomap_topic=octomap_topic,
            mesh_topic=mesh_topic,
            enable_actions=enable_actions,
            enable_goal=enable_goal,
            enable_octomap=enable_octomap,
            max_octomap_triangles=max_octomap_triangles,
            octomap_voxel_scale=octomap_voxel_scale,
            octomap_marker_style=octomap_marker_style,
        )
        relay.start()
    try:
        yield
    finally:
        if relay is not None:
            relay.stop()


def build_registration(
    command_topic: str = DEFAULT_COMMAND_TOPIC,
    goal_topic: str = DEFAULT_GOAL_RELAY_TOPIC,
    octomap_mesh_topic: str = DEFAULT_OCTOMAP_MESH_TOPIC,
    native_octomap_topic: str = DEFAULT_NATIVE_OCTOMAP_TOPIC,
    max_octomap_triangles: int = 120000,
) -> tuple[Robot, object]:
    robot = Robot(
        name="arancino_uav",
        robot_type=RobotType.DRONE,
        dimensions=RobotDimensions(length=0.46, width=0.46, height=0.18),
    )
    robot.configure_ros_binding(
        tf_mode="flat",
        topic_mode="flat",
        base_frame="drone_base_link",
    )
    robot.configure_robot_manager()
    robot.configure_teleop(
        command_topic="/uav_sim/teleop/cmd_vel",
        robot_profile="drone",
        publish_rate_hz=30.0,
        linear_xy_max_mps=1.0,
        linear_z_max_mps=0.6,
        angular_z_max_rps=0.8,
    )
    robot.configure_navigation_tasks(
        waypoint_enabled=False,
        goal_topic=goal_topic,
        cancel_topic=command_topic,
        goal_status_topic="/uav_sim/status",
        waypoint_path_topic="/arancino_uav/waypoint_path",
        waypoint_status_topic="/uav_sim/status",
        frame_id="map",
        position_tolerance_m=0.25,
        yaw_tolerance_deg=12.0,
        min_altitude_m=0.3,
        max_altitude_m=8.0,
    )

    dataviz = robot.create_dataviz("arancino_uav_viz")
    robot.add_path_planning_to_dataviz(
        dataviz,
        global_path_topic="/uav_sim/path",
    )
    dataviz.add_3d_octomap(
        octomap_mesh_topic,
        frame_id="map",
        render_options={
            "render_mode": "surface_mesh",
            "native_topic": native_octomap_topic,
            "native_frame": "map",
            "native_binary_only": False,
            "max_triangles": max_octomap_triangles,
            "alpha": 0.95,
        },
    )
    dataviz.add_tf_tree("/tf", frame_id="map")
    return robot, dataviz


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--no-wait-for-app", action="store_true")
    parser.add_argument("--command-topic", default=DEFAULT_COMMAND_TOPIC)
    parser.add_argument("--goal-topic", default=DEFAULT_GOAL_RELAY_TOPIC)
    parser.add_argument("--sim-goal-topic", default=DEFAULT_SIM_GOAL_TOPIC)
    parser.add_argument(
        "--goal-transform",
        choices=("map_to_ned", "enu", "pass_through"),
        default="enu",
    )
    parser.add_argument("--native-octomap-topic", default=DEFAULT_NATIVE_OCTOMAP_TOPIC)
    parser.add_argument("--octomap-mesh-topic", default=DEFAULT_OCTOMAP_MESH_TOPIC)
    parser.add_argument("--max-octomap-triangles", type=int, default=120000)
    parser.add_argument("--octomap-voxel-scale", type=float, default=2.0)
    parser.add_argument(
        "--octomap-marker-style",
        choices=("rviz_voxels", "surface_mesh"),
        default="surface_mesh",
    )
    parser.add_argument("--no-runtime-relays", action="store_true")
    parser.add_argument("--no-action-relay", action="store_true")
    parser.add_argument("--no-goal-relay", action="store_true")
    parser.add_argument("--no-octomap-relay", action="store_true")
    args = parser.parse_args()

    robot, dataviz = build_registration(
        command_topic=args.command_topic,
        goal_topic=args.goal_topic,
        octomap_mesh_topic=args.octomap_mesh_topic,
        native_octomap_topic=args.native_octomap_topic,
        max_octomap_triangles=args.max_octomap_triangles,
    )
    if args.dry_run:
        print(
            json.dumps(
                {
                    "robot": robot.name,
                    "robot_type": robot.robot_type.value,
                    "base_frame": robot.resolve_tf_frame(),
                    "teleop": "/uav_sim/teleop/cmd_vel",
                    "goal": args.goal_topic,
                    "sim_goal": args.sim_goal_topic,
                    "goal_transform": args.goal_transform,
                    "goal_cancel": args.command_topic,
                    "takeoff": "/arancino_uav/takeoff",
                    "land": "/arancino_uav/land",
                    "hold": "/arancino_uav/hold",
                    "arm": "/arancino_uav/arm",
                    "disarm": "/arancino_uav/disarm",
                    "command": args.command_topic,
                    "action_relay": not args.no_runtime_relays and not args.no_action_relay,
                    "goal_relay": not args.no_runtime_relays and not args.no_goal_relay,
                    "octomap_relay": not args.no_runtime_relays and not args.no_octomap_relay,
                    "status": "/uav_sim/status",
                    "path": "/uav_sim/path",
                    "octomap": args.octomap_mesh_topic,
                    "native_octomap": args.native_octomap_topic,
                    "max_octomap_triangles": args.max_octomap_triangles,
                    "octomap_voxel_scale": args.octomap_voxel_scale,
                    "octomap_marker_style": args.octomap_marker_style,
                    "visualizations": len(dataviz.visualizations),
                },
                indent=2,
            )
        )
        return

    with _maybe_action_relay(
        not args.no_runtime_relays,
        robot.name,
        command_topic=args.command_topic,
        goal_input_topic=args.goal_topic,
        goal_output_topic=args.sim_goal_topic,
        goal_transform=args.goal_transform,
        octomap_topic=args.native_octomap_topic,
        mesh_topic=args.octomap_mesh_topic,
        enable_actions=not args.no_action_relay,
        enable_goal=not args.no_goal_relay,
        enable_octomap=not args.no_octomap_relay,
        max_octomap_triangles=args.max_octomap_triangles,
        octomap_voxel_scale=args.octomap_voxel_scale,
        octomap_marker_style=args.octomap_marker_style,
    ):
        success, result = register_robots(
            [robot],
            datavizs=[dataviz],
            workspace_scale=0.05,
            compass_enabled=False,
            keep_alive=not args.once,
            timeout_sec=args.timeout,
            wait_for_app_before_register=not args.no_wait_for_app,
        )
    if not success:
        raise SystemExit(f"HORUS registration failed: {result}")


if __name__ == "__main__":
    main()
