#!/usr/bin/env python3
"""Register the H-CoRE heterogeneous drone + rover simulation with HORUS MR.

The H-CoRE sim exposes a mixed ROS graph on domain 17. The clean HORUS setup is:

* a drone registered through the local H-CoRE drone bridge topics
  (/drone/cmd_vel and /drone/nav_goal);
* a wheeled rover registered through /rover/cmd_vel and LaserScan data;
* shared world data on /tf, /map, and an optional OctoMap-to-marker relay.

Typical WSL use with the H-CoRE direct DDS profile:

    cd ~/horus_sdk
    source ~/hcore_zenoh_client/hcore_ros_env.sh
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/hcore_heterogeneous_registration.py

The script also sets the same direct DDS defaults itself when the H-CoRE CycloneDDS
profile exists. Use --start-zenoh only when the remote H-CoRE Zenoh listener is
running and you explicitly want to route through Zenoh:

    PYTHONPATH=python:$PYTHONPATH python3 python/examples/hcore_heterogeneous_registration.py --start-zenoh
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack, contextmanager
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
from typing import Iterator, Optional

SDK_PYTHON = Path(__file__).resolve().parents[1]
if (SDK_PYTHON / "horus").is_dir():
    sys.path.insert(0, str(SDK_PYTHON))

from horus.bridge.robot_registry import get_robot_registry_client
from horus.robot import Robot, RobotDimensions, RobotType, is_registration_cancelled, register_robots
from horus.sensors import Camera, LaserScan, Lidar3D


DEFAULT_HCORE_DIR = Path.home() / "hcore_zenoh_client"
DEFAULT_ROS_DOMAIN_ID = 17
DEFAULT_ZENOH_HOST = "arancino"
DEFAULT_ZENOH_PORT = 7447
DEFAULT_OCTOMAP_MESH_TOPIC = "/hcore/octomap_mesh"
DEFAULT_HCORE_TF_TOPIC = "/hcore/tf"
DEFAULT_HCORE_TF_STATIC_TOPIC = "/hcore/tf_static"
DEFAULT_CYCLONEDDS_URI_PATH = Path.home() / "hcore_zenoh_client" / "cyclonedds_hcore_wsl.xml"
DEFAULT_ASSETS_DIR = Path(__file__).resolve().parent / ".local_assets"
DEFAULT_ROBOT_DESCRIPTION_DIR = DEFAULT_ASSETS_DIR / "robot_descriptions"
DEFAULT_HCORE_ROVER_URDF = DEFAULT_ROBOT_DESCRIPTION_DIR / "hcore_rover_prefixed.urdf"
DEFAULT_HCORE_ROVER_MESH_ROOT = DEFAULT_ROBOT_DESCRIPTION_DIR / "meshes_root"


def _default_cyclonedds_uri() -> str:
    explicit = os.environ.get("CYCLONEDDS_URI", "").strip()
    if explicit:
        return explicit
    if DEFAULT_CYCLONEDDS_URI_PATH.exists():
        return f"file://{DEFAULT_CYCLONEDDS_URI_PATH}"
    return ""


def _parse_resolution(value: str) -> tuple[int, int]:
    raw = str(value or "").strip().lower().replace(",", "x")
    if "x" not in raw:
        raise argparse.ArgumentTypeError("resolution must be WIDTHxHEIGHT")
    width_raw, height_raw = raw.split("x", 1)
    try:
        width = int(width_raw)
        height = int(height_raw)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("resolution must contain integer dimensions") from exc
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError("resolution dimensions must be positive")
    return width, height


def _bash_ros_command(
    command: list[str],
    *,
    ros_domain_id: int,
    ros_localhost_only: int,
    rmw_implementation: str,
    cyclonedds_uri: str = "",
    extra_exports: Optional[dict[str, str]] = None,
) -> list[str]:
    exports = [
        "set -e",
        "source /opt/ros/jazzy/setup.bash",
        f"export ROS_DOMAIN_ID={int(ros_domain_id)}",
        f"export ROS_LOCALHOST_ONLY={int(ros_localhost_only)}",
        f"export RMW_IMPLEMENTATION={shlex.quote(rmw_implementation)}",
    ]
    if cyclonedds_uri:
        exports.append(f"export CYCLONEDDS_URI={shlex.quote(cyclonedds_uri)}")
    for key, value in (extra_exports or {}).items():
        exports.append(f"export {key}={shlex.quote(str(value))}")
    exports.append("exec " + shlex.join(command))
    return ["bash", "-lc", "; ".join(exports)]


def _process_exists(pattern: str) -> bool:
    try:
        completed = subprocess.run(
            ["pgrep", "-f", pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return completed.returncode == 0
    except Exception:
        return False


def _terminate_processes(label: str, pattern: str, timeout_sec: float = 3.0) -> None:
    try:
        completed = subprocess.run(
            ["pgrep", "-f", pattern],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except Exception:
        return

    pids = []
    for raw_pid in completed.stdout.split():
        try:
            pid = int(raw_pid)
        except ValueError:
            continue
        if pid == os.getpid():
            continue
        pids.append(pid)

    if not pids:
        return

    print(f"[hcore] Replacing existing {label} process(es): {', '.join(str(pid) for pid in pids)}")
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    deadline = time.monotonic() + max(0.1, timeout_sec)
    while time.monotonic() < deadline:
        alive = []
        for pid in pids:
            try:
                os.kill(pid, 0)
                alive.append(pid)
            except ProcessLookupError:
                pass
        if not alive:
            return
        time.sleep(0.1)

    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


@contextmanager
def _managed_process(
    label: str,
    command: list[str],
    *,
    log_dir: Path,
    existing_pattern: str = "",
    startup_delay_sec: float = 0.0,
    replace_existing: bool = False,
) -> Iterator[None]:
    if existing_pattern and replace_existing:
        _terminate_processes(label, existing_pattern)
    elif existing_pattern and _process_exists(existing_pattern):
        print(f"[hcore] Reusing existing {label} process.")
        yield
        return

    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"{label}.log"
    log_file = log_path.open("ab")
    preexec_fn = os.setsid if hasattr(os, "setsid") else None
    process = subprocess.Popen(
        command,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=preexec_fn,
    )
    print(f"[hcore] Started {label} pid={process.pid}, log={log_path}")
    if startup_delay_sec > 0.0:
        time.sleep(startup_delay_sec)
        if process.poll() is not None:
            log_file.close()
            raise RuntimeError(f"{label} exited early; check {log_path}")
    try:
        yield
    finally:
        if process.poll() is None:
            print(f"[hcore] Stopping {label} pid={process.pid}")
            try:
                if preexec_fn is not None:
                    os.killpg(process.pid, signal.SIGTERM)
                else:
                    process.terminate()
                process.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                if preexec_fn is not None:
                    os.killpg(process.pid, signal.SIGKILL)
                else:
                    process.kill()
                process.wait(timeout=2.0)
        log_file.close()


def _start_zenoh_bridge(args: argparse.Namespace, stack: ExitStack) -> None:
    command = [
        "zenoh-bridge-ros2dds",
        "-d",
        str(args.ros_domain_id),
        "-e",
        f"tcp/{args.zenoh_host}:{args.zenoh_port}",
        "--no-multicast-scouting",
        "--ros-localhost-only",
        "--pub-max-frequency",
        f"{args.drone_camera_topic}={args.drone_camera_max_hz}",
        "--pub-max-frequency",
        f"{args.drone_depth_points_topic}={args.depth_points_max_hz}",
        "--pub-max-frequency",
        f"{args.rover_depth_points_topic}={args.depth_points_max_hz}",
        args.zenoh_mode,
    ]
    stack.enter_context(
        _managed_process(
            "hcore_zenoh_bridge",
            _bash_ros_command(
                command,
                ros_domain_id=args.ros_domain_id,
                ros_localhost_only=1,
                rmw_implementation=args.rmw_implementation,
                cyclonedds_uri=args.cyclonedds_uri,
            ),
            log_dir=args.log_dir,
            existing_pattern=rf"zenoh-bridge-ros2dds.*tcp/{args.zenoh_host}:{args.zenoh_port}",
            startup_delay_sec=4.0,
        )
    )


def _start_drone_bridge(args: argparse.Namespace, stack: ExitStack) -> None:
    bridge_script = args.hcore_dir / "drone_bridge.py"
    if not bridge_script.exists():
        raise FileNotFoundError(f"Missing H-CoRE drone bridge: {bridge_script}")

    command = [
        sys.executable,
        str(bridge_script),
        "--frame",
        args.drone_goal_frame,
        "--base-frame",
        args.drone_source_base_frame,
        "bridge",
        "--cmd-vel-topic",
        args.drone_cmd_vel_topic,
        "--goal-topic",
        args.drone_direct_goal_topic,
        "--nav-goal-topic",
        args.drone_nav_goal_topic,
        "--takeoff-topic",
        f"/{args.drone_name}/takeoff",
        "--land-topic",
        f"/{args.drone_name}/land",
        "--stop-topic",
        f"/{args.drone_name}/stop",
    ]
    stack.enter_context(
        _managed_process(
            "hcore_drone_bridge",
            _bash_ros_command(
                command,
                ros_domain_id=args.ros_domain_id,
                ros_localhost_only=args.ros_localhost_only,
                rmw_implementation=args.rmw_implementation,
                cyclonedds_uri=args.cyclonedds_uri,
            ),
            log_dir=args.log_dir,
            existing_pattern=r"drone_bridge.py.*bridge",
            startup_delay_sec=1.0,
            replace_existing=True,
        )
    )


def _start_tf_bridge(args: argparse.Namespace, stack: ExitStack) -> None:
    bridge_script = Path(__file__).resolve().parent / "tools" / "hcore_tf_bridge.py"
    command = [
        sys.executable,
        str(bridge_script),
        "--output-tf-topic",
        args.tf_topic,
        "--output-tf-static-topic",
        args.tf_static_topic,
        "--world-frame",
        args.world_frame,
        "--drone-prefix",
        args.drone_topic_prefix.strip("/"),
        "--drone-base-frame",
        args.drone_source_base_frame,
        "--rover-prefix",
        args.rover_topic_prefix.strip("/"),
        "--rover-base-frame",
        args.rover_source_base_frame,
    ]
    if args.rover_pose_topic:
        command.extend(["--rover-pose-topic", args.rover_pose_topic])
    stack.enter_context(
        _managed_process(
            "hcore_tf_bridge",
            _bash_ros_command(
                command,
                ros_domain_id=args.ros_domain_id,
                ros_localhost_only=args.ros_localhost_only,
                rmw_implementation=args.rmw_implementation,
                cyclonedds_uri=args.cyclonedds_uri,
            ),
            log_dir=args.log_dir,
            existing_pattern=r"hcore_tf_bridge.py",
            startup_delay_sec=1.0,
            replace_existing=True,
        )
    )


def _start_octomap_relay(args: argparse.Namespace, stack: ExitStack) -> None:
    relay_script = Path(__file__).resolve().parent / "tools" / "uav_sim_octomap_marker_relay.py"
    command = [
        sys.executable,
        str(relay_script),
        "--input-topic",
        args.octomap_topic,
        "--output-topic",
        args.octomap_mesh_topic,
        "--max-triangles",
        str(args.max_octomap_triangles),
        "--voxel-scale",
        str(args.octomap_voxel_scale),
        "--style",
        args.octomap_marker_style,
        "--min-update-interval-sec",
        str(args.octomap_min_update_interval_sec),
        "--chunk-publish-period-ms",
        str(args.octomap_chunk_publish_period_ms),
        "--chunks-per-tick",
        str(args.octomap_chunks_per_tick),
    ]
    stack.enter_context(
        _managed_process(
            "hcore_octomap_mesh_relay",
            _bash_ros_command(
                command,
                ros_domain_id=args.ros_domain_id,
                ros_localhost_only=args.ros_localhost_only,
                rmw_implementation=args.rmw_implementation,
                cyclonedds_uri=args.cyclonedds_uri,
            ),
            log_dir=args.log_dir,
            existing_pattern=r"uav_sim_octomap_marker_relay",
            startup_delay_sec=1.0,
            replace_existing=True,
        )
    )


def _build_drone(args: argparse.Namespace) -> tuple[Robot, object]:
    robot = Robot(
        name=args.drone_name,
        robot_type=RobotType.DRONE,
        dimensions=RobotDimensions(length=0.46, width=0.46, height=0.18),
    )
    robot.configure_ros_binding(
        tf_mode="prefixed",
        topic_mode="prefixed",
        base_frame=args.drone_source_base_frame,
        tf_prefix=args.drone_topic_prefix.strip("/"),
        topic_prefix=args.drone_topic_prefix,
    )
    robot.configure_robot_manager()
    robot.configure_teleop(
        command_topic=args.drone_cmd_vel_topic,
        robot_profile="drone",
        response_mode="analog",
        publish_rate_hz=30.0,
        linear_xy_max_mps=0.8,
        linear_z_max_mps=0.5,
        angular_z_max_rps=0.8,
    )
    robot.configure_navigation_tasks(
        waypoint_enabled=False,
        goal_topic=args.drone_nav_goal_topic,
        cancel_topic=args.drone_command_topic,
        goal_status_topic=args.drone_status_topic,
        waypoint_path_topic="",
        waypoint_status_topic=args.drone_status_topic,
        frame_id=args.drone_goal_frame,
        position_tolerance_m=0.30,
        yaw_tolerance_deg=15.0,
        min_altitude_m=0.3,
        max_altitude_m=8.0,
    )

    camera = Camera(
        name="drone_rgb",
        frame_id=args.drone_camera_frame,
        topic=args.drone_camera_topic,
        resolution=args.drone_camera_resolution,
        fps=args.drone_camera_max_hz,
        fov=args.drone_camera_fov,
        encoding="rgb8",
        streaming_type="ros",
        minimap_streaming_type="ros",
        teleop_streaming_type="ros",
        startup_mode="minimap",
        minimap_topic=args.drone_camera_topic,
        minimap_image_type="raw",
        teleop_topic=args.drone_camera_topic,
        teleop_image_type="raw",
        minimap_max_fps=min(30, args.drone_camera_max_hz),
    )
    camera.add_metadata("camera_info_topic", args.drone_camera_info_topic)
    camera.configure_projected_view(
        position_offset=(0.0, 0.03, 0.0),
        image_scale=0.099,
        focal_length_scale=0.13,
        show_frustum=True,
        frustum_color="#80D9FFA0",
    )
    camera.configure_minimap_view(size=2.025, position_offset=(0.0, 0.75, 0.0))
    camera.configure_immersive_view(ros_flip_x=False, ros_flip_y=True)
    robot.add_sensor(camera)

    if args.enable_depth_clouds:
        robot.add_sensor(
            Lidar3D(
                name="drone_depth_points",
                frame_id=args.drone_depth_frame,
                topic=args.drone_depth_points_topic,
                vertical_fov=58.0,
                horizontal_fov=87.0,
                min_range=0.2,
                max_range=12.0,
                points_per_second=120000,
            )
        )

    dataviz = robot.create_dataviz(f"{args.drone_name}_viz")
    robot.add_path_planning_to_dataviz(
        dataviz,
        global_path_topic=args.drone_global_path_topic,
        local_path_topic=args.drone_local_path_topic,
        trajectory_topic=args.drone_trajectory_topic,
    )
    dataviz.add_robot_velocity_data(
        robot_name=args.drone_name,
        topic=args.drone_odometry_topic,
        frame_id="map",
        render_options={"color": "#80D9FF", "update_hz": 10.0},
    )
    dataviz.add_robot_odometry_trail(
        robot_name=args.drone_name,
        topic=args.drone_odometry_topic,
        frame_id="map",
        render_options={"color": "#80D9FF", "max_points": 56, "history_seconds": 4.0},
    )
    if args.enable_depth_clouds:
        depth_sensor = robot.get_sensor("drone_depth_points")
        if depth_sensor is not None:
            dataviz.add_sensor_visualization(
                depth_sensor,
                robot_name=args.drone_name,
                enabled=True,
                render_options={
                    "point_size": 0.02,
                    "alpha": 0.65,
                    "max_points_per_frame": 70000,
                },
            )
    return robot, dataviz


def _build_rover(args: argparse.Namespace) -> tuple[Robot, object]:
    robot = Robot(
        name=args.rover_name,
        robot_type=RobotType.WHEELED,
        dimensions=RobotDimensions(length=0.50, width=0.38, height=0.28),
    )
    robot.configure_ros_binding(
        tf_mode="prefixed",
        topic_mode="prefixed",
        base_frame="base_footprint",
        tf_prefix=args.rover_topic_prefix.strip("/"),
        topic_prefix=args.rover_topic_prefix,
    )
    if args.enable_rover_model:
        if args.rover_urdf_path.exists():
            robot.configure_robot_description(
                urdf_path=str(args.rover_urdf_path),
                base_frame="base_footprint",
                urdf_package="rover_description_pkg",
                mesh_root=str(args.rover_mesh_root),
                chunk_size_bytes=12000,
                include_visual_meshes=True,
                visual_mesh_triangle_budget=args.rover_model_triangle_budget,
                body_mesh_mode="preview_mesh",
                enabled=True,
            )
        else:
            print(f"[hcore] Rover URDF not found, skipping robot model: {args.rover_urdf_path}")
    robot.configure_robot_manager()
    robot.configure_teleop(
        command_topic=args.rover_cmd_vel_topic,
        robot_profile="wheeled",
        response_mode="analog",
        publish_rate_hz=30.0,
        linear_xy_max_mps=0.35,
        linear_z_max_mps=0.0,
        angular_z_max_rps=0.9,
    )
    if args.enable_rover_navigation:
        robot.configure_navigation_tasks(
            waypoint_enabled=False,
            goal_topic=args.rover_goal_topic,
            cancel_topic=args.rover_cancel_topic,
            goal_status_topic=args.rover_status_topic,
            waypoint_path_topic="",
            waypoint_status_topic=args.rover_status_topic,
            frame_id=args.world_frame,
            position_tolerance_m=0.25,
            yaw_tolerance_deg=12.0,
        )

    if args.enable_rover_camera or args.use_ptz_as_rover_camera:
        camera = Camera(
            name="rover_rgb",
            frame_id=args.rover_camera_frame,
            topic=args.rover_camera_topic,
            resolution=args.rover_camera_resolution,
            fps=args.rover_camera_max_hz,
            fov=args.rover_camera_fov,
            encoding="rgb8",
            streaming_type="ros",
            minimap_streaming_type="ros",
            teleop_streaming_type="ros",
            startup_mode="minimap",
            minimap_topic=args.rover_camera_topic,
            minimap_image_type="raw",
            teleop_topic=args.rover_camera_topic,
            teleop_image_type="raw",
            minimap_max_fps=min(30, args.rover_camera_max_hz),
        )
        camera.add_metadata("camera_info_topic", args.rover_camera_info_topic)
        camera.configure_projected_view(
            position_offset=(0.0, 0.06, 0.0),
            image_scale=0.126,
            focal_length_scale=0.11,
            show_frustum=True,
            frustum_color="#FFE08AA0",
        )
        camera.configure_minimap_view(size=2.25, position_offset=(0.0, 0.85, 0.0))
        camera.configure_immersive_view(ros_flip_x=False, ros_flip_y=True)
        robot.add_sensor(camera)

    scan = LaserScan(
        name="rover_scan",
        frame_id=args.scan_frame,
        topic=args.scan_topic,
        min_angle=-3.14159,
        max_angle=3.14159,
        angle_increment=0.005,
        min_range=0.08,
        max_range=30.0,
        color="#FFE08A",
        point_size=args.scan_point_size,
    )
    robot.add_sensor(scan)

    if args.enable_depth_clouds:
        robot.add_sensor(
            Lidar3D(
                name="rover_depth_points",
                frame_id=args.rover_depth_frame,
                topic=args.rover_depth_points_topic,
                vertical_fov=58.0,
                horizontal_fov=87.0,
                min_range=0.2,
                max_range=12.0,
                points_per_second=120000,
            )
        )

    dataviz = robot.create_dataviz(f"{args.rover_name}_viz")
    if args.enable_rover_navigation:
        robot.add_path_planning_to_dataviz(
            dataviz,
            global_path_topic=args.rover_global_path_topic,
            local_path_topic=args.rover_local_path_topic,
        )
    dataviz.add_sensor_visualization(
        scan,
        robot_name=args.rover_name,
        enabled=True,
        render_options={"color": "#FFE08A", "point_size": args.scan_point_size, "alpha": 0.9},
    )
    if args.enable_rover_odom_visuals:
        dataviz.add_robot_velocity_data(
            robot_name=args.rover_name,
            topic=args.rover_odometry_topic,
            frame_id="map",
            render_options={"color": "#FFE08A", "update_hz": 10.0},
        )
        dataviz.add_robot_odometry_trail(
            robot_name=args.rover_name,
            topic=args.rover_odometry_topic,
            frame_id="map",
            render_options={"color": "#FFE08A", "max_points": 64, "history_seconds": 5.0},
        )
    if args.enable_depth_clouds:
        depth_sensor = robot.get_sensor("rover_depth_points")
        if depth_sensor is not None:
            dataviz.add_sensor_visualization(
                depth_sensor,
                robot_name=args.rover_name,
                enabled=True,
                render_options={
                    "point_size": 0.02,
                    "alpha": 0.65,
                    "max_points_per_frame": 70000,
                },
            )
    return robot, dataviz


def build_registration(args: argparse.Namespace) -> tuple[list[Robot], list[object]]:
    drone, drone_viz = _build_drone(args)
    rover, rover_viz = _build_rover(args)

    _retarget_robot_transforms_to_tf_topic(drone_viz, args.tf_topic)
    _retarget_robot_transforms_to_tf_topic(rover_viz, args.tf_topic)

    drone_viz.add_tf_tree(args.tf_topic, frame_id=args.world_frame)
    if args.enable_occupancy_grid:
        drone_viz.add_occupancy_grid(
            args.occupancy_grid_topic,
            frame_id="map",
            render_options={
                "color_free": "#202020",
                "color_occupied": "#E8E8E8",
                "alpha": 0.55,
            },
            transport_lane="bulk_replaceable",
        )
    if args.enable_octomap:
        drone_viz.add_3d_octomap(
            args.octomap_mesh_topic,
            frame_id=args.drone_goal_frame,
            render_options={
                "render_mode": "surface_mesh",
                "native_topic": args.octomap_topic,
                "native_frame": args.drone_goal_frame,
                "native_binary_only": False,
                "max_triangles": args.max_octomap_triangles,
                "alpha": 0.92,
            },
            transport_lane="bulk_replaceable",
        )

    return [drone, rover], [drone_viz, rover_viz]


def _retarget_robot_transforms_to_tf_topic(dataviz: object, tf_topic: str) -> None:
    for visualization in getattr(dataviz, "visualizations", []) or []:
        data_source = getattr(visualization, "data_source", None)
        source_type = getattr(getattr(data_source, "source_type", None), "value", "")
        if source_type == "robot_transform":
            data_source.topic = tf_topic


def _clear_stale_registrations(robot_names: list[str], timeout_sec: float) -> None:
    """Clear old HORUS entries from interrupted H-CoRE demo runs."""
    registry = get_robot_registry_client()
    for robot_name in robot_names:
        name = str(robot_name or "").strip()
        if not name:
            continue
        success, result = registry.unregister_robot(name, timeout_sec=timeout_sec)
        if success:
            print(f"[hcore] Cleared stale HORUS registration for {name}.")
            continue
        reason = result.get("error") if isinstance(result, dict) else result
        print(f"[hcore] No stale HORUS registration cleared for {name}: {reason}")


def _add_arguments() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Register the H-CoRE drone + rover simulation in HORUS MR."
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--workspace-scale", type=float, default=0.1)
    parser.add_argument("--no-wait-for-app", action="store_true")
    parser.add_argument("--hcore-dir", type=Path, default=DEFAULT_HCORE_DIR)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_HCORE_DIR / "logs")
    parser.add_argument("--ros-domain-id", type=int, default=DEFAULT_ROS_DOMAIN_ID)
    parser.add_argument(
        "--ros-localhost-only",
        type=int,
        choices=(0, 1),
        default=0,
    )
    parser.add_argument("--rmw-implementation", default=os.environ.get("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"))
    parser.add_argument("--cyclonedds-uri", default=_default_cyclonedds_uri())

    parser.add_argument("--start-zenoh", action="store_true")
    parser.add_argument("--zenoh-host", default=os.environ.get("ARANCINO_HOST", DEFAULT_ZENOH_HOST))
    parser.add_argument("--zenoh-port", type=int, default=int(os.environ.get("HCORE_ZENOH_PORT", DEFAULT_ZENOH_PORT)))
    parser.add_argument("--zenoh-mode", choices=("client", "peer"), default=os.environ.get("HCORE_ZENOH_MODE", "client"))

    parser.add_argument("--no-drone-bridge", action="store_true")
    parser.add_argument("--no-tf-bridge", action="store_true")
    parser.add_argument("--no-octomap-relay", action="store_true")
    parser.add_argument("--skip-registration-cleanup", action="store_true")
    parser.add_argument("--registration-cleanup-timeout", type=float, default=2.0)
    parser.add_argument("--world-frame", default="map")
    parser.add_argument("--tf-topic", default=DEFAULT_HCORE_TF_TOPIC)
    parser.add_argument("--tf-static-topic", default=DEFAULT_HCORE_TF_STATIC_TOPIC)

    parser.add_argument("--drone-name", default="hcore_drone")
    parser.add_argument("--drone-topic-prefix", default="/hcore_drone")
    parser.add_argument("--drone-source-base-frame", default="base_link")
    parser.add_argument("--drone-base-frame", default="hcore_drone/base_link")
    parser.add_argument("--drone-goal-frame", default="drone/map")
    parser.add_argument("--drone-cmd-vel-topic", default="/drone/cmd_vel")
    parser.add_argument("--drone-direct-goal-topic", default="/drone/goal")
    parser.add_argument("--drone-nav-goal-topic", default="/drone/nav_goal")
    parser.add_argument("--drone-command-topic", default="/seed_pdt_drone/command")
    parser.add_argument("--drone-status-topic", default="/move_manager/status")
    parser.add_argument("--drone-odometry-topic", default="/model/baby_k_0/odometry")
    parser.add_argument("--drone-global-path-topic", default="/path_planner/optimized_path")
    parser.add_argument("--drone-local-path-topic", default="/local_path")
    parser.add_argument("--drone-trajectory-topic", default="/trajectory_path")
    parser.add_argument("--drone-camera-topic", default="/camera")
    parser.add_argument("--drone-camera-info-topic", default="/camera_info/rgb")
    parser.add_argument("--drone-camera-frame", default="hcore_drone/base_link")
    parser.add_argument("--drone-camera-resolution", type=_parse_resolution, default=(640, 480))
    parser.add_argument("--drone-camera-fov", type=float, default=78.0)
    parser.add_argument("--drone-camera-max-hz", type=int, default=8)
    parser.add_argument("--drone-depth-frame", default="hcore_drone/baby_k_0/OakD-Lite/base_link/StereoOV7251")
    parser.add_argument("--drone-depth-points-topic", default="/depth_camera/points")

    parser.add_argument("--rover-name", default="hcore_rover")
    parser.add_argument("--rover-topic-prefix", default="/hcore_rover")
    parser.add_argument("--rover-source-base-frame", default="rover/base_footprint")
    parser.add_argument("--rover-base-frame", default="hcore_rover/base_footprint")
    parser.add_argument("--rover-cmd-vel-topic", default="/rover/cmd_vel")
    parser.add_argument("--enable-rover-navigation", action="store_true")
    parser.add_argument("--rover-goal-topic", default="/move_base_simple/goal")
    parser.add_argument("--rover-cancel-topic", default="/move_manager/status")
    parser.add_argument("--rover-status-topic", default="/move_manager/status")
    parser.add_argument("--enable-rover-odom-visuals", action="store_true")
    parser.add_argument(
        "--rover-pose-topic",
        default="",
        help="Optional map-frame rover PoseWithCovarianceStamped source. Disabled by default.",
    )
    parser.add_argument("--rover-odometry-topic", default="/odom/wheels")
    parser.add_argument("--rover-global-path-topic", default="/global_path")
    parser.add_argument("--rover-local-path-topic", default="/local_path")
    parser.add_argument("--enable-rover-camera", dest="enable_rover_camera", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--no-rover-camera", dest="enable_rover_camera", action="store_false")
    parser.set_defaults(enable_rover_camera=True)
    parser.add_argument("--rover-camera-topic", default="/rover/color/image_raw")
    parser.add_argument("--rover-camera-info-topic", default="/rover/color/camera_info")
    parser.add_argument("--rover-camera-frame", default="hcore_rover/base_link")
    parser.add_argument("--rover-camera-resolution", type=_parse_resolution, default=(640, 480))
    parser.add_argument("--rover-camera-fov", type=float, default=78.0)
    parser.add_argument("--rover-camera-max-hz", type=int, default=8)
    parser.add_argument("--no-rover-model", dest="enable_rover_model", action="store_false")
    parser.set_defaults(enable_rover_model=True)
    parser.add_argument("--rover-urdf-path", type=Path, default=DEFAULT_HCORE_ROVER_URDF)
    parser.add_argument("--rover-mesh-root", type=Path, default=DEFAULT_HCORE_ROVER_MESH_ROOT)
    parser.add_argument("--rover-model-triangle-budget", type=int, default=120000)
    parser.add_argument(
        "--use-ptz-as-rover-camera",
        action="store_true",
        help="Use the shared PTZ /image_raw stream as a rover camera fallback.",
    )
    parser.add_argument("--rover-depth-frame", default="hcore_rover/depth_optical_frame")
    parser.add_argument("--rover-depth-points-topic", default="/rover/depth/image_raw/points")

    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--scan-frame", default="hcore_rover/laser")
    parser.add_argument("--scan-point-size", type=float, default=0.025)
    parser.add_argument("--enable-depth-clouds", action="store_true")
    parser.add_argument("--depth-points-max-hz", type=int, default=2)

    parser.add_argument("--enable-occupancy-grid", action="store_true")
    parser.add_argument("--occupancy-grid-topic", default="/map")
    parser.add_argument("--no-octomap", dest="enable_octomap", action="store_false")
    parser.set_defaults(enable_octomap=True)
    parser.add_argument("--octomap-topic", default="/octomap_binary")
    parser.add_argument("--octomap-mesh-topic", default=DEFAULT_OCTOMAP_MESH_TOPIC)
    parser.add_argument("--max-octomap-triangles", type=int, default=48000)
    parser.add_argument("--octomap-voxel-scale", type=float, default=1.0)
    parser.add_argument(
        "--octomap-marker-style",
        choices=("rviz_voxels", "surface_mesh"),
        default="surface_mesh",
    )
    parser.add_argument("--octomap-min-update-interval-sec", type=float, default=1.0)
    parser.add_argument("--octomap-chunk-publish-period-ms", type=int, default=40)
    parser.add_argument("--octomap-chunks-per-tick", type=int, default=2)
    return parser


def _summarize(robots: list[Robot], datavizs: list[object], args: argparse.Namespace) -> dict[str, object]:
    return {
        "ros_domain_id": args.ros_domain_id,
        "ros_localhost_only": args.ros_localhost_only,
        "start_zenoh": args.start_zenoh,
        "start_drone_bridge": not args.no_drone_bridge,
        "start_tf_bridge": not args.no_tf_bridge,
        "start_octomap_relay": args.enable_octomap and not args.no_octomap_relay,
        "robots": [
            {
                "name": robot.name,
                "type": robot.robot_type.value,
                "base_frame": robot.resolve_tf_frame(),
                "ros_binding": robot.get_ros_binding(),
                "teleop_topic": robot.get_metadata("teleop_config", {}).get("command_topic"),
                "robot_description": robot.get_metadata("robot_description_config", {}),
                "sensors": [
                    {
                        "name": getattr(sensor, "name", ""),
                        "frame_id": getattr(sensor, "frame_id", ""),
                        "topic": getattr(sensor, "topic", ""),
                        "metadata": getattr(sensor, "metadata", {}),
                    }
                    for sensor in robot.sensors
                ],
            }
            for robot in robots
        ],
        "visualizations": [
            {
                "name": dataviz.name,
                "count": len(dataviz.visualizations),
                "topics": sorted(
                    {
                        viz.data_source.topic
                        for viz in dataviz.visualizations
                        if getattr(viz.data_source, "topic", "")
                    }
                ),
            }
            for dataviz in datavizs
        ],
    }


def main() -> None:
    parser = _add_arguments()
    args = parser.parse_args()
    args.hcore_dir = args.hcore_dir.expanduser()
    args.log_dir = args.log_dir.expanduser()
    args.rover_urdf_path = args.rover_urdf_path.expanduser()
    args.rover_mesh_root = args.rover_mesh_root.expanduser()
    args.cyclonedds_uri = str(args.cyclonedds_uri or "").strip()

    if args.use_ptz_as_rover_camera:
        args.rover_camera_topic = "/image_raw"
        args.rover_camera_info_topic = "/image_raw/camera_info"
        args.rover_camera_frame = "axis_camera_optical"
        args.rover_camera_resolution = (640, 360)

    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = str(args.ros_localhost_only)
    os.environ["RMW_IMPLEMENTATION"] = str(args.rmw_implementation)
    if args.cyclonedds_uri:
        os.environ["CYCLONEDDS_URI"] = args.cyclonedds_uri

    robots, datavizs = build_registration(args)
    if args.dry_run:
        print(json.dumps(_summarize(robots, datavizs, args), indent=2))
        return

    with ExitStack() as stack:
        if args.start_zenoh:
            _start_zenoh_bridge(args, stack)
        if not args.no_drone_bridge:
            _start_drone_bridge(args, stack)
        if not args.no_tf_bridge:
            _start_tf_bridge(args, stack)
        if args.enable_octomap and not args.no_octomap_relay:
            _start_octomap_relay(args, stack)

        if not args.skip_registration_cleanup:
            _clear_stale_registrations(
                [args.drone_name, args.rover_name],
                timeout_sec=args.registration_cleanup_timeout,
            )

        success, result = register_robots(
            robots,
            datavizs=datavizs,
            workspace_scale=args.workspace_scale,
            compass_enabled=False,
            keep_alive=not args.once,
            timeout_sec=args.timeout,
            wait_for_app_before_register=not args.no_wait_for_app,
        )

    if not success:
        if is_registration_cancelled(result):
            print("HORUS registration monitor stopped.")
            raise SystemExit(0)
        raise SystemExit(f"HORUS registration failed: {result}")


if __name__ == "__main__":
    main()
