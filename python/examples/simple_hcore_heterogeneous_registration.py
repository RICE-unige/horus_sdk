#!/usr/bin/env python3
"""Simple HORUS registration for the H-Core drone + rover simulation.

Run from WSL:

    cd ~/horus_sdk
    python3 python/examples/simple_hcore_heterogeneous_registration.py

The script assumes the H-Core simulation is already running and reachable on
ROS_DOMAIN_ID=17. It starts only the small adapter processes HORUS needs:
drone command bridging, clean namespaced TF, and octomap-to-mesh publishing.
"""

from __future__ import annotations

from contextlib import ExitStack, contextmanager
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
from typing import Iterator


ROS_DOMAIN_ID = "17"
ROS_LOCALHOST_ONLY = "0"
RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"

HCORE_DIR = Path.home() / "hcore_zenoh_client"
HCORE_DDS_CONFIG = HCORE_DIR / "cyclonedds_hcore_wsl.xml"
LOG_DIR = HCORE_DIR / "logs"

WORLD_FRAME = "map"
HCORE_TF_TOPIC = "/hcore/tf"
HCORE_TF_STATIC_TOPIC = "/hcore/tf_static"
HCORE_OCTOMAP_MESH_TOPIC = "/hcore/octomap_mesh"

DRONE_NAME = "hcore_drone"
ROVER_NAME = "hcore_rover"

DRONE_FRAME_PREFIX = DRONE_NAME
ROVER_FRAME_PREFIX = ROVER_NAME
DRONE_BASE_FRAME = "base_link"
ROVER_BASE_FRAME = "base_footprint"

DRONE_GOAL_FRAME = "drone/map"

EXAMPLES_DIR = Path(__file__).resolve().parent
SDK_PYTHON = EXAMPLES_DIR.parent
ASSETS_DIR = EXAMPLES_DIR / ".local_assets"
ROBOT_DESCRIPTION_DIR = ASSETS_DIR / "robot_descriptions"
ROVER_URDF = ROBOT_DESCRIPTION_DIR / "hcore_rover_prefixed.urdf"
ROVER_MESH_ROOT = ROBOT_DESCRIPTION_DIR / "meshes_root"


def _bootstrap_ros_environment() -> None:
    """Re-exec through ROS setup when launched from a plain shell."""

    if os.environ.get("HORUS_HCORE_ROS_BOOTSTRAPPED") == "1":
        return
    if os.environ.get("AMENT_PREFIX_PATH"):
        return

    setup = Path("/opt/ros/jazzy/setup.bash")
    if not setup.exists():
        return

    env = os.environ.copy()
    env["HORUS_HCORE_ROS_BOOTSTRAPPED"] = "1"
    env["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID
    env["ROS_LOCALHOST_ONLY"] = ROS_LOCALHOST_ONLY
    env["RMW_IMPLEMENTATION"] = RMW_IMPLEMENTATION
    if HCORE_DDS_CONFIG.exists():
        env["CYCLONEDDS_URI"] = f"file://{HCORE_DDS_CONFIG}"

    args = [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]]
    command = (
        f"source {shlex.quote(str(setup))}; "
        f"export ROS_DOMAIN_ID={shlex.quote(ROS_DOMAIN_ID)}; "
        f"export ROS_LOCALHOST_ONLY={shlex.quote(ROS_LOCALHOST_ONLY)}; "
        f"export RMW_IMPLEMENTATION={shlex.quote(RMW_IMPLEMENTATION)}; "
    )
    if HCORE_DDS_CONFIG.exists():
        command += f"export CYCLONEDDS_URI={shlex.quote(f'file://{HCORE_DDS_CONFIG}')}; "
    command += f"exec {shlex.join(args)}"

    os.execvpe("bash", ["bash", "-lc", command], env)


if __name__ == "__main__":
    _bootstrap_ros_environment()


sys.path.insert(0, str(SDK_PYTHON))

from horus import Robot, RobotType, register_robots  # noqa: E402
from horus.bridge.robot_registry import get_robot_registry_client  # noqa: E402
from horus.sensors import Camera, LaserScan  # noqa: E402


def _set_ros_environment() -> None:
    os.environ.setdefault("ROS_DOMAIN_ID", ROS_DOMAIN_ID)
    os.environ.setdefault("ROS_LOCALHOST_ONLY", ROS_LOCALHOST_ONLY)
    os.environ.setdefault("RMW_IMPLEMENTATION", RMW_IMPLEMENTATION)
    if HCORE_DDS_CONFIG.exists():
        os.environ.setdefault("CYCLONEDDS_URI", f"file://{HCORE_DDS_CONFIG}")


def _ros_command(command: list[str]) -> list[str]:
    setup = "/opt/ros/jazzy/setup.bash"
    script = (
        f"set -e; source {shlex.quote(setup)}; "
        f"export ROS_DOMAIN_ID={shlex.quote(ROS_DOMAIN_ID)}; "
        f"export ROS_LOCALHOST_ONLY={shlex.quote(ROS_LOCALHOST_ONLY)}; "
        f"export RMW_IMPLEMENTATION={shlex.quote(RMW_IMPLEMENTATION)}; "
    )
    if HCORE_DDS_CONFIG.exists():
        script += f"export CYCLONEDDS_URI={shlex.quote(f'file://{HCORE_DDS_CONFIG}')}; "
    script += f"exec {shlex.join(command)}"
    return ["bash", "-lc", script]


def _terminate_matching_processes(pattern: str) -> None:
    try:
        result = subprocess.run(
            ["pgrep", "-f", pattern],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return

    current_pid = os.getpid()
    for raw_pid in result.stdout.split():
        try:
            pid = int(raw_pid)
        except ValueError:
            continue
        if pid == current_pid:
            continue
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass


@contextmanager
def _managed_process(label: str, command: list[str], pattern: str) -> Iterator[None]:
    _terminate_matching_processes(pattern)
    time.sleep(0.2)

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"{label}.log"
    with log_path.open("ab") as log_file:
        process = subprocess.Popen(
            _ros_command(command),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

    print(f"[H-Core] Started {label}; log={log_path}")
    time.sleep(1.0)

    try:
        yield
    finally:
        if process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait(timeout=2.0)
            except ProcessLookupError:
                pass


def _start_helpers(stack: ExitStack) -> None:
    drone_bridge = HCORE_DIR / "drone_bridge.py"
    tf_bridge = EXAMPLES_DIR / "tools" / "hcore_tf_bridge.py"
    octomap_relay = EXAMPLES_DIR / "tools" / "uav_sim_octomap_marker_relay.py"

    for path in (drone_bridge, tf_bridge, octomap_relay):
        if not path.exists():
            raise FileNotFoundError(f"Required H-Core helper is missing: {path}")

    stack.enter_context(
        _managed_process(
            "hcore_drone_bridge",
            [
                sys.executable,
                str(drone_bridge),
                "--frame",
                DRONE_GOAL_FRAME,
                "--base-frame",
                DRONE_BASE_FRAME,
                "bridge",
                "--cmd-vel-topic",
                "/drone/cmd_vel",
                "--goal-topic",
                "/drone/goal",
                "--nav-goal-topic",
                "/drone/nav_goal",
                "--takeoff-topic",
                "/hcore_drone/takeoff",
                "--land-topic",
                "/hcore_drone/land",
                "--stop-topic",
                "/hcore_drone/stop",
            ],
            "drone_bridge.py.*bridge",
        )
    )

    stack.enter_context(
        _managed_process(
            "hcore_tf_bridge",
            [
                sys.executable,
                str(tf_bridge),
                "--output-tf-topic",
                HCORE_TF_TOPIC,
                "--output-tf-static-topic",
                HCORE_TF_STATIC_TOPIC,
                "--world-frame",
                WORLD_FRAME,
                "--drone-prefix",
                DRONE_FRAME_PREFIX,
                "--drone-base-frame",
                DRONE_BASE_FRAME,
                "--rover-prefix",
                ROVER_FRAME_PREFIX,
                "--rover-base-frame",
                f"rover/{ROVER_BASE_FRAME}",
            ],
            "hcore_tf_bridge.py",
        )
    )

    stack.enter_context(
        _managed_process(
            "hcore_octomap_mesh",
            [
                sys.executable,
                str(octomap_relay),
                "--input-topic",
                "/octomap_binary",
                "--output-topic",
                HCORE_OCTOMAP_MESH_TOPIC,
                "--max-triangles",
                "48000",
                "--voxel-scale",
                "1.0",
                "--style",
                "surface_mesh",
                "--min-update-interval-sec",
                "1.0",
                "--chunk-publish-period-ms",
                "40",
                "--chunks-per-tick",
                "2",
            ],
            "uav_sim_octomap_marker_relay.py.*hcore/octomap_mesh",
        )
    )


def _add_drone() -> tuple[Robot, object]:
    drone = Robot(
        name=DRONE_NAME,
        robot_type=RobotType.DRONE,
        metadata={"display_name": "H-Core Drone"},
        dimensions=(0.46, 0.46, 0.18),
    )
    drone.configure_ros_binding(
        tf_mode="prefixed",
        topic_mode="prefixed",
        base_frame=DRONE_BASE_FRAME,
        tf_prefix=DRONE_FRAME_PREFIX,
        topic_prefix=f"/{DRONE_FRAME_PREFIX}",
    )

    drone.configure_teleop(
        command_topic="/drone/cmd_vel",
        robot_profile="drone",
        response_mode="analog",
        publish_rate_hz=30.0,
        linear_xy_max_mps=0.8,
        linear_z_max_mps=0.5,
        angular_z_max_rps=0.8,
    )
    drone.configure_navigation_tasks(
        waypoint_enabled=False,
        goal_topic="/drone/nav_goal",
        cancel_topic="/seed_pdt_drone/command",
        goal_status_topic="/move_manager/status",
        waypoint_path_topic="",
        waypoint_status_topic="/move_manager/status",
        frame_id=DRONE_GOAL_FRAME,
        position_tolerance_m=0.3,
        yaw_tolerance_deg=15.0,
        min_altitude_m=0.3,
        max_altitude_m=8.0,
    )

    camera = Camera(
        name="drone_rgb",
        frame_id=f"{DRONE_FRAME_PREFIX}/{DRONE_BASE_FRAME}",
        topic="/camera",
        resolution=(640, 480),
        fps=8,
        fov=78.0,
        encoding="rgb8",
        streaming_type="ros",
        minimap_streaming_type="ros",
        teleop_streaming_type="ros",
        startup_mode="minimap",
        minimap_topic="/camera",
        minimap_image_type="raw",
        teleop_topic="/camera",
        teleop_image_type="raw",
        minimap_max_fps=8,
    )
    camera.add_metadata("camera_info_topic", "/camera_info/rgb")
    camera.configure_projected_view(
        position_offset=(0.0, 0.03, 0.0),
        image_scale=0.099,
        focal_length_scale=0.13,
        show_frustum=True,
        frustum_color="#80D9FFA0",
    )
    camera.configure_minimap_view(size=2.025, position_offset=(0.0, 0.75, 0.0))
    camera.configure_immersive_view(ros_flip_x=False, ros_flip_y=True)
    drone.add_sensor(camera)

    viz = drone.create_dataviz("hcore_drone_viz")
    drone.add_path_planning_to_dataviz(
        viz,
        global_path_topic="/path_planner/optimized_path",
        local_path_topic="/local_path",
        trajectory_topic="/trajectory_path",
    )
    viz.add_robot_velocity_data(
        robot_name=DRONE_NAME,
        topic="/model/baby_k_0/odometry",
        frame_id=WORLD_FRAME,
        render_options={"color": "#80D9FF", "scale": 0.35, "update_rate": 10.0},
    )
    viz.add_robot_odometry_trail(
        robot_name=DRONE_NAME,
        topic="/model/baby_k_0/odometry",
        frame_id=WORLD_FRAME,
        render_options={
            "color": "#80D9FF",
            "max_points": 56,
            "history_seconds": 4.0,
            "line_width_m": 0.035,
            "update_rate": 5.0,
        },
    )
    viz.add_tf_tree(HCORE_TF_TOPIC, frame_id=WORLD_FRAME)
    viz.add_3d_octomap(
        topic=HCORE_OCTOMAP_MESH_TOPIC,
        frame_id=DRONE_GOAL_FRAME,
        render_options={
            "render_mode": "surface_mesh",
            "native_topic": "/octomap_binary",
            "native_frame": DRONE_GOAL_FRAME,
            "native_binary_only": False,
            "max_triangles": 48000,
            "color_by_axis": True,
            "axis": "z",
            "alpha": 0.92,
        },
        transport_lane="bulk_replaceable",
    )

    return drone, viz


def _add_rover() -> tuple[Robot, object]:
    rover = Robot(
        name=ROVER_NAME,
        robot_type=RobotType.WHEELED,
        metadata={"display_name": "H-Core Rover"},
        dimensions=(0.50, 0.38, 0.28),
    )
    rover.configure_ros_binding(
        tf_mode="prefixed",
        topic_mode="prefixed",
        base_frame=ROVER_BASE_FRAME,
        tf_prefix=ROVER_FRAME_PREFIX,
        topic_prefix=f"/{ROVER_FRAME_PREFIX}",
    )
    rover.configure_teleop(
        command_topic="/rover/cmd_vel",
        robot_profile="wheeled",
        response_mode="analog",
        publish_rate_hz=30.0,
        linear_xy_max_mps=0.35,
        linear_z_max_mps=0.0,
        angular_z_max_rps=0.9,
    )

    if ROVER_URDF.exists():
        rover.configure_robot_description(
            enabled=True,
            urdf_path=str(ROVER_URDF),
            base_frame=ROVER_BASE_FRAME,
            urdf_package="rover_description_pkg",
            mesh_root=str(ROVER_MESH_ROOT),
            include_visual_meshes=True,
            visual_mesh_triangle_budget=120000,
            body_mesh_mode="preview_mesh",
            chunk_size_bytes=12000,
        )
    else:
        print(f"[H-Core] Rover URDF not found, registering fallback body only: {ROVER_URDF}")

    camera = Camera(
        name="rover_rgb",
        frame_id=f"{ROVER_FRAME_PREFIX}/base_link",
        topic="/rover/color/image_raw",
        resolution=(640, 480),
        fps=8,
        fov=78.0,
        encoding="rgb8",
        streaming_type="ros",
        minimap_streaming_type="ros",
        teleop_streaming_type="ros",
        startup_mode="minimap",
        minimap_topic="/rover/color/image_raw",
        minimap_image_type="raw",
        teleop_topic="/rover/color/image_raw",
        teleop_image_type="raw",
        minimap_max_fps=8,
    )
    camera.add_metadata("camera_info_topic", "/rover/color/camera_info")
    camera.configure_projected_view(
        position_offset=(0.0, 0.06, 0.0),
        image_scale=0.126,
        focal_length_scale=0.11,
        show_frustum=True,
        frustum_color="#FFE08AA0",
    )
    camera.configure_minimap_view(size=2.25, position_offset=(0.0, 0.85, 0.0))
    camera.configure_immersive_view(ros_flip_x=False, ros_flip_y=True)
    rover.add_sensor(camera)

    rover.add_sensor(
        LaserScan(
            name="rover_scan",
            frame_id=f"{ROVER_FRAME_PREFIX}/laser",
            topic="/scan",
            min_angle=-3.14159,
            max_angle=3.14159,
            angle_increment=0.005,
            min_range=0.08,
            max_range=30.0,
            color="#FFE08A",
            point_size=0.025,
        )
    )

    viz = rover.create_dataviz("hcore_rover_viz")
    scan = rover.get_sensor("rover_scan")
    if scan is not None:
        viz.add_sensor_visualization(
            scan,
            ROVER_NAME,
            render_options={"color": "#FFE08A", "point_size": 0.025, "alpha": 0.85},
        )

    return rover, viz


def _retarget_robot_transforms_to_hcore_tf(datavizs: list[object]) -> None:
    for data_viz in datavizs:
        for visualization in getattr(data_viz, "visualizations", []) or []:
            data_source = getattr(visualization, "data_source", None)
            source_type = getattr(getattr(data_source, "source_type", None), "value", "")
            if source_type == "robot_transform":
                data_source.topic = HCORE_TF_TOPIC
                data_source.frame_id = WORLD_FRAME


def _clear_stale_registrations() -> None:
    client = get_robot_registry_client()
    for name in (DRONE_NAME, ROVER_NAME):
        try:
            client.unregister_robot(name, timeout_sec=1.0)
        except Exception:
            pass


def main() -> None:
    _set_ros_environment()

    with ExitStack() as stack:
        _start_helpers(stack)
        _clear_stale_registrations()

        drone, drone_viz = _add_drone()
        rover, rover_viz = _add_rover()
        robots = [drone, rover]
        datavizs = [drone_viz, rover_viz]
        _retarget_robot_transforms_to_hcore_tf(datavizs)

        print("[H-Core] Registering H-Core drone and rover with HORUS...")
        success, result = register_robots(
            robots,
            datavizs=datavizs,
            workspace_scale=0.10,
            compass_enabled=False,
            wait_for_app_before_register=True,
            keep_alive=True,
            timeout_sec=10.0,
        )

        if not success:
            raise SystemExit(f"[H-Core] Registration failed: {result}")


if __name__ == "__main__":
    main()
