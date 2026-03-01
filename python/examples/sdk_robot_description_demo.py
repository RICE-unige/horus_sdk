#!/usr/bin/env python3
"""Register one wheeled + one legged robot with Robot Description V1 enabled."""

import argparse
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.join(SCRIPT_DIR, "..")
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

try:
    from horus.robot import Robot, RobotDimensions, RobotType, register_robots
    from horus.sensors import Camera
    from horus.utils import cli
except ImportError:
    print(f"Failed to import horus from {PACKAGE_ROOT}")
    raise


def _first_existing_path(candidates: List[str]) -> str:
    for candidate in candidates:
        if candidate and os.path.isfile(candidate):
            return candidate
    return ""


def _local_asset_path(filename: str) -> str:
    return str(Path(SCRIPT_DIR) / ".local_assets" / "robot_descriptions" / filename)


def _resolve_demo_urdf(explicit: str, local_candidates: Sequence[str], fallback_candidates: Sequence[str]) -> str:
    if explicit:
        candidate = os.path.expandvars(os.path.expanduser(explicit.strip()))
        if os.path.isfile(candidate):
            return candidate
        return ""

    return _first_existing_path([*local_candidates, *fallback_candidates])


def _resolve_wheeled_urdf(explicit: str) -> str:
    return _resolve_demo_urdf(
        explicit=explicit,
        local_candidates=(
            _local_asset_path("jackal.urdf"),
        ),
        fallback_candidates=(
            "/mnt/c/Users/adeko/horus/Assets/URDF/nova_carter_fbx.urdf",
            "C:/Users/adeko/horus/Assets/URDF/nova_carter_fbx.urdf",
            _local_asset_path("jackal.urdf.xacro"),
        ),
    )


def _resolve_legged_urdf(explicit: str) -> str:
    return _resolve_demo_urdf(
        explicit=explicit,
        local_candidates=(
            _local_asset_path("go1.urdf"),
        ),
        fallback_candidates=(
            "/mnt/c/Users/adeko/newton/newton/examples/assets/quadruped.urdf",
            "C:/Users/adeko/newton/newton/examples/assets/quadruped.urdf",
        ),
    )


def _build_camera(robot_name: str, frame_name: str, image_scale: float, overhead_size: float) -> Camera:
    resolved_frame = str(frame_name or "base_link").strip()
    if resolved_frame.startswith(f"{robot_name}/"):
        frame_id = resolved_frame
    else:
        frame_id = f"{robot_name}/{resolved_frame}"
    camera = Camera(
        name="front_camera",
        frame_id=frame_id,
        topic=f"/{robot_name}/camera/image_raw/compressed",
        resolution=(160, 90),
        fps=6,
        encoding="jpeg",
        streaming_type="ros",
        minimap_streaming_type="ros",
        teleop_streaming_type="webrtc",
        startup_mode="minimap",
    )
    camera.add_metadata("image_type", "compressed")
    camera.add_metadata("display_mode", "projected")
    camera.add_metadata("use_tf", True)
    camera.add_metadata("image_scale", float(image_scale))
    camera.add_metadata("focal_length_scale", 0.090)
    camera.add_metadata("view_position_offset", [0.0, 0.0, 0.0])
    camera.add_metadata("view_rotation_offset", [0.0, 0.0, 0.0])
    camera.add_metadata("show_frustum", True)
    camera.add_metadata("frustum_color", "#59D9FF")
    camera.add_metadata("overhead_size", float(overhead_size))
    camera.add_metadata("overhead_position_offset", [0.0, 1.4, 0.0])
    camera.add_metadata("overhead_face_camera", True)
    camera.add_metadata("overhead_rotation_offset", [90.0, 0.0, 0.0])
    return camera


def _apply_common_runtime_metadata(robot: Robot) -> None:
    name = robot.name
    robot.add_metadata(
        "robot_manager_config",
        {
            "enabled": True,
            "prefab_asset_path": "Assets/Prefabs/UI/RobotManager.prefab",
            "prefab_resource_path": "",
            "sections": {
                "status": True,
                "data_viz": True,
                "teleop": True,
                "tasks": True,
            },
        },
    )
    robot.add_metadata(
        "teleop_config",
        {
            "enabled": True,
            "command_topic": f"/{name}/cmd_vel",
            "raw_input_topic": f"/horus/teleop/{name}/joy",
            "head_pose_topic": f"/horus/teleop/{name}/head_pose",
            "robot_profile": "legged" if robot.robot_type == RobotType.LEGGED else "wheeled",
            "response_mode": "analog",
            "publish_rate_hz": 30.0,
        },
    )
    robot.add_metadata(
        "task_config",
        {
            "go_to_point": {
                "enabled": True,
                "goal_topic": f"/{name}/goal_pose",
                "cancel_topic": f"/{name}/goal_cancel",
                "status_topic": f"/{name}/goal_status",
                "frame_id": "map",
                "position_tolerance_m": 0.20,
                "yaw_tolerance_deg": 12.0,
            },
            "waypoint": {
                "enabled": True,
                "path_topic": f"/{name}/waypoint_path",
                "status_topic": f"/{name}/waypoint_status",
                "frame_id": "map",
                "position_tolerance_m": 0.20,
                "yaw_tolerance_deg": 12.0,
            },
        },
    )


def _build_robot(
    name: str,
    robot_type: RobotType,
    dimensions: RobotDimensions,
    urdf_path: Optional[str],
    description_base_frame: str,
    enable_robot_description: bool,
    collision_transparent: bool,
    camera_image_scale: float,
    camera_overhead_size: float,
) -> Dict:
    robot = Robot(name=name, robot_type=robot_type, dimensions=dimensions)
    _apply_common_runtime_metadata(robot)
    robot.add_sensor(
        _build_camera(
            name,
            description_base_frame,
            camera_image_scale,
            camera_overhead_size,
        )
    )

    if enable_robot_description and urdf_path:
        robot.configure_robot_description(
            urdf_path=urdf_path,
            base_frame=str(description_base_frame or "base_link"),
            source="ros",
            chunk_size_bytes=12000,
            is_transparent=bool(collision_transparent),
            enabled=True,
        )

    dataviz = robot.create_dataviz()
    robot.add_path_planning_to_dataviz(
        dataviz,
        global_path_topic=f"/{name}/global_path",
        local_path_topic=f"/{name}/local_path",
    )
    robot.add_navigation_safety_to_dataviz(
        dataviz,
        odom_topic=f"/{name}/odom",
        collision_risk_topic=f"/{name}/collision_risk",
        include_velocity=True,
        include_trail=True,
        include_collision=True,
    )
    return {"robot": robot, "dataviz": dataviz}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Register one wheeled + one legged robot with Robot Description V1 "
            "(collision mesh + joint axes) and standard nav/task topics."
        )
    )
    parser.add_argument(
        "--wheeled-urdf",
        default="",
        help="Path to wheeled robot URDF/xacro. If omitted, resolves local fetched Jackal assets first.",
    )
    parser.add_argument(
        "--legged-urdf",
        default="",
        help="Path to legged robot URDF/xacro. If omitted, resolves local fetched Go1 asset first.",
    )
    parser.add_argument(
        "--enable-robot-description",
        dest="enable_robot_description",
        action="store_true",
        default=True,
        help="Enable Robot Description V1 manifest + chunk transport (default: on).",
    )
    parser.add_argument(
        "--no-enable-robot-description",
        dest="enable_robot_description",
        action="store_false",
        help="Disable robot description transport metadata.",
    )
    parser.add_argument(
        "--workspace-scale",
        type=float,
        default=0.1,
        help="Workspace scale embedded into registration payloads.",
    )
    parser.add_argument(
        "--camera-image-scale",
        type=float,
        default=0.045,
        help="Projected camera image scale metadata (default: 0.045).",
    )
    parser.add_argument(
        "--camera-overhead-size",
        type=float,
        default=0.58,
        help="Overhead camera panel size metadata (default: 0.58).",
    )
    transparency_group = parser.add_mutually_exclusive_group()
    transparency_group.add_argument(
        "--collision-transparent",
        dest="collision_transparent",
        action="store_true",
        default=False,
        help="Render robot-description collision body as transparent.",
    )
    transparency_group.add_argument(
        "--collision-opaque",
        dest="collision_transparent",
        action="store_false",
        help="Render robot-description collision body as opaque (default).",
    )
    parser.add_argument(
        "--keep-alive",
        dest="keep_alive",
        action="store_true",
        default=True,
        help="Keep SDK dashboard alive after registration (default: on).",
    )
    parser.add_argument(
        "--no-keep-alive",
        dest="keep_alive",
        action="store_false",
        help="Exit after successful registration.",
    )
    return parser


def main():
    args = build_parser().parse_args()
    wheeled_urdf = _resolve_wheeled_urdf(args.wheeled_urdf)
    legged_urdf = _resolve_legged_urdf(args.legged_urdf)

    if args.enable_robot_description:
        missing = []
        if not wheeled_urdf or not os.path.isfile(wheeled_urdf):
            missing.append(f"--wheeled-urdf ({args.wheeled_urdf or 'unresolved'})")
        if not legged_urdf or not os.path.isfile(legged_urdf):
            missing.append(f"--legged-urdf ({args.legged_urdf or 'unresolved'})")
        if missing:
            cli.print_error("Robot description URDF path missing/unreadable:")
            for item in missing:
                cli.print_error(f"  - {item}")
            cli.print_info("Tip: fetch local real assets with:")
            cli.print_info("  python3 python/examples/tools/fetch_robot_description_assets.py")
            cli.print_info("Provide explicit paths or disable with --no-enable-robot-description.")
            return
        cli.print_info(f"Wheeled URDF: {wheeled_urdf}")
        cli.print_info(f"Legged URDF: {legged_urdf}")
    cli.print_info(
        f"Camera metadata -> image_scale={args.camera_image_scale:.3f}, overhead_size={args.camera_overhead_size:.3f}"
    )
    cli.print_info(
        f"Robot-description collision transparency -> is_transparent={bool(args.collision_transparent)}"
    )

    setup = [
        _build_robot(
            name="jackal",
            robot_type=RobotType.WHEELED,
            dimensions=RobotDimensions(length=0.50, width=0.38, height=0.32),
            urdf_path=wheeled_urdf,
            description_base_frame="base_link",
            enable_robot_description=bool(args.enable_robot_description),
            collision_transparent=bool(args.collision_transparent),
            camera_image_scale=float(args.camera_image_scale),
            camera_overhead_size=float(args.camera_overhead_size),
        ),
        _build_robot(
            name="go1",
            robot_type=RobotType.LEGGED,
            dimensions=RobotDimensions(length=0.68, width=0.31, height=0.48),
            urdf_path=legged_urdf,
            description_base_frame="base",
            enable_robot_description=bool(args.enable_robot_description),
            collision_transparent=bool(args.collision_transparent),
            camera_image_scale=float(args.camera_image_scale),
            camera_overhead_size=float(args.camera_overhead_size),
        ),
    ]
    robots = [entry["robot"] for entry in setup]
    datavizs = [entry["dataviz"] for entry in setup]

    cli.print_step("Registering robot-description demo robots...")
    success, result = register_robots(
        robots,
        datavizs=datavizs,
        keep_alive=bool(args.keep_alive),
        show_dashboard=True,
        workspace_scale=float(args.workspace_scale),
    )
    if not success:
        cli.print_error(f"Registration failed: {result}")
        return
    cli.print_success("Robot-description demo registration complete.")


if __name__ == "__main__":
    main()
