#!/usr/bin/env python3
"""Typical SDK usage example for 10-robot teleop + task operations."""

import argparse
import os
import sys
from typing import List, Tuple

# Ensure we can import 'horus' package regardless of where script is run from.
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


def parse_resolution(raw_value: str) -> Tuple[int, int]:
    if not raw_value:
        return 160, 90
    value = raw_value.strip().lower()
    if "x" not in value:
        return 160, 90
    width_raw, height_raw = value.split("x", 1)
    try:
        width = max(16, int(width_raw))
        height = max(16, int(height_raw))
        return width, height
    except ValueError:
        return 160, 90


def resolve_robot_names(args) -> List[str]:
    if args.robot_names:
        names = [name.strip() for name in args.robot_names.split(",") if name.strip()]
        if names:
            return names

    count = max(1, int(args.robot_count))
    if count <= len(DEFAULT_ROBOT_NAMES):
        return DEFAULT_ROBOT_NAMES[:count]

    names = list(DEFAULT_ROBOT_NAMES)
    for idx in range(len(DEFAULT_ROBOT_NAMES), count):
        names.append(f"unit_{idx + 1}")
    return names


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Typical HORUS SDK registration flow: 10 named robots with camera, teleop, "
            "go-to-point, and waypoint metadata."
        )
    )
    parser.add_argument(
        "--robot-count",
        type=int,
        default=10,
        help="Number of robots when --robot-names is not supplied (default: 10).",
    )
    parser.add_argument(
        "--robot-names",
        default="",
        help="Comma-separated list of robot names (overrides --robot-count).",
    )
    parser.add_argument(
        "--workspace-scale",
        type=float,
        default=0.1,
        help="Global workspace position scale forwarded in registration payload (default: 0.1).",
    )
    parser.add_argument(
        "--keep-alive",
        dest="keep_alive",
        action="store_true",
        default=True,
        help="Keep dashboard open after registration (default: on).",
    )
    parser.add_argument(
        "--no-keep-alive",
        dest="keep_alive",
        action="store_false",
        help="Exit after registration acknowledgement.",
    )
    parser.add_argument(
        "--camera-image-type",
        choices=["compressed", "raw"],
        default="compressed",
        help="Camera transport type to register (default: compressed).",
    )
    parser.add_argument(
        "--camera-resolution",
        default="160x90",
        help="Camera resolution as WxH (default: 160x90).",
    )
    parser.add_argument(
        "--camera-fps",
        type=int,
        default=6,
        help="Camera FPS metadata for each robot (default: 6).",
    )
    parser.add_argument(
        "--teleop-profile",
        choices=["wheeled", "legged", "aerial", "custom"],
        default="wheeled",
        help="Teleop profile metadata (default: wheeled).",
    )
    parser.add_argument(
        "--teleop-response-mode",
        choices=["analog", "discrete"],
        default="analog",
        help="Teleop joystick response mode metadata (default: analog).",
    )
    parser.add_argument(
        "--teleop-publish-rate",
        type=float,
        default=30.0,
        help="Teleop command publish-rate hint in Hz (default: 30).",
    )
    parser.add_argument(
        "--teleop-passthrough-only",
        action="store_true",
        help="Set teleop custom_passthrough_only=true in metadata.",
    )
    return parser


def build_camera(name: str, args, resolution: Tuple[int, int]) -> Camera:
    width, height = resolution
    image_type = args.camera_image_type.strip().lower()
    if image_type == "compressed":
        topic = f"/{name}/camera/image_raw/compressed"
        encoding = "jpeg"
    else:
        topic = f"/{name}/camera/image_raw"
        encoding = "rgb8"

    camera = Camera(
        name="front_camera",
        frame_id=f"{name}/camera_link",
        topic=topic,
        resolution=(width, height),
        fps=max(1, int(args.camera_fps)),
        encoding=encoding,
        streaming_type="ros",
        minimap_streaming_type="ros",
        teleop_streaming_type="webrtc",
        startup_mode="minimap",
    )

    camera.add_metadata("image_type", image_type)
    camera.add_metadata("streaming_type", "ros")
    camera.add_metadata("minimap_streaming_type", "ros")
    camera.add_metadata("teleop_streaming_type", "webrtc")
    camera.add_metadata("startup_mode", "minimap")
    camera.add_metadata("display_mode", "projected")
    camera.add_metadata("use_tf", True)
    camera.add_metadata("webrtc_client_signal_topic", "/horus/webrtc/client_signal")
    camera.add_metadata("webrtc_server_signal_topic", "/horus/webrtc/server_signal")
    camera.add_metadata("webrtc_bitrate_kbps", 2000)
    camera.add_metadata("webrtc_framerate", 20)
    camera.add_metadata("webrtc_stun_server_url", "stun:stun.l.google.com:19302")
    camera.add_metadata("image_scale", 0.072)
    camera.add_metadata("focal_length_scale", 0.108)
    camera.add_metadata("view_position_offset", [0.0, 0.0, 0.0])
    camera.add_metadata("view_rotation_offset", [0.0, 0.0, 0.0])
    camera.add_metadata("show_frustum", True)
    camera.add_metadata("frustum_color", "#FFFF00")
    camera.add_metadata("overhead_size", 1.0)
    camera.add_metadata("overhead_position_offset", [0.0, 2.0, 0.0])
    camera.add_metadata("overhead_face_camera", True)
    camera.add_metadata("overhead_rotation_offset", [90.0, 0.0, 0.0])
    return camera


def main():
    args = build_parser().parse_args()
    robot_names = resolve_robot_names(args)
    camera_resolution = parse_resolution(args.camera_resolution)

    cli.print_step("Defining typical multi-robot operations configuration...")

    robots: List[Robot] = []
    for index, name in enumerate(robot_names):
        robot = Robot(
            name=name,
            robot_type=RobotType.WHEELED,
            dimensions=RobotDimensions(
                length=0.8 + (0.1 * index),
                width=0.6 + (0.05 * index),
                height=0.4 + (0.03 * index),
            ),
        )

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
                "robot_profile": args.teleop_profile,
                "response_mode": args.teleop_response_mode,
                "publish_rate_hz": max(5.0, float(args.teleop_publish_rate)),
                "custom_passthrough_only": bool(args.teleop_passthrough_only),
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

        robot.add_sensor(build_camera(name, args, camera_resolution))
        robots.append(robot)

    cli.print_step(f"Registering {len(robots)} robot(s)...")
    success, result = register_robots(
        robots,
        keep_alive=args.keep_alive,
        show_dashboard=True,
        workspace_scale=args.workspace_scale,
    )

    if not success:
        if isinstance(result, dict) and result.get("error") == "Cancelled":
            cli.print_info("Registration cancelled by user.")
            return
        cli.print_error(f"Registration failed: {result}")
        return

    cli.print_success("Typical operations registration complete.")


if __name__ == "__main__":
    main()
