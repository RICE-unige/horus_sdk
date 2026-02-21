#!/usr/bin/env python3
"""Register robots with mono/stereo camera metadata for Unity stereo camera testing."""

import argparse
import os
import sys
from typing import List, Tuple

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


DEFAULT_ROBOT_NAME_PREFIX = "stereo_bot"


def parse_resolution(value: str, fallback: Tuple[int, int]) -> Tuple[int, int]:
    raw = (value or "").strip().lower()
    if "x" not in raw:
        return fallback
    left, right = raw.split("x", 1)
    try:
        width = max(64, int(left))
        height = max(64, int(right))
        return width, height
    except ValueError:
        return fallback


def resolve_robot_names(args) -> List[str]:
    if args.robot_names:
        names = [name.strip() for name in args.robot_names.split(",") if name.strip()]
        if names:
            return names

    count = max(1, int(args.robot_count))
    return [f"{DEFAULT_ROBOT_NAME_PREFIX}_{idx + 1}" for idx in range(count)]


def build_parser():
    parser = argparse.ArgumentParser(
        description="Register mono/stereo camera robots for Unity immersive stereo rendering tests."
    )
    parser.add_argument("--robot-count", type=int, default=4, help="Robot count if --robot-names is not provided.")
    parser.add_argument("--robot-names", default="", help="Comma-separated robot names (overrides --robot-count).")
    parser.add_argument(
        "--stereo-robot-count",
        type=int,
        default=-1,
        help="Number of robots (from the head of the list) registered as stereo cameras (default: all robots).",
    )
    parser.add_argument(
        "--stereo-input-mode",
        choices=["sbs", "dual_topic"],
        default="sbs",
        help="Stereo source format: single-topic side-by-side (sbs) or dual_topic (left+right topics).",
    )
    parser.add_argument(
        "--stereo-eye-resolution",
        default="960x540",
        help="Per-eye stereo resolution (total stereo frame width is doubled).",
    )
    parser.add_argument(
        "--mono-resolution",
        default="960x540",
        help="Mono camera resolution for non-stereo robots.",
    )
    parser.add_argument(
        "--camera-image-type",
        choices=["compressed", "raw"],
        default="compressed",
        help="Camera topic type to register (default: compressed).",
    )
    parser.add_argument("--camera-fps", type=int, default=10, help="Camera FPS metadata (default: 10).")
    parser.add_argument(
        "--highres-robot",
        default="",
        help="Robot name to register with a high-load camera profile (default: first robot).",
    )
    parser.add_argument(
        "--highres-eye-resolution",
        default="1920x1080",
        help="Per-eye resolution for the high-load robot (default: 1920x1080).",
    )
    parser.add_argument(
        "--highres-camera-fps",
        type=int,
        default=90,
        help="FPS metadata for the high-load robot (default: 90).",
    )
    parser.add_argument("--workspace-scale", type=float, default=0.1, help="Workspace position scale.")
    parser.add_argument(
        "--keep-alive",
        dest="keep_alive",
        action="store_true",
        default=True,
        help="Keep the SDK dashboard alive after registration (default: on).",
    )
    parser.add_argument(
        "--no-keep-alive",
        dest="keep_alive",
        action="store_false",
        help="Exit immediately after registration acknowledgement.",
    )
    return parser


def build_camera(
    robot_name: str,
    image_type: str,
    fps: int,
    mono_resolution: Tuple[int, int],
    stereo_eye_resolution: Tuple[int, int],
    is_stereo: bool,
    stereo_input_mode: str,
) -> Camera:
    if image_type == "compressed":
        minimap_topic = f"/{robot_name}/camera/minimap/image_raw/compressed"
        teleop_topic_sbs = f"/{robot_name}/camera/teleop/image_raw/compressed"
        teleop_left_topic_dual = f"/{robot_name}/camera/teleop/left/image_raw/compressed"
        teleop_right_topic_dual = f"/{robot_name}/camera/teleop/right/image_raw/compressed"
        encoding = "jpeg"
    else:
        minimap_topic = f"/{robot_name}/camera/minimap/image_raw"
        teleop_topic_sbs = f"/{robot_name}/camera/teleop/image_raw"
        teleop_left_topic_dual = f"/{robot_name}/camera/teleop/left/image_raw"
        teleop_right_topic_dual = f"/{robot_name}/camera/teleop/right/image_raw"
        encoding = "rgb8"

    stereo_mode = (stereo_input_mode or "sbs").strip().lower()
    use_dual_topic = is_stereo and stereo_mode == "dual_topic"
    teleop_topic = teleop_left_topic_dual if use_dual_topic else teleop_topic_sbs
    teleop_right_topic = teleop_right_topic_dual if use_dual_topic else ""
    teleop_stereo_layout = "dual_topic" if use_dual_topic else ("side_by_side" if is_stereo else "mono")
    teleop_transport = "ros" if use_dual_topic else "webrtc"

    if is_stereo and not use_dual_topic:
        eye_w, eye_h = stereo_eye_resolution
        resolution = (eye_w * 2, eye_h)
    elif is_stereo and use_dual_topic:
        resolution = stereo_eye_resolution
    else:
        resolution = mono_resolution

    camera = Camera(
        name="front_camera",
        frame_id=f"{robot_name}/camera_link",
        topic=minimap_topic,
        is_stereo=is_stereo,
        stereo_layout=teleop_stereo_layout,
        right_topic=teleop_right_topic,
        minimap_topic=minimap_topic,
        teleop_topic=teleop_topic,
        minimap_image_type=image_type,
        teleop_image_type=image_type,
        minimap_max_fps=30,
        teleop_right_topic=teleop_right_topic,
        teleop_stereo_layout=teleop_stereo_layout,
        resolution=resolution,
        fps=max(1, int(fps)),
        encoding=encoding,
        streaming_type="ros",
        minimap_streaming_type="ros",
        teleop_streaming_type=teleop_transport,
        startup_mode="minimap",
    )

    per_eye_pixels = max(64, int(stereo_eye_resolution[0])) * max(64, int(stereo_eye_resolution[1]))
    requested_webrtc_bitrate = 8000 if per_eye_pixels >= (1920 * 1080) else 3000
    requested_webrtc_framerate = max(30, min(90, int(fps)))

    camera.add_metadata("image_type", image_type)
    camera.add_metadata("streaming_type", "ros")
    camera.add_metadata("minimap_streaming_type", "ros")
    camera.add_metadata("teleop_streaming_type", teleop_transport)
    camera.add_metadata("startup_mode", "minimap")
    camera.add_metadata("minimap_topic", minimap_topic)
    camera.add_metadata("teleop_topic", teleop_topic)
    camera.add_metadata("minimap_image_type", image_type)
    camera.add_metadata("teleop_image_type", image_type)
    camera.add_metadata("minimap_max_fps", 30)
    camera.add_metadata("webrtc_bitrate_kbps", requested_webrtc_bitrate)
    camera.add_metadata("webrtc_framerate", requested_webrtc_framerate)
    camera.add_metadata("display_mode", "projected")
    camera.add_metadata("use_tf", True)
    camera.add_metadata("show_frustum", True)
    camera.add_metadata("frustum_color", "#FFFF00")
    camera.add_metadata("image_scale", 0.072)
    camera.add_metadata("focal_length_scale", 0.108)
    camera.add_metadata("view_position_offset", [0.0, 0.0, 0.0])
    camera.add_metadata("view_rotation_offset", [0.0, 0.0, 0.0])
    if is_stereo:
        camera.add_metadata("stereo_layout", teleop_stereo_layout)
        camera.add_metadata("teleop_stereo_layout", teleop_stereo_layout)
        if use_dual_topic:
            camera.add_metadata("right_topic", teleop_right_topic)
            camera.add_metadata("teleop_right_topic", teleop_right_topic)
    return camera


def main():
    args = build_parser().parse_args()
    robot_names = resolve_robot_names(args)
    requested_stereo_count = int(args.stereo_robot_count)
    if requested_stereo_count < 0:
        stereo_count = len(robot_names)
    else:
        stereo_count = max(0, min(requested_stereo_count, len(robot_names)))
    stereo_eye_resolution = parse_resolution(args.stereo_eye_resolution, (960, 540))
    mono_resolution = parse_resolution(args.mono_resolution, stereo_eye_resolution)
    highres_eye_resolution = parse_resolution(args.highres_eye_resolution, (1920, 1080))
    highres_camera_fps = max(1, int(args.highres_camera_fps))
    image_type = args.camera_image_type.strip().lower()
    stereo_mode = args.stereo_input_mode.strip().lower()
    requested_highres_robot = (args.highres_robot or "").strip()
    highres_robot_name = requested_highres_robot if requested_highres_robot else (robot_names[0] if robot_names else "")
    if highres_robot_name and highres_robot_name not in robot_names:
        cli.print_info(
            f"Requested --highres-robot '{highres_robot_name}' is not in the active robot list; disabling high-load override."
        )
        highres_robot_name = ""

    cli.print_step(
        "Registering stereo camera demo robots "
        f"({len(robot_names)} robots, {stereo_count} stereo, mode={stereo_mode}, eye={stereo_eye_resolution[0]}x{stereo_eye_resolution[1]})..."
    )
    cli.print_info(
        "Transport policy: minimap=ros, teleop="
        + (
            "ros for dual-topic stereo robots, webrtc for mono robots"
            if stereo_mode == "dual_topic"
            else "webrtc (sbs and mono robots)"
        )
    )
    cli.print_info(f"Robots: {', '.join(robot_names)}")
    if stereo_count > 0:
        cli.print_info(f"Stereo cameras: {', '.join(robot_names[:stereo_count])}")
    if highres_robot_name:
        cli.print_info(
            "High-load camera profile: "
            f"{highres_robot_name} @ {highres_eye_resolution[0]}x{highres_eye_resolution[1]} per eye, {highres_camera_fps} FPS"
        )

    robots = []
    datavizs = []
    for idx, name in enumerate(robot_names):
        robot = Robot(
            name=name,
            robot_type=RobotType.WHEELED,
            dimensions=RobotDimensions(length=0.8, width=0.55, height=0.4),
        )

        robot.add_metadata(
            "robot_manager_config",
            {
                "enabled": True,
                "prefab_asset_path": "Assets/Prefabs/UI/RobotManager.prefab",
                "prefab_resource_path": "",
                "sections": {"status": True, "data_viz": True, "teleop": True, "tasks": True},
            },
        )

        is_stereo = idx < stereo_count
        is_highres_robot = bool(highres_robot_name) and name == highres_robot_name
        selected_eye_resolution = highres_eye_resolution if is_highres_robot else stereo_eye_resolution
        selected_mono_resolution = highres_eye_resolution if is_highres_robot else mono_resolution
        selected_fps = highres_camera_fps if is_highres_robot else args.camera_fps
        camera = build_camera(
            robot_name=name,
            image_type=image_type,
            fps=selected_fps,
            mono_resolution=selected_mono_resolution,
            stereo_eye_resolution=selected_eye_resolution,
            is_stereo=is_stereo,
            stereo_input_mode=stereo_mode,
        )
        robot.add_sensor(camera)

        robots.append(robot)
        datavizs.append(robot.create_dataviz())

    success, result = register_robots(
        robots,
        datavizs=datavizs,
        keep_alive=args.keep_alive,
        show_dashboard=True,
        workspace_scale=args.workspace_scale,
    )
    if not success:
        error_text = str((result or {}).get("error", "")).strip().lower()
        if "cancel" in error_text:
            cli.print_info("Stereo registration demo cancelled by user.")
            return
        cli.print_error(f"Registration failed: {result}")


if __name__ == "__main__":
    main()
