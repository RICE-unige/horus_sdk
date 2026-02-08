#!/usr/bin/env python3
"""
Example of registering multiple robots with Horus using the Python SDK.
This demo sends only the robot transform (TF) using the robot name as the TF prefix
and includes robot dimensions for sizing the interactor surface.

Usage:
  python3 sdk_registration_demo.py
  python3 sdk_registration_demo.py --robot-count 4
  python3 sdk_registration_demo.py --robot-count 4 --with-camera
  python3 sdk_registration_demo.py --robot-count 4 --with-camera --camera-streaming-type webrtc
  python3 sdk_registration_demo.py --robot-names test_bot_1,test_bot_2
"""

import sys
import os
import argparse

# Ensure we can import 'horus' package regardless of where script is run from
script_dir = os.path.dirname(os.path.abspath(__file__))
package_root = os.path.join(script_dir, "..")
if package_root not in sys.path:
    sys.path.insert(0, package_root)

try:
    from horus.robot import Robot, RobotDimensions, RobotType, register_robots
    from horus.sensors import Camera
    from horus.utils import cli
except ImportError:
    # Surface the resolved package path when imports fail.
    print(f"Failed to import horus from {package_root}")
    raise


def parse_resolution_list(raw_value):
    if not raw_value:
        return []

    resolutions = []
    for token in raw_value.split(","):
        item = token.strip().lower()
        if not item or "x" not in item:
            continue
        left, right = item.split("x", 1)
        try:
            width = max(16, int(left))
            height = max(16, int(right))
        except ValueError:
            continue
        resolutions.append((width, height))
    return resolutions


def build_parser():
    parser = argparse.ArgumentParser(
        description="Register multiple robots with Horus for MR testing."
    )
    parser.add_argument(
        "--robot-name",
        default="test_bot",
        help="Robot name (single) or prefix (multi).",
    )
    parser.add_argument(
        "--robot-names",
        default="",
        help="Comma-separated list of robot names (overrides --robot-name/--robot-count).",
    )
    parser.add_argument("--robot-count", type=int, default=10, help="Number of robots")
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
        help="Exit after registering robots.",
    )
    parser.add_argument("--length", type=float, default=0.8, help="Robot length in meters")
    parser.add_argument("--width", type=float, default=0.6, help="Robot width in meters")
    parser.add_argument("--height", type=float, default=0.4, help="Robot height in meters")
    parser.add_argument(
        "--with-camera",
        dest="with_camera",
        action="store_true",
        default=True,
        help="Add one camera sensor per robot on /<robot>/camera/image_raw (default: on).",
    )
    parser.add_argument(
        "--no-camera",
        dest="with_camera",
        action="store_false",
        help="Disable camera sensor registration.",
    )
    parser.add_argument(
        "--camera-image-scale",
        type=float,
        default=0.072,
        help="Projected camera panel scale (default: 0.072).",
    )
    parser.add_argument(
        "--camera-distance-scale",
        type=float,
        default=0.108,
        help="Projected panel distance from camera frame (default: 0.108).",
    )
    parser.add_argument(
        "--camera-image-type",
        choices=["compressed", "raw"],
        default="compressed",
        help="Camera transport type to register (default: compressed).",
    )
    parser.add_argument(
        "--camera-streaming-type",
        choices=["ros", "webrtc"],
        default="ros",
        help="Camera streaming backend for MR visualization (default: ros).",
    )
    parser.add_argument(
        "--webrtc-client-signal-topic",
        default="/horus/webrtc/client_signal",
        help="ROS topic used by Unity client to send WebRTC signaling (default: /horus/webrtc/client_signal).",
    )
    parser.add_argument(
        "--webrtc-server-signal-topic",
        default="/horus/webrtc/server_signal",
        help="ROS topic used by bridge to send WebRTC signaling back to Unity (default: /horus/webrtc/server_signal).",
    )
    parser.add_argument(
        "--webrtc-bitrate-kbps",
        type=int,
        default=2000,
        help="Requested WebRTC video bitrate in kbps (default: 2000).",
    )
    parser.add_argument(
        "--webrtc-framerate",
        type=int,
        default=20,
        help="Requested WebRTC stream framerate in fps (default: 20).",
    )
    parser.add_argument(
        "--webrtc-stun-server-url",
        default="stun:stun.l.google.com:19302",
        help="STUN server URL for Unity WebRTC client (default: stun:stun.l.google.com:19302).",
    )
    parser.add_argument(
        "--webrtc-turn-server-url",
        default="",
        help="Optional TURN server URL for Unity WebRTC client, e.g. turn:127.0.0.1:3478?transport=tcp.",
    )
    parser.add_argument(
        "--webrtc-turn-username",
        default="",
        help="TURN username (optional).",
    )
    parser.add_argument(
        "--webrtc-turn-credential",
        default="",
        help="TURN credential/password (optional).",
    )
    parser.add_argument(
        "--vary-camera-resolution",
        action="store_true",
        default=True,
        help="Vary camera resolution across robots (default: on).",
    )
    parser.add_argument(
        "--no-vary-camera-resolution",
        dest="vary_camera_resolution",
        action="store_false",
        help="Use one camera resolution for all robots.",
    )
    parser.add_argument(
        "--camera-resolutions",
        default="320x180,426x240,640x360,848x480",
        help="Comma-separated WxH list used when varying camera resolution.",
    )
    return parser


def resolve_robot_names(args):
    if args.robot_names:
        names = [name.strip() for name in args.robot_names.split(",") if name.strip()]
        if names:
            return names

    if args.robot_count <= 1:
        return [args.robot_name]

    return [f"{args.robot_name}_{idx + 1}" for idx in range(args.robot_count)]

def main():
    args = build_parser().parse_args()
    robot_names = resolve_robot_names(args)
    camera_resolution_list = parse_resolution_list(args.camera_resolutions)
    if not camera_resolution_list:
        camera_resolution_list = [(320, 180)]

    cli.print_step("Defining Robot Configuration...")
    robots = []
    for idx, name in enumerate(robot_names):
        length = args.length + (0.1 * idx)
        width = args.width + (0.05 * idx)
        height = args.height + (0.03 * idx)
        robot = Robot(
            # Name should match the TF prefix (e.g. test_bot_1/base_link)
            name=name,
            robot_type=RobotType.WHEELED,
            dimensions=RobotDimensions(
                length=length,
                width=width,
                height=height,
            ),
        )
        if args.with_camera:
            image_type = args.camera_image_type.strip().lower()
            if image_type == "compressed":
                topic = f"/{name}/camera/image_raw/compressed"
                encoding = "jpeg"
            else:
                topic = f"/{name}/camera/image_raw"
                encoding = "rgb8"

            if args.vary_camera_resolution:
                resolution = camera_resolution_list[idx % len(camera_resolution_list)]
            else:
                resolution = camera_resolution_list[0]

            width_px, height_px = resolution

            camera = Camera(
                name="front_camera",
                frame_id=f"{name}/camera_link",
                topic=topic,
                resolution=(width_px, height_px),
                fps=6,
                encoding=encoding,
                streaming_type=args.camera_streaming_type,
            )
            camera.add_metadata("image_type", image_type)
            camera.add_metadata("streaming_type", args.camera_streaming_type)
            camera.add_metadata("display_mode", "projected")
            camera.add_metadata("use_tf", True)
            camera.add_metadata("webrtc_client_signal_topic", args.webrtc_client_signal_topic)
            camera.add_metadata("webrtc_server_signal_topic", args.webrtc_server_signal_topic)
            camera.add_metadata("webrtc_bitrate_kbps", max(100, args.webrtc_bitrate_kbps))
            camera.add_metadata("webrtc_framerate", max(1, args.webrtc_framerate))
            camera.add_metadata("webrtc_stun_server_url", args.webrtc_stun_server_url)
            camera.add_metadata("webrtc_turn_server_url", args.webrtc_turn_server_url)
            camera.add_metadata("webrtc_turn_username", args.webrtc_turn_username)
            camera.add_metadata("webrtc_turn_credential", args.webrtc_turn_credential)
            camera.add_metadata("image_scale", max(0.05, args.camera_image_scale))
            camera.add_metadata("focal_length_scale", max(0.05, args.camera_distance_scale))
            camera.add_metadata("view_position_offset", [0.0, 0.0, 0.0])
            camera.add_metadata("view_rotation_offset", [0.0, 0.0, 0.0])
            camera.add_metadata("show_frustum", True)
            camera.add_metadata("frustum_color", "#FFFF00")
            camera.add_metadata("overhead_size", 1.0)
            camera.add_metadata("overhead_position_offset", [0.0, 2.0, 0.0])
            camera.add_metadata("overhead_face_camera", True)
            camera.add_metadata("overhead_rotation_offset", [90.0, 0.0, 0.0])
            robot.add_sensor(camera)

        robots.append(robot)

    cli.print_step(f"Registering {len(robots)} robot(s)...")
    success, result = register_robots(
        robots,
        keep_alive=args.keep_alive,
        show_dashboard=True,
    )
    if not success:
        if isinstance(result, dict) and result.get("error") == "Cancelled":
            cli.print_info("Registration cancelled by user.")
            return
        cli.print_error(f"Registration failed: {result}")
        return


if __name__ == "__main__":
    main()
