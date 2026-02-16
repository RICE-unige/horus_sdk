#!/usr/bin/env python3
"""
Example of registering multiple robots with Horus using the Python SDK.
This demo sends only the robot transform (TF) using the robot name as the TF prefix
and includes robot dimensions for sizing the interactor surface.

Usage:
  python3 sdk_registration_demo.py
  python3 sdk_registration_demo.py --robot-count 4
  python3 sdk_registration_demo.py --robot-count 4 --with-camera
  python3 sdk_registration_demo.py --with-occupancy-grid
  python3 sdk_registration_demo.py --with-3d-map --map-3d-topic /map_3d
  python3 sdk_registration_demo.py --with-fake-tf --with-3d-map
  python3 sdk_registration_demo.py --robot-count 4 --with-camera --camera-streaming-type webrtc
  python3 sdk_registration_demo.py --camera-minimap-streaming-type ros --camera-teleop-streaming-type webrtc
  python3 sdk_registration_demo.py --teleop-profile wheeled --teleop-response-mode analog
  python3 sdk_registration_demo.py --robot-names test_bot_1,test_bot_2
"""

import sys
import os
import argparse
import subprocess
import time

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
        default=None,
        help="Legacy single camera transport profile alias (used as fallback).",
    )
    parser.add_argument(
        "--camera-minimap-streaming-type",
        choices=["ros", "webrtc"],
        default=None,
        help="Camera transport profile for MiniMap mode (default: ros).",
    )
    parser.add_argument(
        "--camera-teleop-streaming-type",
        choices=["ros", "webrtc"],
        default=None,
        help="Camera transport profile for Teleop mode (default: webrtc).",
    )
    parser.add_argument(
        "--camera-startup-mode",
        choices=["minimap", "teleop"],
        default="minimap",
        help="Preferred startup mode metadata (MR currently forces minimap).",
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
        default="160x90,192x108,224x126,256x144,320x180,426x240",
        help="Comma-separated WxH list used when varying camera resolution.",
    )
    parser.add_argument(
        "--with-occupancy-grid",
        dest="with_occupancy_grid",
        action="store_true",
        default=False,
        help="Publish global occupancy-grid visualization config to MR (default: off).",
    )
    parser.add_argument(
        "--no-occupancy-grid",
        dest="with_occupancy_grid",
        action="store_false",
        help="Disable occupancy-grid visualization config in registration payload.",
    )
    parser.add_argument(
        "--occupancy-topic",
        default="/map",
        help="Occupancy grid topic (default: /map).",
    )
    parser.add_argument(
        "--occupancy-frame",
        default="map",
        help="Occupancy grid frame id (default: map).",
    )
    parser.add_argument(
        "--occupancy-position-scale",
        type=float,
        default=None,
        help="Optional occupancy map position scale override for MR visualizer. Ignored when workspace scale is set.",
    )
    parser.add_argument(
        "--workspace-scale",
        type=float,
        default=None,
        help="Optional global workspace position scale applied across MR entities.",
    )
    parser.add_argument(
        "--occupancy-show-unknown-space",
        dest="occupancy_show_unknown_space",
        action="store_true",
        default=True,
        help="Render unknown occupancy cells in MR (default: on).",
    )
    parser.add_argument(
        "--occupancy-hide-unknown-space",
        dest="occupancy_show_unknown_space",
        action="store_false",
        help="Hide unknown occupancy cells in MR.",
    )
    parser.add_argument(
        "--with-3d-map",
        dest="with_3d_map",
        action="store_true",
        default=False,
        help="Publish global 3D map (PointCloud2) visualization config to MR (default: off).",
    )
    parser.add_argument(
        "--map-3d-topic",
        default="/map_3d",
        help="3D map PointCloud2 topic (default: /map_3d).",
    )
    parser.add_argument(
        "--map-3d-frame",
        default="map",
        help="3D map frame id (default: map).",
    )
    parser.add_argument(
        "--with-fake-tf",
        dest="with_fake_tf",
        action="store_true",
        default=False,
        help="Auto-start python/examples/fake_tf_publisher.py with matching robot names.",
    )
    parser.add_argument(
        "--fake-tf-rate",
        type=float,
        default=30.0,
        help="Publish rate used by auto-started fake_tf_publisher.py (Hz).",
    )
    parser.add_argument(
        "--fake-tf-warmup-seconds",
        type=float,
        default=1.0,
        help="Delay after spawning fake TF publisher before registration starts.",
    )
    parser.add_argument(
        "--fake-tf-base-frame",
        default="base_link",
        help="Base frame used by auto-started fake_tf_publisher.py (default: base_link).",
    )
    parser.add_argument(
        "--teleop-profile",
        choices=["wheeled", "legged", "aerial", "custom"],
        default="wheeled",
        help="Teleop robot profile included in registration payload.",
    )
    parser.add_argument(
        "--teleop-response-mode",
        choices=["analog", "discrete"],
        default="analog",
        help="Teleop joystick response mode included in registration payload.",
    )
    parser.add_argument(
        "--teleop-publish-rate",
        type=float,
        default=30.0,
        help="Teleop command publish rate hint (Hz) included in registration payload.",
    )
    parser.add_argument(
        "--teleop-passthrough-only",
        action="store_true",
        help="Set teleop custom_passthrough_only=true in payload.",
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


def start_fake_tf_process(args, robot_names):
    fake_tf_script = os.path.join(script_dir, "fake_tf_publisher.py")
    tf_map_frame = args.map_3d_frame if args.with_3d_map else args.occupancy_frame

    command = [
        sys.executable,
        fake_tf_script,
        "--robot-names",
        ",".join(robot_names),
        "--map-frame",
        tf_map_frame,
        "--base-frame",
        args.fake_tf_base_frame,
        "--rate",
        str(max(0.1, float(args.fake_tf_rate))),
    ]

    if args.with_camera:
        command.extend(["--static-camera", "--publish-compressed-images"])
    else:
        command.extend(["--no-publish-compressed-images", "--no-publish-images"])

    if args.with_occupancy_grid:
        command.extend(["--publish-occupancy-grid", "--occupancy-topic", args.occupancy_topic])
    else:
        command.append("--no-publish-occupancy-grid")

    cli.print_info(f"Starting fake TF publisher: {' '.join(command)}")
    process = subprocess.Popen(command, cwd=script_dir)
    warmup = max(0.0, float(args.fake_tf_warmup_seconds))
    if warmup > 0.0:
        time.sleep(warmup)
    return process


def stop_fake_tf_process(process):
    if process is None or process.poll() is not None:
        return

    process.terminate()
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5.0)


def main():
    args = build_parser().parse_args()
    robot_names = resolve_robot_names(args)
    camera_resolution_list = parse_resolution_list(args.camera_resolutions)
    if not camera_resolution_list:
        camera_resolution_list = [(160, 90)]
    legacy_streaming_choice = (args.camera_streaming_type or "").strip().lower()
    minimap_streaming_choice = (
        args.camera_minimap_streaming_type
        or legacy_streaming_choice
        or "ros"
    )
    teleop_streaming_choice = (
        args.camera_teleop_streaming_type
        or legacy_streaming_choice
        or "webrtc"
    )
    startup_mode_choice = args.camera_startup_mode.strip().lower()
    if startup_mode_choice not in ("minimap", "teleop"):
        startup_mode_choice = "minimap"
    legacy_payload_streaming_type = legacy_streaming_choice or minimap_streaming_choice

    cli.print_step("Defining Robot Configuration...")
    robots = []
    datavizs = []
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
                streaming_type=legacy_payload_streaming_type,
                minimap_streaming_type=minimap_streaming_choice,
                teleop_streaming_type=teleop_streaming_choice,
                startup_mode=startup_mode_choice,
            )
            camera.add_metadata("image_type", image_type)
            camera.add_metadata("streaming_type", legacy_payload_streaming_type)
            camera.add_metadata("minimap_streaming_type", minimap_streaming_choice)
            camera.add_metadata("teleop_streaming_type", teleop_streaming_choice)
            camera.add_metadata("startup_mode", startup_mode_choice)
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

        dataviz = robot.create_dataviz()
        if args.with_occupancy_grid:
            occupancy_render_options = {
                "show_unknown_space": bool(args.occupancy_show_unknown_space),
            }
            if args.occupancy_position_scale is not None:
                occupancy_render_options["position_scale"] = max(0.01, float(args.occupancy_position_scale))

            dataviz.add_occupancy_grid(
                topic=args.occupancy_topic,
                frame_id=args.occupancy_frame,
                render_options=occupancy_render_options,
            )
        if args.with_3d_map:
            dataviz.add_3d_map(
                topic=args.map_3d_topic,
                frame_id=args.map_3d_frame,
                render_options={
                    "point_size": 0.05,
                    "max_points_per_frame": 0,
                    "base_sample_stride": 1,
                    "min_update_interval": 0.0,
                    "enable_adaptive_quality": False,
                    "target_framerate": 72.0,
                    "min_quality_multiplier": 0.6,
                    "min_distance": 0.0,
                    "max_distance": 0.0,
                    "replace_latest": True,
                    "render_all_points": True,
                    "auto_point_size_by_workspace_scale": True,
                    "min_point_size": 0.002,
                    "max_point_size": 0.04,
                    "render_mode": "opaque_fast",
                    "enable_view_frustum_culling": True,
                    "frustum_padding": 0.03,
                    "enable_subpixel_culling": True,
                    "min_screen_radius_px": 0.8,
                    "visible_points_budget": 120000,
                    "max_visible_points_budget": 200000,
                    "map_static_mode": True,
                },
            )
        datavizs.append(dataviz)
        robots.append(robot)

    fake_tf_process = None
    try:
        if args.with_fake_tf:
            fake_tf_process = start_fake_tf_process(args, robot_names)
            if args.with_3d_map:
                cli.print_info(
                    "Fake TF is running. Start fake_3d_map_publisher.py separately for /map_3d data."
                )

        cli.print_step(f"Registering {len(robots)} robot(s)...")
        success, result = register_robots(
            robots,
            datavizs=datavizs,
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
    finally:
        stop_fake_tf_process(fake_tf_process)


if __name__ == "__main__":
    main()
