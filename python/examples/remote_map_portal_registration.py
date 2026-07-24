#!/usr/bin/env python3
"""Register the PC-rendered spatial RGB-D map in HORUS MR.

The SDK process publishes locally rendered packed RGB-D frames. The existing
HORUS ROS2 bridge owns WebRTC signaling, H.264 encoding, RTP, and per-operator
sessions. Unity is not launched or required at runtime.
"""

from __future__ import annotations

import argparse
import atexit
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

from horus.bridge.robot_registry import get_robot_registry_client
from horus.dataviz import MapRenderTarget, MapUpdateMode
from horus.robot import WorkspaceVisualization, is_registration_cancelled, register_entities
from horus.remote_rendering import (
    COW_LADY_DEPTH_FAR_BASE,
    COW_LADY_DEPTH_NEAR_BASE,
    COW_LADY_VIEW_SPECS,
    DEFAULT_VIEWER_POSE_TOPIC,
    DYNAMIC_RGBD_FORMAT_VERSION,
    ETH3D_COURTYARD_DEPTH_FAR_BASE,
    ETH3D_COURTYARD_DEPTH_NEAR_BASE,
    ETH3D_COURTYARD_VIEW_SPECS,
    REMOTE_RGBD_FORMAT_VERSION,
)


STREAM_TOPIC = "/horus/remote_render/map_portal"
ROS_STREAM_TOPIC = "/horus/remote_render/map_rgbd/compressed"
CLIENT_SIGNAL_TOPIC = "/horus/webrtc/client_signal"
SERVER_SIGNAL_TOPIC = "/horus/webrtc/server_signal"
AGENT_STATUS_TOPIC = "/horus/remote_render/agent_status"
ROOT_FRAME = "map"
RENDER_CAMERA_FOV_DEG = 52.0
RENDER_DEPTH_NEAR_M = 8.0
RENDER_DEPTH_FAR_M = 42.0
# Native renderer camera: position=(0, 15, -19), pitch=38 degrees down.
# Converted through the HORUS Unity<->ROS FLU coordinate conventions.
RENDER_CAMERA_POSITION_ROS = (-19.0, 0.0, 15.0)
RENDER_CAMERA_QUATERNION_ROS = (0.0, 0.32556815, 0.0, 0.94551858)
AGENT_SCRIPT = Path(__file__).resolve().parent / "tools" / "remote_map_render_agent.py"
CACHE_ROOT = Path.home() / ".cache" / "horus" / "remote_map"

STREAM_PROFILES = {
    "balanced": {
        "refresh": (960, 540, 30, 12000, 240, 135),
        "static": (1440, 810, 10, 14000, 240, 135),
    },
    "fast60": {
        "refresh": (960, 540, 60, 18000, 240, 135),
        "static": (1440, 810, 10, 18000, 240, 135),
    },
    "quality": {
        "refresh": (1280, 720, 30, 22000, 320, 180),
        "static": (1920, 1080, 10, 26000, 320, 180),
    },
    "quality60": {
        "refresh": (1280, 720, 60, 30000, 320, 180),
        "static": (1920, 1080, 10, 30000, 320, 180),
    },
}


def _unity_view_payload(view, scale: float):
    x, y, z = view.position
    half_yaw = math.radians(view.yaw_degrees) * 0.5
    half_pitch = math.radians(view.pitch_degrees) * 0.5
    sy, cy = math.sin(half_yaw), math.cos(half_yaw)
    sx, cx = math.sin(half_pitch), math.cos(half_pitch)
    unity_x = cy * sx
    unity_y = sy * cx
    unity_z = -sy * sx
    unity_w = cy * cx
    return {
        "id": view.name,
        "atlas_x": view.atlas_x,
        "atlas_y": view.atlas_y,
        "atlas_width": view.atlas_width,
        "atlas_height": view.atlas_height,
        "camera_position": {
            "x": z * scale,
            "y": -x * scale,
            "z": y * scale,
        },
        # RosCoordinateUtility.UnityToFluRotation(Unity yaw * pitch).
        "camera_rotation": {
            "x": unity_z,
            "y": -unity_x,
            "z": unity_y,
            "w": -unity_w,
        },
    }


def build_workspace_visualization(args):
    if args.scene == "cow_lady":
        depth_near_m = COW_LADY_DEPTH_NEAR_BASE * args.dataset_scale
        depth_far_m = COW_LADY_DEPTH_FAR_BASE * args.dataset_scale
        view_payloads = [
            _unity_view_payload(view, args.dataset_scale)
            for view in COW_LADY_VIEW_SPECS
        ]
        camera_position_ros = tuple(view_payloads[0]["camera_position"].values())
        camera_quaternion_ros = tuple(view_payloads[0]["camera_rotation"].values())
    elif args.scene == "eth3d_courtyard":
        depth_near_m = ETH3D_COURTYARD_DEPTH_NEAR_BASE * args.dataset_scale
        depth_far_m = ETH3D_COURTYARD_DEPTH_FAR_BASE * args.dataset_scale
        view_payloads = [
            _unity_view_payload(view, args.dataset_scale)
            for view in ETH3D_COURTYARD_VIEW_SPECS
        ]
        camera_position_ros = tuple(view_payloads[0]["camera_position"].values())
        camera_quaternion_ros = tuple(view_payloads[0]["camera_rotation"].values())
    else:
        depth_near_m = RENDER_DEPTH_NEAR_M
        depth_far_m = RENDER_DEPTH_FAR_M
        camera_position_ros = RENDER_CAMERA_POSITION_ROS
        camera_quaternion_ros = RENDER_CAMERA_QUATERNION_ROS
        view_payloads = [
            {
                "id": "primary",
                "atlas_x": 0.0,
                "atlas_y": 0.0,
                "atlas_width": 1.0,
                "atlas_height": 1.0,
                "camera_position": {
                    "x": camera_position_ros[0],
                    "y": camera_position_ros[1],
                    "z": camera_position_ros[2],
                },
                "camera_rotation": {
                    "x": camera_quaternion_ros[0],
                    "y": camera_quaternion_ros[1],
                    "z": camera_quaternion_ros[2],
                    "w": camera_quaternion_ros[3],
                },
            }
        ]
    if args.dynamic_view:
        if args.scene == "cow_lady":
            depth_near_m = args.depth_near_m or max(0.1, 0.1 * args.dataset_scale)
            depth_far_m = args.depth_far_m or 40.0 * args.dataset_scale
        else:
            depth_near_m = args.depth_near_m or max(0.25, 0.5 * args.dataset_scale)
            depth_far_m = args.depth_far_m or 60.0 * args.dataset_scale
        camera_position_ros = (0.0, 0.0, 0.0)
        camera_quaternion_ros = (0.0, 0.0, 0.0, 1.0)
        view_payloads = [
            {
                "id": "pose_timewarp",
                "atlas_x": 0.0,
                "atlas_y": 0.0,
                "atlas_width": 1.0,
                "atlas_height": 1.0,
                "camera_position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "camera_rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }
        ]
    resource = WorkspaceVisualization("workspace_remote_map")
    dataviz = resource.create_dataviz("workspace_remote_map")
    dataviz.add_3d_map(
        topic=STREAM_TOPIC,
        frame_id=ROOT_FRAME,
        render_target=MapRenderTarget.REMOTE,
        update_mode=MapUpdateMode(args.update_mode),
        render_options={
            "transport": "ros_compressed" if args.transport == "ros" else "webrtc",
            "client_signal_topic": CLIENT_SIGNAL_TOPIC,
            "format_version": (
                DYNAMIC_RGBD_FORMAT_VERSION if args.dynamic_view else REMOTE_RGBD_FORMAT_VERSION
            ),
            "dynamic_view": args.dynamic_view,
            "viewer_pose_topic": args.viewer_pose_topic,
            "viewer_pose_rate_hz": args.viewer_pose_rate_hz,
            "pose_position_range_m": args.pose_position_range,
            "ros_compressed_topic": ROS_STREAM_TOPIC,
            "server_signal_topic": SERVER_SIGNAL_TOPIC,
            "status_topic": AGENT_STATUS_TOPIC,
            "encoder": args.encoder,
            "bitrate_kbps": args.render_bitrate_kbps,
            "framerate": args.render_fps,
            "depth_near_m": depth_near_m,
            "depth_far_m": depth_far_m,
            "vertical_fov_deg": (
                args.dynamic_vertical_fov_deg if args.dynamic_view else RENDER_CAMERA_FOV_DEG
            ),
            "view_aspect": args.render_width / args.render_height,
            "grid_columns": args.reprojection_columns,
            "grid_rows": args.reprojection_rows,
            "flip_y": args.flip_stream_y,
            "camera_position": camera_position_ros,
            "camera_rotation": {
                "x": camera_quaternion_ros[0],
                "y": camera_quaternion_ros[1],
                "z": camera_quaternion_ros[2],
                "w": camera_quaternion_ros[3],
            },
            "views": view_payloads,
        },
    )
    return resource, dataviz


def start_signal_monitor(registry, transport: str):
    if registry is None or not getattr(registry, "ros_initialized", False) or registry.node is None:
        return None

    from std_msgs.msg import String

    state = {
        "client_offer": False,
        "server_answer": False,
        "server_error": False,
        "agent_ready": False,
        "last_offer_time": 0.0,
        "last_missing_answer_warning": 0.0,
    }

    def parse_signal(message):
        try:
            payload = json.loads((message.data or "").replace("\0", "").strip())
        except (TypeError, ValueError, json.JSONDecodeError):
            return None
        if payload.get("stream_topic") not in (None, "", STREAM_TOPIC):
            return None
        return payload

    def on_client_signal(message):
        signal = parse_signal(message)
        if not signal or str(signal.get("type", "")).lower() != "offer":
            return
        state["client_offer"] = True
        state["last_offer_time"] = time.monotonic()
        print(
            "[remote-map] Quest WebRTC offer observed "
            f"session={signal.get('session_id', '')} bitrate={signal.get('bitrate_kbps', '')} "
            f"fps={signal.get('framerate', '')}."
        )

    def on_server_signal(message):
        signal = parse_signal(message)
        if not signal:
            return
        signal_type = str(signal.get("type", "")).lower()
        if signal_type == "answer":
            state["server_answer"] = True
            print(f"[remote-map] HORUS ROS2 WebRTC answer observed session={signal.get('session_id', '')}.")
        elif signal_type == "error":
            state["server_error"] = True
            print(f"[remote-map] HORUS ROS2 WebRTC error: {signal.get('error', '')}")

    def on_agent_status(message):
        try:
            status = json.loads((message.data or "").replace("\0", "").strip())
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if status.get("stream_topic") != STREAM_TOPIC:
            return
        if not state["agent_ready"]:
            state["agent_ready"] = True
            print(
                "[remote-map] Native render agent online "
                f"renderer={status.get('renderer', '')} transport={status.get('transport_owner', '')} "
                f"size={status.get('width', '')}x{status.get('height', '')}@{status.get('fps', '')}."
            )

    def warn_if_answer_missing():
        if not state["client_offer"] or state["server_answer"] or state["server_error"]:
            return
        now = time.monotonic()
        if now - state["last_offer_time"] < 8.0 or now - state["last_missing_answer_warning"] < 15.0:
            return
        state["last_missing_answer_warning"] = now
        print(
            "[remote-map] WARNING: Quest offer has no HORUS ROS2 WebRTC answer. "
            "Inspect the horus_unity_bridge log."
        )

    subscriptions = [
        registry.node.create_subscription(String, AGENT_STATUS_TOPIC, on_agent_status, 32),
    ]
    if transport == "webrtc":
        subscriptions.extend(
            (
                registry.node.create_subscription(String, CLIENT_SIGNAL_TOPIC, on_client_signal, 32),
                registry.node.create_subscription(String, SERVER_SIGNAL_TOPIC, on_server_signal, 32),
            )
        )
    warning_timer = registry.node.create_timer(1.0, warn_if_answer_missing)
    return (*subscriptions, warning_timer)


def _tail(path: Path, lines: int = 30) -> str:
    try:
        return "\n".join(path.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:])
    except OSError:
        return ""


def resolve_render_encoder(
    requested: str,
    width: int = 960,
    height: int = 540,
    bitrate_kbps: int = 5000,
    framerate: int = 30,
) -> str:
    requested = str(requested or "auto").strip().lower()
    if requested == "x264":
        return "x264"

    CACHE_ROOT.mkdir(parents=True, exist_ok=True)
    output_path = CACHE_ROOT / "nvenc-probe.h264"
    output_path.unlink(missing_ok=True)
    command = [
        "gst-launch-1.0",
        "-q",
        "videotestsrc",
        "num-buffers=4",
        "!",
        f"video/x-raw,width={width * 2},height={height},framerate={framerate}/1",
        "!",
        "videoconvert",
        "!",
        "video/x-raw,format=NV12",
        "!",
        "nvcudah264enc",
        "preset=p2",
        "tune=ultra-low-latency",
        "rate-control=cbr",
        f"bitrate={bitrate_kbps}",
        f"gop-size={framerate}",
        "b-frames=0",
        "rc-lookahead=0",
        "repeat-sequence-header=true",
        "!",
        "video/x-h264,profile=constrained-baseline",
        "!",
        "h264parse",
        "!",
        "filesink",
        f"location={output_path}",
    ]
    try:
        probe = subprocess.run(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=10.0,
            check=False,
        )
        encoded_bytes = output_path.stat().st_size if output_path.exists() else 0
        diagnostics = (probe.stderr or "").lower()
        healthy = probe.returncode == 0 and encoded_bytes > 128
    except (OSError, subprocess.SubprocessError):
        healthy = False
        encoded_bytes = 0
    finally:
        output_path.unlink(missing_ok=True)

    if healthy:
        print(f"[remote-map] NVENC runtime probe passed ({encoded_bytes} encoded bytes).")
        if "couldn't compile nvrtc" in diagnostics:
            print(
                "[remote-map] Note: GStreamer could not compile an optional CUDA conversion "
                "kernel, but NVENC produced a valid H.264 stream."
            )
        return "nvenc"
    if requested == "nvenc":
        raise RuntimeError(
            "NVENC was requested but its runtime probe failed. "
            "Use --encoder x264 or repair the CUDA/GStreamer runtime."
        )
    print(
        "[remote-map] NVENC runtime probe failed; using zero-latency x264. "
        "The installed GStreamer NVENC factory is not usable with the current CUDA runtime."
    )
    return "x264"


def start_native_render_agent(args):
    if args.no_launch_render_agent:
        print("[remote-map] Native render-agent auto-launch disabled.")
        return None
    agent_script = AGENT_SCRIPT
    if not agent_script.exists():
        raise RuntimeError(f"render-agent script is missing: {agent_script}")

    CACHE_ROOT.mkdir(parents=True, exist_ok=True)
    ready_path = CACHE_ROOT / f"ready-{uuid.uuid4().hex}.json"
    log_path = CACHE_ROOT / "agent.log"
    preview_path = CACHE_ROOT / "preview.png"
    ready_path.unlink(missing_ok=True)
    command = [
        sys.executable,
        str(agent_script),
        "--width", str(args.render_width),
        "--height", str(args.render_height),
        "--fps", str(args.render_fps),
        "--ready-file", str(ready_path),
        "--preview-path", str(preview_path),
        "--scene", args.scene,
        "--dataset-scale", str(args.dataset_scale),
        "--point-splat-radius", str(args.point_splat_radius),
        "--transport", "ros_compressed" if args.transport == "ros" else "webrtc_source",
        "--jpeg-quality", str(args.jpeg_quality),
    ]
    if args.dynamic_view:
        command.extend(
            (
                "--dynamic-view",
                "--viewer-pose-topic", args.viewer_pose_topic,
                "--dynamic-view-baseline", str(args.dynamic_view_baseline),
                "--render-update-fps", str(args.render_update_fps),
                "--vertical-fov-deg", str(args.dynamic_vertical_fov_deg),
                "--pose-position-range", str(args.pose_position_range),
                "--depth-near-m", str(args.depth_near_m),
                "--depth-far-m", str(args.depth_far_m),
            )
        )
    if args.dataset_path:
        command.extend(("--dataset-path", args.dataset_path))
    environment = os.environ.copy()
    python_root = str(Path(__file__).resolve().parents[1])
    existing_pythonpath = environment.get("PYTHONPATH", "")
    environment["PYTHONPATH"] = python_root + (os.pathsep + existing_pythonpath if existing_pythonpath else "")
    with log_path.open("w", encoding="utf-8") as log_file:
        process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            env=environment,
        )

    preparation_timeout = {
        "synthetic": 25.0,
        "cow_lady": 90.0,
        "eth3d_courtyard": 600.0,
    }[args.scene]
    deadline = time.monotonic() + preparation_timeout
    while time.monotonic() < deadline:
        if ready_path.exists():
            ready = json.loads(ready_path.read_text(encoding="utf-8"))
            print(
                "[remote-map] native renderer ready "
                f"pid={process.pid} transport={ready.get('transport', args.transport)} "
                f"encoder={ready.get('encoder')} "
                f"mode={args.update_mode} profile={args.stream_profile} "
                f"scene={ready.get('scene', args.scene)} "
                f"size={ready.get('width')}x{ready.get('height')}@{ready.get('fps')} "
                f"log={log_path}"
            )
            return {"process": process, "ready_path": ready_path, "log_path": log_path}
        if process.poll() is not None:
            raise RuntimeError(
                f"native render agent exited with code {process.returncode}.\n{_tail(log_path)}"
            )
        time.sleep(0.1)

    process.terminate()
    raise RuntimeError(f"native render agent did not become ready.\n{_tail(log_path)}")


def stop_native_render_agent(runtime) -> None:
    if not runtime:
        return
    process = runtime.get("process")
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)
    runtime.get("ready_path", Path()).unlink(missing_ok=True)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Register and launch the native PC-rendered HORUS spatial RGB-D map."
    )
    parser.add_argument("--no-launch-render-agent", action="store_true")
    parser.add_argument("--transport", choices=("webrtc", "ros"), default="webrtc")
    parser.add_argument(
        "--update-mode",
        choices=("static", "refresh"),
        default=None,
        help="static uses a fixed PC-rendered atlas; refresh follows the Quest viewer pose",
    )
    parser.add_argument(
        "--stream-profile",
        choices=tuple(STREAM_PROFILES),
        default="balanced",
        help="WebRTC quality/rate preset; explicit render options override the preset",
    )
    parser.add_argument("--render-width", type=int, default=None)
    parser.add_argument("--render-height", type=int, default=None)
    parser.add_argument("--render-fps", type=int, default=None)
    parser.add_argument("--render-bitrate-kbps", type=int, default=None)
    parser.add_argument("--encoder", choices=("auto", "nvenc", "x264"), default="auto")
    parser.add_argument(
        "--ice-local-address",
        default="",
        help="Deprecated compatibility option; HORUS ROS2 now owns ICE selection.",
    )
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument(
        "--scene",
        choices=("synthetic", "cow_lady", "eth3d_courtyard"),
        default="synthetic",
        help="PC-side map source. Real scenes use official ETH laser scans.",
    )
    parser.add_argument("--dataset-path", default="")
    parser.add_argument("--dataset-scale", type=float, default=1.0)
    parser.add_argument("--point-splat-radius", type=int, default=None)
    parser.add_argument(
        "--static-view",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--pose-driven",
        action="store_true",
        help="Deprecated compatibility flag; real scenes are pose-driven by default.",
    )
    parser.add_argument("--viewer-pose-topic", default=DEFAULT_VIEWER_POSE_TOPIC)
    parser.add_argument("--viewer-pose-rate-hz", type=float, default=None)
    parser.add_argument(
        "--dynamic-view-baseline",
        type=float,
        default=0.75,
        help="Deprecated compatibility option; ignored by the single-view v5 renderer.",
    )
    parser.add_argument("--render-update-fps", type=float, default=0.0)
    parser.add_argument(
        "--dynamic-vertical-fov-deg",
        type=float,
        default=105.0,
        help="server render FOV; the default guard band absorbs head motion between frames",
    )
    parser.add_argument("--pose-position-range", type=float, default=128.0)
    parser.add_argument("--depth-near-m", type=float, default=0.0)
    parser.add_argument("--depth-far-m", type=float, default=0.0)
    parser.add_argument("--workspace-scale", type=float, default=0.1)
    parser.add_argument("--reprojection-columns", type=int, default=None)
    parser.add_argument("--reprojection-rows", type=int, default=None)
    parser.add_argument("--flip-stream-y", action="store_true")
    args = parser.parse_args()
    if args.pose_driven and args.static_view:
        parser.error("--pose-driven and --static-view cannot be used together")
    if args.static_view:
        if args.update_mode not in (None, "static"):
            parser.error("--static-view conflicts with --update-mode refresh")
        args.update_mode = "static"
    if args.pose_driven:
        if args.update_mode not in (None, "refresh"):
            parser.error("--pose-driven conflicts with --update-mode static")
        args.update_mode = "refresh"
    if args.update_mode is None:
        args.update_mode = "static" if args.scene == "synthetic" else "refresh"
    if args.scene == "synthetic" and args.update_mode == "refresh":
        parser.error("the synthetic fixture currently supports --update-mode static only")
    args.dynamic_view = args.update_mode == "refresh"

    profile_width, profile_height, profile_fps, profile_bitrate, profile_columns, profile_rows = (
        STREAM_PROFILES[args.stream_profile][args.update_mode]
    )
    if args.transport == "ros":
        profile_fps = 10 if args.dynamic_view else 5
    if args.render_width is None:
        args.render_width = profile_width
    if args.render_height is None:
        args.render_height = profile_height
    if args.render_bitrate_kbps is None:
        args.render_bitrate_kbps = profile_bitrate
    if args.point_splat_radius is None:
        args.point_splat_radius = 1 if args.scene == "eth3d_courtyard" else 2
    if args.render_fps is None:
        args.render_fps = profile_fps
    if args.viewer_pose_rate_hz is None:
        args.viewer_pose_rate_hz = float(args.render_fps)
    if args.reprojection_columns is None:
        args.reprojection_columns = profile_columns
    if args.reprojection_rows is None:
        args.reprojection_rows = profile_rows
    if not 320 <= args.render_width <= 1920 or not 180 <= args.render_height <= 1080:
        parser.error("render dimensions must be within 320x180 and 1920x1080")
    if args.render_width % 2 or args.render_height % 2:
        parser.error("render width and height must be even for H.264 RGB-D packing")
    if args.dynamic_view and args.render_width % 4:
        parser.error("refresh render width must be divisible by four for 16-bit depth packing")
    if args.scene != "synthetic" and not args.dynamic_view and (
        args.render_width % 3 or args.render_height % 3
    ):
        parser.error("complete static render dimensions must be divisible by three")
    if not 1 <= args.render_fps <= 60:
        parser.error("--render-fps must be between 1 and 60")
    if args.transport == "ros" and args.render_fps > 15:
        parser.error("the ROS debug transport is capped at 15 FPS")
    if not 80 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be between 80 and 100")
    if args.render_bitrate_kbps < 1000:
        parser.error("--render-bitrate-kbps must be at least 1000")
    if not 0.001 <= args.workspace_scale <= 10.0:
        parser.error("--workspace-scale must be between 0.001 and 10")
    if not 32 <= args.reprojection_columns <= 320 or not 18 <= args.reprojection_rows <= 180:
        parser.error("reprojection grid is outside the supported range")
    if not 0.1 <= args.dataset_scale <= 10.0:
        parser.error("--dataset-scale must be between 0.1 and 10")
    if not 0 <= args.point_splat_radius <= 4:
        parser.error("--point-splat-radius must be between 0 and 4")
    if not args.viewer_pose_topic.startswith("/"):
        parser.error("--viewer-pose-topic must be an absolute ROS topic")
    if not 1.0 <= args.viewer_pose_rate_hz <= 120.0:
        parser.error("--viewer-pose-rate-hz must be between 1 and 120")
    if args.render_update_fps <= 0.0:
        args.render_update_fps = min(
            args.render_fps,
            30 if args.scene == "eth3d_courtyard" else 60,
        )
    if not 1.0 <= args.render_update_fps <= 60.0:
        parser.error("--render-update-fps must be between 1 and 60")
    if not 0.0 <= args.dynamic_view_baseline <= 5.0:
        parser.error("--dynamic-view-baseline must be between 0 and 5 metres")
    if not 40.0 <= args.dynamic_vertical_fov_deg <= 140.0:
        parser.error("--dynamic-vertical-fov-deg must be between 40 and 140")
    if not 8.0 <= args.pose_position_range <= 2048.0:
        parser.error("--pose-position-range must be between 8 and 2048 metres")
    if args.depth_near_m < 0.0 or args.depth_far_m < 0.0:
        parser.error("depth overrides cannot be negative")
    if (
        args.depth_far_m > 0.0
        and args.depth_near_m > 0.0
        and args.depth_far_m <= args.depth_near_m
    ):
        parser.error("--depth-far-m must be greater than --depth-near-m")
    return args


def main() -> int:
    args = parse_args()
    if args.transport == "webrtc":
        args.encoder = resolve_render_encoder(
            args.encoder,
            width=args.render_width,
            height=args.render_height,
            bitrate_kbps=args.render_bitrate_kbps,
            framerate=args.render_fps,
        )
    runtime = start_native_render_agent(args)
    atexit.register(stop_native_render_agent, runtime)
    try:
        registry = get_robot_registry_client()
        _signal_monitor_runtime = start_signal_monitor(registry, args.transport)
        resource, dataviz = build_workspace_visualization(args)
        success, result = register_entities(
            [resource],
            datavizs=[dataviz],
            workspace_scale=args.workspace_scale,
            compass_enabled=False,
            keep_alive=True,
        )
        if not success:
            if is_registration_cancelled(result):
                print("HORUS registration monitor stopped.")
                return 0
            raise RuntimeError(f"HORUS registration failed: {result}")
        return 0
    finally:
        stop_native_render_agent(runtime)


if __name__ == "__main__":
    raise SystemExit(main())
