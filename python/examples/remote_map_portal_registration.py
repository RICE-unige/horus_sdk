#!/usr/bin/env python3
"""Register and run the pose-adaptive HORUS remote map renderer.

The PC renders the dense scene from the Quest's predicted pose. HORUS ROS2
encodes color with WebRTC and sends exact depth/pose metadata on the paired
data channel. The Quest only reprojects the latest synchronized frame.
"""

from __future__ import annotations

import argparse
import atexit
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

from horus.robot import WorkspaceVisualization, is_registration_cancelled, register_entities
from horus.remote_rendering import (
    ADVANCED_REMOTE_SCENE_IDS,
    DEFAULT_VIEWER_POSE_TOPIC,
    ETH3D_REMOTE_SCENE_IDS,
    REMOTE_FRAME_DATA_TOPIC,
    REMOTE_FRAME_FORMAT_VERSION,
    ROS_DEBUG_FRAME_FORMAT_VERSION,
    ROS_DEBUG_FRAME_TOPIC,
)


STREAM_TOPIC = "/horus/remote_render/map_portal"
CLIENT_SIGNAL_TOPIC = "/horus/webrtc/client_signal"
SERVER_SIGNAL_TOPIC = "/horus/webrtc/server_signal"
AGENT_STATUS_TOPIC = "/horus/remote_render/agent_status"
AGENT_SCRIPT = Path(__file__).resolve().parent / "tools" / "remote_map_render_agent.py"
CACHE_ROOT = Path.home() / ".cache" / "horus" / "remote_map"

# Render buffers are near-square on purpose.
#
# The agent renders the headset's own frustum, expanded by the guard band. For
# a Quest 3 that frustum is essentially square (measured aspect 1.001), so a
# 16:9 buffer spends its pixels on horizontal angle that does not exist and
# starves the vertical: 1280x720 delivered 10.9 px/deg horizontally but only
# 6.1 vertically, against a display that resolves about 14 px/deg. Matching the
# buffer to the frustum redistributes the same bytes evenly.
#
# Sizes are bounded by the measured raw-frame ceiling between the render agent
# and the bridge: 5.5 MB/frame sustains 59.8 Hz, 12.4 MB/frame collapses to
# 35 Hz. Because both eyes now share one view, a frame costs
# width * height * 3 bytes rather than twice that, so the whole budget buys
# resolution instead of a duplicate copy.
#
# What matters perceptually is sharpness at the CENTRE of vision, not the flat
# average. The frustum is rendered rectilinearly across ~117 deg, where tan()
# grows fast toward the edges, so the periphery consumes pixels out of all
# proportion to the solid angle it covers. Centre sharpness over the central
# +/-20 deg is roughly 0.0182 * (buffer_half_width / tan(fov/2)) px/deg:
#
#     old 1280x720 side-by-side : 7.10 px/deg H, 4.00 px/deg V
#     960x960 side-by-side      : 5.33 px/deg H, 5.33 px/deg V
#     1344x1344 single view     : 7.46 px/deg H, 7.46 px/deg V
PROFILES = {
    "fast": {
        "width": 896,
        "height": 896,
        "depth_width": 448,
        "depth_height": 448,
        "fps": 60,
        "bitrate_kbps": 20_000,
        "ros_jpeg_quality": 85,
    },
    "balanced": {
        "width": 1152,
        "height": 1152,
        "depth_width": 384,
        "depth_height": 384,
        "fps": 60,
        "bitrate_kbps": 35_000,
        "ros_jpeg_quality": 90,
    },
    "quality": {
        "width": 1344,
        "height": 1344,
        "depth_width": 448,
        "depth_height": 448,
        "fps": 60,
        "bitrate_kbps": 50_000,
        "ros_jpeg_quality": 92,
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the synchronized PC-rendered HORUS 3D map."
    )
    parser.add_argument(
        "--scene",
        choices=(
            "synthetic",
            *ADVANCED_REMOTE_SCENE_IDS,
            "cow_lady",
            "eth3d_courtyard",
            *ETH3D_REMOTE_SCENE_IDS,
        ),
        default="delivery_area",
    )
    parser.add_argument("--profile", choices=tuple(PROFILES), default="balanced")
    parser.add_argument(
        "--transport",
        choices=("ros", "ros_debug", "webrtc"),
        default="ros",
        help=(
            "Use the validated ROS-compressed path by default. "
            "ros_debug is retained as a compatibility alias; WebRTC is experimental."
        ),
    )
    parser.add_argument("--dataset-path", default="")
    parser.add_argument("--dataset-scale", type=float, default=1.0)
    parser.add_argument("--workspace-scale", type=float, default=0.1)
    parser.add_argument("--point-splat-radius", type=int, default=2)
    parser.add_argument(
        "--source-voxel-size",
        type=float,
        default=0.0125,
        help=(
            "PC-side render LOD voxel size in scene metres. The full source "
            "dataset is retained."
        ),
    )
    parser.add_argument("--render-update-fps", type=float, default=60.0)
    parser.add_argument("--guard-band-degrees", type=float, default=6.0)
    parser.add_argument("--minimum-vertical-fov-deg", type=float, default=90.0)
    parser.add_argument("--encoder", choices=("auto", "nvenc", "x264"), default="nvenc")
    parser.add_argument("--no-launch-render-agent", action="store_true")
    args = parser.parse_args()
    if not 0.001 <= args.workspace_scale <= 10.0:
        parser.error("--workspace-scale must be between 0.001 and 10")
    if not 0.1 <= args.dataset_scale <= 10.0:
        parser.error("--dataset-scale must be between 0.1 and 10")
    if not 0 <= args.point_splat_radius <= 4:
        parser.error("--point-splat-radius must be between 0 and 4")
    if not 0.001 <= args.source_voxel_size <= 0.25:
        parser.error("--source-voxel-size must be between 0.001 and 0.25 metres")
    if not 1.0 <= args.render_update_fps <= 60.0:
        parser.error("--render-update-fps must be between 1 and 60")
    if not 0.0 <= args.guard_band_degrees <= 25.0:
        parser.error("--guard-band-degrees must be between 0 and 25")
    return args


def build_workspace_visualization(args: argparse.Namespace):
    profile = PROFILES[args.profile]
    uses_ros_transport = args.transport in {"ros", "ros_debug"}
    resource = WorkspaceVisualization("workspace_remote_map")
    dataviz = resource.create_dataviz("workspace_remote_map")
    dataviz.add_remote_rendered_map(
        stream_topic=STREAM_TOPIC,
        frame_id="map",
        render_options={
            "transport": "ros_compressed" if uses_ros_transport else "webrtc",
            "format_version": (
                ROS_DEBUG_FRAME_FORMAT_VERSION
                if uses_ros_transport
                else REMOTE_FRAME_FORMAT_VERSION
            ),
            "ros_compressed_topic": ROS_DEBUG_FRAME_TOPIC,
            "frame_data_topic": REMOTE_FRAME_DATA_TOPIC,
            "viewer_pose_topic": DEFAULT_VIEWER_POSE_TOPIC,
            "viewer_pose_rate_hz": 60.0,
            "client_signal_topic": CLIENT_SIGNAL_TOPIC,
            "server_signal_topic": SERVER_SIGNAL_TOPIC,
            "status_topic": AGENT_STATUS_TOPIC,
            "encoder": args.encoder,
            "bitrate_kbps": profile["bitrate_kbps"],
            "framerate": (
                min(profile["fps"], 15)
                if uses_ros_transport
                else profile["fps"]
            ),
        },
    )
    return resource, dataviz


def _tail(path: Path, lines: int = 40) -> str:
    try:
        return "\n".join(
            path.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:]
        )
    except OSError:
        return ""


def start_render_agent(args: argparse.Namespace):
    if args.no_launch_render_agent:
        return None
    if not AGENT_SCRIPT.exists():
        raise RuntimeError(f"remote render agent is missing: {AGENT_SCRIPT}")

    profile = PROFILES[args.profile]
    uses_ros_transport = args.transport in {"ros", "ros_debug"}
    width = profile["width"]
    height = profile["height"]
    fps = (
        min(profile["fps"], 15)
        if uses_ros_transport
        else profile["fps"]
    )
    CACHE_ROOT.mkdir(parents=True, exist_ok=True)
    ready_path = CACHE_ROOT / f"ready-{uuid.uuid4().hex}.json"
    log_path = CACHE_ROOT / "agent.log"
    preview_path = CACHE_ROOT / "preview.png"
    command = [
        sys.executable,
        str(AGENT_SCRIPT),
        "--width",
        str(width),
        "--height",
        str(height),
        "--depth-width",
        str(profile["depth_width"]),
        "--depth-height",
        str(profile["depth_height"]),
        "--fps",
        str(fps),
        "--ros-jpeg-quality",
        str(profile["ros_jpeg_quality"]),
        "--transport",
        "ros_compressed" if uses_ros_transport else "webrtc",
        "--scene",
        args.scene,
        "--dataset-scale",
        str(args.dataset_scale),
        "--point-splat-radius",
        str(args.point_splat_radius),
        "--source-voxel-size",
        str(args.source_voxel_size),
        "--render-update-fps",
        str(args.render_update_fps),
        "--minimum-vertical-fov-deg",
        str(args.minimum_vertical_fov_deg),
        "--guard-band-degrees",
        str(args.guard_band_degrees),
        "--viewer-pose-topic",
        DEFAULT_VIEWER_POSE_TOPIC,
        "--ready-file",
        str(ready_path),
        "--preview-path",
        str(preview_path),
    ]
    if args.dataset_path:
        command.extend(("--dataset-path", args.dataset_path))

    environment = os.environ.copy()
    python_root = str(Path(__file__).resolve().parents[1])
    current_pythonpath = environment.get("PYTHONPATH", "")
    environment["PYTHONPATH"] = (
        python_root + (os.pathsep + current_pythonpath if current_pythonpath else "")
    )
    with log_path.open("w", encoding="utf-8") as log_file:
        process = subprocess.Popen(
            command,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            env=environment,
        )

    timeout = 30.0 if args.scene == "synthetic" else 600.0
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if ready_path.exists():
            ready = json.loads(ready_path.read_text(encoding="utf-8"))
            print(
                "[remote-map] renderer ready "
                f"pid={process.pid} scene={ready['scene']} "
                f"color={ready['color_width']}x{ready['color_height']} "
                f"depth={ready['depth_width']}x{ready['depth_height']} "
                f"fps={ready['fps']} transport={ready['transport']} "
                f"preview={preview_path} log={log_path}"
            )
            return process, ready_path
        if process.poll() is not None:
            raise RuntimeError(
                f"remote renderer exited with code {process.returncode}.\n"
                f"{_tail(log_path)}"
            )
        time.sleep(0.1)
    process.terminate()
    raise RuntimeError(f"remote renderer did not become ready.\n{_tail(log_path)}")


def stop_render_agent(runtime) -> None:
    if runtime is None:
        return
    process, ready_path = runtime
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)
    ready_path.unlink(missing_ok=True)


def main() -> int:
    args = parse_args()
    runtime = start_render_agent(args)
    atexit.register(stop_render_agent, runtime)
    try:
        resource, dataviz = build_workspace_visualization(args)
        success, result = register_entities(
            [resource],
            datavizs=[dataviz],
            workspace_scale=args.workspace_scale,
            compass_enabled=False,
            keep_alive=True,
        )
        if success:
            return 0
        if is_registration_cancelled(result):
            print("HORUS registration monitor stopped.")
            return 0
        raise RuntimeError(f"HORUS registration failed: {result}")
    finally:
        stop_render_agent(runtime)


if __name__ == "__main__":
    raise SystemExit(main())
