#!/usr/bin/env python3
"""Register and run the pose-adaptive HORUS remote map renderer.

The PC renders the complete dense scene from the Quest's predicted stereo
views. HORUS ROS2 hardware-encodes a fixed-size stereo carrier with WebRTC;
the Quest decodes and composites that carrier without receiving or rebuilding
the source mesh, point cloud, or Gaussian scene.
"""

from __future__ import annotations

import argparse
import atexit
import json
import os
from pathlib import Path
import re
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
    V3_DEFAULT_ROS_DIAGNOSTIC_TOPIC,
    V3_DEFAULT_STATUS_TOPIC,
    V3_DEFAULT_VIEW_STATE_TOPIC,
    V3_PROTOCOL_VERSION,
    V3_ROS_DIAGNOSTIC_FORMAT_VERSION,
    V3_WEBRTC_FORMAT_VERSION,
)


STREAM_TOPIC = "/horus/remote_render/map_portal"
CLIENT_SIGNAL_TOPIC = "/horus/webrtc/client_signal"
SERVER_SIGNAL_TOPIC = "/horus/webrtc/server_signal"
AGENT_STATUS_TOPIC = "/horus/remote_render/agent_status"
AGENT_SCRIPT = Path(__file__).resolve().parent / "tools" / "remote_map_render_agent.py"
CACHE_ROOT = Path.home() / ".cache" / "horus" / "remote_map"
_DDS_INTERFACE_PATTERN = re.compile(r"^[A-Za-z0-9_.:-]+$")

# Render buffers are near-square on purpose.
#
# The agent renders the headset's own frustum, expanded by the guard band. For
# a Quest 3 that frustum is essentially square (measured aspect 1.001), so a
# 16:9 buffer spends its pixels on horizontal angle that does not exist and
# starves the vertical: 1280x720 delivered 10.9 px/deg horizontally but only
# 6.1 vertically, against a display that resolves about 14 px/deg. Matching the
# buffer to the frustum redistributes the same bytes evenly.
#
# V3 sends independent left/right eye images. The map geometry stays entirely
# on the PC; these dimensions only control the fixed-size stereo video carrier
# decoded by the headset. They never decimate the source mesh, points, or
# Gaussian splats.
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
        "visibility_mask_height": 224,
        "fps": 60,
        "ros_fps": 15,
        "bitrate_kbps": 20_000,
        "ros_jpeg_quality": 85,
    },
    "balanced": {
        "width": 1152,
        "height": 1152,
        "depth_width": 384,
        "depth_height": 384,
        "visibility_mask_height": 288,
        "fps": 60,
        "ros_fps": 15,
        "bitrate_kbps": 35_000,
        "ros_jpeg_quality": 90,
    },
    "quality": {
        "width": 1344,
        "height": 1344,
        "depth_width": 448,
        "depth_height": 448,
        "visibility_mask_height": 336,
        "fps": 60,
        "ros_fps": 15,
        "bitrate_kbps": 50_000,
        "ros_jpeg_quality": 92,
    },
}


def uses_ros_transport(transport: str) -> bool:
    return transport in {"ros", "ros_debug"}


def stream_fps(profile: dict, transport: str) -> int:
    return profile["ros_fps"] if uses_ros_transport(transport) else profile["fps"]


def render_update_fps(
    profile: dict,
    transport: str,
    requested_fps: float,
) -> float:
    # Never rasterize work that the selected carrier cannot deliver.
    return min(requested_fps, stream_fps(profile, transport))


def _default_route_interface() -> str | None:
    try:
        completed = subprocess.run(
            ("ip", "-j", "route", "show", "default"),
            check=True,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
        routes = json.loads(completed.stdout)
    except (FileNotFoundError, subprocess.SubprocessError, json.JSONDecodeError):
        return None
    candidates = [
        route for route in routes
        if isinstance(route, dict) and route.get("dev")
    ]
    if not candidates:
        return None
    route = min(candidates, key=lambda item: int(item.get("metric", 0)))
    interface = str(route["dev"]).strip()
    return interface if _DDS_INTERFACE_PATTERN.fullmatch(interface) else None


def _fastdds_default_route_profile(interface: str) -> str:
    if not _DDS_INTERFACE_PATTERN.fullmatch(interface):
        raise ValueError("invalid Fast DDS interface name")
    return f'''<?xml version="1.0" encoding="UTF-8" ?>
<dds>
  <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>horus_default_udp</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList><interface>{interface}</interface></interfaceWhiteList>
        <TTL>1</TTL>
      </transport_descriptor>
      <transport_descriptor>
        <transport_id>horus_local_shm</transport_id>
        <type>SHM</type>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="horus_default_route" is_default_profile="true">
      <rtps>
        <useBuiltinTransports>false</useBuiltinTransports>
        <userTransports>
          <transport_id>horus_default_udp</transport_id>
          <transport_id>horus_local_shm</transport_id>
        </userTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
'''


def _configure_v3_fastdds_default_route(
    profile_root: Path | None = None,
) -> Path | None:
    if (
        os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE")
        or os.environ.get("FASTDDS_DEFAULT_PROFILES_FILE")
    ):
        return None
    rmw = os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    if "fastrtps" not in rmw:
        return None
    # Local ROS traffic must retain loopback discovery. The Quest connects to
    # the TCP/WebRTC bridge; it does not need to be a DDS participant.
    local_only = os.environ.get("ROS_LOCALHOST_ONLY", "").strip().lower() in {"1", "true"}
    local_only |= os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "").upper() == "LOCALHOST"
    interface = "lo" if local_only else _default_route_interface()
    if interface is None:
        return None
    root = profile_root or CACHE_ROOT
    root.mkdir(parents=True, exist_ok=True)
    path = root / f"fastdds_default_route_{interface}.xml"
    content = _fastdds_default_route_profile(interface)
    if not path.exists() or path.read_text(encoding="utf-8") != content:
        path.write_text(content, encoding="utf-8")
    value = str(path)
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = value
    os.environ["FASTDDS_DEFAULT_PROFILES_FILE"] = value
    print(f"[remote-map] Fast DDS interface: {interface} ({path})")
    return path


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the synchronized PC-rendered HORUS 3D map."
    )
    parser.add_argument(
        "--scene",
        choices=(
            "synthetic",
            *ADVANCED_REMOTE_SCENE_IDS,
            "cow_lady",
            "cow_lady_points",
            "recorded_mesh",
            "recorded_points",
            "eth3d_courtyard",
            *ETH3D_REMOTE_SCENE_IDS,
        ),
        default="delivery_area",
    )
    parser.add_argument("--profile", choices=tuple(PROFILES), default="balanced")
    parser.add_argument(
        "--protocol",
        choices=("v2", "v3"),
        default="v3",
        help="Use the exact per-eye V3 path; V2 is retained only for rollback.",
    )
    parser.add_argument(
        "--transport",
        choices=("ros", "ros_debug", "webrtc"),
        default="webrtc",
        help=(
            "Use hardware-decoded WebRTC by default. ROS is a low-rate "
            "diagnostic path and ros_debug is its compatibility alias."
        ),
    )
    parser.add_argument("--dataset-path", default="")
    parser.add_argument(
        "--session-id",
        default="",
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--dataset-scale", type=float, default=1.0)
    parser.add_argument("--workspace-scale", type=float, default=0.1)
    parser.add_argument("--point-splat-radius", type=int, default=2)
    parser.add_argument(
        "--source-voxel-size",
        type=float,
        default=0.0,
        help=(
            "Optional PC-side point-cloud LOD voxel size in scene metres. "
            "Zero preserves every source point and is the default."
        ),
    )
    parser.add_argument("--render-update-fps", type=float, default=60.0)
    parser.add_argument(
        "--maximum-frame-age-ms",
        type=int,
        default=250,
        help=(
            "Reject a decoded frame when its source view is older than this "
            "bounded deadline. This does not queue or reduce source geometry."
        ),
    )
    parser.add_argument("--guard-band-degrees", type=float, default=6.0)
    parser.add_argument("--minimum-vertical-fov-deg", type=float, default=90.0)
    parser.add_argument("--encoder", choices=("auto", "nvenc", "x264"), default="nvenc")
    parser.add_argument("--no-launch-render-agent", action="store_true")
    args = parser.parse_args(argv)
    if not args.session_id:
        args.session_id = uuid.uuid4().hex
    if not 0.001 <= args.workspace_scale <= 10.0:
        parser.error("--workspace-scale must be between 0.001 and 10")
    if not 0.1 <= args.dataset_scale <= 10.0:
        parser.error("--dataset-scale must be between 0.1 and 10")
    if not 0 <= args.point_splat_radius <= 4:
        parser.error("--point-splat-radius must be between 0 and 4")
    if not 0.0 <= args.source_voxel_size <= 0.25:
        parser.error("--source-voxel-size must be zero or between 0.001 and 0.25 metres")
    if 0.0 < args.source_voxel_size < 0.001:
        parser.error("--source-voxel-size must be zero or between 0.001 and 0.25 metres")
    if not 1.0 <= args.render_update_fps <= 60.0:
        parser.error("--render-update-fps must be between 1 and 60")
    if not 50 <= args.maximum_frame_age_ms <= 1000:
        parser.error("--maximum-frame-age-ms must be between 50 and 1000")
    if not 0.0 <= args.guard_band_degrees <= 25.0:
        parser.error("--guard-band-degrees must be between 0 and 25")
    return args


def build_workspace_visualization(args: argparse.Namespace):
    profile = PROFILES[args.profile]
    uses_ros = uses_ros_transport(args.transport)
    uses_v3 = args.protocol == "v3"
    resource = WorkspaceVisualization("workspace_remote_map")
    dataviz = resource.create_dataviz("workspace_remote_map")
    dataviz.add_remote_rendered_map(
        stream_topic=STREAM_TOPIC,
        frame_id="map",
        render_options={
            "protocol_version": (
                V3_PROTOCOL_VERSION if uses_v3 else "remote_stereo_rgbd_v2"
            ),
            "session_id": args.session_id,
            "reference_space_id": "workspace-local",
            "workspace_revision": 0,
            "transport": "ros_compressed" if uses_ros else "webrtc",
            "format_version": (
                (
                    V3_ROS_DIAGNOSTIC_FORMAT_VERSION
                    if uses_ros
                    else V3_WEBRTC_FORMAT_VERSION
                )
                if uses_v3
                else (
                    ROS_DEBUG_FRAME_FORMAT_VERSION
                    if uses_ros
                    else REMOTE_FRAME_FORMAT_VERSION
                )
            ),
            "ros_compressed_topic": (
                V3_DEFAULT_ROS_DIAGNOSTIC_TOPIC
                if uses_v3
                else ROS_DEBUG_FRAME_TOPIC
            ),
            "frame_data_topic": (
                "" if uses_v3 and not uses_ros else REMOTE_FRAME_DATA_TOPIC
            ),
            "viewer_pose_topic": (
                V3_DEFAULT_VIEW_STATE_TOPIC
                if uses_v3
                else DEFAULT_VIEWER_POSE_TOPIC
            ),
            "viewer_pose_rate_hz": 72.0 if uses_v3 else 60.0,
            "client_signal_topic": CLIENT_SIGNAL_TOPIC,
            "server_signal_topic": SERVER_SIGNAL_TOPIC,
            "status_topic": V3_DEFAULT_STATUS_TOPIC if uses_v3 else AGENT_STATUS_TOPIC,
            "encoder": args.encoder,
            "image_type": getattr(args, "webrtc_image_type", "raw"),
            "bitrate_kbps": profile["bitrate_kbps"],
            "framerate": (
                stream_fps(profile, args.transport)
            ),
            "viewport_width": profile["width"],
            "viewport_height": profile["height"],
            "display_refresh_rate_hz": 72.0,
            "maximum_frame_age_ms": args.maximum_frame_age_ms,
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
    uses_ros = uses_ros_transport(args.transport)
    width = profile["width"]
    height = profile["height"]
    fps = stream_fps(profile, args.transport)
    effective_render_fps = render_update_fps(
        profile,
        args.transport,
        args.render_update_fps,
    )
    CACHE_ROOT.mkdir(parents=True, exist_ok=True)
    run_root = CACHE_ROOT / uuid.uuid4().hex
    run_root.mkdir()
    ready_path = run_root / "ready.json"
    log_path = run_root / "agent.log"
    preview_path = run_root / "preview.png"
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
        "--bitrate-kbps",
        str(profile["bitrate_kbps"]),
        "--protocol",
        args.protocol,
        "--session-id",
        args.session_id,
        "--ros-jpeg-quality",
        str(profile["ros_jpeg_quality"]),
        "--transport",
        "ros_compressed" if uses_ros else "webrtc",
        "--scene",
        args.scene,
        "--dataset-scale",
        str(args.dataset_scale),
        "--point-splat-radius",
        str(args.point_splat_radius),
        "--source-voxel-size",
        str(args.source_voxel_size),
        "--render-update-fps",
        str(effective_render_fps),
        "--minimum-vertical-fov-deg",
        str(args.minimum_vertical_fov_deg),
        "--guard-band-degrees",
        str(args.guard_band_degrees),
        "--visibility-mask-height",
        str(profile["visibility_mask_height"]),
        "--viewer-pose-topic",
        (
            V3_DEFAULT_VIEW_STATE_TOPIC
            if args.protocol == "v3"
            else DEFAULT_VIEWER_POSE_TOPIC
        ),
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
    print(f"[remote-map] preparing scene; log={log_path}", flush=True)
    try:
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(
                    f"remote renderer exited with code {process.returncode}.\n"
                    f"{_tail(log_path)}"
                )
            if ready_path.exists():
                ready = json.loads(ready_path.read_text(encoding="utf-8"))
                args.webrtc_image_type = str(
                    ready.get("webrtc_image_type", "raw") or "raw"
                ).strip().lower()
                print(
                    "[remote-map] renderer ready "
                    f"pid={process.pid} scene={ready['scene']} "
                    f"color={ready['color_width']}x{ready['color_height']} "
                    f"depth={ready['depth_width']}x{ready['depth_height']} "
                    f"fps={ready['fps']} transport={ready['transport']} "
                    f"source={args.webrtc_image_type} "
                    f"preview={preview_path} log={log_path}"
                )
                return process, ready_path
            time.sleep(0.1)
        raise RuntimeError(f"remote renderer did not become ready.\n{_tail(log_path)}")
    except BaseException:
        # Includes Ctrl+C before main has registered its shutdown handler.
        stop_render_agent((process, ready_path))
        raise


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


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.protocol == "v3":
        _configure_v3_fastdds_default_route()
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
