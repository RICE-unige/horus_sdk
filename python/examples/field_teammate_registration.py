#!/usr/bin/env python3
"""Register a human field teammate (HoloLens-class wearable) in HORUS MR.

A field teammate is represented through the same registration pipeline as a
robot, but as a *guidable, non-controllable* entity: teleop, the navigation
tasks, and every robot-control capability are denied by default, and the
serializer fails closed if any of them are re-enabled.

One-command mock demo (HORUS app + bridge up):
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/field_teammate_registration.py --mock-feed

Exercise the contract offline (no app, no bridge) -- prints the baked payload:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/field_teammate_registration.py --dry-run

    Manual mock feed, if you prefer two terminals:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/tools/mock_field_teammate_node.py

Live HoloLens path: the relay is owned by horus_connector (the `teammate`
role) -- the SDK ships no copy. Managed launch:
    cd ~/horus_connector && ./horus setup teammate && ./horus launch teammate
Endpoint check:
    cd ~/horus_connector && python3 scripts/field_teammate_hololens_relay.py \
        --hololens-host <HOLOLENS_IP_FROM_STATUS_PANEL> --check-endpoint
`--live-hololens` below resolves that same connector relay via
HORUS_CONNECTOR_ROOT (default ~/horus_connector). It is a demo convenience for
single-terminal runs; the connector-managed `./horus launch teammate` path is
the normal deployment path and should not be run at the same time for the same
teammate.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

from horus.robot import FieldTeammate, RobotDimensions, is_registration_cancelled
from horus.sensors import Camera

from tools.field_teammate_articulated_assets import (
    FieldTeammateProfile,
    ensure_field_teammate_articulated_assets,
    ensure_field_teammate_head_hand_assets,
    ensure_field_teammate_head_assets,
)


SCRIPT_DIR = Path(__file__).resolve().parent
HUMAN_DESCRIPTION_DIR = SCRIPT_DIR / ".local_assets" / "field_teammate_description"
HUMAN_URDF_PATH = HUMAN_DESCRIPTION_DIR / "urdf" / "field_teammate_human.urdf"
DEFAULT_WORKSPACE_SCALE = 0.1
DEFAULT_MOCK_FEED_RATE_HZ = 15.0
DEFAULT_LIVE_VIDEO_PROFILE = "fast60"
DEFAULT_CAMERA_HEIGHT_RATIO = 0.92
DEFAULT_PROJECTED_IMAGE_SCALE = 0.09
DEFAULT_PROJECTED_FOCAL_LENGTH_SCALE = 0.045
DEFAULT_PROJECTED_FORWARD_OFFSET = 0.035


def build_field_teammate(args) -> FieldTeammate:
    profile = FieldTeammateProfile(height_m=args.profile_height, sex=args.profile_sex)
    human_height = profile.normalized_height_m
    human_width = 0.42 if profile.normalized_sex == "male" else 0.38 if profile.normalized_sex == "female" else 0.40
    dimensions = RobotDimensions(length=0.34, width=human_width, height=human_height)
    teammate = FieldTeammate(
        args.name,
        wearable_type=args.wearable,
        dimensions=dimensions,
        profile_height_m=human_height,
        profile_sex=profile.normalized_sex,
        body_model=args.human_model_source,
    )

    if not args.no_human_model:
        if args.human_model_source == "meta_avatar":
            # Meta Avatars are an MR-side runtime concern. Do not send an SDK
            # body mesh for this mode; the registration still carries the
            # human profile, TF, FPV camera, status, confidence, and guidance
            # topics needed to bind a Meta avatar once the Avatar SDK package is
            # installed in HORUS MR.
            pass
        elif args.human_model_source == "head_hands":
            bundle = ensure_field_teammate_head_hand_assets(profile)
            teammate.configure_robot_description(
                source="local",
                urdf_path=str(bundle.urdf_path),
                base_frame="camera",
                mesh_root=str(bundle.mesh_root),
                include_visual_meshes=True,
                visual_mesh_triangle_budget=30000,
                body_mesh_mode="runtime_high_mesh",
                visual_link_pose_source="tf",
                chunk_size_bytes=64000,
            )
        elif args.human_model_source == "head_only":
            bundle = ensure_field_teammate_head_assets(profile)
            teammate.configure_robot_description(
                source="local",
                urdf_path=str(bundle.urdf_path),
                base_frame="camera",
                mesh_root=str(bundle.mesh_root),
                include_visual_meshes=True,
                visual_mesh_triangle_budget=12000,
                body_mesh_mode="runtime_high_mesh",
                visual_link_pose_source="static",
                chunk_size_bytes=64000,
            )
        elif args.human_model_source == "makehuman_static":
            teammate.configure_robot_description(
                source="local",
                urdf_path=str(HUMAN_URDF_PATH),
                base_frame="base",
                mesh_root=str(HUMAN_DESCRIPTION_DIR),
                include_visual_meshes=True,
                visual_mesh_triangle_budget=16000,
                body_mesh_mode="runtime_high_mesh",
                chunk_size_bytes=64000,
            )
        else:
            bundle = ensure_field_teammate_articulated_assets(profile)
            teammate.configure_robot_description(
                source="local",
                urdf_path=str(bundle.urdf_path),
                base_frame="base",
                mesh_root=str(bundle.mesh_root),
                include_visual_meshes=True,
                visual_mesh_triangle_budget=80000,
                body_mesh_mode="runtime_high_mesh",
                visual_link_pose_source="static",
                chunk_size_bytes=64000,
            )

    if not args.no_fpv_camera:
        # First-person view: compressed ROS frames matching the FPV
        # contract a HoloLens companion app publishes.
        fpv_config = teammate.get_field_teammate_config() or {}
        fpv_topic = (fpv_config.get("topics") or {}).get(
            "first_person_video", f"/{args.name}/fpv/image_raw/compressed"
        )
        camera = Camera(
            name="fpv_camera",
            frame_id=fpv_config.get("camera_frame", f"{args.name}/camera"),
            topic=fpv_topic,
            resolution=(640, 360),
            fps=15,
            encoding="jpeg",
            streaming_type="ros",
            minimap_streaming_type="ros",
            teleop_streaming_type="ros",
            minimap_topic=fpv_topic,
            teleop_topic=fpv_topic,
            minimap_image_type="compressed",
            teleop_image_type="compressed",
            minimap_max_fps=15,
        )
        camera.add_metadata("image_type", "compressed")
        # The live/mock feeds publish this camera frame at eye height. Keep the
        # projected panel close to that frame; height belongs in TF, not in a
        # large projected-view offset from the floor-level base frame.
        camera.configure_projected_view(
            position_offset=(0.0, 0.0, DEFAULT_PROJECTED_FORWARD_OFFSET),
            rotation_offset=(0.0, 0.0, float(args.camera_roll_deg)),
            image_scale=DEFAULT_PROJECTED_IMAGE_SCALE,
            focal_length_scale=DEFAULT_PROJECTED_FOCAL_LENGTH_SCALE,
            show_frustum=True,
            frustum_color="#66D9FFA0",
        )
        camera.configure_minimap_view(
            size=0.55,
            position_offset=(0.0, 0.48, 0.0),
            face_camera=True,
        )
        camera.configure_immersive_view(ros_flip_x=False, ros_flip_y=False)
        teammate.add_sensor(camera)

    return teammate


def _build_offline_payload(teammate: FieldTeammate) -> dict:
    """Build the registration payload without a live bridge (offline validation).

    Mirrors the registration serializer's inputs without opening a ROS/bridge
    connection, so the capability contract can be inspected before going
    on-device.
    """
    from horus.bridge.robot_registry import RobotRegistryClient

    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    client._robot_description_resolver = None
    client._robot_description_by_robot = {}
    client._robot_description_by_id = {}

    dataviz = teammate.create_dataviz()
    payload = client._build_robot_config_dict(teammate, dataviz)
    payload.pop("timestamp", None)
    return payload


def _start_mock_feed(args) -> subprocess.Popen:
    """Start the local mock HoloLens feed for the one-command demo path."""
    env = os.environ.copy()
    python_path = str(SCRIPT_DIR.parent)
    current_pythonpath = env.get("PYTHONPATH", "")
    if current_pythonpath:
        env["PYTHONPATH"] = f"{python_path}{os.pathsep}{current_pythonpath}"
    else:
        env["PYTHONPATH"] = python_path

    cmd = [
        sys.executable,
        str(SCRIPT_DIR / "tools" / "mock_field_teammate_node.py"),
        "--name",
        args.name,
        "--wearable",
        args.wearable,
        "--rate",
        str(float(args.mock_feed_rate)),
        "--profile-height",
        str(float(args.profile_height)),
        "--profile-sex",
        str(args.profile_sex),
        "--fpv-source",
        str(args.mock_fpv_source),
    ]
    if args.mock_fpv_video:
        cmd.extend(["--fpv-video", str(args.mock_fpv_video)])
    process = subprocess.Popen(cmd, env=env)
    time.sleep(0.75)
    if process.poll() is not None:
        raise RuntimeError(
            "Mock field-teammate feed exited before registration. "
            "Make sure ROS 2 Jazzy is sourced in this terminal."
        )
    print(
        f"Started mock field teammate feed for '{args.name}' "
        f"({float(args.mock_feed_rate):.1f} Hz)."
    )
    return process


def _resolve_connector_relay() -> Path:
    """Locate the canonical HoloLens relay in the horus_connector checkout.

    The live relay is owned by horus_connector (the `teammate` role); the SDK
    deliberately ships no copy. Set HORUS_CONNECTOR_ROOT when the checkout is
    not at ~/horus_connector.
    """
    root = Path(os.environ.get("HORUS_CONNECTOR_ROOT", "~/horus_connector")).expanduser()
    relay = root / "scripts" / "field_teammate_hololens_relay.py"
    if not relay.is_file():
        raise RuntimeError(
            f"horus_connector relay not found at {relay}. Clone "
            "RICE-unige/horus_connector (or set HORUS_CONNECTOR_ROOT), or run the "
            "relay yourself with './horus launch teammate' from the connector and "
            "rerun this registration without --live-hololens."
        )
    return relay


def _start_live_hololens_relay(args) -> subprocess.Popen:
    """Start the connector relay as a demo convenience path."""
    if not args.hololens_host:
        raise ValueError("--hololens-host is required with --live-hololens")

    relay = _resolve_connector_relay()
    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

    cmd = [
        sys.executable,
        str(relay),
        "--name",
        args.name,
        "--hololens-host",
        args.hololens_host,
        "--video-profile",
        args.video_profile,
    ]
    if args.no_raw_hololens_image:
        cmd.append("--no-raw-image")
    cmd.extend(
        [
            "--profile-height",
            str(float(args.profile_height)),
            "--camera-height",
            str(float(args.profile_height) * DEFAULT_CAMERA_HEIGHT_RATIO),
        ]
    )
    # cwd = the connector checkout so the relay resolves its relative paths
    # exactly as it does under `./horus launch teammate`.
    process = subprocess.Popen(cmd, env=env, cwd=str(relay.parent.parent))
    time.sleep(1.5)
    if process.poll() is not None:
        raise RuntimeError(
            "Live HoloLens relay exited before registration. "
            "Make sure the HoloLens app is open and ROS 2 Jazzy is sourced."
        )
    print(
        f"Started live HoloLens relay for '{args.name}' at {args.hololens_host} "
        f"using video profile '{args.video_profile}'."
    )
    return process


def _stop_mock_feed(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=3.0)


def _stop_live_hololens_relay(process: subprocess.Popen | None) -> None:
    _stop_mock_feed(process)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--name", default="field_teammate_1", help="Teammate entity name/namespace.")
    parser.add_argument(
        "--wearable",
        default="hololens2",
        help="Wearable device type (hololens2, aria, quest_pro, generic).",
    )
    parser.add_argument(
        "--no-fpv-camera",
        action="store_true",
        help="Register without the first-person-view camera sensor.",
    )
    parser.add_argument(
        "--no-human-model",
        action="store_true",
        help="Register without the human body model.",
    )
    parser.add_argument(
        "--profile-height",
        type=float,
        default=1.75,
        help="Human teammate height in meters. The demo model and profile payload are generated from this value.",
    )
    parser.add_argument(
        "--profile-sex",
        choices=("female", "male", "unspecified"),
        default="unspecified",
        help="Profile sex used for broad body-proportion selection in the demo model.",
    )
    parser.add_argument(
        "--human-model-source",
        choices=("head_hands", "head_only", "meta_avatar", "skinned_profile_v1", "makehuman_static"),
        default="head_hands",
        help=(
            "Human model source. Default sends a lightweight SDK head mesh bound to "
            "the live camera frame plus tracked hand meshes bound to HoloLens hand "
            "TF frames. Use skinned_profile_v1/makehuman_static only for local "
            "full-body mesh debugging."
        ),
    )
    parser.add_argument(
        "--camera-roll-deg",
        type=float,
        default=0.0,
        help="Projected FPV panel clockwise roll offset in degrees. This rotates the image plane only, not the camera TF yaw.",
    )
    parser.add_argument(
        "--no-keep-alive",
        action="store_true",
        help="Do not keep the registration monitor alive after registering.",
    )
    parser.add_argument(
        "--workspace-scale",
        type=float,
        default=DEFAULT_WORKSPACE_SCALE,
        help=f"HORUS workspace scale. Default: {DEFAULT_WORKSPACE_SCALE}.",
    )
    parser.add_argument(
        "--mock-feed",
        action="store_true",
        help=(
            "Start the local mock HoloLens pose/status/FPV feed before registration. "
            "Use this for the one-command offline demo; do not use it with a real "
            "field-teammate publisher."
        ),
    )
    parser.add_argument(
        "--mock-feed-rate",
        type=float,
        default=DEFAULT_MOCK_FEED_RATE_HZ,
        help=f"Mock feed publish rate in Hz when --mock-feed is enabled. Default: {DEFAULT_MOCK_FEED_RATE_HZ}.",
    )
    parser.add_argument(
        "--mock-fpv-source",
        choices=("real", "synthetic", "auto"),
        default="real",
        help="Mock FPV source. Default downloads/caches a real CC-BY walking video and streams its frames.",
    )
    parser.add_argument(
        "--mock-fpv-video",
        default="",
        help="Optional local video path or URL for the mock FPV stream.",
    )
    parser.add_argument(
        "--live-hololens",
        action="store_true",
        help="Start the live HoloLens relay before registering the field teammate.",
    )
    parser.add_argument(
        "--hololens-host",
        default="",
        help="HoloLens IPv4/hostname shown in the HORUS Field status panel. Required with --live-hololens.",
    )
    parser.add_argument(
        "--video-profile",
        choices=("app", "balanced", "fast60", "hd720"),
        default=DEFAULT_LIVE_VIDEO_PROFILE,
        help=f"Live HoloLens video profile. Default: {DEFAULT_LIVE_VIDEO_PROFILE}.",
    )
    raw_image_group = parser.add_mutually_exclusive_group()
    raw_image_group.add_argument(
        "--raw-hololens-image",
        dest="no_raw_hololens_image",
        action="store_false",
        help="Also publish the decoded RGB helper topic. This costs CPU and can reduce live FPV framerate.",
    )
    raw_image_group.add_argument(
        "--no-raw-hololens-image",
        dest="no_raw_hololens_image",
        action="store_true",
        help="Publish only compressed FPV frames; skip local decoded RGB helper topic.",
    )
    parser.set_defaults(no_raw_hololens_image=True)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Build and print the baked payload offline; no HORUS connection.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    teammate = build_field_teammate(args)

    if args.dry_run:
        payload = _build_offline_payload(teammate)
        print(json.dumps(payload, indent=2))
        caps = payload["capabilities"]
        assert payload["entity_kind"] == "field_teammate"
        assert not caps["controllable"] and not caps["teleoperable"] and not caps["taskable"]
        assert payload["control"]["teleop"]["enabled"] is False
        if not args.no_human_model and args.human_model_source != "meta_avatar":
            manifest = payload.get("robot_description_manifest") or {}
            assert manifest.get("supports_visual_meshes") is True
            assert int(manifest.get("mesh_asset_count") or 0) >= 1
        print("DRYRUN_OK (default-deny contract verified)")
        return 0

    mock_feed = None
    live_relay = None
    try:
        if args.mock_feed and args.live_hololens:
            raise ValueError("Use either --mock-feed or --live-hololens, not both.")
        if args.mock_feed:
            mock_feed = _start_mock_feed(args)
        if args.live_hololens:
            live_relay = _start_live_hololens_relay(args)

        success, result = teammate.register_with_horus(
            keep_alive=not args.no_keep_alive,
            workspace_scale=float(args.workspace_scale),
        )
        if not success:
            if is_registration_cancelled(result):
                print("HORUS registration monitor stopped.")
                return 0
            print(f"HORUS registration failed: {result}")
            return 1
        print(f"Field teammate '{args.name}' registered.")
        return 0
    finally:
        _stop_mock_feed(mock_feed)
        _stop_live_hololens_relay(live_relay)


if __name__ == "__main__":
    sys.exit(main())
