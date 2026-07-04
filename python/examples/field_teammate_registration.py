#!/usr/bin/env python3
"""Register a human field teammate (HoloLens-class wearable) in HORUS MR.

A field teammate is represented through the same registration pipeline as a
robot, but as a *guidable, non-controllable* entity: teleop, the navigation
tasks, and every robot-control capability are denied by default, and the
serializer fails closed if any of them are re-enabled.

Typical run (HORUS app + bridge up):
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/field_teammate_registration.py

Exercise the contract offline (no app, no bridge) -- prints the baked payload:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/field_teammate_registration.py --dry-run

Pair an offline pose/FPV/guidance feed (no HoloLens needed):
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/tools/mock_field_teammate_node.py
"""

from __future__ import annotations

import argparse
import json
import sys

from horus.robot import FieldTeammate, is_registration_cancelled
from horus.sensors import Camera


def build_field_teammate(args) -> FieldTeammate:
    teammate = FieldTeammate(
        args.name,
        wearable_type=args.wearable,
    )

    if not args.no_fpv_camera:
        # First-person view: compressed frames over WebRTC, matching the FPV
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
            fps=30,
            encoding="jpeg",
            teleop_streaming_type="webrtc",
            minimap_streaming_type="webrtc",
            minimap_image_type="compressed",
            teleop_image_type="compressed",
        )
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
        "--no-keep-alive",
        action="store_true",
        help="Do not keep the registration monitor alive after registering.",
    )
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
        print("DRYRUN_OK (default-deny contract verified)")
        return 0

    success, result = teammate.register_with_horus(
        keep_alive=not args.no_keep_alive,
    )
    if not success:
        if is_registration_cancelled(result):
            print("HORUS registration monitor stopped.")
            return 0
        print(f"HORUS registration failed: {result}")
        return 1
    print(f"Field teammate '{args.name}' registered.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
