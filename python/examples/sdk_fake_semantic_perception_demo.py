#!/usr/bin/env python3
"""Fake-data semantic perception demo using the existing ops-suite topic layout."""

import argparse
import os
import sys
from dataclasses import dataclass
from typing import List, Tuple

# Ensure we can import the local horus package regardless of CWD.
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

@dataclass(frozen=True)
class RobotProfile:
    urdf_path: str
    base_frame: str
    robot_type: RobotType
    dimensions: RobotDimensions
    teleop_profile: str


LOCAL_DESCRIPTION_DIR = os.path.join(
    SCRIPT_DIR,
    ".local_assets",
    "robot_descriptions",
)

ROBOT_PROFILES = {
    "anymal": RobotProfile(
        urdf_path=os.path.join(LOCAL_DESCRIPTION_DIR, "anymal_c.urdf"),
        base_frame="base",
        robot_type=RobotType.LEGGED,
        dimensions=RobotDimensions(length=1.0, width=0.56, height=0.78),
        teleop_profile="legged",
    ),
    "anymal_c": RobotProfile(
        urdf_path=os.path.join(LOCAL_DESCRIPTION_DIR, "anymal_c.urdf"),
        base_frame="base",
        robot_type=RobotType.LEGGED,
        dimensions=RobotDimensions(length=1.0, width=0.56, height=0.78),
        teleop_profile="legged",
    ),
    "go1": RobotProfile(
        urdf_path=os.path.join(LOCAL_DESCRIPTION_DIR, "go1.urdf"),
        base_frame="base",
        robot_type=RobotType.LEGGED,
        dimensions=RobotDimensions(length=0.65, width=0.30, height=0.40),
        teleop_profile="legged",
    ),
    "h1": RobotProfile(
        urdf_path=os.path.join(LOCAL_DESCRIPTION_DIR, "h1.urdf"),
        base_frame="pelvis",
        robot_type=RobotType.LEGGED,
        dimensions=RobotDimensions(length=0.76, width=0.42, height=1.80),
        teleop_profile="legged",
    ),
    "jackal": RobotProfile(
        urdf_path=os.path.join(LOCAL_DESCRIPTION_DIR, "jackal.urdf"),
        base_frame="base_link",
        robot_type=RobotType.WHEELED,
        dimensions=RobotDimensions(length=0.51, width=0.43, height=0.31),
        teleop_profile="wheeled",
    ),
}

DEFAULT_ROBOT_NAMES = ["anymal", "go1", "h1", "jackal"]


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


def parse_robot_names(raw_value: str) -> List[str]:
    names = [name.strip() for name in str(raw_value or "").split(",") if name.strip()]
    return names or list(DEFAULT_ROBOT_NAMES)


def build_camera(name: str, image_type: str, resolution: Tuple[int, int], fps: int) -> Camera:
    width, height = resolution
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
        fps=max(1, int(fps)),
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


def add_semantic_demo_boxes(dataviz) -> None:
    demo_boxes = [
        {
            "id": "injured_person_1",
            "label": "Injured Person",
            "center": (2.1, -1.0, 0.0),
            "size": (0.85, 0.75, 1.7),
            "rotation_offset_euler": (0.0, 0.0, 18.0),
        },
        {
            "id": "table_1",
            "label": "Table",
            "center": (5.0, 0.7, 0.0),
            "size": (1.6, 0.85, 0.78),
            "rotation_offset_euler": (0.0, 0.0, 0.0),
        },
        {
            "id": "stairs_1",
            "label": "Stairs",
            "center": (7.2, 3.5, 0.0),
            "size": (1.2, 2.3, 0.55),
            "rotation_offset_euler": (0.0, 0.0, -32.0),
        },
        {
            "id": "supply_cart_1",
            "label": "Supply Cart",
            "center": (3.9, 4.6, 0.0),
            "size": (1.05, 0.62, 1.18),
            "rotation_offset_euler": (0.0, 0.0, 46.0),
        },
    ]

    for box in demo_boxes:
        dataviz.add_semantic_box(
            semantic_id=box["id"],
            label=box["label"],
            center=box["center"],
            size=box["size"],
            frame_id="map",
            rotation_offset_euler=box["rotation_offset_euler"],
        )


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Register a fake multi-robot ops scenario plus semantic perception boxes. "
            "Pair this with fake_tf_ops_suite.py for deterministic semantic-box testing."
        )
    )
    parser.add_argument(
        "--robot-names",
        default="anymal,go1,h1,jackal",
        help=(
            "Comma-separated robot names/models (default: anymal,go1,h1,jackal). "
            "Supported: anymal, anymal_c, go1, h1, jackal."
        ),
    )
    parser.add_argument(
        "--workspace-scale",
        type=float,
        default=0.1,
        help="Global workspace position scale forwarded in registration payload (default: 0.1).",
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
    return parser


def main():
    args = build_parser().parse_args()
    robot_names = parse_robot_names(args.robot_names)
    camera_resolution = parse_resolution(args.camera_resolution)
    resolved_profiles = {}
    for name in robot_names:
        profile = ROBOT_PROFILES.get(str(name).strip().lower())
        if profile is None:
            cli.print_error(
                "Unsupported fake semantic robot "
                f"'{name}'. Supported: anymal, anymal_c, go1, h1, jackal."
            )
            return 1
        if not os.path.isfile(profile.urdf_path):
            cli.print_error(
                f"Collision-body URDF not found for '{name}': {profile.urdf_path}"
            )
            return 1
        resolved_profiles[name] = profile

    cli.print_step("Defining fake semantic perception test configuration...")
    cli.print_info(
        "This registration demo is intended to be paired with fake_tf_ops_suite.py "
        "so semantic perception boxes can be tested without the Carter sim."
    )
    cli.print_info(
        "Recommended fake TF base-frame mapping: "
        + ",".join(
            f"{name}:{resolved_profiles[name].base_frame}" for name in robot_names
        )
    )

    robots = []
    datavizs = []

    for name in robot_names:
        profile = resolved_profiles[name]
        robot = Robot(
            name=name,
            robot_type=profile.robot_type,
            dimensions=profile.dimensions,
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
                "robot_profile": profile.teleop_profile,
                "response_mode": "analog",
                "publish_rate_hz": 30.0,
                "custom_passthrough_only": False,
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

        robot.configure_robot_description(
            urdf_path=profile.urdf_path,
            base_frame=profile.base_frame,
            source="ros",
            chunk_size_bytes=12000,
            is_transparent=True,
            include_visual_meshes=False,
            body_mesh_mode="collision_only",
            enabled=True,
        )

        robot.add_sensor(
            build_camera(
                name=name,
                image_type=args.camera_image_type,
                resolution=camera_resolution,
                fps=args.camera_fps,
            )
        )

        dataviz = robot.create_dataviz()
        dataviz.add_robot_transform(
            robot_name=name,
            topic="/tf",
            frame_id=f"{name}/{profile.base_frame}",
        )
        dataviz.add_robot_velocity_data(
            robot_name=name,
            topic=f"/{name}/odom",
            frame_id="map",
        )
        dataviz.add_robot_odometry_trail(
            robot_name=name,
            topic=f"/{name}/odom",
            frame_id="map",
        )
        dataviz.add_robot_global_path(
            robot_name=name,
            topic=f"/{name}/global_path",
            frame_id="map",
        )
        dataviz.add_robot_local_path(
            robot_name=name,
            topic=f"/{name}/local_path",
            frame_id="map",
        )
        dataviz.add_robot_collision_risk(
            robot_name=name,
            topic=f"/{name}/collision_risk",
            frame_id=f"{name}/{profile.base_frame}",
            render_options={
                "threshold_m": 1.2,
                "radius_m": 1.2,
                "source": "laser_scan",
                "alpha_min": 0.0,
                "alpha_max": 0.55,
            },
        )

        robots.append(robot)
        datavizs.append(dataviz)

    if datavizs:
        add_semantic_demo_boxes(datavizs[0])

    cli.print_info("Semantic perception demo boxes enabled (4 map-anchored boxes, no occupancy grid).")
    cli.print_step(f"Registering {len(robots)} fake robot(s) with HORUS...")

    success, result = register_robots(
        robots,
        datavizs=datavizs,
        keep_alive=args.keep_alive,
        show_dashboard=True,
        workspace_scale=args.workspace_scale,
    )

    if not success:
        cli.print_error(f"Registration failed: {result}")
        return 1

    cli.print_success("Fake semantic perception registration complete.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
