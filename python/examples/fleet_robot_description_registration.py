#!/usr/bin/env python3
"""Register a heterogeneous 4-robot fleet in HORUS MR via the robot_description workflow.

This demonstrates the robot-model upgrade: the SDK resolves each robot's URDF straight
from a normal ROS source -- the latched ``/<ns>/robot_description`` topic (default) or the
``robot_state_publisher`` parameter (``--source ros``) -- and bakes the real visual meshes
referenced by the URDF (``package://``), resolving them via the ament index or a local
``mesh_root`` tree. No per-robot URDF path is hand-fed to the SDK.

Fleet: jackal (wheeled), go1 + anymal_c (legged), h1 (humanoid).

Typical run:
    # 1) one-time: fetch URDFs + meshes
    python3 python/examples/tools/fetch_robot_description_assets.py
    # 2) bring up the ROS robot_description graph
    ros2 launch python/examples/launch/fleet_robot_state_publishers.launch.py
    # 3) register with HORUS MR (this script)
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/fleet_robot_description_registration.py

Validate offline (no ROS graph, no HORUS bridge) -- resolves + bakes each robot from the
local files and prints the baked manifest:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/fleet_robot_description_registration.py --dry-run
"""

import argparse
import os
import sys
from pathlib import Path

from horus.robot import (
    Robot,
    RobotDimensions,
    RobotType,
    is_registration_cancelled,
    register_robots,
)

SCRIPT_DIR = Path(__file__).resolve().parent
ASSETS_DIR = SCRIPT_DIR / ".local_assets" / "robot_descriptions"
DEFAULT_MESH_ROOT = ASSETS_DIR / "meshes_root"

# name == launch namespace == default tf_prefix (tf_mode="prefixed" defaults prefix to name).
# (name, RobotType, root link, (length,width,height), body_mesh_mode, urdf basename, xacro basename)
FLEET = [
    ("jackal", RobotType.WHEELED, "base_link", (0.508, 0.430, 0.250), "preview_mesh", "jackal.urdf", "jackal.urdf.xacro"),
    ("go1", RobotType.LEGGED, "base", (0.540, 0.300, 0.180), "runtime_high_mesh", "go1.urdf", None),
    ("anymal_c", RobotType.LEGGED, "base", (0.930, 0.530, 0.700), "preview_mesh", "anymal_c.urdf", None),
    # HORUS RobotType has no HUMANOID; LEGGED is the closest classification for H1.
    ("h1", RobotType.LEGGED, "pelvis", (0.300, 0.400, 1.800), "runtime_high_mesh", "h1.urdf", None),
]

HIGH_MESH_TRIANGLE_BUDGET = 220000
PREVIEW_MESH_TRIANGLE_BUDGET = 90000


def _local_urdf_path(urdf_name: str, xacro_name) -> str:
    """Best available local URDF source for offline dry-run resolution."""
    urdf_path = ASSETS_DIR / urdf_name
    if urdf_path.is_file():
        return str(urdf_path)
    if xacro_name and (ASSETS_DIR / xacro_name).is_file():
        return str(ASSETS_DIR / xacro_name)
    return ""


def build_robot(spec, args):
    name, robot_type, root_link, dims, default_mode, urdf_name, xacro_name = spec
    body_mesh_mode = args.body_mesh_mode or default_mode
    triangle_budget = (
        HIGH_MESH_TRIANGLE_BUDGET if body_mesh_mode == "runtime_high_mesh" else PREVIEW_MESH_TRIANGLE_BUDGET
    )

    robot = Robot(name=name, robot_type=robot_type, dimensions=RobotDimensions(*dims))
    # tf_mode="prefixed" with empty tf_prefix -> prefix defaults to the robot name,
    # matching robot_state_publisher's frame_prefix "<name>/".
    robot.configure_ros_binding(base_frame=root_link)

    description_kwargs = dict(
        source=args.source,
        robot_description_topic=f"/{name}/robot_description",
        ros_param_node=f"/{name}/robot_state_publisher",
        base_frame=root_link,
        mesh_root=("" if args.no_meshes else str(args.mesh_root)),
        include_visual_meshes=not args.no_meshes,
        visual_mesh_triangle_budget=triangle_budget,
        body_mesh_mode=("collision_only" if args.no_meshes else body_mesh_mode),
        chunk_size_bytes=64000,
    )
    if args.dry_run:
        # Offline validation reads the local file directly (urdf_path takes precedence),
        # so it proves URDF + mesh resolution without a live ROS graph.
        description_kwargs["urdf_path"] = _local_urdf_path(urdf_name, xacro_name)

    robot.configure_robot_description(**description_kwargs)
    return robot


def run_dry_run(robots) -> int:
    from horus.description.robot_description_resolver import RobotDescriptionResolver

    resolver = RobotDescriptionResolver()
    all_ok = True
    for robot in robots:
        artifact = resolver.resolve_for_robot(robot)
        if artifact is None:
            print(f"{robot.name:9s}: RESOLVE_FAILED: {resolver.last_error}")
            all_ok = False
            continue
        m = artifact.manifest
        print(
            f"{robot.name:9s}: links={m.link_count:3d} joints={m.joint_count:3d} "
            f"collisions={m.collision_count:3d} mesh_assets={m.mesh_asset_count:3d} "
            f"meshes={m.supports_visual_meshes!s:5s} mesh_kb={m.mesh_asset_encoded_bytes // 1024:5d} "
            f"chunks={len(artifact.chunks):3d} base={m.base_frame}"
        )
        if not robot.get_metadata("robot_description_config", {}).get("urdf_path"):
            all_ok = False  # dry-run requires a local source
    print("DRYRUN_OK" if all_ok else "DRYRUN_HAD_ISSUES")
    return 0 if all_ok else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--source",
        choices=["topic", "ros"],
        default="topic",
        help="Where the SDK reads each URDF: latched /<ns>/robot_description topic (default) "
        "or the robot_state_publisher parameter (ros).",
    )
    parser.add_argument(
        "--mesh-root",
        dest="mesh_root",
        default=str(DEFAULT_MESH_ROOT),
        help="Filesystem root for resolving package:// meshes when the ROS packages are not installed.",
    )
    parser.add_argument(
        "--body-mesh-mode",
        choices=["collision_only", "preview_mesh", "runtime_high_mesh"],
        default="",
        help="Override the per-robot body mesh mode for all robots.",
    )
    parser.add_argument("--no-meshes", action="store_true", help="Register collision-only (skip visual meshes).")
    parser.add_argument("--workspace-scale", type=float, default=0.1, help="HORUS workspace scale.")
    parser.add_argument("--no-keep-alive", action="store_true", help="Do not keep the registration monitor alive.")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Resolve + bake each robot from local files and print the manifest; no HORUS connection.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    args.mesh_root = Path(args.mesh_root).expanduser()

    robots = [build_robot(spec, args) for spec in FLEET]

    if args.dry_run:
        return run_dry_run(robots)

    print(f"[fleet] registering {len(robots)} robots via source='{args.source}' "
          f"(mesh_root={'<disabled>' if args.no_meshes else args.mesh_root})")
    datavizs = [robot.create_dataviz() for robot in robots]

    success, result = register_robots(
        robots,
        datavizs=datavizs,
        workspace_scale=float(args.workspace_scale),
        compass_enabled=False,
        keep_alive=not args.no_keep_alive,
        show_dashboard=True,
    )
    if not success:
        if is_registration_cancelled(result):
            print("[fleet] registration monitor stopped.")
            return 0
        print(f"[fleet] registration failed: {result}")
        return 1
    print("[fleet] registration complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
