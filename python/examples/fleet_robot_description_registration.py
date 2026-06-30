#!/usr/bin/env python3
"""Register a heterogeneous 5-robot showroom fleet in HORUS MR.

This demonstrates the robot-model upgrade with a static showroom fleet. By default the
SDK resolves each robot's URDF from the cached local assets fetched by
``fetch_robot_description_assets.py`` and bakes the real visual meshes referenced by the
URDF (``package://``), resolving them via the ament index or a local ``mesh_root`` tree.
The live ROS graph is still used for TF. Pass ``--source topic`` or ``--source ros`` when
you specifically want to prove the RViz-style robot_description transport path.

Fleet: Unitree G1/H1, ANYmal C, Boston Dynamics Spot, and Jackal.

Typical run:
    # 1) one-time: fetch URDFs + meshes
    python3 python/examples/tools/fetch_robot_description_assets.py
    # 2) bring up the base-anchor TF graph
    ros2 launch python/examples/launch/fleet_robot_state_publishers.launch.py
    # 3) register with HORUS MR (this script)
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/fleet_robot_description_registration.py

Validate offline (no ROS graph, no HORUS bridge) -- resolves + bakes each robot from the
same local files and prints the baked manifest:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/fleet_robot_description_registration.py --dry-run
"""

import argparse
import os
import sys
from pathlib import Path

from robot_description_showroom_specs import SHOWROOM_FLEET, ShowroomRobotSpec

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

ROBOT_TYPE_BY_KEY = {
    "wheeled": RobotType.WHEELED,
    "legged": RobotType.LEGGED,
}
FLEET = SHOWROOM_FLEET

HIGH_MESH_TRIANGLE_BUDGET = 500000
PREVIEW_MESH_TRIANGLE_BUDGET = 25000


def _local_urdf_path(urdf_name: str, xacro_name) -> str:
    """Best available local URDF source for offline dry-run resolution."""
    urdf_path = ASSETS_DIR / urdf_name
    if urdf_path.is_file():
        return str(urdf_path)
    if xacro_name and (ASSETS_DIR / xacro_name).is_file():
        return str(ASSETS_DIR / xacro_name)
    return ""


def build_robot(spec: ShowroomRobotSpec, args):
    body_mesh_mode = args.body_mesh_mode or spec.body_mesh_mode
    triangle_budget = args.visual_mesh_triangle_budget
    if triangle_budget <= 0:
        triangle_budget = (
            HIGH_MESH_TRIANGLE_BUDGET if body_mesh_mode == "runtime_high_mesh" else PREVIEW_MESH_TRIANGLE_BUDGET
        )

    robot_type = ROBOT_TYPE_BY_KEY.get(spec.robot_type, RobotType.LEGGED)
    robot = Robot(name=spec.name, robot_type=robot_type, dimensions=RobotDimensions(*spec.dimensions))
    # tf_mode="prefixed" with empty tf_prefix -> prefix defaults to the robot name,
    # matching robot_state_publisher's frame_prefix "<name>/".
    robot.configure_ros_binding(base_frame=spec.base_frame)

    use_local_urdf = args.source == "local" or args.dry_run

    description_kwargs = dict(
        source=args.source,
        robot_description_topic=f"/{spec.name}/robot_description",
        ros_param_node=f"/{spec.name}/robot_state_publisher",
        base_frame=spec.base_frame,
        mesh_root=("" if args.no_meshes else str(args.mesh_root)),
        include_visual_meshes=not args.no_meshes,
        visual_mesh_triangle_budget=triangle_budget,
        body_mesh_mode=("collision_only" if args.no_meshes else body_mesh_mode),
        chunk_size_bytes=64000,
    )
    if use_local_urdf:
        # The showroom bodies are static. Reading the local URDF avoids creating a burst
        # of ROS CLI participants just to fetch ten robot_description strings.
        description_kwargs["urdf_path"] = _local_urdf_path(spec.urdf_name, spec.xacro_name)

    robot.configure_robot_description(**description_kwargs)
    robot.configure_robot_manager(
        status=True,
        data_viz=True,
        teleop=False,
        tasks=False,
    )
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
        choices=["local", "topic", "ros"],
        default="local",
        help="Where the SDK reads each URDF: cached local assets (default), latched "
        "/<ns>/robot_description topic, or the robot_state_publisher parameter (ros).",
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
    parser.add_argument(
        "--visual-mesh-triangle-budget",
        type=int,
        default=0,
        help=f"Override the visual mesh triangle budget per robot. Default high-detail budget is {HIGH_MESH_TRIANGLE_BUDGET}.",
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

    print(
        f"[showroom] registering {len(robots)} robots via source='{args.source}' "
        f"(mesh_root={'<disabled>' if args.no_meshes else args.mesh_root})"
    )
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
            print("[showroom] registration monitor stopped.")
            return 0
        print(f"[showroom] registration failed: {result}")
        return 1
    print("[showroom] registration complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
