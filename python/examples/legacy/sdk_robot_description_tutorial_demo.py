#!/usr/bin/env python3
"""Register the robot-description demo with workspace tutorial mode enabled."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.join(SCRIPT_DIR, "..", "..")
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

try:
    import sdk_robot_description_demo as base_demo
    from horus.robot import register_robots
    from horus.utils import cli
    from horus.utils.map_3d_workflow import resolve_map_3d_mode
except ImportError:
    print(f"Failed to import tutorial demo dependencies from {PACKAGE_ROOT}")
    raise


WORKSPACE_TUTORIAL_PRESET_ID = "robot_description_onboarding_v1"


def build_parser():
    parser = base_demo.build_parser()
    parser.description = (
        "Register robot-description demo profiles with tutorial mode enabled "
        "for first-time MR onboarding."
    )
    return parser


def main():
    args = build_parser().parse_args()
    map_3d_mode, map_mode_warnings = resolve_map_3d_mode(
        args.map_3d_mode,
        with_3d_map=bool(args.with_3d_map),
        with_3d_mesh=bool(args.with_3d_mesh),
    )
    mesh_transport = base_demo._apply_mesh_transport_defaults(args, map_3d_mode)
    base_demo._apply_high_detail_mesh_registration_defaults(args, map_3d_mode)
    base_demo._apply_high_detail_octomap_registration_defaults(args, map_3d_mode)

    for warning in map_mode_warnings:
        cli.print_warning(f"[3D-MAP] {warning}")

    cli.print_info(f"[3D-MAP] Registration mode -> {map_3d_mode.value}")
    if map_3d_mode == base_demo.Map3DMode.MESH:
        cli.print_info(
            f"[3D-MAP] Mesh transport -> {mesh_transport.value} (topic={args.map_3d_mesh_topic})"
        )
    if map_3d_mode == base_demo.Map3DMode.POINTCLOUD:
        cli.print_warning(
            "[3D-MAP] Pointcloud mode is not recommended on Quest 3 for regular operations. "
            "Prefer mesh mode for stable performance."
        )
    if map_3d_mode == base_demo.Map3DMode.OCTOMAP:
        cli.print_info(
            "[3D-MAP] Octomap mode uses mesh-marker rendering in MR for Quest performance."
        )

    setup_entries = base_demo._build_robot_profile_entries(args)

    if args.enable_robot_description:
        missing = []
        for entry in setup_entries:
            urdf_path = str(entry.get("urdf_path", "") or "")
            if not urdf_path or not os.path.isfile(urdf_path):
                raw_value = str(entry.get("arg_raw", "") or "").strip() or "unresolved"
                missing.append(f"{entry['arg_label']} ({raw_value})")
        if missing:
            cli.print_error("Robot description URDF path missing/unreadable:")
            for item in missing:
                cli.print_error(f"  - {item}")
            cli.print_info("Tip: fetch local real assets with:")
            cli.print_info("  python3 python/examples/tools/fetch_robot_description_assets.py")
            cli.print_info("Provide explicit paths or disable with --no-enable-robot-description.")
            return
        cli.print_info(f"Robot profile: {args.robot_profile}")
        for entry in setup_entries:
            cli.print_info(f"{entry['name']} URDF: {entry['urdf_path']}")

    cli.print_info(
        f"Camera metadata -> image_scale={args.camera_image_scale:.3f}, overhead_size={args.camera_overhead_size:.3f}"
    )
    cli.print_info(
        f"Robot-description collision transparency -> is_transparent={bool(args.collision_transparent)}"
    )
    cli.print_info(f"Workspace tutorial preset -> {WORKSPACE_TUTORIAL_PRESET_ID}")

    setup = []
    for entry in setup_entries:
        setup_entry = base_demo._build_robot(
            name=str(entry["name"]),
            robot_type=entry["robot_type"],
            dimensions=entry["dimensions"],
            urdf_path=str(entry["urdf_path"] or ""),
            description_base_frame=str(entry["base_frame"]),
            enable_robot_description=bool(args.enable_robot_description),
            collision_transparent=bool(args.collision_transparent),
            camera_image_scale=float(args.camera_image_scale),
            camera_overhead_size=float(args.camera_overhead_size),
        )
        setup_entry["robot"].configure_workspace_tutorial(
            WORKSPACE_TUTORIAL_PRESET_ID,
            enabled=True,
        )
        base_demo._add_map_3d_visualization(
            setup_entry["dataviz"],
            args,
            map_3d_mode,
        )
        setup.append(setup_entry)

    robots = [entry["robot"] for entry in setup]
    datavizs = [entry["dataviz"] for entry in setup]

    cli.print_step("Registering robot-description tutorial demo robots...")
    success, result = register_robots(
        robots,
        datavizs=datavizs,
        keep_alive=bool(args.keep_alive),
        show_dashboard=True,
        workspace_scale=float(args.workspace_scale),
    )
    if not success:
        cli.print_error(f"Registration failed: {result}")
        return
    cli.print_success("Robot-description tutorial demo registration complete.")


if __name__ == "__main__":
    main()
