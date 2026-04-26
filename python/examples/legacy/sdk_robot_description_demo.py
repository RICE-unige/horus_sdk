#!/usr/bin/env python3
"""Register robot-description demo profiles with Robot Description V1 enabled."""

import argparse
import os
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.join(SCRIPT_DIR, "..", "..")
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

try:
    from horus.robot import Robot, RobotDimensions, RobotType, register_robots
    from horus.sensors import Camera
    from horus.utils import cli
    from horus.utils.map_3d_workflow import (
        Map3DMode,
        MeshTransport,
        add_map_3d_mode_arguments,
        coerce_mesh_transport_to_marker,
        resolve_map_3d_mode,
    )
except ImportError:
    print(f"Failed to import horus from {PACKAGE_ROOT}")
    raise

DETAILED_MESH_MAX_TRIANGLES_DEFAULT = 100000
DETAILED_OCTOMAP_MAX_TRIANGLES_DEFAULT = 100000


def _first_existing_path(candidates: List[str]) -> str:
    for candidate in candidates:
        if candidate and os.path.isfile(candidate):
            return candidate
    return ""


def _local_asset_path(filename: str) -> str:
    return str(Path(SCRIPT_DIR).resolve().parent / ".local_assets" / "robot_descriptions" / filename)


def _resolve_demo_urdf(explicit: str, local_candidates: Sequence[str], fallback_candidates: Sequence[str]) -> str:
    if explicit:
        candidate = os.path.expandvars(os.path.expanduser(explicit.strip()))
        if os.path.isfile(candidate):
            return candidate
        return ""

    return _first_existing_path([*local_candidates, *fallback_candidates])


def _resolve_wheeled_urdf(explicit: str) -> str:
    return _resolve_demo_urdf(
        explicit=explicit,
        local_candidates=(
            _local_asset_path("jackal.urdf"),
        ),
        fallback_candidates=(
            "/mnt/c/Users/adeko/horus/Assets/URDF/nova_carter_fbx.urdf",
            "C:/Users/adeko/horus/Assets/URDF/nova_carter_fbx.urdf",
            _local_asset_path("jackal.urdf.xacro"),
        ),
    )


def _resolve_legged_urdf(explicit: str) -> str:
    return _resolve_demo_urdf(
        explicit=explicit,
        local_candidates=(
            _local_asset_path("go1.urdf"),
        ),
        fallback_candidates=(
            "/mnt/c/Users/adeko/newton/newton/examples/assets/quadruped.urdf",
            "C:/Users/adeko/newton/newton/examples/assets/quadruped.urdf",
        ),
    )


def _resolve_anymal_urdf(explicit: str) -> str:
    return _resolve_demo_urdf(
        explicit=explicit,
        local_candidates=(
            _local_asset_path("anymal_c.urdf"),
        ),
        fallback_candidates=(),
    )


def _resolve_h1_urdf(explicit: str) -> str:
    return _resolve_demo_urdf(
        explicit=explicit,
        local_candidates=(
            _local_asset_path("h1.urdf"),
        ),
        fallback_candidates=(),
    )


def _build_camera(robot_name: str, frame_name: str, image_scale: float, overhead_size: float) -> Camera:
    resolved_frame = str(frame_name or "base_link").strip()
    if resolved_frame.startswith(f"{robot_name}/"):
        frame_id = resolved_frame
    else:
        frame_id = f"{robot_name}/{resolved_frame}"
    camera = Camera(
        name="front_camera",
        frame_id=frame_id,
        topic=f"/{robot_name}/camera/image_raw/compressed",
        resolution=(160, 90),
        fps=6,
        encoding="jpeg",
        streaming_type="ros",
        minimap_streaming_type="ros",
        teleop_streaming_type="webrtc",
        startup_mode="minimap",
    )
    camera.add_metadata("image_type", "compressed")
    camera.add_metadata("display_mode", "projected")
    camera.add_metadata("use_tf", True)
    camera.add_metadata("image_scale", float(image_scale))
    camera.add_metadata("focal_length_scale", 0.090)
    camera.add_metadata("view_position_offset", [0.0, 0.0, 0.0])
    camera.add_metadata("view_rotation_offset", [0.0, 0.0, 0.0])
    camera.add_metadata("show_frustum", True)
    camera.add_metadata("frustum_color", "#59D9FF")
    camera.add_metadata("overhead_size", float(overhead_size))
    camera.add_metadata("overhead_position_offset", [0.0, 1.4, 0.0])
    camera.add_metadata("overhead_face_camera", True)
    camera.add_metadata("overhead_rotation_offset", [90.0, 0.0, 0.0])
    return camera


def _apply_common_runtime_metadata(robot: Robot) -> None:
    name = robot.name
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
            "robot_profile": "legged" if robot.robot_type == RobotType.LEGGED else "wheeled",
            "response_mode": "analog",
            "publish_rate_hz": 30.0,
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


def _build_robot(
    name: str,
    robot_type: RobotType,
    dimensions: RobotDimensions,
    urdf_path: Optional[str],
    description_base_frame: str,
    enable_robot_description: bool,
    collision_transparent: bool,
    camera_image_scale: float,
    camera_overhead_size: float,
) -> Dict:
    robot = Robot(name=name, robot_type=robot_type, dimensions=dimensions)
    _apply_common_runtime_metadata(robot)
    robot.add_sensor(
        _build_camera(
            name,
            description_base_frame,
            camera_image_scale,
            camera_overhead_size,
        )
    )

    if enable_robot_description and urdf_path:
        robot.configure_robot_description(
            urdf_path=urdf_path,
            base_frame=str(description_base_frame or "base_link"),
            source="ros",
            chunk_size_bytes=12000,
            is_transparent=bool(collision_transparent),
            enabled=True,
        )

    dataviz = robot.create_dataviz()
    robot.add_path_planning_to_dataviz(
        dataviz,
        global_path_topic=f"/{name}/global_path",
        local_path_topic=f"/{name}/local_path",
    )
    robot.add_navigation_safety_to_dataviz(
        dataviz,
        odom_topic=f"/{name}/odom",
        collision_risk_topic=f"/{name}/collision_risk",
        include_velocity=True,
        include_trail=True,
        include_collision=True,
    )
    return {"robot": robot, "dataviz": dataviz}


def _add_map_3d_visualization(dataviz, args, mode: Map3DMode) -> None:
    if dataviz is None or mode == Map3DMode.OFF:
        return

    if mode == Map3DMode.POINTCLOUD:
        dataviz.add_3d_map(
            topic=str(args.map_3d_topic or "/map_3d").strip() or "/map_3d",
            frame_id=str(args.map_3d_frame or "map").strip() or "map",
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
        return

    if mode == Map3DMode.OCTOMAP:
        dataviz.add_3d_octomap(
            topic=str(args.map_3d_octomap_mesh_topic or "/map_3d_octomap_mesh").strip() or "/map_3d_octomap_mesh",
            frame_id=str(args.map_3d_octomap_frame or "map").strip() or "map",
            render_options={
                "render_mode": "surface_mesh",
                "use_vertex_colors": True,
                "alpha": 1.0,
                "double_sided": False,
                "max_triangles": max(1000, int(getattr(args, "map_3d_octomap_max_triangles", 60000))),
                "source_coordinate_space": "enu",
                "native_topic": str(args.map_3d_octomap_topic or "/map_3d_octomap").strip() or "/map_3d_octomap",
                "native_frame": str(args.map_3d_octomap_frame or "map").strip() or "map",
                "native_binary_only": True,
            },
        )
        return

    marker_topic = "/map_3d_mesh"
    dataviz.add_3d_mesh(
        topic=marker_topic,
        frame_id=str(args.map_3d_mesh_frame or "map").strip() or "map",
        render_options={
            "use_vertex_colors": True,
            "alpha": 1.0,
            "double_sided": False,
            "max_triangles": max(1000, int(getattr(args, "map_3d_mesh_max_triangles", 60000))),
            "source_coordinate_space": "enu",
        },
    )


def _flag_was_provided(flag: str) -> bool:
    prefix = f"{flag}="
    return any(arg == flag or arg.startswith(prefix) for arg in sys.argv[1:])


def _resolve_mesh_transport(args) -> MeshTransport:
    transport, _ = coerce_mesh_transport_to_marker(
        str(getattr(args, "map_3d_mesh_transport", MeshTransport.MARKER.value) or "")
    )
    return MeshTransport(transport)


def _apply_mesh_transport_defaults(args, mode: Map3DMode) -> MeshTransport:
    requested_topic = str(getattr(args, "map_3d_mesh_topic", "/map_3d_mesh") or "/map_3d_mesh").strip() or "/map_3d_mesh"
    args.map_3d_mesh_topic = "/map_3d_mesh"
    transport_raw = str(getattr(args, "map_3d_mesh_transport", MeshTransport.MARKER.value) or "")
    _, requested_marker_array = coerce_mesh_transport_to_marker(transport_raw)
    transport = MeshTransport.MARKER
    args.map_3d_mesh_transport = MeshTransport.MARKER.value
    if mode != Map3DMode.MESH:
        return transport

    if requested_marker_array or _flag_was_provided("--map-3d-mesh-transport"):
        cli.print_warning(
            "[3D-MAP] Marker-only rollback active. Ignoring --map-3d-mesh-transport and forcing marker transport."
        )
    if _flag_was_provided("--map-3d-mesh-array-topic"):
        cli.print_warning(
            "[3D-MAP] Marker-only rollback active. Ignoring --map-3d-mesh-array-topic."
        )
    if _flag_was_provided("--map-3d-mesh-chunk-max-triangles"):
        cli.print_warning(
            "[3D-MAP] Marker-only rollback active. Ignoring --map-3d-mesh-chunk-max-triangles."
        )
    if requested_topic != "/map_3d_mesh":
        cli.print_warning(
            "[3D-MAP] Marker-only rollback active. Forcing mesh topic to /map_3d_mesh."
        )

    return transport


def _apply_high_detail_mesh_registration_defaults(args, mode: Map3DMode) -> None:
    if mode != Map3DMode.MESH or not (
        bool(getattr(args, "map_3d_detailed", False))
        or str(getattr(args, "map_3d_profile", "") or "").strip().lower() == "realistic"
    ):
        return

    if _flag_was_provided("--map-3d-mesh-max-triangles"):
        return

    current = int(getattr(args, "map_3d_mesh_max_triangles", 0))
    if current >= DETAILED_MESH_MAX_TRIANGLES_DEFAULT:
        return

    args.map_3d_mesh_max_triangles = DETAILED_MESH_MAX_TRIANGLES_DEFAULT
    cli.print_info(
        "[3D-MAP] Detailed mesh mode auto-set registration --map-3d-mesh-max-triangles=100000 "
        "(override with --map-3d-mesh-max-triangles)."
    )


def _apply_high_detail_octomap_registration_defaults(args, mode: Map3DMode) -> None:
    if mode != Map3DMode.OCTOMAP or not (
        bool(getattr(args, "map_3d_detailed", False))
        or str(getattr(args, "map_3d_profile", "") or "").strip().lower() == "realistic"
    ):
        return

    if _flag_was_provided("--map-3d-octomap-max-triangles"):
        return

    current = int(getattr(args, "map_3d_octomap_max_triangles", 0))
    if current >= DETAILED_OCTOMAP_MAX_TRIANGLES_DEFAULT:
        return

    args.map_3d_octomap_max_triangles = DETAILED_OCTOMAP_MAX_TRIANGLES_DEFAULT
    cli.print_info(
        "[3D-MAP] Detailed octomap mode auto-set registration --map-3d-octomap-max-triangles=100000 "
        "(override with --map-3d-octomap-max-triangles)."
    )


def _build_robot_profile_entries(args) -> List[Dict]:
    profile = str(args.robot_profile or "classic").strip().lower()
    if profile == "real_models":
        anymal_urdf = _resolve_anymal_urdf(args.anymal_urdf)
        wheeled_urdf = _resolve_wheeled_urdf(args.wheeled_urdf)
        legged_urdf = _resolve_legged_urdf(args.legged_urdf)
        h1_urdf = _resolve_h1_urdf(args.h1_urdf)
        return [
            {
                "name": "anymal_c",
                "robot_type": RobotType.LEGGED,
                "dimensions": RobotDimensions(length=0.85, width=0.52, height=0.76),
                "urdf_path": anymal_urdf,
                "base_frame": "base",
                "arg_label": "--anymal-urdf",
                "arg_raw": args.anymal_urdf,
            },
            {
                "name": "jackal",
                "robot_type": RobotType.WHEELED,
                "dimensions": RobotDimensions(length=0.50, width=0.38, height=0.32),
                "urdf_path": wheeled_urdf,
                "base_frame": "base_link",
                "arg_label": "--wheeled-urdf",
                "arg_raw": args.wheeled_urdf,
            },
            {
                "name": "go1",
                "robot_type": RobotType.LEGGED,
                "dimensions": RobotDimensions(length=0.68, width=0.31, height=0.48),
                "urdf_path": legged_urdf,
                "base_frame": "base",
                "arg_label": "--legged-urdf",
                "arg_raw": args.legged_urdf,
            },
            {
                "name": "h1",
                "robot_type": RobotType.LEGGED,
                # Conservative dimensions so mesh-proxy fallback does not overinflate joint links.
                "dimensions": RobotDimensions(length=0.42, width=0.20, height=0.28),
                "urdf_path": h1_urdf,
                "base_frame": "pelvis",
                "arg_label": "--h1-urdf",
                "arg_raw": args.h1_urdf,
            },
        ]

    wheeled_urdf = _resolve_wheeled_urdf(args.wheeled_urdf)
    legged_urdf = _resolve_legged_urdf(args.legged_urdf)
    return [
        {
            "name": "jackal",
            "robot_type": RobotType.WHEELED,
            "dimensions": RobotDimensions(length=0.50, width=0.38, height=0.32),
            "urdf_path": wheeled_urdf,
            "base_frame": "base_link",
            "arg_label": "--wheeled-urdf",
            "arg_raw": args.wheeled_urdf,
        },
        {
            "name": "go1",
            "robot_type": RobotType.LEGGED,
            "dimensions": RobotDimensions(length=0.68, width=0.31, height=0.48),
            "urdf_path": legged_urdf,
            "base_frame": "base",
            "arg_label": "--legged-urdf",
            "arg_raw": args.legged_urdf,
        },
    ]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Register robot-description demo profiles with Robot Description V1 "
            "(collision mesh + joint axes) and standard nav/task topics."
        )
    )
    parser.add_argument(
        "--robot-profile",
        choices=["classic", "real_models"],
        default="classic",
        help=(
            "Robot profile selection. classic=jackal+go1 (default), "
            "real_models=anymal_c+jackal+go1+h1."
        ),
    )
    parser.add_argument(
        "--wheeled-urdf",
        default="",
        help="Path to wheeled robot URDF/xacro. If omitted, resolves local fetched Jackal assets first.",
    )
    parser.add_argument(
        "--legged-urdf",
        default="",
        help="Path to legged robot URDF/xacro. If omitted, resolves local fetched Go1 asset first.",
    )
    parser.add_argument(
        "--anymal-urdf",
        default="",
        help="Path to Anymal C URDF/xacro. Used by --robot-profile real_models.",
    )
    parser.add_argument(
        "--h1-urdf",
        default="",
        help="Path to Unitree H1 URDF/xacro. Used by --robot-profile real_models.",
    )
    parser.add_argument(
        "--enable-robot-description",
        dest="enable_robot_description",
        action="store_true",
        default=True,
        help="Enable Robot Description V1 manifest + chunk transport (default: on).",
    )
    parser.add_argument(
        "--no-enable-robot-description",
        dest="enable_robot_description",
        action="store_false",
        help="Disable robot description transport metadata.",
    )
    parser.add_argument(
        "--workspace-scale",
        type=float,
        default=0.1,
        help="Workspace scale embedded into registration payloads.",
    )
    parser.add_argument(
        "--camera-image-scale",
        type=float,
        default=0.045,
        help="Projected camera image scale metadata (default: 0.045).",
    )
    parser.add_argument(
        "--camera-overhead-size",
        type=float,
        default=0.58,
        help="Overhead camera panel size metadata (default: 0.58).",
    )
    transparency_group = parser.add_mutually_exclusive_group()
    transparency_group.add_argument(
        "--collision-transparent",
        dest="collision_transparent",
        action="store_true",
        default=False,
        help="Render robot-description collision body as transparent.",
    )
    transparency_group.add_argument(
        "--collision-opaque",
        dest="collision_transparent",
        action="store_false",
        help="Render robot-description collision body as opaque (default).",
    )
    add_map_3d_mode_arguments(parser)
    parser.add_argument(
        "--keep-alive",
        dest="keep_alive",
        action="store_true",
        default=True,
        help="Keep SDK dashboard alive after registration (default: on).",
    )
    parser.add_argument(
        "--no-keep-alive",
        dest="keep_alive",
        action="store_false",
        help="Exit after successful registration.",
    )
    return parser


def main():
    args = build_parser().parse_args()
    map_3d_mode, map_mode_warnings = resolve_map_3d_mode(
        args.map_3d_mode,
        with_3d_map=bool(args.with_3d_map),
        with_3d_mesh=bool(args.with_3d_mesh),
    )
    mesh_transport = _apply_mesh_transport_defaults(args, map_3d_mode)
    _apply_high_detail_mesh_registration_defaults(args, map_3d_mode)
    _apply_high_detail_octomap_registration_defaults(args, map_3d_mode)

    for warning in map_mode_warnings:
        cli.print_warning(f"[3D-MAP] {warning}")

    cli.print_info(f"[3D-MAP] Registration mode -> {map_3d_mode.value}")
    if map_3d_mode == Map3DMode.MESH:
        cli.print_info(
            f"[3D-MAP] Mesh transport -> {mesh_transport.value} (topic={args.map_3d_mesh_topic})"
        )
    if map_3d_mode == Map3DMode.POINTCLOUD:
        cli.print_warning(
            "[3D-MAP] Pointcloud mode is not recommended on Quest 3 for regular operations. "
            "Prefer mesh mode for stable performance."
        )
    if map_3d_mode == Map3DMode.OCTOMAP:
        cli.print_info(
            "[3D-MAP] Octomap mode uses mesh-marker rendering in MR for Quest performance."
        )

    setup_entries = _build_robot_profile_entries(args)

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

    setup = []
    for entry in setup_entries:
        setup_entry = _build_robot(
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
        _add_map_3d_visualization(
            setup_entry["dataviz"],
            args,
            map_3d_mode,
        )
        setup.append(setup_entry)
    robots = [entry["robot"] for entry in setup]
    datavizs = [entry["dataviz"] for entry in setup]

    cli.print_step("Registering robot-description demo robots...")
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
    cli.print_success("Robot-description demo registration complete.")


if __name__ == "__main__":
    main()
