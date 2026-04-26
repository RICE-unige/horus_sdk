#!/usr/bin/env python3
"""Run fake TF ops suite with profile-based robot-description demo defaults."""

import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.join(SCRIPT_DIR, "..", "..")
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from fake_tf_ops_suite import build_ops_suite_parser, run_from_args
from horus.utils import cli
from horus.utils.map_3d_workflow import (
    Map3DMode,
    MeshTransport,
    add_map_3d_mode_arguments,
    build_map_3d_process_specs,
    coerce_mesh_transport_to_marker,
    resolve_map_3d_mode,
    resolve_map_3d_profile,
    start_managed_processes,
    stop_managed_processes,
)

DETAILED_MESH_MAX_VOXELS_DEFAULT = 100000
DETAILED_MESH_MAX_TRIANGLES_DEFAULT = 100000
DETAILED_OCTOMAP_MAX_VOXELS_DEFAULT = 100000
DETAILED_OCTOMAP_MAX_TRIANGLES_DEFAULT = 100000


def _remove_option(parser, option_name: str) -> None:
    action = parser._option_string_actions.get(option_name)  # pylint: disable=protected-access
    if action is None:
        return
    for option in list(action.option_strings):
        parser._option_string_actions.pop(option, None)  # pylint: disable=protected-access
    if action in parser._actions:  # pylint: disable=protected-access
        parser._actions.remove(action)  # pylint: disable=protected-access
    for group in parser._action_groups:  # pylint: disable=protected-access
        if action in group._group_actions:  # pylint: disable=protected-access
            group._group_actions.remove(action)  # pylint: disable=protected-access


def build_parser():
    parser = build_ops_suite_parser()
    parser.description = (
        "Robot-description demo fake runtime with profile-based robot sets "
        "(classic=jackal+go1, real_models=anymal_c+jackal+go1+h1)."
    )
    # Keep workspace scaling authority in MR only for this demo and avoid exposing a TF scale knob.
    _remove_option(parser, "--scale")
    parser.set_defaults(
        robot_count=2,
        robot_names="jackal,go1",
        scale=1.0,
        robot_base_frames="jackal:base_link,go1:base",
        no_static_frames=True,
        static_camera=False,
        publish_compressed_images=True,
        publish_occupancy_grid=False,
        strict_single_tf_publisher=True,
    )
    parser.add_argument(
        "--robot-profile",
        choices=["classic", "real_models"],
        default="classic",
        help="Robot profile preset for names/base frames (default: classic).",
    )
    add_map_3d_mode_arguments(parser)
    parser.add_argument(
        "--map-3d-pipeline-warmup-seconds",
        type=float,
        default=1.0,
        help="Warmup delay after auto-starting 3D map helper nodes (default: 1.0).",
    )
    return parser


def _apply_profile_defaults(args: argparse.Namespace) -> None:
    profile = str(getattr(args, "robot_profile", "classic") or "classic").strip().lower()
    if profile == "real_models":
        args.robot_count = 4
        args.robot_names = "anymal_c,jackal,go1,h1"
        args.robot_base_frames = "anymal_c:base,jackal:base_link,go1:base,h1:pelvis"
    else:
        args.robot_count = 2
        args.robot_names = "jackal,go1"
        args.robot_base_frames = "jackal:base_link,go1:base"


def _apply_map_runtime_defaults(args: argparse.Namespace, mode: Map3DMode) -> None:
    if mode == Map3DMode.OCTOMAP:
        args.map_frame = str(args.map_3d_octomap_frame or "map").strip() or "map"
    elif mode in (Map3DMode.POINTCLOUD, Map3DMode.MESH):
        args.map_frame = str(args.map_3d_frame or "map").strip() or "map"


def _flag_was_provided(flag: str) -> bool:
    prefix = f"{flag}="
    return any(arg == flag or arg.startswith(prefix) for arg in sys.argv[1:])


def _apply_high_detail_mesh_defaults(args: argparse.Namespace, mode: Map3DMode) -> None:
    if mode != Map3DMode.MESH or not (
        bool(getattr(args, "map_3d_detailed", False))
        or str(getattr(args, "map_3d_profile", "") or "").strip().lower() == "realistic"
    ):
        return

    if not _flag_was_provided("--map-3d-mesh-voxel-size"):
        current_voxel_size = float(getattr(args, "map_3d_mesh_voxel_size", 0.0) or 0.0)
        if current_voxel_size <= 0.0 or abs(current_voxel_size - 0.10) < 1e-9:
            args.map_3d_mesh_voxel_size = 0.07
            cli.print_info(
                "[3D-MAP] Detailed mesh mode auto-set --map-3d-mesh-voxel-size=0.07 "
                "(override with --map-3d-mesh-voxel-size)."
            )

    if not _flag_was_provided("--map-3d-mesh-max-voxels"):
        current_voxels = int(getattr(args, "map_3d_mesh_max_voxels", 0))
        if current_voxels < DETAILED_MESH_MAX_VOXELS_DEFAULT:
            args.map_3d_mesh_max_voxels = DETAILED_MESH_MAX_VOXELS_DEFAULT
            cli.print_info(
                "[3D-MAP] Detailed mesh mode auto-set --map-3d-mesh-max-voxels=100000 "
                "(override with --map-3d-mesh-max-voxels)."
            )

    if not _flag_was_provided("--map-3d-mesh-max-triangles"):
        current_triangles = int(getattr(args, "map_3d_mesh_max_triangles", 0))
        if current_triangles < DETAILED_MESH_MAX_TRIANGLES_DEFAULT:
            args.map_3d_mesh_max_triangles = DETAILED_MESH_MAX_TRIANGLES_DEFAULT
            cli.print_info(
                "[3D-MAP] Detailed mesh mode auto-set --map-3d-mesh-max-triangles=100000 "
                "(override with --map-3d-mesh-max-triangles)."
            )


def _apply_high_detail_octomap_defaults(args: argparse.Namespace, mode: Map3DMode) -> None:
    if mode != Map3DMode.OCTOMAP or not (
        bool(getattr(args, "map_3d_detailed", False))
        or str(getattr(args, "map_3d_profile", "") or "").strip().lower() == "realistic"
    ):
        return

    if not _flag_was_provided("--map-3d-octomap-max-voxels"):
        current_voxels = int(getattr(args, "map_3d_octomap_max_voxels", 0))
        if current_voxels < DETAILED_OCTOMAP_MAX_VOXELS_DEFAULT:
            args.map_3d_octomap_max_voxels = DETAILED_OCTOMAP_MAX_VOXELS_DEFAULT
            cli.print_info(
                "[3D-MAP] Detailed octomap mode auto-set --map-3d-octomap-max-voxels=100000 "
                "(override with --map-3d-octomap-max-voxels)."
            )

    if not _flag_was_provided("--map-3d-octomap-max-triangles"):
        current_triangles = int(getattr(args, "map_3d_octomap_max_triangles", 0))
        if current_triangles < DETAILED_OCTOMAP_MAX_TRIANGLES_DEFAULT:
            args.map_3d_octomap_max_triangles = DETAILED_OCTOMAP_MAX_TRIANGLES_DEFAULT
            cli.print_info(
                "[3D-MAP] Detailed octomap mode auto-set --map-3d-octomap-max-triangles=100000 "
                "(override with --map-3d-octomap-max-triangles)."
            )


def _resolve_mesh_transport(args: argparse.Namespace) -> MeshTransport:
    transport, _ = coerce_mesh_transport_to_marker(
        str(getattr(args, "map_3d_mesh_transport", MeshTransport.MARKER.value) or "")
    )
    return MeshTransport(transport)


def _apply_mesh_transport_defaults(args: argparse.Namespace, mode: Map3DMode) -> MeshTransport:
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


def main():
    parser = build_parser()
    args = parser.parse_args()
    _apply_profile_defaults(args)

    mode, warnings = resolve_map_3d_mode(
        args.map_3d_mode,
        with_3d_map=bool(args.with_3d_map),
        with_3d_mesh=bool(args.with_3d_mesh),
    )
    map_profile, profile_warnings = resolve_map_3d_profile(
        args.map_3d_profile,
        detailed=bool(getattr(args, "map_3d_detailed", False)),
    )
    args.map_3d_profile = map_profile.value
    mesh_transport = _apply_mesh_transport_defaults(args, mode)

    for warning in [*warnings, *profile_warnings]:
        cli.print_warning(f"[3D-MAP] {warning}")

    if mode == Map3DMode.OFF:
        cli.print_info("[3D-MAP] Mode=off. No fake map publisher/converter will be auto-started.")
    elif mode == Map3DMode.POINTCLOUD:
        cli.print_info(
            f"[3D-MAP] Mode=pointcloud (profile={map_profile.value}). "
            f"Auto-starting fake pointcloud publisher on {args.map_3d_topic}."
        )
    elif mode == Map3DMode.OCTOMAP:
        cli.print_info(
            "[3D-MAP] Mode=octomap "
            f"(profile={map_profile.value}). Auto-starting fake octomap publisher "
            f"(octomap={args.map_3d_octomap_topic}, mesh={args.map_3d_octomap_mesh_topic})."
        )
    else:
        cli.print_info(
            "[3D-MAP] Mode=mesh "
            f"(profile={map_profile.value}). Auto-starting fake pointcloud publisher ({args.map_3d_topic}) "
            f"and converter transport={mesh_transport.value} "
            f"(marker={args.map_3d_mesh_topic}) "
            f"policy={args.map_3d_mesh_update_policy}."
        )

    args.scale = 1.0
    _apply_map_runtime_defaults(args, mode)
    _apply_high_detail_mesh_defaults(args, mode)
    _apply_high_detail_octomap_defaults(args, mode)

    processes = []
    try:
        if mode != Map3DMode.OFF:
            specs = build_map_3d_process_specs(
                mode=mode,
                python_executable=sys.executable,
                script_dir=SCRIPT_DIR,
                map_3d_topic=args.map_3d_topic,
                map_3d_frame=args.map_3d_frame,
                map_3d_mesh_topic=args.map_3d_mesh_topic,
                map_3d_mesh_array_topic=args.map_3d_mesh_array_topic,
                map_3d_mesh_transport=args.map_3d_mesh_transport,
                map_3d_mesh_chunk_max_triangles=args.map_3d_mesh_chunk_max_triangles,
                map_3d_detailed=bool(getattr(args, "map_3d_detailed", False)),
                map_3d_profile=map_profile.value,
                mesh_voxel_size=args.map_3d_mesh_voxel_size,
                mesh_max_voxels=args.map_3d_mesh_max_voxels,
                mesh_max_triangles=args.map_3d_mesh_max_triangles,
                mesh_update_policy=args.map_3d_mesh_update_policy,
                mesh_republish_interval=args.map_3d_mesh_republish_interval,
                map_3d_octomap_topic=args.map_3d_octomap_topic,
                map_3d_octomap_mesh_topic=args.map_3d_octomap_mesh_topic,
                map_3d_octomap_frame=args.map_3d_octomap_frame,
                map_3d_octomap_max_voxels=args.map_3d_octomap_max_voxels,
                map_3d_octomap_max_triangles=args.map_3d_octomap_max_triangles,
                map_3d_octomap_republish_interval=args.map_3d_octomap_republish_interval,
            )
            processes = start_managed_processes(
                specs,
                log_fn=lambda message: cli.print_info(f"[3D-MAP] {message}"),
                warmup_seconds=max(0.0, float(args.map_3d_pipeline_warmup_seconds)),
            )

        run_from_args(args)
    finally:
        stop_managed_processes(
            processes,
            log_fn=lambda message: cli.print_info(f"[3D-MAP] {message}"),
        )


if __name__ == "__main__":
    main()
