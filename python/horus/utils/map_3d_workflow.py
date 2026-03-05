"""Helpers for 3D map demo mode selection and subprocess orchestration."""

from __future__ import annotations

import argparse
import os
import shlex
import signal
import subprocess
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, List, Optional, Sequence, Tuple


class Map3DMode(str, Enum):
    """Supported 3D map visualization modes."""

    OFF = "off"
    POINTCLOUD = "pointcloud"
    MESH = "mesh"


class MeshUpdatePolicy(str, Enum):
    """Mesh converter update policies exposed by SDK demo CLIs."""

    SNAPSHOT = "snapshot"
    PERIODIC = "periodic"
    CONTINUOUS = "continuous"


class MeshTransport(str, Enum):
    """Mesh transport payload formats supported by SDK demo CLIs."""

    MARKER = "marker"
    MARKER_ARRAY = "marker_array"


@dataclass(frozen=True)
class ManagedProcessSpec:
    """Launch specification for a managed subprocess."""

    name: str
    command: List[str]
    cwd: str


@dataclass
class ManagedProcessHandle:
    """Running subprocess handle tracked by this module."""

    name: str
    command: List[str]
    cwd: str
    process: subprocess.Popen


def add_map_3d_mode_arguments(parser: argparse.ArgumentParser) -> None:
    """Attach common 3D-map arguments to example CLIs."""

    parser.add_argument(
        "--map-3d-mode",
        choices=[mode.value for mode in Map3DMode],
        default=None,
        help=(
            "Global 3D map visualization mode: off|pointcloud|mesh "
            "(default: off)."
        ),
    )
    parser.add_argument(
        "--map-3d-topic",
        default="/map_3d",
        help="PointCloud2 map topic (default: /map_3d).",
    )
    parser.add_argument(
        "--map-3d-detailed",
        action="store_true",
        default=False,
        help=(
            "Use the detailed pointcloud publisher profile "
            "(fake_3d_map_publisher_realistic.py)."
        ),
    )
    parser.add_argument(
        "--map-3d-frame",
        default="map",
        help="PointCloud2 map frame id (default: map).",
    )
    parser.add_argument(
        "--map-3d-mesh-topic",
        default="/map_3d_mesh",
        help="Mesh marker map topic (default: /map_3d_mesh).",
    )
    parser.add_argument(
        "--map-3d-mesh-array-topic",
        default="/map_3d_mesh_array",
        help=(
            "Chunked mesh MarkerArray topic (default: /map_3d_mesh_array). "
            "Accepted for compatibility; ignored in marker-only rollback mode."
        ),
    )
    parser.add_argument(
        "--map-3d-mesh-transport",
        choices=[transport.value for transport in MeshTransport],
        default=MeshTransport.MARKER.value,
        help=(
            "Mesh transport mode: marker|marker_array "
            "(default: marker). Accepted for compatibility; marker is enforced "
            "in marker-only rollback mode."
        ),
    )
    parser.add_argument(
        "--map-3d-mesh-chunk-max-triangles",
        type=int,
        default=3000,
        help=(
            "Chunk triangle cap for marker_array transport "
            "(default: 3000). Accepted for compatibility; ignored in marker-only rollback mode."
        ),
    )
    parser.add_argument(
        "--map-3d-mesh-frame",
        default="map",
        help="Mesh marker map frame id (default: map).",
    )
    parser.add_argument(
        "--map-3d-mesh-voxel-size",
        type=float,
        default=0.10,
        help="Mesh converter voxel size in meters (default: 0.10).",
    )
    parser.add_argument(
        "--map-3d-mesh-max-voxels",
        type=int,
        default=60000,
        help="Mesh converter voxel cap (default: 60000, 0=unlimited).",
    )
    parser.add_argument(
        "--map-3d-mesh-max-triangles",
        type=int,
        default=60000,
        help="Mesh converter triangle cap (default: 60000, 0=unlimited).",
    )
    parser.add_argument(
        "--map-3d-mesh-update-policy",
        choices=[policy.value for policy in MeshUpdatePolicy],
        default=MeshUpdatePolicy.SNAPSHOT.value,
        help=(
            "Mesh conversion update policy: "
            "snapshot|periodic|continuous (default: snapshot)."
        ),
    )
    parser.add_argument(
        "--map-3d-mesh-republish-interval",
        type=float,
        default=0.0,
        help=(
            "Republish interval in seconds for mesh marker keepalive. "
            "Default 0 disables periodic keepalive bursts."
        ),
    )
    parser.add_argument(
        "--with-3d-map",
        dest="with_3d_map",
        action="store_true",
        default=False,
        help="[Deprecated] Legacy alias for --map-3d-mode pointcloud.",
    )
    parser.add_argument(
        "--with-3d-mesh",
        dest="with_3d_mesh",
        action="store_true",
        default=False,
        help="[Deprecated] Legacy alias for --map-3d-mode mesh.",
    )


def resolve_map_3d_mode(
    map_3d_mode_raw: Optional[str],
    with_3d_map: bool = False,
    with_3d_mesh: bool = False,
) -> Tuple[Map3DMode, List[str]]:
    """Resolve mode from primary flag and deprecated aliases."""

    warnings: List[str] = []
    explicit_mode: Optional[Map3DMode] = None
    legacy_mode: Optional[Map3DMode] = None

    if map_3d_mode_raw is not None:
        explicit_mode = Map3DMode(str(map_3d_mode_raw).strip().lower())

    if with_3d_map or with_3d_mesh:
        warnings.append(
            "Deprecated flags detected (--with-3d-map/--with-3d-mesh). "
            "Use --map-3d-mode {off|pointcloud|mesh}."
        )
        if with_3d_map and with_3d_mesh:
            legacy_mode = Map3DMode.MESH
            warnings.append(
                "Both legacy 3D-map flags were set; selecting mesh mode by precedence."
            )
        elif with_3d_mesh:
            legacy_mode = Map3DMode.MESH
        elif with_3d_map:
            legacy_mode = Map3DMode.POINTCLOUD

    if explicit_mode is not None:
        if legacy_mode is not None and legacy_mode != explicit_mode:
            warnings.append(
                f"Ignoring legacy mode alias ({legacy_mode.value}) because "
                f"--map-3d-mode={explicit_mode.value} was set explicitly."
            )
        return explicit_mode, warnings

    if legacy_mode is not None:
        return legacy_mode, warnings

    return Map3DMode.OFF, warnings


def resolve_mesh_update_policy(policy_raw: Optional[str]) -> MeshUpdatePolicy:
    """Resolve and normalize mesh update policy string."""

    if policy_raw is None:
        return MeshUpdatePolicy.SNAPSHOT

    normalized = str(policy_raw).strip().lower()
    if normalized == MeshUpdatePolicy.PERIODIC.value:
        return MeshUpdatePolicy.PERIODIC
    if normalized == MeshUpdatePolicy.CONTINUOUS.value:
        return MeshUpdatePolicy.CONTINUOUS
    return MeshUpdatePolicy.SNAPSHOT


def resolve_converter_update_mode(
    update_policy: MeshUpdatePolicy,
    republish_interval: float,
) -> Tuple[str, float]:
    """Map SDK mesh policy to converter wire arguments."""

    policy = update_policy
    if policy == MeshUpdatePolicy.CONTINUOUS:
        return "continuous", 0.0

    if policy == MeshUpdatePolicy.PERIODIC:
        if republish_interval > 0.0:
            return "on_change", max(0.5, float(republish_interval))
        return "on_change", 2.0

    if republish_interval > 0.0:
        return "once", max(0.5, float(republish_interval))
    return "once", 0.0


def coerce_mesh_transport_to_marker(mesh_transport_raw: Optional[str]) -> Tuple[str, bool]:
    """Force marker transport and report whether a non-marker value was requested."""

    normalized = str(mesh_transport_raw or MeshTransport.MARKER.value).strip().lower()
    requested_marker_array = normalized == MeshTransport.MARKER_ARRAY.value
    return MeshTransport.MARKER.value, requested_marker_array


def build_fake_3d_map_publisher_command(
    python_executable: str,
    script_dir: str,
    topic: str,
    frame_id: str,
    detailed: bool = False,
) -> List[str]:
    script_name = (
        "fake_3d_map_publisher_realistic.py"
        if detailed
        else "fake_3d_map_publisher.py"
    )
    return [
        python_executable,
        os.path.join(script_dir, script_name),
        "--topic",
        str(topic),
        "--frame",
        str(frame_id),
    ]


def build_pointcloud_to_mesh_converter_command(
    python_executable: str,
    script_dir: str,
    cloud_topic: str,
    mesh_topic: str,
    mesh_array_topic: str = "/map_3d_mesh_array",
    mesh_transport: str = MeshTransport.MARKER.value,
    mesh_chunk_max_triangles: int = 3000,
    voxel_size: float = 0.10,
    max_voxels: int = 60000,
    max_triangles: int = 60000,
    update_policy: str = MeshUpdatePolicy.SNAPSHOT.value,
    republish_interval: float = 0.0,
) -> List[str]:
    policy = resolve_mesh_update_policy(update_policy)
    update_mode, resolved_republish = resolve_converter_update_mode(policy, republish_interval)
    transport, _ = coerce_mesh_transport_to_marker(mesh_transport)

    return [
        python_executable,
        os.path.join(script_dir, "pointcloud_to_voxel_mesh_marker.py"),
        "--cloud-topic",
        str(cloud_topic),
        "--mesh-topic",
        str(mesh_topic),
        "--mesh-array-topic",
        str(mesh_array_topic),
        "--mesh-transport",
        transport,
        "--chunk-max-triangles",
        str(max(256, int(mesh_chunk_max_triangles))),
        "--voxel-size",
        str(max(0.02, float(voxel_size))),
        "--max-voxels",
        str(max(0, int(max_voxels))),
        "--max-triangles",
        str(max(0, int(max_triangles))),
        "--update-policy",
        policy.value,
        "--update-mode",
        update_mode,
        "--on-change-republish-interval",
        str(max(0.0, float(resolved_republish))),
    ]


def build_map_3d_process_specs(
    mode: Map3DMode,
    python_executable: str,
    script_dir: str,
    map_3d_topic: str,
    map_3d_frame: str,
    map_3d_mesh_topic: str,
    map_3d_mesh_array_topic: str = "/map_3d_mesh_array",
    map_3d_mesh_transport: str = MeshTransport.MARKER.value,
    map_3d_mesh_chunk_max_triangles: int = 3000,
    map_3d_detailed: bool = False,
    mesh_voxel_size: float = 0.10,
    mesh_max_voxels: int = 60000,
    mesh_max_triangles: int = 60000,
    mesh_update_policy: str = MeshUpdatePolicy.SNAPSHOT.value,
    mesh_republish_interval: float = 0.0,
) -> List[ManagedProcessSpec]:
    """Build subprocess launch specs required for a selected mode."""

    if mode == Map3DMode.OFF:
        return []

    specs = [
        ManagedProcessSpec(
            name="fake_3d_map_publisher",
            command=build_fake_3d_map_publisher_command(
                python_executable=python_executable,
                script_dir=script_dir,
                topic=map_3d_topic,
                frame_id=map_3d_frame,
                detailed=map_3d_detailed,
            ),
            cwd=script_dir,
        )
    ]

    if mode == Map3DMode.MESH:
        specs.append(
            ManagedProcessSpec(
                name="pointcloud_to_voxel_mesh_marker",
                command=build_pointcloud_to_mesh_converter_command(
                    python_executable=python_executable,
                    script_dir=script_dir,
                    cloud_topic=map_3d_topic,
                    mesh_topic=map_3d_mesh_topic,
                    mesh_array_topic=map_3d_mesh_array_topic,
                    mesh_transport=map_3d_mesh_transport,
                    mesh_chunk_max_triangles=map_3d_mesh_chunk_max_triangles,
                    voxel_size=mesh_voxel_size,
                    max_voxels=mesh_max_voxels,
                    max_triangles=mesh_max_triangles,
                    update_policy=mesh_update_policy,
                    republish_interval=mesh_republish_interval,
                ),
                cwd=script_dir,
            )
        )

    return specs


def start_managed_processes(
    specs: Sequence[ManagedProcessSpec],
    log_fn: Optional[Callable[[str], None]] = None,
    warmup_seconds: float = 0.0,
) -> List[ManagedProcessHandle]:
    """Start subprocesses and return running handles."""

    handles: List[ManagedProcessHandle] = []
    for spec in specs:
        command_text = " ".join(shlex.quote(part) for part in spec.command)
        if log_fn is not None:
            log_fn(f"Starting {spec.name}: {command_text}")
        try:
            process = subprocess.Popen(  # nosec B603
                spec.command,
                cwd=spec.cwd,
                start_new_session=True,
            )
        except Exception:
            stop_managed_processes(handles, log_fn=log_fn)
            raise

        handles.append(
            ManagedProcessHandle(
                name=spec.name,
                command=list(spec.command),
                cwd=spec.cwd,
                process=process,
            )
        )

    if warmup_seconds > 0.0 and handles:
        time.sleep(float(warmup_seconds))
    return handles


def stop_managed_processes(
    handles: Sequence[ManagedProcessHandle],
    log_fn: Optional[Callable[[str], None]] = None,
    timeout_sec: float = 5.0,
) -> None:
    """Terminate subprocesses in reverse launch order."""

    for handle in reversed(list(handles)):
        process = handle.process
        if process is None or process.poll() is not None:
            continue

        if log_fn is not None:
            log_fn(f"Stopping {handle.name} (pid={process.pid})")

        _terminate_process(process)

        try:
            process.wait(timeout=max(0.1, float(timeout_sec)))
        except subprocess.TimeoutExpired:
            _kill_process(process)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass


def _terminate_process(process: subprocess.Popen) -> None:
    try:
        if os.name != "nt":
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        else:
            process.terminate()
    except Exception:
        try:
            process.terminate()
        except Exception:
            pass


def _kill_process(process: subprocess.Popen) -> None:
    try:
        if os.name != "nt":
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        else:
            process.kill()
    except Exception:
        try:
            process.kill()
        except Exception:
            pass
