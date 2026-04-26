#!/usr/bin/env python3
"""Register live Carter robots against the shared SLAM/Nav graph.

Expected live topics per robot by default:
- shared `/tf`
- shared `/tf_static`
- shared `/shared_map`
- `/<robot>/front_stereo_camera/left/image_raw/compressed`
- `/<robot>/front_stereo_camera/left/camera_info`
- `/<robot>/chassis/odom`
- `/<robot>/front_2d_lidar/scan`
- `/rviz/<robot>/plan`
- `/rviz/<robot>/local_plan`
- Nav2 actions under `/<robot>/navigate_to_pose` and `/<robot>/navigate_through_poses`

This demo:
1. Probes required live topics and samples first messages.
2. Uses the live compressed image topics directly.
3. Uses the shared `/tf` and `/tf_static` graph from `carter_multi_nav`.
4. Registers one shared occupancy grid from `/shared_map` in `map`.
5. Uses the RViz-relayed global and local plans from `global_odom`.
6. Bridges HORUS go-to / waypoint topics onto per-robot Nav2 actions.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
import urllib.request
import uuid
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Set, Tuple

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.join(SCRIPT_DIR, "..", "..")
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

try:
    from horus.dataviz import DataViz
    from horus.robot import Robot, RobotDimensions, RobotType, register_robots
    from horus.sensors import Camera, LaserScan
    from horus.utils import cli
except ImportError:
    print(f"Failed to import horus from {PACKAGE_ROOT}")
    raise


DEFAULT_ROBOT_NAMES = ("carter1", "carter2", "carter3")
DEFAULT_CAMERA_FPS = 20
DEFAULT_TF_TOPIC = "/tf"
DEFAULT_TF_STATIC_TOPIC = "/tf_static"
DEFAULT_SHARED_MAP_TOPIC = "/shared_map"
DEFAULT_SHARED_MAP_FRAME = "map"
DEFAULT_RVIZ_ROOT_FRAME = "global_odom"
DEFAULT_APRILTAG_DETECTIONS_TOPIC = "/april_tag/detections_by_robot"
DEFAULT_APRILTAG_SEMANTIC_BOX_SIZE = (0.6, 0.6, 1.7)
DEFAULT_APRILTAG_SCENE_PROFILE = "hospital"
DEFAULT_APRILTAG_WORLD_POSITIONS_BY_SCENE: Dict[str, Dict[int, Tuple[float, float, float]]] = {
    "hospital": {
        1: (-47.780966508350595, 9.253047326267392, 0.5390588048650494),
        2: (-47.825433425657415, 31.31671386718752, 0.5325260213106301),
        3: (12.045346704726485, 26.44529965786674, 0.5412456866482279),
        4: (26.016314965415788, 20.264474470321122, 0.5320971696497737),
        5: (12.68741541247874, 3.5158409870380307, 0.5457641816876574),
        6: (22.188201333573637, 2.261410239786365, 0.5213475846469872),
        7: (25.0451904296875, -1.961320386525618, 0.5220328481793512),
        8: (-14.32625071502537, 14.311163792197549, 0.6505159942022498),
        9: (-13.512517256229335, 7.811550875800454, 0.5511879084272279),
        10: (-2.632030628617786, -0.6653609672384561, 0.7186511932485986),
    },
    "office": {
        1: (4.910504642109061, 30.97664047071059, 0.4973964158136842),
        2: (0.5827527406821116, 34.748954086202545, 0.32859851446801663),
        3: (1.5865299526820311, 46.770076227205365, 0.3151346876367264),
        4: (0.8464887157491048, 49.28514813433961, 0.5228103315535189),
        5: (-21.831578705298394, 55.754486975546506, 0.41116598687485983),
        6: (0.795903598816899, 13.269574971827057, 0.5836132810783681),
        7: (-20.556490073819877, -11.162325696927983, 0.32975355322420963),
        8: (-7.1000001057982445, -25.5, 0.4000000059604645),
        9: (-20.500000000000004, 31.5, 1.0),
        10: (-12.1118798828125, 49.510115966796874, 0.3),
    },
}
DEFAULT_APRILTAG_WORLD_POSITIONS = DEFAULT_APRILTAG_WORLD_POSITIONS_BY_SCENE[
    DEFAULT_APRILTAG_SCENE_PROFILE
]
GLOBAL_ROOT_FRAMES = {"map", "world", DEFAULT_RVIZ_ROOT_FRAME}
NOVA_CARTER_REPO_URL = "https://github.com/NVIDIA-ISAAC-ROS/nova_carter.git"
NOVA_CARTER_MEDIA_BASE_URL = "https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/nova_carter/main"
NOVA_CARTER_CACHE_ROOT = os.path.expanduser("~/.cache/horus/robot_description_sources/nova_carter")
NOVA_CARTER_PACKAGE_DIR = os.path.join(NOVA_CARTER_CACHE_ROOT, "nova_carter_description")
NOVA_CARTER_REQUIRED_MESHES = (
    "chassis_link.obj",
    "caster_frame_base.obj",
    "nova_carter_wheel_left.obj",
    "nova_carter_wheel_right.obj",
    "caster_swivel.obj",
    "caster_wheel.obj",
)
NOVA_CARTER_STATIC_SHELL_URDF = os.path.join(
    NOVA_CARTER_PACKAGE_DIR,
    "urdf",
    "horus_nova_carter_static_shell.urdf",
)
_NOVA_CARTER_SOURCE_READY = False

DEFAULT_CARTER_BODY_MESH_MODE = "runtime_high_mesh"
DEFAULT_CARTER_PREVIEW_TRIANGLE_BUDGET = 90000
DEFAULT_CARTER_RUNTIME_HIGH_TRIANGLE_BUDGET = 240000

NOVA_CARTER_STATIC_SHELL_URDF_XML = """<?xml version="1.0"?>
<robot name="nova_carter_static_shell">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://nova_carter_description/meshes/chassis_link.obj"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://nova_carter_description/meshes/chassis_link.obj"/>
      </geometry>
    </collision>
  </link>
  <link name="nova_carter_caster_frame_base">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://nova_carter_description/meshes/caster_frame_base.obj"/>
      </geometry>
    </visual>
  </link>
  <link name="wheel_left">
    <visual>
      <origin xyz="0 0 0.034" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://nova_carter_description/meshes/nova_carter_wheel_left.obj"/>
      </geometry>
    </visual>
  </link>
  <link name="wheel_right">
    <visual>
      <origin xyz="0 0 -0.034" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://nova_carter_description/meshes/nova_carter_wheel_right.obj"/>
      </geometry>
    </visual>
  </link>
  <link name="caster_swivel_left">
    <visual>
      <origin xyz="-0.000002 0.000004 -0.000002" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://nova_carter_description/meshes/caster_swivel.obj"/>
      </geometry>
    </visual>
  </link>
  <link name="caster_swivel_right">
    <visual>
      <origin xyz="-0.0000021 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://nova_carter_description/meshes/caster_swivel.obj"/>
      </geometry>
    </visual>
  </link>
  <link name="caster_wheel_left">
    <visual>
      <origin xyz="0 -0.000005 -0.023039" rpy="3.141592653589793 0 0"/>
      <geometry>
        <mesh filename="package://nova_carter_description/meshes/caster_wheel.obj"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0.0000001 -0.0000045 -0.0230392" rpy="3.141592653589793 0 0"/>
      <geometry>
        <cylinder radius="0.0755" length="0.025"/>
      </geometry>
    </collision>
  </link>
  <link name="caster_wheel_right">
    <visual>
      <origin xyz="0 -0.000005 -0.0230391" rpy="3.141592653589793 0 0"/>
      <geometry>
        <mesh filename="package://nova_carter_description/meshes/caster_wheel.obj"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 -0.0000046 -0.0230391" rpy="3.141592653589793 0 0"/>
      <geometry>
        <cylinder radius="0.0755" length="0.025"/>
      </geometry>
    </collision>
  </link>
  <joint name="joint_wheel_left" type="continuous">
    <origin xyz="0 0.1726 0.14" rpy="-1.5707963267948966 0 0"/>
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="joint_wheel_right" type="continuous">
    <origin xyz="0 -0.1726 0.14" rpy="-1.5707963267948966 0 0"/>
    <parent link="base_link"/>
    <child link="wheel_right"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="joint_caster_base" type="revolute">
    <origin xyz="-0.47195 0 0.22289" rpy="-1.5707963267948966 0 1.5707963267948966"/>
    <parent link="base_link"/>
    <child link="nova_carter_caster_frame_base"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="joint_swing_left" type="continuous">
    <origin xyz="0.132296 0.031188 0.000002" rpy="0 0 -1.5707963267948966"/>
    <parent link="nova_carter_caster_frame_base"/>
    <child link="caster_swivel_left"/>
    <axis xyz="1 0 0"/>
  </joint>
  <joint name="joint_swing_right" type="continuous">
    <origin xyz="-0.1322037 0.0311879 0.0000001" rpy="0 0 -1.5707963267948966"/>
    <parent link="nova_carter_caster_frame_base"/>
    <child link="caster_swivel_right"/>
    <axis xyz="1 0 0"/>
  </joint>
  <joint name="joint_caster_left" type="continuous">
    <origin xyz="-0.1153999 -0.0230851 0.0399971" rpy="1.5707963267948966 -1.5707963267948966 0"/>
    <parent link="caster_swivel_left"/>
    <child link="caster_wheel_left"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="joint_caster_right" type="continuous">
    <origin xyz="-0.1154 -0.022989 0.039999" rpy="1.5707963267948966 -1.5707963267948966 0"/>
    <parent link="caster_swivel_right"/>
    <child link="caster_wheel_right"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
"""


@dataclass(frozen=True)
class CameraProbeResult:
    topic: str
    frame_id: str
    width: int
    height: int
    encoding: str


@dataclass(frozen=True)
class CameraInfoProbeResult:
    topic: str
    frame_id: str
    width: int
    height: int


@dataclass(frozen=True)
class OdomProbeResult:
    topic: str
    frame_id: str
    child_frame_id: str


@dataclass(frozen=True)
class ScanProbeResult:
    topic: str
    frame_id: str
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float


@dataclass(frozen=True)
class RobotProbeResult:
    robot_name: str
    tf_topic: str
    cmd_vel_topic: str
    goal_topic: str
    goal_cancel_topic: str
    goal_status_topic: str
    waypoint_path_topic: str
    waypoint_status_topic: str
    global_path_topic: str
    controller_path_topic: str
    camera_raw_topic: str
    camera_compressed_topic: str
    camera_frame: str
    camera: CameraProbeResult
    odom: OdomProbeResult
    scan: Optional[ScanProbeResult]


@dataclass(frozen=True)
class NavGraphProbeResult:
    shared_tf_topic: str
    shared_tf_static_topic: str
    shared_map_topic: str
    probes: List[RobotProbeResult]


def _tf_source_topic(robot_name: str) -> str:
    return f"/{robot_name}/tf"


def _camera_raw_topic(robot_name: str) -> str:
    return f"/{robot_name}/front_stereo_camera/left/image_raw"


def _camera_compressed_topic(robot_name: str) -> str:
    return f"/{robot_name}/front_stereo_camera/left/image_raw/compressed"


def _camera_info_topic(robot_name: str) -> str:
    return f"/{robot_name}/front_stereo_camera/left/camera_info"


def _odom_topic(robot_name: str) -> str:
    return f"/{robot_name}/chassis/odom"


def _scan_topic(robot_name: str) -> str:
    return f"/{robot_name}/front_2d_lidar/scan"


def _cmd_vel_topic(robot_name: str) -> str:
    return f"/{robot_name}/cmd_vel"


def _goal_pose_topic(robot_name: str) -> str:
    return f"/{robot_name}/goal_pose"


def _goal_cancel_topic(robot_name: str) -> str:
    return f"/{robot_name}/goal_cancel"


def _goal_status_topic(robot_name: str) -> str:
    return f"/{robot_name}/goal_status"


def _waypoint_path_topic(robot_name: str) -> str:
    return f"/{robot_name}/waypoint_path"


def _waypoint_status_topic(robot_name: str) -> str:
    return f"/{robot_name}/waypoint_status"


def _rviz_global_plan_topic(robot_name: str) -> str:
    return f"/rviz/{robot_name}/plan"


def _rviz_local_plan_topic(robot_name: str) -> str:
    return f"/rviz/{robot_name}/local_plan"


def _normalize_transport(value: str, default: str) -> str:
    normalized = str(value or "").strip().lower()
    if normalized in ("ros", "webrtc"):
        return normalized
    return default


def _parse_robot_names(raw_value: str) -> List[str]:
    tokens = [token.strip() for token in str(raw_value or "").split(",")]
    names: List[str] = []
    for token in tokens:
        if token and token not in names:
            names.append(token)
    return names or list(DEFAULT_ROBOT_NAMES)


def _normalize_body_mesh_mode(raw_value: Optional[str]) -> str:
    normalized = str(raw_value or DEFAULT_CARTER_BODY_MESH_MODE).strip().lower()
    if normalized == "max_quality_mesh":
        return "runtime_high_mesh"
    if normalized in {"collision_only", "preview_mesh", "runtime_high_mesh"}:
        return normalized
    return DEFAULT_CARTER_BODY_MESH_MODE


def _nova_carter_media_path_needs_download(path: str) -> bool:
    if not os.path.isfile(path):
        return True
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as handle:
            first_line = handle.readline().strip()
    except OSError:
        return True
    return first_line.startswith("version https://git-lfs.github.com/spec/v1")


def _download_nova_carter_media_file(relative_path: str) -> str:
    normalized_relative = str(relative_path or "").strip().lstrip("/")
    if not normalized_relative:
        raise RuntimeError("Invalid Nova Carter media path.")
    local_path = os.path.join(NOVA_CARTER_CACHE_ROOT, normalized_relative)
    os.makedirs(os.path.dirname(local_path), exist_ok=True)
    if _nova_carter_media_path_needs_download(local_path):
        cli.print_step(f"Downloading Nova Carter asset '{normalized_relative}'...")
        asset_url = f"{NOVA_CARTER_MEDIA_BASE_URL}/{normalized_relative}"
        request = urllib.request.Request(asset_url, headers={"User-Agent": "horus-sdk/robot-description"})
        with urllib.request.urlopen(request, timeout=240.0) as response, open(local_path, "wb") as output:
            output.write(response.read())
    return local_path


def _extract_obj_mtllib_names(mesh_path: str) -> List[str]:
    names: List[str] = []
    try:
        with open(mesh_path, "r", encoding="utf-8", errors="ignore") as handle:
            for raw_line in handle:
                line = raw_line.strip()
                if line.startswith("mtllib "):
                    candidate = line.split(maxsplit=1)[1].strip()
                    if candidate and candidate not in names:
                        names.append(candidate)
    except OSError:
        return []
    return names


def _ensure_nova_carter_mesh_with_sidecars(mesh_name: str) -> str:
    mesh_relative = f"nova_carter_description/meshes/{mesh_name}"
    mesh_path = _download_nova_carter_media_file(mesh_relative)
    for mtllib_name in _extract_obj_mtllib_names(mesh_path):
        _download_nova_carter_media_file(f"nova_carter_description/meshes/{mtllib_name}")
    return mesh_path


def ensure_nova_carter_description_source() -> str:
    global _NOVA_CARTER_SOURCE_READY
    if _NOVA_CARTER_SOURCE_READY and os.path.isfile(NOVA_CARTER_STATIC_SHELL_URDF):
        return NOVA_CARTER_STATIC_SHELL_URDF

    package_dir = NOVA_CARTER_PACKAGE_DIR
    if not os.path.isdir(package_dir):
        os.makedirs(os.path.dirname(package_dir), exist_ok=True)
        cli.print_step("Fetching official Nova Carter description package...")
        clone_cmd = [
            "git",
            "clone",
            "--filter=blob:none",
            "--no-checkout",
            "--depth",
            "1",
            NOVA_CARTER_REPO_URL,
            NOVA_CARTER_CACHE_ROOT,
        ]
        clone_result = subprocess.run(clone_cmd, check=False, capture_output=True, text=True, timeout=240.0)
        if clone_result.returncode != 0:
            raise RuntimeError(
                f"Failed to clone nova_carter repo: {(clone_result.stderr or clone_result.stdout or '').strip()}"
            )
        sparse_init = subprocess.run(
            ["git", "-C", NOVA_CARTER_CACHE_ROOT, "sparse-checkout", "init", "--cone"],
            check=False,
            capture_output=True,
            text=True,
            timeout=30.0,
        )
        if sparse_init.returncode != 0:
            raise RuntimeError(
                f"Failed to initialize sparse checkout: {(sparse_init.stderr or sparse_init.stdout or '').strip()}"
            )
        sparse_set = subprocess.run(
            ["git", "-C", NOVA_CARTER_CACHE_ROOT, "sparse-checkout", "set", "nova_carter_description"],
            check=False,
            capture_output=True,
            text=True,
            timeout=30.0,
        )
        if sparse_set.returncode != 0:
            raise RuntimeError(
                f"Failed to set sparse checkout: {(sparse_set.stderr or sparse_set.stdout or '').strip()}"
            )
        checkout = subprocess.run(
            ["git", "-C", NOVA_CARTER_CACHE_ROOT, "checkout"],
            check=False,
            capture_output=True,
            text=True,
            timeout=120.0,
        )
        if checkout.returncode != 0:
            raise RuntimeError(
                f"Failed to checkout nova_carter_description: {(checkout.stderr or checkout.stdout or '').strip()}"
            )

    meshes_dir = os.path.join(package_dir, "meshes")
    os.makedirs(meshes_dir, exist_ok=True)
    for mesh_name in NOVA_CARTER_REQUIRED_MESHES:
        _ensure_nova_carter_mesh_with_sidecars(mesh_name)

    if not os.path.isfile(NOVA_CARTER_STATIC_SHELL_URDF):
        os.makedirs(os.path.dirname(NOVA_CARTER_STATIC_SHELL_URDF), exist_ok=True)
        with open(NOVA_CARTER_STATIC_SHELL_URDF, "w", encoding="utf-8") as handle:
            handle.write(NOVA_CARTER_STATIC_SHELL_URDF_XML)

    _NOVA_CARTER_SOURCE_READY = True
    return NOVA_CARTER_STATIC_SHELL_URDF


def warm_nova_carter_robot_description_artifact(
    body_mesh_mode_override: Optional[str] = None,
    visual_mesh_triangle_budget_override: Optional[int] = None,
) -> Tuple[str, Dict[str, int | str]]:
    description_urdf_path = ensure_nova_carter_description_source()

    from horus.description.robot_description_resolver import RobotDescriptionResolver

    warm_robot = Robot(name="nova_carter_warmup", robot_type=RobotType.WHEELED)
    body_mesh_mode, visual_mesh_triangle_budget = resolve_carter_body_mesh_settings(
        body_mesh_mode_override=body_mesh_mode_override,
        visual_mesh_triangle_budget_override=visual_mesh_triangle_budget_override,
    )
    warm_robot.configure_robot_description(
        urdf_path=description_urdf_path,
        base_frame="base_link",
        include_visual_meshes=True,
        visual_mesh_triangle_budget=visual_mesh_triangle_budget,
        body_mesh_mode=body_mesh_mode,
        chunk_size_bytes=48000,
    )

    resolver = RobotDescriptionResolver()
    artifact = resolver.resolve_for_robot(warm_robot)
    if artifact is None:
        raise RuntimeError(
            f"Failed preparing Nova Carter robot description artifact: {resolver.last_error or 'unknown error'}"
        )

    payload = artifact.payload_dict or {}
    mesh_assets = payload.get("mesh_assets") or []
    total_triangles = sum(int(asset.get("triangle_count") or 0) for asset in mesh_assets)
    stats = {
        "body_mesh_mode": body_mesh_mode,
        "triangle_budget": int(visual_mesh_triangle_budget),
        "triangle_count": int(total_triangles),
        "mesh_asset_count": int(len(mesh_assets)),
        "chunk_count": int(len(getattr(artifact, "chunks", []) or [])),
        "encoded_bytes": int(len(str(getattr(artifact, "encoded_payload", "") or ""))),
        "cache_status": str(getattr(resolver, "last_resolution_cache_status", "miss") or "miss"),
    }

    if body_mesh_mode == "runtime_high_mesh":
        allowed = max(int(math.ceil(visual_mesh_triangle_budget * 1.05)), int(visual_mesh_triangle_budget) + 5000)
        if total_triangles > allowed:
            raise RuntimeError(
                f"Runtime-high mesh artifact exceeds Quest-safe cap: triangles={total_triangles} budget={visual_mesh_triangle_budget}."
            )

    return description_urdf_path, stats


def resolve_carter_body_mesh_settings(
    body_mesh_mode_override: Optional[str] = None,
    visual_mesh_triangle_budget_override: Optional[int] = None,
) -> Tuple[str, int]:
    raw_mode = str(
        body_mesh_mode_override
        or os.environ.get("HORUS_CARTER_BODY_MESH_MODE", DEFAULT_CARTER_BODY_MESH_MODE)
        or DEFAULT_CARTER_BODY_MESH_MODE
    ).strip().lower()
    if raw_mode == "max_quality_mesh":
        cli.print_warning(
            "body_mesh_mode='max_quality_mesh' is deprecated; using 'runtime_high_mesh' instead."
        )
    if raw_mode not in {"collision_only", "preview_mesh", "runtime_high_mesh", "max_quality_mesh"}:
        raw_mode = DEFAULT_CARTER_BODY_MESH_MODE
    raw_mode = _normalize_body_mesh_mode(raw_mode)

    default_budget = (
        DEFAULT_CARTER_PREVIEW_TRIANGLE_BUDGET
        if raw_mode == "preview_mesh"
        else DEFAULT_CARTER_RUNTIME_HIGH_TRIANGLE_BUDGET
    )
    raw_budget = (
        visual_mesh_triangle_budget_override
        if visual_mesh_triangle_budget_override is not None
        else os.environ.get("HORUS_CARTER_VISUAL_MESH_TRIANGLE_BUDGET", default_budget)
    )
    try:
        triangle_budget = int(raw_budget)
    except (TypeError, ValueError):
        triangle_budget = default_budget

    triangle_budget = max(2000, min(500000, triangle_budget))
    return raw_mode, triangle_budget


def _topic_types_by_name(node) -> Dict[str, Tuple[str, ...]]:
    return {
        str(topic): tuple(str(item) for item in types)
        for topic, types in node.get_topic_names_and_types()
    }


def _topic_types_from_ros2_cli(timeout_sec: float) -> Dict[str, Tuple[str, ...]]:
    import subprocess

    try:
        result = subprocess.run(
            ["ros2", "topic", "list", "-t"],
            check=False,
            capture_output=True,
            text=True,
            timeout=max(1.0, float(timeout_sec)),
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return {}

    if result.returncode != 0:
        return {}

    topic_types: Dict[str, Tuple[str, ...]] = {}
    for raw_line in result.stdout.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if " [" not in line or not line.endswith("]"):
            topic_types[line] = tuple()
            continue
        topic_name, type_suffix = line.split(" [", 1)
        type_tokens = [
            token.strip() for token in type_suffix[:-1].split(",") if token.strip()
        ]
        topic_types[topic_name.strip()] = tuple(type_tokens)
    return topic_types


def _missing_or_wrong_topics(
    topic_types: Dict[str, Tuple[str, ...]],
    expected: Dict[str, str],
) -> List[str]:
    problems: List[str] = []
    for topic_name, expected_type in expected.items():
        available_types = topic_types.get(topic_name)
        if not available_types:
            problems.append(f"missing topic {topic_name}")
            continue
        if expected_type not in available_types:
            joined_types = ", ".join(available_types)
            problems.append(
                f"topic {topic_name} has type(s) [{joined_types}] but expected {expected_type}"
            )
    return problems


def prefix_frame_id(frame_id: str, robot_name: str) -> str:
    normalized_frame = str(frame_id or "").strip().lstrip("/")
    normalized_robot = str(robot_name or "").strip().strip("/")
    if not normalized_frame:
        return ""
    if normalized_frame in GLOBAL_ROOT_FRAMES:
        return normalized_frame
    if not normalized_robot:
        return normalized_frame
    if normalized_frame.startswith(f"{normalized_robot}/"):
        return normalized_frame
    return f"{normalized_robot}/{normalized_frame}"


def _normalize_apriltag_scene_profile(scene_profile: str) -> str:
    normalized = str(scene_profile or "").strip().lower()
    if normalized in DEFAULT_APRILTAG_WORLD_POSITIONS_BY_SCENE:
        return normalized
    return DEFAULT_APRILTAG_SCENE_PROFILE


def _semantic_box_center_for_tag(
    tag_id: int,
    scene_profile: str = DEFAULT_APRILTAG_SCENE_PROFILE,
) -> Optional[Tuple[float, float, float]]:
    world_positions = DEFAULT_APRILTAG_WORLD_POSITIONS_BY_SCENE[
        _normalize_apriltag_scene_profile(scene_profile)
    ]
    center = world_positions.get(int(tag_id))
    if center is None:
        return None
    return (float(center[0]), float(center[1]), float(center[2]))


def _apriltag_person_semantic_id(tag_id: int) -> str:
    return f"apriltag_person_{int(tag_id)}"


class AprilTagSemanticOverlayBridge:
    """Promote AprilTag detections into HORUS semantic boxes."""

    def __init__(
        self,
        probes: Sequence[RobotProbeResult],
        robots: Sequence[Robot],
        datavizs: Sequence[DataViz],
        workspace_scale: float,
        detections_topic: str = DEFAULT_APRILTAG_DETECTIONS_TOPIC,
        scene_profile: str = DEFAULT_APRILTAG_SCENE_PROFILE,
    ) -> None:
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        try:
            from carter_multi_nav_msgs.msg import RobotTagDetectionsArray
        except ImportError as exc:
            raise RuntimeError(
                "AprilTag semantic labeling requires 'carter_multi_nav_msgs'. "
                "Source the carter_multi_nav workspace before running this demo."
            ) from exc

        if not robots or not datavizs or len(robots) != len(datavizs):
            raise RuntimeError(
                "AprilTag semantic labeling requires matching live robots and DataViz instances."
            )

        self._string_type = String
        self._workspace_scale = float(workspace_scale)
        self._robots = list(robots)
        self._datavizs = list(datavizs)
        self._dataviz_anchor = self._datavizs[0]
        self._scene_profile = _normalize_apriltag_scene_profile(scene_profile)
        self._target_robot_names: Set[str] = {probe.robot_name for probe in probes}
        self._registration_ready = False
        self._pending_publish = False
        self._seen_tag_ids: Set[int] = set()
        self._acked_robot_names: Set[str] = set()
        self._lock = threading.Lock()

        node_name = f"horus_apriltag_semantic_overlay_{uuid.uuid4().hex[:8]}"
        self.node = rclpy.create_node(node_name)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._subscriptions = [
            self.node.create_subscription(
                RobotTagDetectionsArray,
                str(detections_topic or "").strip() or DEFAULT_APRILTAG_DETECTIONS_TOPIC,
                self._handle_detections,
                sensor_qos,
            ),
            self.node.create_subscription(
                String,
                "/horus/registration_ack",
                self._handle_registration_ack,
                status_qos,
            ),
        ]

    def destroy(self) -> None:
        for subscription in self._subscriptions:
            try:
                self.node.destroy_subscription(subscription)
            except Exception:
                pass
        try:
            self.node.destroy_node()
        except Exception:
            pass

    def _handle_registration_ack(self, msg) -> None:
        try:
            ack = json.loads(str(msg.data or ""))
        except Exception:
            return

        robot_name = str(ack.get("robot_name", "") or "").strip()
        if robot_name not in self._target_robot_names or not bool(ack.get("success")):
            return

        should_publish = False
        with self._lock:
            self._acked_robot_names.add(robot_name)
            if not self._registration_ready and self._acked_robot_names >= self._target_robot_names:
                self._registration_ready = True
                should_publish = self._pending_publish

        if should_publish:
            self._publish_registration_update("initial AprilTag semantic overlays")

    def _handle_detections(self, msg) -> None:
        added_tags: List[Tuple[int, str, Tuple[float, float, float]]] = []
        should_publish = False

        with self._lock:
            detections = list(getattr(msg, "detections", []) or [])
            for detection in detections:
                robot_name = str(getattr(detection, "robot_name", "") or "").strip()
                if robot_name not in self._target_robot_names:
                    continue

                for raw_tag_id in list(getattr(detection, "tag_ids", []) or []):
                    tag_id = int(raw_tag_id)
                    if tag_id in self._seen_tag_ids:
                        continue

                    center = _semantic_box_center_for_tag(tag_id, self._scene_profile)
                    if center is None:
                        continue
                    self._dataviz_anchor.add_semantic_box(
                        semantic_id=_apriltag_person_semantic_id(tag_id),
                        label=f"person: {tag_id}",
                        center=center,
                        size=DEFAULT_APRILTAG_SEMANTIC_BOX_SIZE,
                        frame_id=DEFAULT_SHARED_MAP_FRAME,
                        rotation_offset_euler=(0.0, 0.0, 0.0),
                    )
                    self._seen_tag_ids.add(tag_id)
                    added_tags.append((tag_id, robot_name, center))

            if added_tags:
                if self._registration_ready:
                    should_publish = True
                else:
                    self._pending_publish = True

        for tag_id, robot_name, center in added_tags:
            self.node.get_logger().info(
                "Queued semantic box for AprilTag %d from %s at map center=(%.2f, %.2f, %.2f)"
                % (tag_id, robot_name, center[0], center[1], center[2])
            )

        if should_publish:
            tag_list = ", ".join(str(tag_id) for tag_id, _, _ in added_tags)
            self._publish_registration_update(f"new AprilTag detections: {tag_list}")

    def _publish_registration_update(self, reason: str) -> None:
        from horus.bridge.robot_registry import get_robot_registry_client

        client = get_robot_registry_client()
        publisher = getattr(client, "publisher", None)
        if publisher is None:
            with self._lock:
                self._pending_publish = True
            self.node.get_logger().warn(
                "Skipping AprilTag semantic registration update; HORUS registration publisher is not ready yet."
            )
            return

        try:
            with self._lock:
                global_visualizations = client._build_global_visualizations_payload(self._datavizs)
                payloads = []
                for robot, dataviz in zip(self._robots, self._datavizs):
                    config = client._build_robot_config_dict(
                        robot,
                        dataviz,
                        global_visualizations=global_visualizations,
                        workspace_scale=self._workspace_scale,
                    )
                    payloads.append(json.dumps(config))

            for payload in payloads:
                reg_msg = self._string_type()
                reg_msg.data = payload
                publisher.publish(reg_msg)

            with self._lock:
                self._pending_publish = False

            self.node.get_logger().info(
                "Published updated HORUS registrations for AprilTag semantic overlays (%s)."
                % reason
            )
        except Exception as exc:
            with self._lock:
                self._pending_publish = True
            self.node.get_logger().warn(
                "Failed to publish AprilTag semantic registration update: %s" % exc
            )


class AprilTagSemanticOverlayRunner:
    """Run the AprilTag semantic overlay bridge on a background executor thread."""

    def __init__(self, bridge: AprilTagSemanticOverlayBridge) -> None:
        from rclpy.executors import SingleThreadedExecutor

        self.bridge = bridge
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.bridge.node)
        self._stop_requested = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_requested.clear()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        import rclpy

        while not self._stop_requested.is_set() and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)

    def stop(self) -> None:
        self._stop_requested.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        try:
            self.executor.remove_node(self.bridge.node)
        except Exception:
            pass
        self.bridge.destroy()
        try:
            self.executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass


def resolve_camera_frame_id(
    robot_name: str,
    camera_header_frame: str,
    available_tf_frames: Sequence[str],
) -> str:
    header_frame = str(camera_header_frame or "").strip().lstrip("/")
    if header_frame:
        if header_frame.startswith(f"{robot_name}/"):
            return header_frame
        return prefix_frame_id(header_frame, robot_name)

    normalized_frames = set()
    normalized_robot = str(robot_name or "").strip().strip("/")
    for frame in available_tf_frames:
        value = str(frame).strip().lstrip("/")
        if not value:
            continue
        normalized_frames.add(value)
        if normalized_robot and value.startswith(f"{normalized_robot}/"):
            normalized_frames.add(value[len(normalized_robot) + 1 :])
    for candidate in ("front_stereo_camera_left_rgb", "front_stereo_camera_left_optical", "base_link"):
        if candidate in normalized_frames:
            return prefix_frame_id(candidate, robot_name)

    return prefix_frame_id("base_link", robot_name)


def probe_live_nav_graph(
    robot_names: Sequence[str],
    shared_tf_topic: str,
    shared_tf_static_topic: str,
    shared_map_topic: str,
    include_scan: bool,
    timeout_sec: float,
) -> NavGraphProbeResult:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
    from tf2_msgs.msg import TFMessage

    timeout_sec = max(0.5, float(timeout_sec))
    expected_types: Dict[str, str] = {}
    expected_types[shared_tf_topic] = "tf2_msgs/msg/TFMessage"
    expected_types[shared_tf_static_topic] = "tf2_msgs/msg/TFMessage"
    expected_types[shared_map_topic] = "nav_msgs/msg/OccupancyGrid"

    for robot_name in robot_names:
        expected_types[_camera_compressed_topic(robot_name)] = "sensor_msgs/msg/CompressedImage"
        expected_types[_camera_info_topic(robot_name)] = "sensor_msgs/msg/CameraInfo"
        expected_types[_odom_topic(robot_name)] = "nav_msgs/msg/Odometry"
        expected_types[_rviz_global_plan_topic(robot_name)] = "nav_msgs/msg/Path"
        expected_types[_rviz_local_plan_topic(robot_name)] = "nav_msgs/msg/Path"
        if include_scan:
            expected_types[_scan_topic(robot_name)] = "sensor_msgs/msg/LaserScan"

    node_name = f"horus_hospital_probe_{uuid.uuid4().hex[:8]}"
    node = rclpy.create_node(node_name)

    camera_results: Dict[str, Optional[CameraProbeResult]] = {
        name: None for name in robot_names
    }
    camera_info_results: Dict[str, Optional[CameraInfoProbeResult]] = {
        name: None for name in robot_names
    }
    odom_results: Dict[str, Optional[OdomProbeResult]] = {name: None for name in robot_names}
    scan_results: Dict[str, Optional[ScanProbeResult]] = {name: None for name in robot_names}
    tf_frames: Dict[str, set[str]] = {name: set() for name in robot_names}
    tf_seen: Dict[str, bool] = {name: False for name in robot_names}
    subscriptions = []

    def make_camera_callback(name: str):
        def _callback(msg: CompressedImage):
            if camera_results[name] is not None and camera_info_results[name] is not None:
                return
            camera_results[name] = CameraProbeResult(
                topic=_camera_compressed_topic(name),
                frame_id=str(msg.header.frame_id or "").strip(),
                width=int(max(1, camera_info_results[name].width if camera_info_results[name] else 1)),
                height=int(max(1, camera_info_results[name].height if camera_info_results[name] else 1)),
                encoding="jpeg",
            )

        return _callback

    def make_camera_info_callback(name: str):
        def _callback(msg: CameraInfo):
            if camera_info_results[name] is not None:
                return
            camera_info_results[name] = CameraInfoProbeResult(
                topic=_camera_info_topic(name),
                frame_id=str(msg.header.frame_id or "").strip(),
                width=int(max(1, msg.width)),
                height=int(max(1, msg.height)),
            )
            if camera_results[name] is not None:
                camera_results[name] = CameraProbeResult(
                    topic=camera_results[name].topic,
                    frame_id=camera_results[name].frame_id or camera_info_results[name].frame_id,
                    width=camera_info_results[name].width,
                    height=camera_info_results[name].height,
                    encoding=camera_results[name].encoding,
                )

        return _callback

    def make_odom_callback(name: str):
        def _callback(msg: Odometry):
            if odom_results[name] is not None:
                return
            odom_results[name] = OdomProbeResult(
                topic=_odom_topic(name),
                frame_id=str(msg.header.frame_id or "").strip(),
                child_frame_id=str(msg.child_frame_id or "").strip(),
            )

        return _callback

    def make_scan_callback(name: str):
        def _callback(msg: LaserScan):
            if scan_results[name] is not None:
                return
            scan_results[name] = ScanProbeResult(
                topic=_scan_topic(name),
                frame_id=str(msg.header.frame_id or "").strip(),
                angle_min=float(msg.angle_min),
                angle_max=float(msg.angle_max),
                angle_increment=float(msg.angle_increment),
                range_min=float(msg.range_min),
                range_max=float(msg.range_max),
            )

        return _callback

    def _record_tf_frames(msg: TFMessage):
        transforms = list(getattr(msg, "transforms", []))
        if not transforms:
            return
        for transform in transforms:
            parent_frame = str(transform.header.frame_id or "").strip().lstrip("/")
            child_frame = str(transform.child_frame_id or "").strip().lstrip("/")
            for frame in (parent_frame, child_frame):
                if not frame:
                    continue
                for name in robot_names:
                    prefix = f"{name}/"
                    if frame.startswith(prefix):
                        tf_frames[name].add(frame)
                        tf_seen[name] = True

    def _dynamic_tf_callback(msg: TFMessage):
        _record_tf_frames(msg)

    def _static_tf_callback(msg: TFMessage):
        _record_tf_frames(msg)

    try:
        for robot_name in robot_names:
            subscriptions.append(
                node.create_subscription(
                    CompressedImage,
                    _camera_compressed_topic(robot_name),
                    make_camera_callback(robot_name),
                    10,
                )
            )
            subscriptions.append(
                node.create_subscription(
                    CameraInfo,
                    _camera_info_topic(robot_name),
                    make_camera_info_callback(robot_name),
                    10,
                )
            )
            subscriptions.append(
                node.create_subscription(
                    Odometry,
                    _odom_topic(robot_name),
                    make_odom_callback(robot_name),
                    10,
                )
            )
            if include_scan:
                subscriptions.append(
                    node.create_subscription(
                        LaserScan,
                        _scan_topic(robot_name),
                        make_scan_callback(robot_name),
                        10,
                    )
                )

        dynamic_tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscriptions.append(
            node.create_subscription(
                TFMessage,
                shared_tf_topic,
                _dynamic_tf_callback,
                dynamic_tf_qos,
            )
        )
        subscriptions.append(
            node.create_subscription(
                TFMessage,
                shared_tf_static_topic,
                _static_tf_callback,
                static_tf_qos,
            )
        )

        last_topic_types = _topic_types_from_ros2_cli(timeout_sec=min(2.0, timeout_sec))
        last_topic_types.update(_topic_types_by_name(node))

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            topic_problems = _missing_or_wrong_topics(last_topic_types, expected_types)
            all_cameras = all(
                camera_results[name] is not None and camera_info_results[name] is not None
                for name in robot_names
            )
            all_odom = all(odom_results[name] is not None for name in robot_names)
            all_tf = all(tf_seen[name] for name in robot_names)
            all_scan = True
            if include_scan:
                all_scan = all(scan_results[name] is not None for name in robot_names)

            if not topic_problems and all_cameras and all_odom and all_tf and all_scan:
                break

            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(node, timeout_sec=min(0.2, remaining))
            last_topic_types.update(_topic_types_by_name(node))

        last_topic_types.update(_topic_types_from_ros2_cli(timeout_sec=min(2.0, timeout_sec)))
        last_topic_types.update(_topic_types_by_name(node))
        topic_problems = _missing_or_wrong_topics(last_topic_types, expected_types)
        if topic_problems:
            raise RuntimeError("Live ROS probe failed. " + "; ".join(topic_problems))

        for robot_name in robot_names:
            if camera_results[robot_name] is None:
                raise RuntimeError(
                    f"Timed out waiting for first compressed image on {_camera_compressed_topic(robot_name)}."
                )
            if camera_info_results[robot_name] is None:
                raise RuntimeError(
                    f"Timed out waiting for first camera info on {_camera_info_topic(robot_name)}."
                )
            if odom_results[robot_name] is None:
                raise RuntimeError(
                    f"Timed out waiting for first odometry message on {_odom_topic(robot_name)}."
                )
            if not tf_seen[robot_name]:
                raise RuntimeError(
                    f"Timed out waiting for shared TF frames for '{robot_name}' on {shared_tf_topic}/{shared_tf_static_topic}."
                )
            if include_scan and scan_results[robot_name] is None:
                raise RuntimeError(
                    f"Timed out waiting for LaserScan message on {_scan_topic(robot_name)}."
                )

        probes: List[RobotProbeResult] = []
        for robot_name in robot_names:
            resolved_camera_frame = resolve_camera_frame_id(
                robot_name=robot_name,
                camera_header_frame=(
                    camera_info_results[robot_name].frame_id
                    or camera_results[robot_name].frame_id  # type: ignore[union-attr]
                ),
                available_tf_frames=sorted(tf_frames[robot_name]),
            )
            probes.append(
                RobotProbeResult(
                    robot_name=robot_name,
                    tf_topic=shared_tf_topic,
                    cmd_vel_topic=_cmd_vel_topic(robot_name),
                    goal_topic=_goal_pose_topic(robot_name),
                    goal_cancel_topic=_goal_cancel_topic(robot_name),
                    goal_status_topic=_goal_status_topic(robot_name),
                    waypoint_path_topic=_waypoint_path_topic(robot_name),
                    waypoint_status_topic=_waypoint_status_topic(robot_name),
                    global_path_topic=_rviz_global_plan_topic(robot_name),
                    controller_path_topic=_rviz_local_plan_topic(robot_name),
                    camera_raw_topic="",
                    camera_compressed_topic=_camera_compressed_topic(robot_name),
                    camera_frame=resolved_camera_frame,
                    camera=camera_results[robot_name],  # type: ignore[arg-type]
                    odom=odom_results[robot_name],  # type: ignore[arg-type]
                    scan=scan_results[robot_name] if include_scan else None,
                )
            )
        return NavGraphProbeResult(
            shared_tf_topic=shared_tf_topic,
            shared_tf_static_topic=shared_tf_static_topic,
            shared_map_topic=shared_map_topic,
            probes=probes,
        )
    finally:
        for subscription in subscriptions:
            try:
                node.destroy_subscription(subscription)
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass


def wait_for_compressed_topics(
    probes: Sequence[RobotProbeResult],
    timeout_sec: float,
) -> None:
    import rclpy
    from sensor_msgs.msg import CompressedImage

    timeout_sec = max(0.5, float(timeout_sec))
    node_name = f"horus_hospital_compressed_probe_{uuid.uuid4().hex[:8]}"
    node = rclpy.create_node(node_name)
    subscriptions = []
    received: Dict[str, bool] = {probe.robot_name: False for probe in probes}

    def make_callback(name: str):
        def _callback(msg: CompressedImage):
            if msg.data:
                received[name] = True

        return _callback

    try:
        for probe in probes:
            subscriptions.append(
                node.create_subscription(
                    CompressedImage,
                    probe.camera_compressed_topic,
                    make_callback(probe.robot_name),
                    10,
                )
            )

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if all(received.values()):
                return
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(node, timeout_sec=min(0.2, remaining))

        missing = [name for name, seen in received.items() if not seen]
        raise RuntimeError(
            "Timed out waiting for compressed image topics: " + ", ".join(missing)
        )
    finally:
        for subscription in subscriptions:
            try:
                node.destroy_subscription(subscription)
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass


@dataclass
class _TaskBridgeState:
    goal_client: object
    waypoint_client: object
    goal_status_publisher: object
    waypoint_status_publisher: object
    active_goal_handle: Optional[object] = None
    active_waypoint_handle: Optional[object] = None
    active_goal_pose: Optional[object] = None
    active_waypoint_total: int = 0
    last_waypoint_remaining: Optional[int] = None


class CarterNavTaskBridge:
    """Bridge HORUS task topics onto the live Carter Nav2 actions."""

    def __init__(self, probes: Sequence[RobotProbeResult], status_frame_id: str = "map") -> None:
        import rclpy
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
        from nav_msgs.msg import Path
        from rclpy.action import ActionClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self._goal_status_enum = GoalStatus
        self._navigate_to_pose_type = NavigateToPose
        self._navigate_through_poses_type = NavigateThroughPoses
        self._string_type = String
        self.status_frame_id = str(status_frame_id or "").strip() or "map"
        self._lock = threading.Lock()

        node_name = f"horus_carter_nav_task_bridge_{uuid.uuid4().hex[:8]}"
        self.node = rclpy.create_node(node_name)

        command_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._subscriptions = []
        self._states: Dict[str, _TaskBridgeState] = {}
        for probe in probes:
            goal_client = ActionClient(
                self.node,
                NavigateToPose,
                f"/{probe.robot_name}/navigate_to_pose",
            )
            waypoint_client = ActionClient(
                self.node,
                NavigateThroughPoses,
                f"/{probe.robot_name}/navigate_through_poses",
            )
            goal_status_pub = self.node.create_publisher(String, probe.goal_status_topic, command_qos)
            waypoint_status_pub = self.node.create_publisher(String, probe.waypoint_status_topic, command_qos)

            self._states[probe.robot_name] = _TaskBridgeState(
                goal_client=goal_client,
                waypoint_client=waypoint_client,
                goal_status_publisher=goal_status_pub,
                waypoint_status_publisher=waypoint_status_pub,
            )

            self._subscriptions.append(
                self.node.create_subscription(
                    PoseStamped,
                    probe.goal_topic,
                    self._make_goal_callback(probe.robot_name),
                    command_qos,
                )
            )
            self._subscriptions.append(
                self.node.create_subscription(
                    String,
                    probe.goal_cancel_topic,
                    self._make_cancel_callback(probe.robot_name),
                    command_qos,
                )
            )
            self._subscriptions.append(
                self.node.create_subscription(
                    Path,
                    probe.waypoint_path_topic,
                    self._make_waypoint_callback(probe.robot_name),
                    command_qos,
                )
            )

    def wait_until_ready(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.5, float(timeout_sec))
        while time.monotonic() < deadline:
            if not self.missing_servers():
                return True
            time.sleep(0.1)
        return not self.missing_servers()

    def missing_servers(self) -> List[str]:
        missing: List[str] = []
        for robot_name, state in self._states.items():
            if not state.goal_client.wait_for_server(timeout_sec=0.0):
                missing.append(f"/{robot_name}/navigate_to_pose")
            if not state.waypoint_client.wait_for_server(timeout_sec=0.0):
                missing.append(f"/{robot_name}/navigate_through_poses")
        return missing

    def destroy(self) -> None:
        for subscription in self._subscriptions:
            try:
                self.node.destroy_subscription(subscription)
            except Exception:
                pass
        for state in self._states.values():
            try:
                state.goal_client.destroy()
            except Exception:
                pass
            try:
                state.waypoint_client.destroy()
            except Exception:
                pass
        try:
            self.node.destroy_node()
        except Exception:
            pass

    def _yaw_from_pose(self, pose_msg) -> float:
        orientation = pose_msg.pose.orientation
        siny_cosp = 2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y))
        cosy_cosp = 1.0 - (2.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z)))
        return math.atan2(siny_cosp, cosy_cosp)

    def _publish_goal_status(self, robot_name: str, state_text: str, goal_pose=None, error_msg: str = "") -> None:
        state = self._states.get(robot_name)
        if state is None:
            return
        payload = {
            "robot_name": robot_name,
            "status": state_text,
            "state": state_text,
            "frame_id": self.status_frame_id,
            "ts_unix_ms": int(time.time() * 1000.0),
        }
        if goal_pose is not None:
            payload["goal_pose"] = {
                "x": round(float(goal_pose.pose.position.x), 4),
                "y": round(float(goal_pose.pose.position.y), 4),
                "yaw": round(self._yaw_from_pose(goal_pose), 4),
            }
        if error_msg:
            payload["error"] = str(error_msg)
        msg = self._string_type()
        msg.data = json.dumps(payload, separators=(",", ":"))
        state.goal_status_publisher.publish(msg)

    def _publish_waypoint_status(
        self,
        robot_name: str,
        state_text: str,
        current_index: int,
        total: int,
        error_msg: str = "",
    ) -> None:
        state = self._states.get(robot_name)
        if state is None:
            return
        payload = {
            "robot_name": robot_name,
            "status": state_text,
            "state": state_text,
            "current_index": int(current_index),
            "total": int(max(0, total)),
            "frame_id": self.status_frame_id,
            "ts_unix_ms": int(time.time() * 1000.0),
        }
        if error_msg:
            payload["error"] = str(error_msg)
        msg = self._string_type()
        msg.data = json.dumps(payload, separators=(",", ":"))
        state.waypoint_status_publisher.publish(msg)

    def _cancel_active_handles(self, robot_name: str, cancel_waypoint: bool = True, cancel_goal: bool = True) -> None:
        with self._lock:
            state = self._states[robot_name]
            goal_handle = state.active_goal_handle if cancel_goal else None
            waypoint_handle = state.active_waypoint_handle if cancel_waypoint else None
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
        if waypoint_handle is not None:
            try:
                waypoint_handle.cancel_goal_async()
            except Exception:
                pass

    def _make_goal_callback(self, robot_name: str):
        def _callback(msg):
            self._cancel_active_handles(robot_name, cancel_waypoint=True, cancel_goal=True)
            with self._lock:
                self._states[robot_name].active_goal_pose = msg
            self._publish_goal_status(robot_name, "goal_sent", goal_pose=msg)
            goal_msg = self._navigate_to_pose_type.Goal()
            goal_msg.pose = msg
            goal_msg.behavior_tree = ""
            future = self._states[robot_name].goal_client.send_goal_async(goal_msg)
            future.add_done_callback(lambda f: self._on_goal_response(robot_name, f))

        return _callback

    def _on_goal_response(self, robot_name: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_goal_status(robot_name, "goal_failed", error_msg=str(exc))
            return

        if goal_handle is None or not goal_handle.accepted:
            self._publish_goal_status(robot_name, "goal_failed", error_msg="navigate_to_pose goal rejected")
            return

        with self._lock:
            state = self._states[robot_name]
            state.active_goal_handle = goal_handle
            goal_pose = state.active_goal_pose
        self._publish_goal_status(robot_name, "goal_active", goal_pose=goal_pose)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f, handle=goal_handle: self._on_goal_result(robot_name, handle, f))

    def _on_goal_result(self, robot_name: str, goal_handle, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self._publish_goal_status(robot_name, "goal_failed", error_msg=str(exc))
            return

        with self._lock:
            state = self._states[robot_name]
            goal_pose = state.active_goal_pose
            if state.active_goal_handle is goal_handle:
                state.active_goal_handle = None
                state.active_goal_pose = None

        status_code = getattr(wrapped, "status", None)
        result = getattr(wrapped, "result", None)
        error_msg = getattr(result, "error_msg", "") if result is not None else ""
        if status_code == self._goal_status_enum.STATUS_SUCCEEDED:
            self._publish_goal_status(robot_name, "goal_reached", goal_pose=goal_pose)
        elif status_code == self._goal_status_enum.STATUS_CANCELED:
            self._publish_goal_status(robot_name, "goal_cancelled", goal_pose=goal_pose)
        else:
            self._publish_goal_status(robot_name, "goal_failed", goal_pose=goal_pose, error_msg=error_msg)

    def _make_cancel_callback(self, robot_name: str):
        def _callback(_msg):
            with self._lock:
                state = self._states[robot_name]
                goal_pose = state.active_goal_pose
                waypoint_total = state.active_waypoint_total
                has_waypoint = state.active_waypoint_handle is not None
            self._cancel_active_handles(robot_name, cancel_waypoint=True, cancel_goal=True)
            if goal_pose is not None:
                self._publish_goal_status(robot_name, "goal_cancelled", goal_pose=goal_pose)
            if has_waypoint:
                self._publish_waypoint_status(robot_name, "path_cancelled", current_index=-1, total=waypoint_total)

        return _callback

    def _make_waypoint_callback(self, robot_name: str):
        def _callback(msg):
            poses = list(getattr(msg, "poses", []))
            if not poses:
                self._publish_waypoint_status(robot_name, "path_failed", current_index=-1, total=0, error_msg="empty path")
                return

            self._cancel_active_handles(robot_name, cancel_waypoint=True, cancel_goal=True)
            total = len(poses)
            with self._lock:
                state = self._states[robot_name]
                state.active_waypoint_total = total
                state.last_waypoint_remaining = None
            self._publish_waypoint_status(robot_name, "path_sent", current_index=0, total=total)

            goal_msg = self._navigate_through_poses_type.Goal()
            goal_msg.poses = poses
            goal_msg.behavior_tree = ""
            future = self._states[robot_name].waypoint_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback: self._on_waypoint_feedback(robot_name, feedback),
            )
            future.add_done_callback(lambda f: self._on_waypoint_response(robot_name, f))

        return _callback

    def _on_waypoint_response(self, robot_name: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_waypoint_status(robot_name, "path_failed", current_index=-1, total=0, error_msg=str(exc))
            return

        with self._lock:
            state = self._states[robot_name]
            total = state.active_waypoint_total

        if goal_handle is None or not goal_handle.accepted:
            self._publish_waypoint_status(robot_name, "path_failed", current_index=-1, total=total, error_msg="navigate_through_poses goal rejected")
            return

        with self._lock:
            self._states[robot_name].active_waypoint_handle = goal_handle
        self._publish_waypoint_status(robot_name, "path_active", current_index=0, total=total)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f, handle=goal_handle: self._on_waypoint_result(robot_name, handle, f))

    def _on_waypoint_feedback(self, robot_name: str, feedback_msg) -> None:
        feedback = getattr(feedback_msg, "feedback", None)
        if feedback is None:
            return
        remaining = int(max(0, getattr(feedback, "number_of_poses_remaining", 0)))
        with self._lock:
            state = self._states[robot_name]
            total = state.active_waypoint_total
            if state.last_waypoint_remaining == remaining:
                return
            state.last_waypoint_remaining = remaining
        current_index = max(0, total - remaining)
        self._publish_waypoint_status(robot_name, "path_active", current_index=current_index, total=total)

    def _on_waypoint_result(self, robot_name: str, goal_handle, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self._publish_waypoint_status(robot_name, "path_failed", current_index=-1, total=0, error_msg=str(exc))
            return

        with self._lock:
            state = self._states[robot_name]
            total = state.active_waypoint_total
            if state.active_waypoint_handle is goal_handle:
                state.active_waypoint_handle = None
                state.active_waypoint_total = 0
                state.last_waypoint_remaining = None

        status_code = getattr(wrapped, "status", None)
        result = getattr(wrapped, "result", None)
        error_msg = getattr(result, "error_msg", "") if result is not None else ""
        if status_code == self._goal_status_enum.STATUS_SUCCEEDED:
            self._publish_waypoint_status(robot_name, "path_completed", current_index=total, total=total)
        elif status_code == self._goal_status_enum.STATUS_CANCELED:
            self._publish_waypoint_status(robot_name, "path_cancelled", current_index=-1, total=total)
        else:
            self._publish_waypoint_status(robot_name, "path_failed", current_index=-1, total=total, error_msg=error_msg)


class TaskBridgeRunner:
    """Run the Carter Nav task bridge on a background executor thread."""

    def __init__(self, bridge: CarterNavTaskBridge) -> None:
        from rclpy.executors import SingleThreadedExecutor

        self.bridge = bridge
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.bridge.node)
        self._stop_requested = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_requested.clear()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        import rclpy

        while not self._stop_requested.is_set() and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)

    def stop(self) -> None:
        self._stop_requested.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        try:
            self.executor.remove_node(self.bridge.node)
        except Exception:
            pass
        self.bridge.destroy()
        try:
            self.executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass


def build_robot_and_dataviz(
    probe: RobotProbeResult,
    include_scan: bool,
    minimap_streaming_type: str,
    teleop_streaming_type: str,
    description_urdf_path: str,
    body_mesh_mode: str,
    visual_mesh_triangle_budget: int,
) -> Tuple[Robot, DataViz]:
    robot = Robot(
        name=probe.robot_name,
        robot_type=RobotType.WHEELED,
        dimensions=RobotDimensions(length=0.82, width=0.56, height=0.60),
    )
    robot.configure_robot_description(
        urdf_path=description_urdf_path,
        base_frame="base_link",
        include_visual_meshes=True,
        visual_mesh_triangle_budget=visual_mesh_triangle_budget,
        body_mesh_mode=body_mesh_mode,
        chunk_size_bytes=48000,
    )
    robot.add_metadata(
        "teleop_config",
        {
            "enabled": True,
            "command_topic": probe.cmd_vel_topic,
            "raw_input_topic": f"/horus/teleop/{probe.robot_name}/joy",
            "head_pose_topic": f"/horus/teleop/{probe.robot_name}/head_pose",
            "robot_profile": "wheeled",
            "response_mode": "analog",
            "publish_rate_hz": 30.0,
        },
    )
    robot.add_metadata(
        "task_config",
        {
            "go_to_point": {
                "enabled": True,
                "goal_topic": probe.goal_topic,
                "cancel_topic": probe.goal_cancel_topic,
                "status_topic": probe.goal_status_topic,
                "frame_id": "map",
                "position_tolerance_m": 0.20,
                "yaw_tolerance_deg": 12.0,
                "min_altitude_m": 0.0,
                "max_altitude_m": 1.0,
            },
            "waypoint": {
                "enabled": True,
                "path_topic": probe.waypoint_path_topic,
                "status_topic": probe.waypoint_status_topic,
                "frame_id": "map",
                "position_tolerance_m": 0.20,
                "yaw_tolerance_deg": 12.0,
            },
        },
    )

    robot_tf_frame = prefix_frame_id(probe.odom.child_frame_id, probe.robot_name)
    if not robot_tf_frame:
        robot_tf_frame = f"{probe.robot_name}/base_link"

    camera = Camera(
        name="front_camera",
        frame_id=probe.camera_frame or robot_tf_frame,
        topic=probe.camera_compressed_topic,
        resolution=(probe.camera.width, probe.camera.height),
        fps=DEFAULT_CAMERA_FPS,
        encoding="jpeg",
        streaming_type=minimap_streaming_type,
        minimap_streaming_type=minimap_streaming_type,
        teleop_streaming_type=teleop_streaming_type,
        startup_mode="minimap",
    )
    camera.add_metadata("image_type", "compressed")
    camera.add_metadata("streaming_type", minimap_streaming_type)
    camera.add_metadata("minimap_streaming_type", minimap_streaming_type)
    camera.add_metadata("teleop_streaming_type", teleop_streaming_type)
    camera.add_metadata("startup_mode", "minimap")
    camera.add_metadata("display_mode", "projected")
    camera.add_metadata("use_tf", True)
    camera.add_metadata("webrtc_client_signal_topic", "/horus/webrtc/client_signal")
    camera.add_metadata("webrtc_server_signal_topic", "/horus/webrtc/server_signal")
    camera.add_metadata("webrtc_bitrate_kbps", 2000)
    camera.add_metadata("webrtc_framerate", 20)
    camera.add_metadata("webrtc_stun_server_url", "stun:stun.l.google.com:19302")
    camera.add_metadata("view_position_offset", [0.0, 0.0, 0.0])
    camera.configure_projected_view(
        position_offset=(0.04, 0.28, 0.0),
        rotation_offset=(0.0, 0.0, 0.0),
        image_scale=1.037,
        focal_length_scale=0.55,
        show_frustum=True,
        frustum_color="#E6E6E0A0",
    )
    camera.add_metadata("overhead_size", 10.0)
    camera.add_metadata("overhead_position_offset", [0.0, 2.0, 0.0])
    camera.add_metadata("overhead_face_camera", True)
    camera.add_metadata("overhead_rotation_offset", [90.0, 0.0, 0.0])
    robot.add_sensor(camera)

    scan_sensor: Optional[LaserScan] = None
    if include_scan and probe.scan is not None:
        scan_frame = prefix_frame_id(probe.scan.frame_id, probe.robot_name)
        scan_sensor = LaserScan(
            name="front_2d_lidar",
            frame_id=scan_frame,
            topic=probe.scan.topic,
            min_angle=probe.scan.angle_min,
            max_angle=probe.scan.angle_max,
            angle_increment=max(1e-6, probe.scan.angle_increment),
            min_range=max(0.0, probe.scan.range_min),
            max_range=max(0.0, probe.scan.range_max),
        )
        robot.add_sensor(scan_sensor)

    odom_frame = prefix_frame_id(probe.odom.frame_id, probe.robot_name) or robot_tf_frame

    dataviz = DataViz(name=f"{probe.robot_name}_hospital_live_viz")
    dataviz.add_sensor_visualization(
        camera,
        robot_name=probe.robot_name,
        enabled=True,
    )
    if scan_sensor is not None:
        scan_sensor.color = DataViz._deterministic_robot_laser_hex_color(probe.robot_name)
        dataviz.add_sensor_visualization(
            scan_sensor,
            robot_name=probe.robot_name,
            enabled=True,
        )

    dataviz.add_robot_transform(
        robot_name=probe.robot_name,
        topic=probe.tf_topic,
        frame_id=robot_tf_frame,
    )
    dataviz.add_robot_velocity_data(
        robot_name=probe.robot_name,
        topic=probe.odom.topic,
        frame_id=odom_frame,
    )
    dataviz.add_robot_odometry_trail(
        robot_name=probe.robot_name,
        topic=probe.odom.topic,
        frame_id=odom_frame,
    )
    dataviz.add_robot_global_path(
        robot_name=probe.robot_name,
        topic=probe.global_path_topic,
        frame_id=DEFAULT_RVIZ_ROOT_FRAME,
    )
    dataviz.add_robot_local_path(
        robot_name=probe.robot_name,
        topic=probe.controller_path_topic,
        frame_id=DEFAULT_RVIZ_ROOT_FRAME,
    )

    return robot, dataviz


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Register live hospital Carter robots against the shared SLAM/Nav graph "
            "with teleop, go-to, waypoint, occupancy grid, and robot-description body visuals."
        )
    )
    parser.add_argument(
        "--robot-names",
        default="carter1,carter2,carter3",
        help="Comma-separated robot names (default: carter1,carter2,carter3).",
    )
    parser.add_argument(
        "--workspace-scale",
        type=float,
        default=0.1,
        help="Optional workspace scale forwarded in registration payload (default: 0.1).",
    )
    parser.add_argument(
        "--probe-timeout-sec",
        type=float,
        default=8.0,
        help="Timeout for topic probing and readiness checks (default: 8.0).",
    )
    parser.add_argument(
        "--tf-topic",
        default=DEFAULT_TF_TOPIC,
        help=f"Shared TF topic from the Carter nav graph (default: {DEFAULT_TF_TOPIC}).",
    )
    parser.add_argument(
        "--tf-static-topic",
        default=DEFAULT_TF_STATIC_TOPIC,
        help=f"Shared static TF topic from the Carter nav graph (default: {DEFAULT_TF_STATIC_TOPIC}).",
    )
    parser.add_argument(
        "--shared-map-topic",
        default=DEFAULT_SHARED_MAP_TOPIC,
        help=f"Shared occupancy grid topic from the Carter nav graph (default: {DEFAULT_SHARED_MAP_TOPIC}).",
    )
    parser.add_argument(
        "--camera-minimap-streaming-type",
        choices=["ros", "webrtc"],
        default="ros",
        help="Camera transport profile for minimap mode (default: ros).",
    )
    parser.add_argument(
        "--camera-teleop-streaming-type",
        choices=["ros", "webrtc"],
        default="webrtc",
        help="Camera transport profile for teleop mode (default: webrtc).",
    )
    parser.add_argument(
        "--keep-alive",
        dest="keep_alive",
        action="store_true",
        default=True,
        help="Keep dashboard/session alive after registration (default: on).",
    )
    parser.add_argument(
        "--no-keep-alive",
        dest="keep_alive",
        action="store_false",
        help="Exit immediately after registration acknowledgement.",
    )
    parser.add_argument(
        "--no-scan",
        dest="with_scan",
        action="store_false",
        default=True,
        help="Skip LaserScan sensor registration.",
    )
    parser.add_argument(
        "--body-mesh-mode",
        choices=["collision_only", "preview_mesh", "runtime_high_mesh", "max_quality_mesh"],
        default=None,
        help=(
            "Robot body delivery mode. CLI overrides HORUS_CARTER_BODY_MESH_MODE. "
            "Default: env or runtime_high_mesh."
        ),
    )
    parser.add_argument(
        "--visual-mesh-triangle-budget",
        type=int,
        default=None,
        help=(
            "Triangle budget used for preview mesh baking. "
            "CLI overrides HORUS_CARTER_VISUAL_MESH_TRIANGLE_BUDGET."
        ),
    )
    parser.add_argument(
        "--apriltag-semantic-labeling",
        action="store_true",
        default=False,
        help=(
            "Subscribe to /april_tag/detections_by_robot and promote first-seen tag IDs "
            "into HORUS semantic boxes placed at the configured AprilTag world positions."
        ),
    )
    parser.add_argument(
        "--apriltag-detections-topic",
        default=DEFAULT_APRILTAG_DETECTIONS_TOPIC,
        help=(
            "Per-robot AprilTag detections topic used for semantic labeling "
            f"(default: {DEFAULT_APRILTAG_DETECTIONS_TOPIC})."
        ),
    )
    parser.add_argument(
        "--apriltag-scene-profile",
        choices=sorted(DEFAULT_APRILTAG_WORLD_POSITIONS_BY_SCENE.keys()),
        default=DEFAULT_APRILTAG_SCENE_PROFILE,
        help=(
            "Select the fixed AprilTag world-position map used for semantic labeling "
            f"(default: {DEFAULT_APRILTAG_SCENE_PROFILE})."
        ),
    )
    return parser


def _print_probe_summary(nav_graph: NavGraphProbeResult, include_scan: bool) -> None:
    cli.print_info(f"shared_tf_topic={nav_graph.shared_tf_topic}")
    cli.print_info(f"shared_tf_static_topic={nav_graph.shared_tf_static_topic}")
    cli.print_info(f"shared_map_topic={nav_graph.shared_map_topic}")
    for probe in nav_graph.probes:
        cli.print_info(
            f"[{probe.robot_name}] camera_compressed={probe.camera_compressed_topic} "
            f"({probe.camera.width}x{probe.camera.height}, {probe.camera.encoding})"
        )
        cli.print_info(
            f"[{probe.robot_name}] camera_frame={probe.camera_frame}"
        )
        cli.print_info(
            f"[{probe.robot_name}] tf={probe.tf_topic} "
            f"odom={probe.odom.topic} frame={prefix_frame_id(probe.odom.frame_id, probe.robot_name)} "
            f"cmd_vel={probe.cmd_vel_topic}"
        )
        if include_scan and probe.scan is not None:
            cli.print_info(
                f"[{probe.robot_name}] scan={probe.scan.topic} frame={probe.scan.frame_id}"
            )
        cli.print_info(
            f"[{probe.robot_name}] goal={probe.goal_topic} waypoint={probe.waypoint_path_topic} "
            f"rviz_global_path={probe.global_path_topic} rviz_local_path={probe.controller_path_topic} "
            f"path_frame={DEFAULT_RVIZ_ROOT_FRAME}"
        )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    robot_names = _parse_robot_names(args.robot_names)
    shared_tf_topic = str(args.tf_topic or "").strip() or DEFAULT_TF_TOPIC
    if not shared_tf_topic.startswith("/"):
        shared_tf_topic = "/" + shared_tf_topic.lstrip("/")
    shared_tf_static_topic = str(args.tf_static_topic or "").strip() or DEFAULT_TF_STATIC_TOPIC
    if not shared_tf_static_topic.startswith("/"):
        shared_tf_static_topic = "/" + shared_tf_static_topic.lstrip("/")
    shared_map_topic = str(args.shared_map_topic or "").strip() or DEFAULT_SHARED_MAP_TOPIC
    if not shared_map_topic.startswith("/"):
        shared_map_topic = "/" + shared_map_topic.lstrip("/")
    include_scan = bool(args.with_scan)
    timeout_sec = max(0.5, float(args.probe_timeout_sec))
    minimap_streaming_type = _normalize_transport(
        args.camera_minimap_streaming_type, "ros"
    )
    teleop_streaming_type = _normalize_transport(
        args.camera_teleop_streaming_type, "webrtc"
    )

    task_bridge_runner: Optional[TaskBridgeRunner] = None
    semantic_overlay_runner: Optional[AprilTagSemanticOverlayRunner] = None
    rclpy_initialized = False

    try:
        import rclpy

        if not rclpy.ok():
            rclpy.init()
            rclpy_initialized = True

        cli.print_step("Probing live hospital Carter nav graph...")
        nav_graph = probe_live_nav_graph(
            robot_names=robot_names,
            shared_tf_topic=shared_tf_topic,
            shared_tf_static_topic=shared_tf_static_topic,
            shared_map_topic=shared_map_topic,
            include_scan=include_scan,
            timeout_sec=timeout_sec,
        )
        cli.print_success("Carter nav graph detected.")
        _print_probe_summary(nav_graph, include_scan)

        cli.print_step("Preparing Nova Carter robot description meshes...")
        body_mesh_mode, visual_mesh_triangle_budget = resolve_carter_body_mesh_settings(
            body_mesh_mode_override=args.body_mesh_mode,
            visual_mesh_triangle_budget_override=args.visual_mesh_triangle_budget,
        )
        cli.print_info(
            f"carter_body_mesh_mode={body_mesh_mode} "
            f"visual_mesh_triangle_budget={visual_mesh_triangle_budget}"
        )
        description_urdf_path, warm_stats = warm_nova_carter_robot_description_artifact(
            body_mesh_mode_override=body_mesh_mode,
            visual_mesh_triangle_budget_override=visual_mesh_triangle_budget,
        )
        cli.print_info(
            "Nova Carter artifact stats: "
            f"groups={warm_stats['mesh_asset_count']} triangles={warm_stats['triangle_count']} "
            f"chunks={warm_stats['chunk_count']} bytes={warm_stats['encoded_bytes']} "
            f"cache={warm_stats['cache_status']}"
        )
        cli.print_success("Nova Carter robot description meshes are ready.")

        cli.print_step("Waiting for live compressed image topics...")
        wait_for_compressed_topics(
            probes=nav_graph.probes,
            timeout_sec=timeout_sec,
        )
        cli.print_success("Compressed camera topics are active.")

        cli.print_step("Starting Carter Nav task bridges...")
        task_bridge = CarterNavTaskBridge(nav_graph.probes, status_frame_id="map")
        task_bridge_runner = TaskBridgeRunner(task_bridge)
        task_bridge_runner.start()
        if not task_bridge.wait_until_ready(timeout_sec=timeout_sec):
            raise RuntimeError(
                "Timed out waiting for Nav2 action servers: "
                + ", ".join(task_bridge.missing_servers())
            )
        cli.print_success("Task bridges are active.")

        robots: List[Robot] = []
        datavizs: List[DataViz] = []
        for probe in nav_graph.probes:
            robot, dataviz = build_robot_and_dataviz(
                probe=probe,
                include_scan=include_scan,
                minimap_streaming_type=minimap_streaming_type,
                teleop_streaming_type=teleop_streaming_type,
                description_urdf_path=description_urdf_path,
                body_mesh_mode=body_mesh_mode,
                visual_mesh_triangle_budget=visual_mesh_triangle_budget,
            )
            robots.append(robot)
            datavizs.append(dataviz)
        if datavizs:
            datavizs[0].add_occupancy_grid(
                topic=nav_graph.shared_map_topic,
                frame_id=DEFAULT_SHARED_MAP_FRAME,
            )

        if bool(args.apriltag_semantic_labeling):
            cli.print_step("Starting AprilTag semantic overlay bridge...")
            semantic_overlay = AprilTagSemanticOverlayBridge(
                probes=nav_graph.probes,
                robots=robots,
                datavizs=datavizs,
                workspace_scale=float(args.workspace_scale),
                detections_topic=args.apriltag_detections_topic,
                scene_profile=args.apriltag_scene_profile,
            )
            semantic_overlay_runner = AprilTagSemanticOverlayRunner(semantic_overlay)
            semantic_overlay_runner.start()
            cli.print_success("AprilTag semantic overlay bridge is active.")

        cli.print_step(f"Registering {len(robots)} robot(s) with HORUS...")
        cli.print_info("Bridge ready state will be followed by immediate registration seeding.")
        success, result = register_robots(
            robots,
            datavizs=datavizs,
            keep_alive=bool(args.keep_alive),
            show_dashboard=True,
            workspace_scale=float(args.workspace_scale),
            wait_for_app_before_register=False,
        )
        if not success:
            cli.print_error(f"Registration failed: {result}")
            return 1

        cli.print_success("Hospital Carter live registration complete.")
        return 0
    except KeyboardInterrupt:
        cli.print_info("Hospital Carter live demo interrupted.")
        return 1
    except Exception as exc:
        cli.print_error(str(exc))
        return 1
    finally:
        if semantic_overlay_runner is not None:
            cli.print_info("Stopping AprilTag semantic overlay bridge...")
            semantic_overlay_runner.stop()

        if task_bridge_runner is not None:
            cli.print_info("Stopping Carter Nav task bridges...")
            task_bridge_runner.stop()

        if rclpy_initialized:
            try:
                import rclpy

                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
