"""
URDF-backed robot description resolver and chunk encoder for MR transport.
"""

from __future__ import annotations

import base64
import gzip
import hashlib
import json
import math
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from .robot_description_models import (
    CompiledCollision,
    CompiledJoint,
    CompiledLink,
    CompiledVisual,
    RobotDescriptionArtifact,
    RobotDescriptionManifestV2,
    RobotDescriptionV2,
)
from .robot_mesh_baker import RobotMeshBaker


def _parse_vec3(raw_value: Optional[str], default_xyz: Tuple[float, float, float]) -> List[float]:
    if not raw_value:
        return [float(default_xyz[0]), float(default_xyz[1]), float(default_xyz[2])]
    parts = str(raw_value).replace(",", " ").split()
    if len(parts) != 3:
        return [float(default_xyz[0]), float(default_xyz[1]), float(default_xyz[2])]
    parsed: List[float] = []
    for index, part in enumerate(parts):
        try:
            parsed.append(float(part))
        except (TypeError, ValueError):
            parsed.append(float(default_xyz[index]))
    return parsed


def _normalize_joint_axis(axis_xyz: List[float]) -> List[float]:
    x, y, z = axis_xyz
    magnitude = math.sqrt((x * x) + (y * y) + (z * z))
    if magnitude <= 1e-8:
        return [0.0, 0.0, 1.0]
    return [x / magnitude, y / magnitude, z / magnitude]


def _coerce_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
        return default
    if isinstance(value, (int, float)):
        return bool(value)
    return default


def _normalize_body_mesh_mode(value: Any) -> str:
    normalized = str(value or "preview_mesh").strip().lower()
    if normalized == "max_quality_mesh":
        return "runtime_high_mesh"
    if normalized in {"collision_only", "preview_mesh", "runtime_high_mesh"}:
        return normalized
    return "preview_mesh"


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    """Build ROS intrinsic XYZ (roll/pitch/yaw) rotation matrix."""
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return [
        [(cy * cp), (cy * sp * sr) - (sy * cr), (cy * sp * cr) + (sy * sr)],
        [(sy * cp), (sy * sp * sr) + (cy * cr), (sy * sp * cr) - (cy * sr)],
        [(-sp), cp * sr, cp * cr],
    ]


def _matrix_multiply(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [
        [
            (a[row][0] * b[0][col]) + (a[row][1] * b[1][col]) + (a[row][2] * b[2][col])
            for col in range(3)
        ]
        for row in range(3)
    ]


def _matrix_vector_multiply(m: List[List[float]], v: List[float]) -> List[float]:
    return [
        (m[0][0] * v[0]) + (m[0][1] * v[1]) + (m[0][2] * v[2]),
        (m[1][0] * v[0]) + (m[1][1] * v[1]) + (m[1][2] * v[2]),
        (m[2][0] * v[0]) + (m[2][1] * v[1]) + (m[2][2] * v[2]),
    ]


def _matrix_transpose(m: List[List[float]]) -> List[List[float]]:
    return [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]


def _compose_transforms(
    parent_rotation: List[List[float]],
    parent_translation: List[float],
    local_xyz: List[float],
    local_rpy: List[float],
) -> Tuple[List[List[float]], List[float]]:
    local_rotation = _rpy_to_matrix(local_rpy[0], local_rpy[1], local_rpy[2])
    combined_rotation = _matrix_multiply(parent_rotation, local_rotation)
    translated = _matrix_vector_multiply(parent_rotation, local_xyz)
    combined_translation = [
        parent_translation[0] + translated[0],
        parent_translation[1] + translated[1],
        parent_translation[2] + translated[2],
    ]
    return combined_rotation, combined_translation


def _relative_transform(
    parent_rotation: List[List[float]],
    parent_translation: List[float],
    child_rotation: List[List[float]],
    child_translation: List[float],
) -> Tuple[List[List[float]], List[float]]:
    inverse_parent = _matrix_transpose(parent_rotation)
    relative_rotation = _matrix_multiply(inverse_parent, child_rotation)
    delta = [
        child_translation[0] - parent_translation[0],
        child_translation[1] - parent_translation[1],
        child_translation[2] - parent_translation[2],
    ]
    relative_translation = _matrix_vector_multiply(inverse_parent, delta)
    return relative_rotation, relative_translation


def _matrix_to_rpy(rotation: List[List[float]]) -> List[float]:
    """Convert rotation matrix to ROS roll/pitch/yaw."""
    r20 = max(-1.0, min(1.0, rotation[2][0]))
    pitch = math.asin(-r20)
    cos_pitch = math.cos(pitch)
    if abs(cos_pitch) > 1e-6:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        # Gimbal lock fallback.
        roll = math.atan2(-rotation[1][2], rotation[1][1])
        yaw = 0.0
    return [roll, pitch, yaw]


def _identity_rotation() -> List[List[float]]:
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


@dataclass
class _JointSourceData:
    name: str
    type: str
    parent_link: str
    child_link: str
    origin_xyz: List[float]
    origin_rpy: List[float]
    axis_xyz: List[float]


@dataclass
class _VisualSourceData:
    link_name: str
    mesh_uri: str
    origin_xyz: List[float]
    origin_rpy: List[float]
    mesh_scale_xyz: List[float]
    color_rgb: List[float]


@dataclass
class RobotDescriptionResolveConfig:
    enabled: bool = False
    source: str = "ros"
    urdf_path: str = ""
    ros_param_node: str = ""
    ros_param_name: str = "robot_description"
    base_frame: str = "base_link"
    chunk_size_bytes: int = 12000
    is_transparent: bool = False
    include_visual_meshes: bool = True
    visual_mesh_triangle_budget: int = 90000
    body_mesh_mode: str = "preview_mesh"


class RobotDescriptionResolver:
    """
    Resolve robot URDF sources into compact collision+joint payloads.
    """

    def __init__(self):
        self._artifact_by_hash: Dict[str, RobotDescriptionArtifact] = {}
        self._artifact_by_resolve_key: Dict[str, RobotDescriptionArtifact] = {}
        self._last_error: str = ""
        self._last_resolution_cache_status: str = "miss"
        self._mesh_baker = RobotMeshBaker()

    @property
    def last_error(self) -> str:
        return self._last_error

    @property
    def last_resolution_cache_status(self) -> str:
        return self._last_resolution_cache_status

    def resolve_for_robot(self, robot: Any) -> Optional[RobotDescriptionArtifact]:
        self._last_error = ""
        self._last_resolution_cache_status = "miss"
        config = self._parse_config(getattr(robot, "metadata", {}) or {})
        if not config.enabled:
            return None

        urdf_xml, error = self._resolve_urdf_xml(config)
        if not urdf_xml:
            self._last_error = error or "Failed to resolve URDF XML."
            return None

        resolve_key = hashlib.sha256(
            json.dumps(
                {
                    "urdf_xml": urdf_xml,
                    "source": config.source,
                    "base_frame": config.base_frame,
                    "include_visual_meshes": bool(config.include_visual_meshes),
                    "body_mesh_mode": config.body_mesh_mode,
                    "visual_mesh_triangle_budget": int(config.visual_mesh_triangle_budget),
                },
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()

        cached_by_resolve_key = self._artifact_by_resolve_key.get(resolve_key)
        if cached_by_resolve_key is not None:
            self._last_resolution_cache_status = "hit"
            return self._build_variant_artifact(cached_by_resolve_key, config)

        try:
            payload = self._compile_robot_description_payload(robot, urdf_xml, config)
        except ET.ParseError as exc:
            self._last_error = f"Invalid URDF/XML payload: {exc}"
            return None
        except Exception as exc:
            self._last_error = f"Robot description compile failed: {exc}"
            return None
        payload_json = json.dumps(payload.to_dict(), sort_keys=True, separators=(",", ":"))
        description_hash = hashlib.sha256(payload_json.encode("utf-8")).hexdigest()
        description_id = f"sha256:{description_hash}"

        chunk_size = max(1024, int(config.chunk_size_bytes))
        requested_transparent = bool(config.is_transparent)

        if description_id in self._artifact_by_hash:
            self._last_resolution_cache_status = "hit"
            return self._build_variant_artifact(self._artifact_by_hash[description_id], config)

        encoded_payload = self._encode_payload(payload_json)
        chunks = [
            encoded_payload[index:index + chunk_size]
            for index in range(0, len(encoded_payload), chunk_size)
        ] or [""]

        links = payload.links
        joints = payload.joints
        collision_count = sum(len(link.collisions) for link in links)
        mesh_asset_count = len(payload.mesh_assets)
        mesh_asset_encoded_bytes = sum(
            len(asset.positions_b64) + len(asset.normals_b64) + len(asset.indices_b64)
            for asset in payload.mesh_assets
        )
        manifest = RobotDescriptionManifestV2(
            version="v2",
            description_id=description_id,
            source=config.source,
            base_frame=payload.base_frame,
            link_count=len(links),
            joint_count=len(joints),
            collision_count=collision_count,
            supports_collision=collision_count > 0,
            supports_joints=len(joints) > 0,
            supports_visual_meshes=mesh_asset_count > 0,
            mesh_asset_count=mesh_asset_count,
            mesh_asset_encoded_bytes=mesh_asset_encoded_bytes,
            is_transparent=requested_transparent,
            encoding="json+gzip+base64",
            chunk_size_bytes=chunk_size,
        )

        artifact = RobotDescriptionArtifact(
            manifest=manifest,
            payload_dict=payload.to_dict(),
            payload_json=payload_json,
            encoded_payload=encoded_payload,
            chunks=chunks,
            urdf_path=config.urdf_path or None,
        )
        self._artifact_by_hash[description_id] = artifact
        self._artifact_by_resolve_key[resolve_key] = artifact
        self._last_resolution_cache_status = "built"
        return self._build_variant_artifact(artifact, config)

    def _build_variant_artifact(
        self,
        cached: RobotDescriptionArtifact,
        config: RobotDescriptionResolveConfig,
    ) -> RobotDescriptionArtifact:
        chunk_size = max(1024, int(config.chunk_size_bytes))
        requested_transparent = bool(config.is_transparent)
        if (
            bool(cached.manifest.is_transparent) == requested_transparent
            and int(cached.manifest.chunk_size_bytes) == chunk_size
        ):
            return cached

        variant_chunks = [
            cached.encoded_payload[index:index + chunk_size]
            for index in range(0, len(cached.encoded_payload), chunk_size)
        ] or [""]
        variant_manifest = RobotDescriptionManifestV2(
            version=cached.manifest.version,
            description_id=cached.manifest.description_id,
            source=cached.manifest.source,
            base_frame=cached.manifest.base_frame,
            link_count=cached.manifest.link_count,
            joint_count=cached.manifest.joint_count,
            collision_count=cached.manifest.collision_count,
            supports_collision=cached.manifest.supports_collision,
            supports_joints=cached.manifest.supports_joints,
            supports_visual_meshes=cached.manifest.supports_visual_meshes,
            mesh_asset_count=cached.manifest.mesh_asset_count,
            mesh_asset_encoded_bytes=cached.manifest.mesh_asset_encoded_bytes,
            is_transparent=requested_transparent,
            encoding=cached.manifest.encoding,
            chunk_size_bytes=chunk_size,
        )
        return RobotDescriptionArtifact(
            manifest=variant_manifest,
            payload_dict=cached.payload_dict,
            payload_json=cached.payload_json,
            encoded_payload=cached.encoded_payload,
            chunks=variant_chunks,
            urdf_path=config.urdf_path or cached.urdf_path,
        )

    def _parse_config(self, metadata: Dict[str, Any]) -> RobotDescriptionResolveConfig:
        config = metadata.get("robot_description_config")
        if not isinstance(config, dict):
            return RobotDescriptionResolveConfig(enabled=False)

        enabled = _coerce_bool(config.get("enabled", True), True)
        is_transparent = _coerce_bool(config.get("is_transparent", False), False)
        include_visual_meshes = _coerce_bool(config.get("include_visual_meshes", True), True)
        source = str(config.get("source", "ros") or "ros").strip().lower()
        if source not in {"ros"}:
            source = "ros"

        chunk_size = config.get("chunk_size_bytes", 12000)
        try:
            chunk_size = int(chunk_size)
        except (TypeError, ValueError):
            chunk_size = 12000

        raw_body_mesh_mode = config.get("body_mesh_mode", "preview_mesh")
        body_mesh_mode = _normalize_body_mesh_mode(raw_body_mesh_mode)

        default_budget = 240000 if body_mesh_mode == "runtime_high_mesh" else 90000
        visual_mesh_triangle_budget = config.get("visual_mesh_triangle_budget", default_budget)
        try:
            visual_mesh_triangle_budget = int(visual_mesh_triangle_budget)
        except (TypeError, ValueError):
            visual_mesh_triangle_budget = default_budget

        if body_mesh_mode == "collision_only":
            include_visual_meshes = False

        return RobotDescriptionResolveConfig(
            enabled=enabled,
            source=source,
            urdf_path=str(config.get("urdf_path", "") or "").strip(),
            ros_param_node=str(config.get("ros_param_node", "") or "").strip(),
            ros_param_name=str(config.get("ros_param_name", "robot_description") or "robot_description").strip(),
            base_frame=str(config.get("base_frame", "base_link") or "base_link").strip() or "base_link",
            chunk_size_bytes=max(1024, min(64000, chunk_size)),
            is_transparent=is_transparent,
            include_visual_meshes=include_visual_meshes,
            visual_mesh_triangle_budget=max(2000, min(500000, visual_mesh_triangle_budget)),
            body_mesh_mode=body_mesh_mode,
        )

    def _resolve_urdf_xml(self, config: RobotDescriptionResolveConfig) -> Tuple[str, str]:
        if config.urdf_path:
            candidate = os.path.expandvars(os.path.expanduser(config.urdf_path))
            if not os.path.isfile(candidate):
                return "", f"URDF path does not exist: {candidate}"

            lower_candidate = candidate.lower()
            if lower_candidate.endswith(".xacro"):
                xacro_xml, xacro_error = self._expand_xacro(candidate)
                if xacro_xml:
                    return xacro_xml, ""
                return "", xacro_error or f"Failed to expand xacro file: {candidate}"

            try:
                with open(candidate, "r", encoding="utf-8") as handle:
                    return handle.read(), ""
            except OSError as exc:
                return "", f"Failed reading URDF file '{candidate}': {exc}"

        if not config.ros_param_node:
            return "", "No URDF path and no ros_param_node configured."

        try:
            process = subprocess.run(
                [
                    "ros2",
                    "param",
                    "get",
                    config.ros_param_node,
                    config.ros_param_name,
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=8.0,
            )
        except Exception as exc:
            return "", f"Failed to query ROS parameter '{config.ros_param_name}': {exc}"

        if process.returncode != 0:
            stderr = str(process.stderr or "").strip()
            return "", f"ROS parameter lookup failed (node={config.ros_param_node}, param={config.ros_param_name}): {stderr}"

        raw_stdout = str(process.stdout or "")
        _, _, value = raw_stdout.partition(":")
        candidate_xml = value.strip() if value else raw_stdout.strip()
        if "<robot" not in candidate_xml:
            return "", "ROS parameter did not contain a <robot ...> URDF payload."
        return candidate_xml, ""

    def _expand_xacro(self, xacro_path: str) -> Tuple[str, str]:
        commands: List[List[str]] = []
        direct_xacro = shutil.which("xacro")
        if direct_xacro:
            commands.append([direct_xacro, xacro_path])

        ros2_exec = shutil.which("ros2")
        if ros2_exec:
            commands.append([ros2_exec, "run", "xacro", "xacro", xacro_path])

        commands.append(["python3", "-m", "xacro", xacro_path])

        last_error = "xacro tool not found. Install xacro or provide a resolved .urdf file."
        for command in commands:
            try:
                process = subprocess.run(
                    command,
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=20.0,
                    cwd=os.path.dirname(xacro_path) or None,
                )
            except Exception as exc:
                last_error = f"Failed running {' '.join(command)}: {exc}"
                continue

            if process.returncode != 0:
                stderr = str(process.stderr or process.stdout or "").strip()
                if stderr:
                    last_error = f"xacro failed ({' '.join(command)}): {stderr}"
                else:
                    last_error = f"xacro failed ({' '.join(command)}) with return code {process.returncode}"
                continue

            payload = str(process.stdout or "")
            if "<robot" in payload:
                return payload, ""

            last_error = f"xacro succeeded but output did not contain <robot>: {' '.join(command)}"

        return "", last_error

    def _compile_robot_description_payload(
        self,
        robot: Any,
        urdf_xml: str,
        config: RobotDescriptionResolveConfig,
    ) -> RobotDescriptionV2:
        root = ET.fromstring(urdf_xml)
        robot_name = str(root.attrib.get("name", "robot")).strip() or str(getattr(robot, "name", "") or "").strip() or "robot"
        fallback_proxy = self._default_mesh_proxy_size(robot)
        package_root = self._resolve_package_root(config.urdf_path)

        links_local: Dict[str, List[CompiledCollision]] = {}
        visuals_local: Dict[str, List[_VisualSourceData]] = {}
        link_names: List[str] = []
        for link_el in root.findall("link"):
            link_name = str(link_el.attrib.get("name", "")).strip()
            if not link_name:
                continue

            compiled_collisions: List[CompiledCollision] = []
            for collision_el in link_el.findall("collision"):
                compiled = self._parse_collision(collision_el, fallback_proxy)
                if compiled is not None:
                    compiled_collisions.append(compiled)

            compiled_visuals: List[_VisualSourceData] = []
            for visual_el in link_el.findall("visual"):
                compiled_visual = self._parse_visual(visual_el, link_name)
                if compiled_visual is not None:
                    compiled_visuals.append(compiled_visual)

            links_local[link_name] = compiled_collisions
            visuals_local[link_name] = compiled_visuals
            link_names.append(link_name)

        source_joints: List[_JointSourceData] = []
        child_links: set[str] = set()
        for joint_el in root.findall("joint"):
            joint_name = str(joint_el.attrib.get("name", "")).strip()
            joint_type = str(joint_el.attrib.get("type", "fixed")).strip().lower() or "fixed"
            parent_link = ""
            child_link = ""

            parent_el = joint_el.find("parent")
            child_el = joint_el.find("child")
            if parent_el is not None:
                parent_link = str(parent_el.attrib.get("link", "")).strip()
            if child_el is not None:
                child_link = str(child_el.attrib.get("link", "")).strip()
            if not joint_name or not child_link:
                continue

            origin_el = joint_el.find("origin")
            axis_el = joint_el.find("axis")
            origin_xyz = _parse_vec3(origin_el.attrib.get("xyz") if origin_el is not None else None, (0.0, 0.0, 0.0))
            origin_rpy = _parse_vec3(origin_el.attrib.get("rpy") if origin_el is not None else None, (0.0, 0.0, 0.0))
            axis_xyz = _normalize_joint_axis(
                _parse_vec3(axis_el.attrib.get("xyz") if axis_el is not None else None, (0.0, 0.0, 1.0))
            )

            source_joints.append(
                _JointSourceData(
                    name=joint_name,
                    type=joint_type,
                    parent_link=parent_link,
                    child_link=child_link,
                    origin_xyz=origin_xyz,
                    origin_rpy=origin_rpy,
                    axis_xyz=axis_xyz,
                )
            )
            child_links.add(child_link)

        base_frame = config.base_frame
        if base_frame not in link_names:
            root_link = ""
            for candidate_link in link_names:
                if candidate_link not in child_links:
                    root_link = candidate_link
                    break

            if root_link:
                base_frame = root_link
            elif "base_link" in link_names:
                base_frame = "base_link"
            elif link_names:
                base_frame = link_names[0]
            else:
                base_frame = "base_link"

        link_transforms = self._build_link_transforms(base_frame, source_joints)
        joints_by_child = {joint.child_link: joint for joint in source_joints if joint.child_link}
        links: List[CompiledLink] = []
        mesh_entries_by_group: Dict[str, List[Dict[str, object]]] = {}
        for link_name in link_names:
            local_collisions = links_local.get(link_name, [])
            link_rotation, link_translation = link_transforms.get(link_name, (_identity_rotation(), [0.0, 0.0, 0.0]))
            compiled_collisions: List[CompiledCollision] = []
            for local_collision in local_collisions:
                world_rotation, world_translation = _compose_transforms(
                    link_rotation,
                    link_translation,
                    local_collision.origin_xyz,
                    local_collision.origin_rpy,
                )
                world_collision = CompiledCollision(
                    type=local_collision.type,
                    origin_xyz=world_translation,
                    origin_rpy=_matrix_to_rpy(world_rotation),
                    size_xyz=list(local_collision.size_xyz),
                    radius=local_collision.radius,
                    length=local_collision.length,
                    mesh_uri=local_collision.mesh_uri,
                    proxy_size_xyz=list(local_collision.proxy_size_xyz),
                )
                compiled_collisions.append(world_collision)
            links.append(CompiledLink(name=link_name, collisions=compiled_collisions))

            if config.include_visual_meshes:
                visual_group_root = self._resolve_visual_group_root(link_name, joints_by_child)
                group_rotation, group_translation = link_transforms.get(
                    visual_group_root,
                    (_identity_rotation(), [0.0, 0.0, 0.0]),
                )
                relative_link_rotation, relative_link_translation = _relative_transform(
                    group_rotation,
                    group_translation,
                    link_rotation,
                    link_translation,
                )
                for visual in visuals_local.get(link_name, []):
                    source_path = self._resolve_mesh_path(
                        mesh_uri=visual.mesh_uri,
                        urdf_path=config.urdf_path,
                        package_root=package_root,
                    )
                    if source_path is None:
                        continue
                    mesh_entries_by_group.setdefault(visual_group_root, []).append(
                        {
                            "link_name": link_name,
                            "mesh_uri": visual.mesh_uri,
                            "mesh_path": str(source_path),
                            "origin_xyz": visual.origin_xyz,
                            "origin_rpy": visual.origin_rpy,
                            "link_xyz": relative_link_translation,
                            "link_rpy": _matrix_to_rpy(relative_link_rotation),
                            "mesh_scale_xyz": visual.mesh_scale_xyz,
                            "color_rgb": visual.color_rgb,
                        }
                    )

        joints: List[CompiledJoint] = []
        for joint in source_joints:
            parent_rotation, parent_translation = link_transforms.get(
                joint.parent_link,
                (_identity_rotation(), [0.0, 0.0, 0.0]),
            )
            joint_rotation, joint_translation = _compose_transforms(
                parent_rotation,
                parent_translation,
                joint.origin_xyz,
                joint.origin_rpy,
            )
            joints.append(
                CompiledJoint(
                    name=joint.name,
                    type=joint.type,
                    parent_link=joint.parent_link,
                    child_link=joint.child_link,
                    origin_xyz=joint_translation,
                    origin_rpy=_matrix_to_rpy(joint_rotation),
                    axis_xyz=joint.axis_xyz,
                )
            )

        visual_links: List[CompiledVisual] = []
        mesh_assets = []
        if config.include_visual_meshes and mesh_entries_by_group:
            total_source_triangles = 0
            group_source_triangles: Dict[str, int] = {}
            for visual_group_root, mesh_entries in mesh_entries_by_group.items():
                group_triangles = 0
                for mesh_entry in mesh_entries:
                    source_path = Path(str(mesh_entry.get("mesh_path") or "")).resolve()
                    if not source_path.is_file():
                        continue
                    source_mesh = self._mesh_baker._load_source_mesh(source_path)
                    if source_mesh is None:
                        continue
                    group_triangles += len(source_mesh.faces)
                group_source_triangles[visual_group_root] = group_triangles
                total_source_triangles += group_triangles

            total_budget = None
            if config.body_mesh_mode in {"preview_mesh", "runtime_high_mesh"}:
                total_budget = int(config.visual_mesh_triangle_budget)

            for _ in range(3):
                attempt_visual_links: List[CompiledVisual] = []
                attempt_mesh_assets = []
                actual_total_triangles = 0

                for visual_group_root, mesh_entries in mesh_entries_by_group.items():
                    target_triangles = None
                    if total_budget is not None and config.body_mesh_mode == "preview_mesh":
                        source_triangles = max(0, group_source_triangles.get(visual_group_root, 0))
                        proportional = int(
                            math.ceil(total_budget * (float(source_triangles) / float(max(1, total_source_triangles))))
                        )
                        target_triangles = max(2000, min(60000, proportional))
                    elif total_budget is not None and config.body_mesh_mode == "runtime_high_mesh":
                        source_triangles = max(0, group_source_triangles.get(visual_group_root, 0))
                        proportional = int(
                            math.ceil(total_budget * (float(source_triangles) / float(max(1, total_source_triangles))))
                        )
                        target_triangles = max(5000, min(150000, proportional))

                    combined_asset = None
                    attempt_target = target_triangles
                    for _ in range(4):
                        combined_asset = self._mesh_baker.build_combined_asset(
                            mesh_entries=mesh_entries,
                            description_id_seed=f"{config.urdf_path}:{config.base_frame}:{visual_group_root}",
                            target_triangles=attempt_target,
                            mesh_mode=config.body_mesh_mode,
                        )
                        if combined_asset is None:
                            break
                        asset, _, _ = combined_asset
                        if attempt_target is None or asset.triangle_count <= int(math.ceil(max(1, attempt_target) * 1.05)):
                            break
                        if config.body_mesh_mode == "runtime_high_mesh":
                            attempt_target = max(5000, int(math.floor(attempt_target * 0.85)))
                        else:
                            attempt_target = max(2000, int(math.floor(attempt_target * 0.85)))
                    if combined_asset is None:
                        continue

                    asset, _, _ = combined_asset
                    actual_total_triangles += int(asset.triangle_count)
                    attempt_mesh_assets.append(asset)
                    attempt_visual_links.append(
                        CompiledVisual(
                            name=visual_group_root,
                            frame_id=visual_group_root,
                            mesh_id=asset.mesh_id,
                            origin_xyz=[0.0, 0.0, 0.0],
                            origin_rpy=[0.0, 0.0, 0.0],
                            color_rgb=list(asset.color_rgb),
                        )
                    )

                mesh_assets = attempt_mesh_assets
                visual_links = attempt_visual_links

                if total_budget is None or actual_total_triangles <= int(math.ceil(max(1, total_budget) * 1.05)):
                    break

                per_group_floor = 5000 if config.body_mesh_mode == "runtime_high_mesh" else 2000
                next_budget = int(
                    math.floor(float(total_budget) * (float(total_budget) / float(max(1, actual_total_triangles))) * 0.98)
                )
                next_budget = max(per_group_floor * max(1, len(mesh_entries_by_group)), next_budget)
                if next_budget >= total_budget:
                    break
                total_budget = next_budget

        return RobotDescriptionV2(
            version="v2",
            robot_name=robot_name,
            base_frame=base_frame,
            links=links,
            joints=joints,
            visual_links=visual_links,
            mesh_assets=mesh_assets,
        )

    def _build_link_transforms(
        self,
        base_frame: str,
        joints: List[_JointSourceData],
    ) -> Dict[str, Tuple[List[List[float]], List[float]]]:
        transforms: Dict[str, Tuple[List[List[float]], List[float]]] = {
            base_frame: (_identity_rotation(), [0.0, 0.0, 0.0])
        }

        unresolved = list(joints)
        max_iterations = max(1, len(unresolved) * 2)
        for _ in range(max_iterations):
            if not unresolved:
                break
            progressed = False
            next_unresolved: List[_JointSourceData] = []
            for joint in unresolved:
                parent_transform = transforms.get(joint.parent_link)
                if parent_transform is None:
                    next_unresolved.append(joint)
                    continue

                parent_rotation, parent_translation = parent_transform
                child_rotation, child_translation = _compose_transforms(
                    parent_rotation,
                    parent_translation,
                    joint.origin_xyz,
                    joint.origin_rpy,
                )
                transforms[joint.child_link] = (child_rotation, child_translation)
                progressed = True

            unresolved = next_unresolved
            if not progressed:
                break

        return transforms

    def _resolve_visual_group_root(
        self,
        link_name: str,
        joints_by_child: Dict[str, _JointSourceData],
    ) -> str:
        current = str(link_name or "").strip()
        if not current:
            return current

        while True:
            joint = joints_by_child.get(current)
            if joint is None or joint.type != "fixed" or not str(joint.parent_link or "").strip():
                return current
            current = str(joint.parent_link).strip()

    def _parse_collision(
        self,
        collision_el: ET.Element,
        fallback_mesh_proxy: Tuple[float, float, float],
    ) -> Optional[CompiledCollision]:
        geometry_el = collision_el.find("geometry")
        if geometry_el is None:
            return None

        origin_el = collision_el.find("origin")
        origin_xyz = _parse_vec3(origin_el.attrib.get("xyz") if origin_el is not None else None, (0.0, 0.0, 0.0))
        origin_rpy = _parse_vec3(origin_el.attrib.get("rpy") if origin_el is not None else None, (0.0, 0.0, 0.0))

        box_el = geometry_el.find("box")
        if box_el is not None:
            size_xyz = _parse_vec3(box_el.attrib.get("size"), (0.1, 0.1, 0.1))
            return CompiledCollision(
                type="box",
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                size_xyz=size_xyz,
            )

        sphere_el = geometry_el.find("sphere")
        if sphere_el is not None:
            try:
                radius = float(sphere_el.attrib.get("radius", 0.05))
            except (TypeError, ValueError):
                radius = 0.05
            return CompiledCollision(
                type="sphere",
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                radius=max(0.001, radius),
            )

        cylinder_el = geometry_el.find("cylinder")
        if cylinder_el is not None:
            try:
                radius = float(cylinder_el.attrib.get("radius", 0.05))
            except (TypeError, ValueError):
                radius = 0.05
            try:
                length = float(cylinder_el.attrib.get("length", 0.1))
            except (TypeError, ValueError):
                length = 0.1
            return CompiledCollision(
                type="cylinder",
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                radius=max(0.001, radius),
                length=max(0.001, length),
            )

        capsule_el = geometry_el.find("capsule")
        if capsule_el is not None:
            try:
                radius = float(capsule_el.attrib.get("radius", 0.05))
            except (TypeError, ValueError):
                radius = 0.05
            try:
                length = float(capsule_el.attrib.get("length", 0.1))
            except (TypeError, ValueError):
                length = 0.1
            return CompiledCollision(
                type="capsule",
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                radius=max(0.001, radius),
                length=max(0.001, length),
            )

        mesh_el = geometry_el.find("mesh")
        if mesh_el is not None:
            mesh_uri = str(mesh_el.attrib.get("filename", "")).strip()
            mesh_scale = _parse_vec3(mesh_el.attrib.get("scale"), (1.0, 1.0, 1.0))
            proxy_size = [
                max(0.03, abs(fallback_mesh_proxy[0] * mesh_scale[0])),
                max(0.03, abs(fallback_mesh_proxy[1] * mesh_scale[1])),
                max(0.03, abs(fallback_mesh_proxy[2] * mesh_scale[2])),
            ]
            return CompiledCollision(
                type="mesh_proxy",
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                mesh_uri=mesh_uri,
                proxy_size_xyz=proxy_size,
            )

        return None

    def _parse_visual(
        self,
        visual_el: ET.Element,
        link_name: str,
    ) -> Optional[_VisualSourceData]:
        geometry_el = visual_el.find("geometry")
        if geometry_el is None:
            return None

        mesh_el = geometry_el.find("mesh")
        if mesh_el is None:
            return None

        mesh_uri = str(mesh_el.attrib.get("filename", "")).strip()
        if not mesh_uri:
            return None

        origin_el = visual_el.find("origin")
        origin_xyz = _parse_vec3(origin_el.attrib.get("xyz") if origin_el is not None else None, (0.0, 0.0, 0.0))
        origin_rpy = _parse_vec3(origin_el.attrib.get("rpy") if origin_el is not None else None, (0.0, 0.0, 0.0))
        mesh_scale = _parse_vec3(mesh_el.attrib.get("scale"), (1.0, 1.0, 1.0))

        color_rgb: List[float] = []
        material_el = visual_el.find("material")
        if material_el is not None:
            color_el = material_el.find("color")
            if color_el is not None:
                rgba_tokens = str(color_el.attrib.get("rgba", "")).replace(",", " ").split()
                if len(rgba_tokens) >= 3:
                    try:
                        color_rgb = [
                            max(0.0, min(1.0, float(rgba_tokens[0]))),
                            max(0.0, min(1.0, float(rgba_tokens[1]))),
                            max(0.0, min(1.0, float(rgba_tokens[2]))),
                        ]
                    except (TypeError, ValueError):
                        color_rgb = []

        return _VisualSourceData(
            link_name=link_name,
            mesh_uri=mesh_uri,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            mesh_scale_xyz=mesh_scale,
            color_rgb=color_rgb,
        )

    def _resolve_package_root(self, urdf_path: str) -> Optional[Path]:
        candidate = Path(str(urdf_path or "")).expanduser()
        if not candidate:
            return None
        start_dir = candidate.parent if candidate.suffix else candidate
        current = start_dir.resolve()
        while True:
            if (current / "package.xml").is_file():
                return current
            if current.parent == current:
                break
            current = current.parent
        return None

    def _resolve_mesh_path(
        self,
        mesh_uri: str,
        urdf_path: str,
        package_root: Optional[Path],
    ) -> Optional[Path]:
        raw_uri = str(mesh_uri or "").strip()
        if not raw_uri:
            return None

        if raw_uri.startswith("package://"):
            package_relative = raw_uri[len("package://"):]
            package_name, _, relative_path = package_relative.partition("/")
            if package_root is not None and package_root.name == package_name and relative_path:
                candidate = (package_root / relative_path).resolve()
                if candidate.is_file():
                    return candidate
            return None

        candidate = Path(raw_uri).expanduser()
        if not candidate.is_absolute():
            urdf_file = Path(str(urdf_path or "")).expanduser()
            if urdf_file.is_file():
                candidate = (urdf_file.parent / candidate).resolve()
        if candidate.is_file():
            return candidate
        return None

    def _default_mesh_proxy_size(self, robot: Any) -> Tuple[float, float, float]:
        dimensions = getattr(robot, "dimensions", None)
        if dimensions is not None:
            try:
                length = float(getattr(dimensions, "length", 0.3))
                width = float(getattr(dimensions, "width", 0.2))
                height = float(getattr(dimensions, "height", 0.2))
                return (
                    max(0.05, min(length * 0.35, 0.35)),
                    max(0.05, min(width * 0.35, 0.30)),
                    max(0.05, min((height if height > 0 else 0.2) * 0.25, 0.30)),
                )
            except (TypeError, ValueError):
                pass
        return (0.28, 0.20, 0.18)

    @staticmethod
    def _encode_payload(payload_json: str) -> str:
        compressed = gzip.compress(payload_json.encode("utf-8"))
        return base64.b64encode(compressed).decode("ascii")
