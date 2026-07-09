#!/usr/bin/env python3
"""Generate a profile-driven static human URDF for field teammate mesh debugging.

The visible model is one continuous MakeHuman hm08 body mesh with baked vertex
colors. The normal field-teammate demo uses an MR-side avatar body instead of
this SDK mesh. Keep this generator as an explicit local fallback for validating
mesh transport, orientation, floor placement, and vertex-color preservation.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import io
import json
import math
from pathlib import Path
import struct
from typing import Iterable
from xml.sax.saxutils import escape


@dataclass(frozen=True)
class FieldTeammateProfile:
    height_m: float = 1.75
    sex: str = "unspecified"

    @property
    def normalized_height_m(self) -> float:
        return max(1.35, min(2.10, float(self.height_m)))

    @property
    def normalized_sex(self) -> str:
        value = str(self.sex or "unspecified").strip().lower()
        return value if value in {"female", "male", "unspecified"} else "unspecified"

    @property
    def cache_key(self) -> str:
        raw = f"makehuman_skinned_v1:{self.normalized_height_m:.3f}:{self.normalized_sex}"
        return hashlib.sha1(raw.encode("utf-8")).hexdigest()[:10]


@dataclass(frozen=True)
class FieldTeammateAssetBundle:
    urdf_path: Path
    mesh_root: Path
    profile: FieldTeammateProfile


@dataclass(frozen=True)
class _SegmentedHumanMesh:
    parts: dict[str, list[tuple[list[tuple[float, float, float]], tuple[int, int, int]]]]
    centers: dict[str, tuple[float, float, float]]


_TOOLS_DIR = Path(__file__).resolve().parent
_EXAMPLES_DIR = _TOOLS_DIR.parent
_MAKEHUMAN_BODY_OBJ = (
    _EXAMPLES_DIR
    / ".local_assets"
    / "field_teammate_description"
    / "vendor"
    / "makehuman_hm08_cc0"
    / "field_teammate_human_body.obj"
)
_HOLOLENS2_FABER_GLB = (
    _EXAMPLES_DIR
    / ".local_assets"
    / "field_teammate_description"
    / "vendor"
    / "hololens2_faber_ccby"
    / "hololens_2.glb"
)
_ARTICULATED_PARTS = (
    "pelvis",
    "torso",
    "chest_vest",
    "neck",
    "head",
    "left_upper_arm",
    "left_forearm",
    "left_hand",
    "right_upper_arm",
    "right_forearm",
    "right_hand",
    "left_thigh",
    "left_shin",
    "left_foot",
    "right_thigh",
    "right_shin",
    "right_foot",
)
_HAND_JOINTS = (
    "wrist",
    "palm",
    "thumb_metacarpal",
    "thumb_proximal",
    "thumb_distal",
    "thumb_tip",
    "index_metacarpal",
    "index_proximal",
    "index_intermediate",
    "index_distal",
    "index_tip",
    "middle_metacarpal",
    "middle_proximal",
    "middle_intermediate",
    "middle_distal",
    "middle_tip",
    "ring_metacarpal",
    "ring_proximal",
    "ring_intermediate",
    "ring_distal",
    "ring_tip",
    "little_metacarpal",
    "little_proximal",
    "little_intermediate",
    "little_distal",
    "little_tip",
)


def ensure_field_teammate_articulated_assets(
    profile: FieldTeammateProfile,
    *,
    cache_root: Path | None = None,
    force: bool = False,
) -> FieldTeammateAssetBundle:
    root = cache_root or (Path.home() / ".cache" / "horus" / "field_teammate_profiles")
    bundle_root = root / f"skinned_{profile.cache_key}"
    mesh_root = bundle_root / "meshes"
    urdf_path = bundle_root / "field_teammate_skinned.urdf"

    if force or not urdf_path.is_file() or not (mesh_root / "body.obj").is_file():
        mesh_root.mkdir(parents=True, exist_ok=True)
        _write_materials(mesh_root / "field_teammate_materials.mtl")
        _write_meshes(mesh_root, profile)
        urdf_path.write_text(_build_urdf(profile), encoding="utf-8")

    return FieldTeammateAssetBundle(urdf_path=urdf_path, mesh_root=bundle_root, profile=profile)


def ensure_field_teammate_head_assets(
    profile: FieldTeammateProfile,
    *,
    cache_root: Path | None = None,
    force: bool = False,
) -> FieldTeammateAssetBundle:
    """Generate a compact head-only SDK mesh bound to the live camera frame."""
    root = cache_root or (Path.home() / ".cache" / "horus" / "field_teammate_profiles")
    bundle_root = root / f"head_real_hl2_faber_yaw90_proportional_camera_anchor_{profile.cache_key}"
    mesh_root = bundle_root / "meshes"
    urdf_path = bundle_root / "field_teammate_head.urdf"

    if force or not urdf_path.is_file() or not (mesh_root / "head.obj").is_file():
        mesh_root.mkdir(parents=True, exist_ok=True)
        _write_head_obj(mesh_root / "head.obj", profile)
        urdf_path.write_text(_build_head_urdf(profile), encoding="utf-8")

    return FieldTeammateAssetBundle(urdf_path=urdf_path, mesh_root=bundle_root, profile=profile)


def ensure_field_teammate_head_hand_assets(
    profile: FieldTeammateProfile,
    *,
    cache_root: Path | None = None,
    force: bool = False,
) -> FieldTeammateAssetBundle:
    """Generate a camera-bound head plus TF-bound tracked hand visuals."""
    root = cache_root or (Path.home() / ".cache" / "horus" / "field_teammate_profiles")
    bundle_root = root / f"head_hands_real_hl2_faber_yaw90_proportional_camera_anchor_{profile.cache_key}"
    mesh_root = bundle_root / "meshes"
    urdf_path = bundle_root / "field_teammate_head_hands.urdf"

    required_meshes = (
        mesh_root / "head.obj",
        mesh_root / "hand_palm.obj",
        mesh_root / "hand_joint.obj",
        mesh_root / "hand_tip.obj",
    )
    if force or not urdf_path.is_file() or not all(path.is_file() for path in required_meshes):
        mesh_root.mkdir(parents=True, exist_ok=True)
        _write_head_obj(mesh_root / "head.obj", profile)
        _write_hand_visual_obj(mesh_root / "hand_palm.obj", profile, kind="palm")
        _write_hand_visual_obj(mesh_root / "hand_joint.obj", profile, kind="joint")
        _write_hand_visual_obj(mesh_root / "hand_tip.obj", profile, kind="tip")
        urdf_path.write_text(_build_head_hand_urdf(profile), encoding="utf-8")

    return FieldTeammateAssetBundle(urdf_path=urdf_path, mesh_root=bundle_root, profile=profile)


def _profile_scale(profile: FieldTeammateProfile) -> float:
    return profile.normalized_height_m / 1.75


def _sex_shape(profile: FieldTeammateProfile) -> tuple[float, float, float]:
    scale = _profile_scale(profile)
    sex = profile.normalized_sex
    if sex == "female":
        shoulder_width = 0.38 * scale
        hip_width = 0.34 * scale
        torso_depth = 0.18 * scale
    elif sex == "male":
        shoulder_width = 0.44 * scale
        hip_width = 0.30 * scale
        torso_depth = 0.20 * scale
    else:
        shoulder_width = 0.41 * scale
        hip_width = 0.32 * scale
        torso_depth = 0.19 * scale
    return shoulder_width, hip_width, torso_depth


def _write_materials(path: Path) -> None:
    lines: list[str] = []
    for material, color in _MATERIAL_COLORS.items():
        lines.extend(
            [
                f"newmtl {material}",
                f"Kd {color[0]:.6f} {color[1]:.6f} {color[2]:.6f}",
            ]
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def _write_meshes(mesh_root: Path, profile: FieldTeammateProfile) -> None:
    _write_body_obj(mesh_root / "body.obj", profile)


_MATERIAL_COLORS: dict[str, tuple[float, float, float]] = {
    "skin": (0.78, 0.58, 0.43),
    "shirt": (0.12, 0.27, 0.48),
    "vest": (0.95, 0.48, 0.10),
    "pants": (0.12, 0.15, 0.20),
    "boots": (0.04, 0.04, 0.04),
    "gloves": (0.06, 0.07, 0.08),
}


def _write_body_obj(path: Path, profile: FieldTeammateProfile) -> None:
    native_vertices, faces = _load_makehuman_body()
    y_values = [vertex[1] for vertex in native_vertices]
    z_values = [vertex[2] for vertex in native_vertices]
    y_min = min(y_values)
    y_max = max(y_values)
    z_center = (min(z_values) + max(z_values)) * 0.5

    ros_vertices = [
        _native_to_ros(vertex, profile=profile, y_min=y_min, y_max=y_max, z_center=z_center)
        for vertex in native_vertices
    ]
    color_sums = [[0.0, 0.0, 0.0] for _ in native_vertices]
    color_counts = [0 for _ in native_vertices]

    for face in faces:
        native_face = [native_vertices[index] for index in face]
        centroid = (
            sum(vertex[0] for vertex in native_face) / 3.0,
            sum(vertex[1] for vertex in native_face) / 3.0,
            sum(vertex[2] for vertex in native_face) / 3.0,
        )
        material = _part_material(_classify_part(centroid, y_min=y_min, y_max=y_max))
        color = _MATERIAL_COLORS.get(material, _MATERIAL_COLORS["skin"])
        for vertex_index in face:
            color_sums[vertex_index][0] += color[0]
            color_sums[vertex_index][1] += color[1]
            color_sums[vertex_index][2] += color[2]
            color_counts[vertex_index] += 1

    with path.open("w", encoding="utf-8") as handle:
        handle.write("mtllib field_teammate_materials.mtl\n")
        handle.write("usemtl skin\n")
        for vertex_index, (x, y, z) in enumerate(ros_vertices):
            count = max(1, color_counts[vertex_index])
            color = (
                color_sums[vertex_index][0] / count,
                color_sums[vertex_index][1] / count,
                color_sums[vertex_index][2] / count,
            )
            handle.write(
                f"v {x:.6f} {y:.6f} {z:.6f} "
                f"{color[0]:.6f} {color[1]:.6f} {color[2]:.6f}\n"
            )
        for i0, i1, i2 in faces:
            handle.write(f"f {i0 + 1} {i1 + 1} {i2 + 1}\n")


def field_teammate_body_link_layout(profile: FieldTeammateProfile) -> dict[str, tuple[float, float, float]]:
    """Return base-relative centers for the generated true-mesh body links."""
    return dict(_build_segmented_human_mesh(profile).centers)


def _load_makehuman_body() -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    if not _MAKEHUMAN_BODY_OBJ.is_file():
        raise FileNotFoundError(f"MakeHuman field teammate mesh not found: {_MAKEHUMAN_BODY_OBJ}")

    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    with _MAKEHUMAN_BODY_OBJ.open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.startswith("v "):
                parts = line.split()
                if len(parts) >= 4:
                    vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif line.startswith("f "):
                indices = []
                for token in line.split()[1:]:
                    head = token.split("/", 1)[0]
                    if head:
                        indices.append(int(head) - 1)
                if len(indices) >= 3:
                    for index in range(1, len(indices) - 1):
                        faces.append((indices[0], indices[index], indices[index + 1]))
    if not vertices or not faces:
        raise ValueError(f"MakeHuman field teammate mesh is empty: {_MAKEHUMAN_BODY_OBJ}")
    return vertices, faces


def _height_fraction(native_y: float, y_min: float, y_max: float) -> float:
    return (native_y - y_min) / max(1e-6, y_max - y_min)


def _profile_lateral_factor(height_fraction: float, profile: FieldTeammateProfile) -> float:
    sex = profile.normalized_sex
    if sex == "female":
        shoulder = 0.96
        hip = 1.10
    elif sex == "male":
        shoulder = 1.09
        hip = 0.98
    else:
        shoulder = 1.0
        hip = 1.0

    if height_fraction < 0.45:
        return hip
    if height_fraction > 0.62:
        return shoulder
    t = (height_fraction - 0.45) / max(1e-6, 0.62 - 0.45)
    return hip + (shoulder - hip) * t


def _profile_depth_factor(height_fraction: float, profile: FieldTeammateProfile) -> float:
    if profile.normalized_sex == "male" and 0.50 < height_fraction < 0.82:
        return 1.04
    if profile.normalized_sex == "female" and 0.36 < height_fraction < 0.58:
        return 1.03
    return 1.0


def _native_to_ros(
    vertex: tuple[float, float, float],
    *,
    profile: FieldTeammateProfile,
    y_min: float,
    y_max: float,
    z_center: float,
) -> tuple[float, float, float]:
    native_x, native_y, native_z = vertex
    scale = profile.normalized_height_m / max(1e-6, y_max - y_min)
    h = _height_fraction(native_y, y_min, y_max)
    lateral = _profile_lateral_factor(h, profile)
    depth = _profile_depth_factor(h, profile)
    ros_x = ((native_z - z_center) * depth) * scale
    ros_y = (-native_x * lateral) * scale
    ros_z = (native_y - y_min) * scale
    return (ros_x, ros_y, ros_z)


def _side(native_x: float) -> str:
    return "left" if native_x >= 0.0 else "right"


def _classify_part(
    centroid: tuple[float, float, float],
    *,
    y_min: float,
    y_max: float,
) -> str:
    x, y, z = centroid
    h = _height_fraction(y, y_min, y_max)
    abs_x = abs(x)
    side = _side(x)

    if h >= 0.90 and abs_x < 1.20:
        return "head"
    if h >= 0.845 and abs_x < 1.15:
        return "neck"

    # The MakeHuman hm08 mesh is close to a T-pose/A-pose. Arms are high and
    # far from the trunk; feet/legs are the low geometry near the floor.
    if 0.46 <= h <= 0.78 and abs_x > 3.65:
        return f"{side}_hand"
    if 0.45 <= h <= 0.82 and abs_x > 2.55:
        return f"{side}_forearm"
    if 0.50 <= h <= 0.86 and abs_x > 1.25:
        return f"{side}_upper_arm"

    if h < 0.12:
        return f"{side}_foot"
    if h < 0.32:
        return f"{side}_shin"
    if h < 0.52:
        return f"{side}_thigh"
    if h < 0.615:
        return "pelvis"
    if h < 0.735:
        return "torso"
    if h < 0.855:
        return "chest_vest"
    return "head"


def _part_material(part: str) -> str:
    if part in {"head", "neck"} or part.endswith("_forearm") or part.endswith("_hand"):
        return "skin"
    if part.endswith("_upper_arm") or part == "torso":
        return "shirt"
    if part == "chest_vest":
        return "vest"
    if part.endswith("_foot"):
        return "boots"
    if part == "pelvis" or part.endswith("_thigh") or part.endswith("_shin"):
        return "pants"
    return "skin"


def _build_segmented_human_mesh(profile: FieldTeammateProfile) -> _SegmentedHumanMesh:
    native_vertices, faces = _load_makehuman_body()
    y_values = [vertex[1] for vertex in native_vertices]
    z_values = [vertex[2] for vertex in native_vertices]
    y_min = min(y_values)
    y_max = max(y_values)
    z_center = (min(z_values) + max(z_values)) * 0.5

    parts: dict[str, list[tuple[list[tuple[float, float, float]], tuple[int, int, int]]]] = {
        part: [] for part in _ARTICULATED_PARTS
    }
    all_part_vertices: dict[str, list[tuple[float, float, float]]] = {
        part: [] for part in _ARTICULATED_PARTS
    }
    for face in faces:
        native_face = [native_vertices[index] for index in face]
        centroid = (
            sum(vertex[0] for vertex in native_face) / 3.0,
            sum(vertex[1] for vertex in native_face) / 3.0,
            sum(vertex[2] for vertex in native_face) / 3.0,
        )
        part = _classify_part(centroid, y_min=y_min, y_max=y_max)
        ros_face = [
            _native_to_ros(vertex, profile=profile, y_min=y_min, y_max=y_max, z_center=z_center)
            for vertex in native_face
        ]
        parts[part].append((ros_face, (1, 2, 3)))
        all_part_vertices[part].extend(ros_face)

    centers: dict[str, tuple[float, float, float]] = {}
    for part in _ARTICULATED_PARTS:
        vertices = all_part_vertices.get(part) or []
        if not vertices:
            centers[part] = _fallback_part_center(part, profile)
            continue
        mins = [min(vertex[i] for vertex in vertices) for i in range(3)]
        maxs = [max(vertex[i] for vertex in vertices) for i in range(3)]
        centers[part] = (
            (mins[0] + maxs[0]) * 0.5,
            (mins[1] + maxs[1]) * 0.5,
            (mins[2] + maxs[2]) * 0.5,
        )

    local_parts: dict[str, list[tuple[list[tuple[float, float, float]], tuple[int, int, int]]]] = {}
    for part, face_entries in parts.items():
        center = centers[part]
        local_parts[part] = [
            (
                [
                    (vertex[0] - center[0], vertex[1] - center[1], vertex[2] - center[2])
                    for vertex in ros_face
                ],
                face,
            )
            for ros_face, face in face_entries
        ]

    return _SegmentedHumanMesh(parts=local_parts, centers=centers)


def _fallback_part_center(part: str, profile: FieldTeammateProfile) -> tuple[float, float, float]:
    height = profile.normalized_height_m
    side = 1.0 if part.startswith("left") else -1.0
    if part == "head":
        return (0.0, 0.0, 0.93 * height)
    if part == "neck":
        return (0.0, 0.0, 0.855 * height)
    if part == "chest_vest":
        return (0.0, 0.0, 0.775 * height)
    if part == "torso":
        return (0.0, 0.0, 0.675 * height)
    if part == "pelvis":
        return (0.0, 0.0, 0.54 * height)
    if "upper_arm" in part:
        return (0.0, side * 0.26, 0.66 * height)
    if "forearm" in part:
        return (0.0, side * 0.30, 0.46 * height)
    if "hand" in part:
        return (0.0, side * 0.28, 0.23 * height)
    if "thigh" in part:
        return (0.0, side * 0.09, 0.34 * height)
    if "shin" in part:
        return (0.0, side * 0.08, 0.18 * height)
    if "foot" in part:
        return (0.055, side * 0.08, 0.04 * height)
    return (0.0, 0.0, height * 0.5)


def _write_obj_part(
    path: Path,
    faces: list[tuple[list[tuple[float, float, float]], tuple[int, int, int]]],
    material: str,
) -> None:
    with path.open("w", encoding="utf-8") as handle:
        handle.write("mtllib field_teammate_materials.mtl\n")
        handle.write(f"usemtl {material}\n")
        vertex_index = 1
        for vertices, _ in faces:
            for x, y, z in vertices:
                handle.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
            handle.write(f"f {vertex_index} {vertex_index + 1} {vertex_index + 2}\n")
            vertex_index += 3


def _visual(link: str, mesh: str) -> str:
    return (
        f'  <link name="{escape(link)}">\n'
        "    <visual>\n"
        f'      <geometry><mesh filename="meshes/{escape(mesh)}.obj"/></geometry>\n'
        "    </visual>\n"
        "  </link>\n"
    )


def _joint(parent: str, child: str, xyz: tuple[float, float, float]) -> str:
    return (
        f'  <joint name="{escape(parent)}_to_{escape(child)}" type="continuous">\n'
        f'    <parent link="{escape(parent)}"/>\n'
        f'    <child link="{escape(child)}"/>\n'
        f'    <origin xyz="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" rpy="0 0 0"/>\n'
        '    <axis xyz="0 0 1"/>\n'
        "  </joint>\n"
    )


def _append_uv_ellipsoid(
    vertices: list[tuple[float, float, float, float, float, float]],
    faces: list[tuple[int, int, int]],
    *,
    center: tuple[float, float, float],
    radii: tuple[float, float, float],
    color: tuple[float, float, float],
    rings: int = 12,
    segments: int = 24,
) -> None:
    start = len(vertices)
    for ring in range(rings + 1):
        phi = math.pi * float(ring) / float(rings)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
        for segment in range(segments):
            theta = math.tau * float(segment) / float(segments)
            x = center[0] + (radii[0] * sin_phi * math.cos(theta))
            y = center[1] + (radii[1] * sin_phi * math.sin(theta))
            z = center[2] + (radii[2] * cos_phi)
            vertices.append((x, y, z, color[0], color[1], color[2]))

    for ring in range(rings):
        for segment in range(segments):
            next_segment = (segment + 1) % segments
            i0 = start + (ring * segments) + segment
            i1 = start + ((ring + 1) * segments) + segment
            i2 = start + ((ring + 1) * segments) + next_segment
            i3 = start + (ring * segments) + next_segment
            if ring > 0:
                faces.append((i0, i1, i3))
            if ring < rings - 1:
                faces.append((i3, i1, i2))


def _append_box(
    vertices: list[tuple[float, float, float, float, float, float]],
    faces: list[tuple[int, int, int]],
    *,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    color: tuple[float, float, float],
) -> None:
    start = len(vertices)
    hx, hy, hz = size[0] * 0.5, size[1] * 0.5, size[2] * 0.5
    corners = (
        (-hx, -hy, -hz),
        (hx, -hy, -hz),
        (hx, hy, -hz),
        (-hx, hy, -hz),
        (-hx, -hy, hz),
        (hx, -hy, hz),
        (hx, hy, hz),
        (-hx, hy, hz),
    )
    for x, y, z in corners:
        vertices.append((center[0] + x, center[1] + y, center[2] + z, color[0], color[1], color[2]))

    for i0, i1, i2 in (
        (0, 1, 2), (0, 2, 3),
        (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1),
        (1, 5, 6), (1, 6, 2),
        (2, 6, 7), (2, 7, 3),
        (3, 7, 4), (3, 4, 0),
    ):
        faces.append((start + i0, start + i1, start + i2))


def _rotate_xyz(point: tuple[float, float, float], rotation: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = point
    roll, pitch, yaw = rotation
    if roll:
        cos_r = math.cos(roll)
        sin_r = math.sin(roll)
        y, z = (y * cos_r) - (z * sin_r), (y * sin_r) + (z * cos_r)
    if pitch:
        cos_p = math.cos(pitch)
        sin_p = math.sin(pitch)
        x, z = (x * cos_p) + (z * sin_p), (-x * sin_p) + (z * cos_p)
    if yaw:
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        x, y = (x * cos_y) - (y * sin_y), (x * sin_y) + (y * cos_y)
    return (x, y, z)


def _append_oriented_box(
    vertices: list[tuple[float, float, float, float, float, float]],
    faces: list[tuple[int, int, int]],
    *,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    color: tuple[float, float, float],
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    start = len(vertices)
    hx, hy, hz = size[0] * 0.5, size[1] * 0.5, size[2] * 0.5
    corners = (
        (-hx, -hy, -hz),
        (hx, -hy, -hz),
        (hx, hy, -hz),
        (-hx, hy, -hz),
        (-hx, -hy, hz),
        (hx, -hy, hz),
        (hx, hy, hz),
        (-hx, hy, hz),
    )
    for corner in corners:
        x, y, z = _rotate_xyz(corner, rotation)
        vertices.append((center[0] + x, center[1] + y, center[2] + z, color[0], color[1], color[2]))

    for i0, i1, i2 in (
        (0, 1, 2), (0, 2, 3),
        (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1),
        (1, 5, 6), (1, 6, 2),
        (2, 6, 7), (2, 7, 3),
        (3, 7, 4), (3, 4, 0),
    ):
        faces.append((start + i0, start + i1, start + i2))


_GLTF_COMPONENT_FORMATS = {
    5120: ("b", 1),
    5121: ("B", 1),
    5122: ("h", 2),
    5123: ("H", 2),
    5125: ("I", 4),
    5126: ("f", 4),
}
_GLTF_TYPE_COMPONENTS = {
    "SCALAR": 1,
    "VEC2": 2,
    "VEC3": 3,
    "VEC4": 4,
}


def _read_glb(path: Path) -> tuple[dict, bytes]:
    data = path.read_bytes()
    if len(data) < 20:
        raise ValueError(f"GLB file is too small: {path}")
    magic, version, declared_length = struct.unpack_from("<4sII", data, 0)
    if magic != b"glTF" or version != 2:
        raise ValueError(f"Unsupported GLB header in {path}")
    if declared_length > len(data):
        raise ValueError(f"Truncated GLB file: {path}")

    gltf: dict | None = None
    binary: bytes | None = None
    offset = 12
    while offset + 8 <= declared_length:
        chunk_length, chunk_type = struct.unpack_from("<I4s", data, offset)
        offset += 8
        chunk = data[offset : offset + chunk_length]
        offset += chunk_length
        if chunk_type == b"JSON":
            gltf = json.loads(chunk.decode("utf-8"))
        elif chunk_type == b"BIN\x00":
            binary = chunk

    if gltf is None or binary is None:
        raise ValueError(f"GLB file must contain JSON and BIN chunks: {path}")
    return gltf, binary


def _read_gltf_accessor(gltf: dict, binary: bytes, accessor_index: int) -> list[tuple[float, ...]]:
    accessor = gltf["accessors"][accessor_index]
    buffer_view = gltf["bufferViews"][accessor["bufferView"]]
    component_format, component_size = _GLTF_COMPONENT_FORMATS[accessor["componentType"]]
    component_count = _GLTF_TYPE_COMPONENTS[accessor["type"]]
    byte_stride = buffer_view.get("byteStride", component_size * component_count)
    byte_offset = buffer_view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    unpack_format = "<" + (component_format * component_count)

    values: list[tuple[float, ...]] = []
    for index in range(accessor["count"]):
        raw = struct.unpack_from(unpack_format, binary, byte_offset + (index * byte_stride))
        values.append(tuple(float(value) for value in raw))
    return values


def _load_gltf_texture_sampler(gltf: dict, binary: bytes, primitive: dict):
    material_index = primitive.get("material")
    if material_index is None:
        return None
    material = gltf.get("materials", [])[material_index]
    texture_info = material.get("pbrMetallicRoughness", {}).get("baseColorTexture")
    if not texture_info:
        return None
    texture = gltf.get("textures", [])[texture_info["index"]]
    image = gltf.get("images", [])[texture["source"]]
    buffer_view = gltf["bufferViews"][image["bufferView"]]
    byte_offset = buffer_view.get("byteOffset", 0)
    byte_length = buffer_view["byteLength"]
    try:
        from PIL import Image

        loaded = Image.open(io.BytesIO(binary[byte_offset : byte_offset + byte_length])).convert("RGB")
    except Exception:
        return None

    width, height = loaded.size
    pixels = loaded.load()

    def sample(uv: tuple[float, float]) -> tuple[float, float, float]:
        u = uv[0] % 1.0
        v = uv[1] % 1.0
        px = min(width - 1, max(0, int(round(u * (width - 1)))))
        py = min(height - 1, max(0, int(round((1.0 - v) * (height - 1)))))
        red, green, blue = pixels[px, py]
        # Keep dark headset surfaces visible in MR without losing the texture.
        return (
            max(0.035, red / 255.0),
            max(0.035, green / 255.0),
            max(0.035, blue / 255.0),
        )

    return sample


def _append_hololens2_glb_mesh(
    vertices: list[tuple[float, float, float, float, float, float]],
    faces: list[tuple[int, int, int]],
    *,
    profile: FieldTeammateProfile,
) -> bool:
    if not _HOLOLENS2_FABER_GLB.is_file():
        return False

    try:
        gltf, binary = _read_glb(_HOLOLENS2_FABER_GLB)
        mesh = gltf["meshes"][0]
        primitive = mesh["primitives"][0]
        positions = _read_gltf_accessor(gltf, binary, primitive["attributes"]["POSITION"])
        texcoords = _read_gltf_accessor(gltf, binary, primitive["attributes"].get("TEXCOORD_0", -1)) \
            if "TEXCOORD_0" in primitive["attributes"] else []
        raw_indices = _read_gltf_accessor(gltf, binary, primitive["indices"]) if "indices" in primitive else []
    except Exception:
        return False

    if not positions:
        return False

    indices = [int(value[0]) for value in raw_indices] if raw_indices else list(range(len(positions)))
    if len(indices) < 3:
        return False

    scale = _profile_scale(profile)
    xs = [position[0] for position in positions]
    ys = [position[1] for position in positions]
    zs = [position[2] for position in positions]
    center = (
        (min(xs) + max(xs)) * 0.5,
        (min(ys) + max(ys)) * 0.5,
        (min(zs) + max(zs)) * 0.5,
    )
    rotated_positions = [
        _rotate_xyz(
            (position[0] - center[0], position[1] - center[1], position[2] - center[2]),
            (0.0, 0.0, -math.pi * 0.5),
        )
        for position in positions
    ]
    rotated_xs = [position[0] for position in rotated_positions]
    rotated_ys = [position[1] for position in rotated_positions]
    rotated_zs = [position[2] for position in rotated_positions]
    depth = max(1e-6, max(rotated_xs) - min(rotated_xs))
    width = max(1e-6, max(rotated_ys) - min(rotated_ys))
    height = max(1e-6, max(rotated_zs) - min(rotated_zs))
    asset_scale = (0.300 * scale) / max(depth, width, height)
    front_offset = (0.030 * scale) - (max(rotated_xs) * asset_scale)
    z_offset = 0.018 * scale
    sample_color = _load_gltf_texture_sampler(gltf, binary, primitive)

    start = len(vertices)
    for index, position in enumerate(positions):
        color = (0.090, 0.095, 0.105)
        if sample_color is not None and index < len(texcoords):
            color = sample_color((texcoords[index][0], texcoords[index][1]))
        local_x, local_y, local_z = rotated_positions[index]
        x = (local_x * asset_scale) + front_offset
        y = local_y * asset_scale
        z = (local_z * asset_scale) + z_offset
        vertices.append((x, y, z, color[0], color[1], color[2]))

    for offset in range(0, len(indices) - 2, 3):
        i0, i1, i2 = indices[offset], indices[offset + 1], indices[offset + 2]
        if 0 <= i0 < len(positions) and 0 <= i1 < len(positions) and 0 <= i2 < len(positions):
            faces.append((start + i0, start + i1, start + i2))
    return True


def _append_procedural_hololens2(
    vertices: list[tuple[float, float, float, float, float, float]],
    faces: list[tuple[int, int, int]],
    *,
    profile: FieldTeammateProfile,
) -> None:
    scale = _profile_scale(profile)
    dark_shell = (0.028, 0.032, 0.038)
    graphite = (0.095, 0.100, 0.108)
    black_glass = (0.010, 0.014, 0.018)
    lens_blue = (0.045, 0.135, 0.180)
    sensor_glass = (0.006, 0.010, 0.014)
    silver = (0.58, 0.61, 0.64)

    for side in (-1.0, 1.0):
        _append_box(
            vertices,
            faces,
            center=(-0.002 * scale, side * 0.074 * scale, -0.016 * scale),
            size=(0.026 * scale, 0.115 * scale, 0.066 * scale),
            color=lens_blue,
        )
    _append_box(
        vertices,
        faces,
        center=(-0.004 * scale, 0.0, 0.018 * scale),
        size=(0.030 * scale, 0.290 * scale, 0.030 * scale),
        color=black_glass,
    )
    _append_box(
        vertices,
        faces,
        center=(-0.045 * scale, 0.0, 0.067 * scale),
        size=(0.070 * scale, 0.335 * scale, 0.046 * scale),
        color=dark_shell,
    )
    _append_box(
        vertices,
        faces,
        center=(0.004 * scale, 0.0, 0.055 * scale),
        size=(0.024 * scale, 0.242 * scale, 0.020 * scale),
        color=graphite,
    )
    for y in (-0.096, -0.040, 0.040, 0.096):
        _append_uv_ellipsoid(
            vertices,
            faces,
            center=(0.019 * scale, y * scale, 0.056 * scale),
            radii=(0.010 * scale, 0.010 * scale, 0.010 * scale),
            color=sensor_glass,
            rings=5,
            segments=10,
        )
    _append_box(
        vertices,
        faces,
        center=(0.017 * scale, 0.0, 0.032 * scale),
        size=(0.018 * scale, 0.050 * scale, 0.020 * scale),
        color=silver,
    )
    _append_box(
        vertices,
        faces,
        center=(-0.017 * scale, 0.0, -0.057 * scale),
        size=(0.034 * scale, 0.072 * scale, 0.038 * scale),
        color=graphite,
    )
    for side, yaw in ((1.0, -0.18), (-1.0, 0.18)):
        _append_uv_ellipsoid(
            vertices,
            faces,
            center=(-0.060 * scale, side * 0.159 * scale, 0.020 * scale),
            radii=(0.052 * scale, 0.028 * scale, 0.048 * scale),
            color=dark_shell,
            rings=8,
            segments=14,
        )
        _append_oriented_box(
            vertices,
            faces,
            center=(-0.128 * scale, side * 0.150 * scale, 0.036 * scale),
            size=(0.165 * scale, 0.027 * scale, 0.032 * scale),
            color=dark_shell,
            rotation=(0.0, 0.0, yaw),
        )
        _append_oriented_box(
            vertices,
            faces,
            center=(-0.130 * scale, side * 0.146 * scale, -0.014 * scale),
            size=(0.135 * scale, 0.018 * scale, 0.020 * scale),
            color=graphite,
            rotation=(0.0, 0.0, yaw),
        )
    _append_box(
        vertices,
        faces,
        center=(-0.218 * scale, 0.0, 0.015 * scale),
        size=(0.050 * scale, 0.285 * scale, 0.056 * scale),
        color=dark_shell,
    )
    _append_uv_ellipsoid(
        vertices,
        faces,
        center=(-0.254 * scale, 0.0, 0.015 * scale),
        radii=(0.024 * scale, 0.058 * scale, 0.058 * scale),
        color=graphite,
        rings=8,
        segments=18,
    )
    _append_box(
        vertices,
        faces,
        center=(-0.282 * scale, 0.0, 0.015 * scale),
        size=(0.014 * scale, 0.070 * scale, 0.070 * scale),
        color=black_glass,
    )


def _write_head_obj(path: Path, profile: FieldTeammateProfile) -> None:
    scale = _profile_scale(profile)
    skin_color = (0.78, 0.58, 0.43)

    vertices: list[tuple[float, float, float, float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    # ROS head-camera frame convention in this demo: +X forward, +Y left, +Z up.
    # The HoloLens camera frame is near the face. Extract the real MakeHuman
    # CC0 head/neck geometry and recenter it so the head volume sits mostly
    # behind that camera frame.
    if _MAKEHUMAN_BODY_OBJ.is_file():
        native_vertices, native_faces = _load_makehuman_body()
        y_values = [vertex[1] for vertex in native_vertices]
        z_values = [vertex[2] for vertex in native_vertices]
        y_min = min(y_values)
        y_max = max(y_values)
        z_center = (min(z_values) + max(z_values)) * 0.5
        ros_vertices = [
            _native_to_ros(vertex, profile=profile, y_min=y_min, y_max=y_max, z_center=z_center)
            for vertex in native_vertices
        ]
        selected_faces: list[tuple[int, int, int]] = []
        selected_indices: set[int] = set()
        for face in native_faces:
            centroid = (
                sum(native_vertices[index][0] for index in face) / 3.0,
                sum(native_vertices[index][1] for index in face) / 3.0,
                sum(native_vertices[index][2] for index in face) / 3.0,
            )
            if _classify_part(centroid, y_min=y_min, y_max=y_max) not in {"head", "neck"}:
                continue
            selected_faces.append(face)
            selected_indices.update(face)

        if selected_faces and selected_indices:
            selected_points = [ros_vertices[index] for index in selected_indices]
            front_x = max(point[0] for point in selected_points)
            eye_z = profile.normalized_height_m * 0.922
            index_map: dict[int, int] = {}
            for source_index in sorted(selected_indices):
                x, y, z = ros_vertices[source_index]
                index_map[source_index] = len(vertices)
                vertices.append(
                    (
                        x - (front_x + (0.018 * scale)),
                        y,
                        z - eye_z,
                        skin_color[0],
                        skin_color[1],
                        skin_color[2],
                    )
                )
            for i0, i1, i2 in selected_faces:
                if i0 in index_map and i1 in index_map and i2 in index_map:
                    faces.append((index_map[i0], index_map[i1], index_map[i2]))

    if not vertices:
        _append_uv_ellipsoid(
            vertices,
            faces,
            center=(-0.095 * scale, 0.0, -0.020 * scale),
            radii=(0.105 * scale, 0.125 * scale, 0.160 * scale),
            color=skin_color,
        )

    if not _append_hololens2_glb_mesh(vertices, faces, profile=profile):
        _append_procedural_hololens2(vertices, faces, profile=profile)

    with path.open("w", encoding="utf-8") as handle:
        handle.write("# HORUS field teammate head visual mesh from MakeHuman CC0 with HoloLens 2 headset overlay\n")
        handle.write("# HoloLens asset credit: \"Hololens 2\" (https://skfb.ly/onHXB) by Faber is licensed under Creative Commons Attribution (http://creativecommons.org/licenses/by/4.0/).\n")
        for x, y, z, r, g, b in vertices:
            handle.write(f"v {x:.6f} {y:.6f} {z:.6f} {r:.6f} {g:.6f} {b:.6f}\n")
        for i0, i1, i2 in faces:
            handle.write(f"f {i0 + 1} {i1 + 1} {i2 + 1}\n")


def _write_hand_visual_obj(path: Path, profile: FieldTeammateProfile, *, kind: str) -> None:
    scale = _profile_scale(profile)
    skin_color = (0.78, 0.58, 0.43)
    glove_color = (0.06, 0.07, 0.08)

    vertices: list[tuple[float, float, float, float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    if kind == "palm":
        _append_uv_ellipsoid(
            vertices,
            faces,
            center=(0.0, 0.0, 0.0),
            radii=(0.050 * scale, 0.040 * scale, 0.018 * scale),
            color=skin_color,
            rings=8,
            segments=16,
        )
        _append_box(
            vertices,
            faces,
            center=(-0.030 * scale, 0.0, -0.002 * scale),
            size=(0.050 * scale, 0.060 * scale, 0.026 * scale),
            color=glove_color,
        )
    elif kind == "tip":
        _append_uv_ellipsoid(
            vertices,
            faces,
            center=(0.0, 0.0, 0.0),
            radii=(0.013 * scale, 0.013 * scale, 0.011 * scale),
            color=skin_color,
            rings=6,
            segments=12,
        )
    else:
        _append_uv_ellipsoid(
            vertices,
            faces,
            center=(0.0, 0.0, 0.0),
            radii=(0.016 * scale, 0.014 * scale, 0.012 * scale),
            color=skin_color,
            rings=6,
            segments=12,
        )

    with path.open("w", encoding="utf-8") as handle:
        handle.write(f"# HORUS field teammate hand {kind} visual mesh\n")
        for x, y, z, r, g, b in vertices:
            handle.write(f"v {x:.6f} {y:.6f} {z:.6f} {r:.6f} {g:.6f} {b:.6f}\n")
        for i0, i1, i2 in faces:
            handle.write(f"f {i0 + 1} {i1 + 1} {i2 + 1}\n")


def _hand_mesh_name(joint: str) -> str:
    if joint == "palm":
        return "hand_palm"
    if joint.endswith("_tip"):
        return "hand_tip"
    return "hand_joint"


def _hand_fallback_xyz(side: str, joint: str, profile: FieldTeammateProfile) -> tuple[float, float, float]:
    scale = _profile_scale(profile)
    side_sign = 1.0 if side == "left" else -1.0
    finger_y_offsets = {
        "thumb": -0.060,
        "index": 0.030,
        "middle": 0.000,
        "ring": -0.030,
        "little": -0.058,
    }
    segment_x_offsets = {
        "metacarpal": 0.025,
        "proximal": 0.070,
        "intermediate": 0.105,
        "distal": 0.135,
        "tip": 0.165,
    }
    if joint == "wrist":
        return (0.180 * scale, side_sign * 0.205 * scale, -0.410 * scale)
    if joint == "palm":
        return (0.235 * scale, side_sign * 0.220 * scale, -0.390 * scale)

    finger = joint.split("_", 1)[0]
    segment = joint.rsplit("_", 1)[-1]
    base_y = 0.220 + finger_y_offsets.get(finger, 0.0)
    x = 0.245 + segment_x_offsets.get(segment, 0.055)
    z = -0.385
    if finger == "thumb":
        x -= 0.030
        z -= 0.015
    return (x * scale, side_sign * base_y * scale, z * scale)


def _build_head_urdf(profile: FieldTeammateProfile) -> str:
    return (
        '<?xml version="1.0"?>\n'
        '<robot name="field_teammate_head">\n'
        '  <link name="camera"/>\n'
        '  <link name="head_visual">\n'
        "    <visual>\n"
        '      <geometry><mesh filename="meshes/head.obj"/></geometry>\n'
        "    </visual>\n"
        "  </link>\n"
        '  <joint name="camera_to_head_visual" type="fixed">\n'
        '    <parent link="camera"/>\n'
        '    <child link="head_visual"/>\n'
        '    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        "  </joint>\n"
        "</robot>\n"
    )


def _build_head_hand_urdf(profile: FieldTeammateProfile) -> str:
    lines = [
        '<?xml version="1.0"?>',
        '<robot name="field_teammate_head_hands">',
        '  <link name="camera"/>',
        '  <link name="head_visual">',
        "    <visual>",
        '      <geometry><mesh filename="meshes/head.obj"/></geometry>',
        "    </visual>",
        "  </link>",
        '  <joint name="camera_to_head_visual" type="fixed">',
        '    <parent link="camera"/>',
        '    <child link="head_visual"/>',
        '    <origin xyz="0 0 0" rpy="0 0 0"/>',
        "  </joint>",
    ]

    for side in ("left", "right"):
        for joint in _HAND_JOINTS:
            link = f"{side}_hand_{joint}"
            mesh = _hand_mesh_name(joint)
            xyz = _hand_fallback_xyz(side, joint, profile)
            lines.extend(
                [
                    f'  <link name="{escape(link)}">',
                    "    <visual>",
                    f'      <geometry><mesh filename="meshes/{escape(mesh)}.obj"/></geometry>',
                    "    </visual>",
                    "  </link>",
                    f'  <joint name="camera_to_{escape(link)}" type="continuous">',
                    '    <parent link="camera"/>',
                    f'    <child link="{escape(link)}"/>',
                    f'    <origin xyz="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" rpy="0 0 0"/>',
                    '    <axis xyz="0 0 1"/>',
                    "  </joint>",
                ]
            )

    lines.append("</robot>")
    return "\n".join(lines) + "\n"


def _build_urdf(profile: FieldTeammateProfile) -> str:
    return (
        '<?xml version="1.0"?>\n'
        '<robot name="field_teammate_skinned">\n'
        '  <link name="base"><collision><origin xyz="0 0 0.02"/><geometry><box size="0.18 0.18 0.04"/></geometry></collision></link>\n'
        '  <link name="body">\n'
        "    <visual>\n"
        '      <geometry><mesh filename="meshes/body.obj"/></geometry>\n'
        "    </visual>\n"
        "  </link>\n"
        '  <joint name="base_to_body" type="fixed">\n'
        '    <parent link="base"/>\n'
        '    <child link="body"/>\n'
        '    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        "  </joint>\n"
        "</robot>\n"
    )


def articulated_link_names(name: str) -> Iterable[str]:
    prefix = str(name or "field_teammate").strip().strip("/")
    for leaf in (
        "pelvis",
        "torso",
        "chest_vest",
        "neck",
        "head",
        "left_upper_arm",
        "left_forearm",
        "left_hand",
        "right_upper_arm",
        "right_forearm",
        "right_hand",
        "left_thigh",
        "left_shin",
        "left_foot",
        "right_thigh",
        "right_shin",
        "right_foot",
    ):
        yield f"{prefix}/{leaf}"
