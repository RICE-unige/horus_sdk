"""Deterministic synthetic RGB-D site used by the remote-map proof.

The renderer intentionally has no Unity dependency. It builds a triangle mesh,
rasterizes color and metric eye depth from the registered map camera, and packs
the result for the Quest-side depth-reprojection shader.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterable, Sequence

import numpy as np


VERTICAL_FOV_DEGREES = 52.0
DEPTH_NEAR_METERS = 8.0
DEPTH_FAR_METERS = 42.0
CAMERA_POSITION = np.array((0.0, 15.0, -19.0), dtype=np.float32)
CAMERA_PITCH_DEGREES = 38.0


Color = tuple[int, int, int]


@dataclass
class TriangleScene:
    vertices: list[np.ndarray] = field(default_factory=list)
    faces: list[np.ndarray] = field(default_factory=list)
    colors: list[np.ndarray] = field(default_factory=list)
    _vertex_count: int = 0

    def add_mesh(
        self,
        vertices: Sequence[Sequence[float]],
        faces: Sequence[Sequence[int]],
        color: Color | Sequence[Color],
    ) -> None:
        vertex_array = np.asarray(vertices, dtype=np.float32)
        face_array = np.asarray(faces, dtype=np.int32)
        if vertex_array.ndim != 2 or vertex_array.shape[1] != 3:
            raise ValueError("vertices must have shape (N, 3)")
        if face_array.ndim != 2 or face_array.shape[1] != 3:
            raise ValueError("faces must have shape (M, 3)")

        face_array = face_array + self._vertex_count
        if isinstance(color, tuple):
            color_array = np.repeat(
                np.asarray(color, dtype=np.float32)[None, :],
                face_array.shape[0],
                axis=0,
            )
        else:
            color_array = np.asarray(color, dtype=np.float32)
            if color_array.shape != (face_array.shape[0], 3):
                raise ValueError("per-face colors must have shape (M, 3)")

        self.vertices.append(vertex_array)
        self.faces.append(face_array)
        self.colors.append(color_array)
        self._vertex_count += vertex_array.shape[0]

    def arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return (
            np.concatenate(self.vertices, axis=0),
            np.concatenate(self.faces, axis=0),
            np.concatenate(self.colors, axis=0),
        )

    def add_box(
        self,
        center: Sequence[float],
        size: Sequence[float],
        color: Color,
        yaw_degrees: float = 0.0,
    ) -> None:
        cx, cy, cz = center
        sx, sy, sz = (value * 0.5 for value in size)
        vertices = np.array(
            [
                (-sx, -sy, -sz), (sx, -sy, -sz), (sx, sy, -sz), (-sx, sy, -sz),
                (-sx, -sy, sz), (sx, -sy, sz), (sx, sy, sz), (-sx, sy, sz),
            ],
            dtype=np.float32,
        )
        angle = math.radians(yaw_degrees)
        rotation = np.array(
            ((math.cos(angle), 0.0, math.sin(angle)), (0.0, 1.0, 0.0),
             (-math.sin(angle), 0.0, math.cos(angle))),
            dtype=np.float32,
        )
        vertices = vertices @ rotation.T + np.array((cx, cy, cz), dtype=np.float32)
        faces = (
            (0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
            (0, 1, 5), (0, 5, 4), (3, 7, 6), (3, 6, 2),
            (1, 2, 6), (1, 6, 5), (0, 4, 7), (0, 7, 3),
        )
        self.add_mesh(vertices, faces, color)

    def add_gabled_roof(
        self,
        center: Sequence[float],
        width: float,
        depth: float,
        height: float,
        color: Color,
    ) -> None:
        cx, cy, cz = center
        x = width * 0.5
        z = depth * 0.5
        vertices = np.array(
            [
                (cx - x, cy, cz - z), (cx + x, cy, cz - z),
                (cx - x, cy, cz + z), (cx + x, cy, cz + z),
                (cx, cy + height, cz - z), (cx, cy + height, cz + z),
            ],
            dtype=np.float32,
        )
        faces = ((0, 2, 4), (4, 2, 5), (4, 5, 1), (1, 5, 3), (0, 4, 1), (2, 3, 5))
        self.add_mesh(vertices, faces, color)

    def add_cylinder(
        self,
        center: Sequence[float],
        radius: float,
        height: float,
        color: Color,
        segments: int = 12,
    ) -> None:
        cx, cy, cz = center
        bottom_y = cy - height * 0.5
        top_y = cy + height * 0.5
        vertices: list[tuple[float, float, float]] = []
        for y in (bottom_y, top_y):
            for index in range(segments):
                angle = 2.0 * math.pi * index / segments
                vertices.append((cx + radius * math.cos(angle), y, cz + radius * math.sin(angle)))
        vertices.extend(((cx, bottom_y, cz), (cx, top_y, cz)))
        bottom_center = segments * 2
        top_center = bottom_center + 1
        faces: list[tuple[int, int, int]] = []
        for index in range(segments):
            nxt = (index + 1) % segments
            faces.extend(
                (
                    (index, nxt, segments + index),
                    (nxt, segments + nxt, segments + index),
                    (bottom_center, nxt, index),
                    (top_center, segments + index, segments + nxt),
                )
            )
        self.add_mesh(vertices, faces, color)

    def add_cylinder_between(
        self,
        start: Sequence[float],
        end: Sequence[float],
        radius: float,
        color: Color,
        segments: int = 10,
    ) -> None:
        start_array = np.asarray(start, dtype=np.float32)
        end_array = np.asarray(end, dtype=np.float32)
        axis = end_array - start_array
        length = float(np.linalg.norm(axis))
        if length < 1e-5:
            return
        axis /= length
        helper = np.array((0.0, 1.0, 0.0), dtype=np.float32)
        if abs(float(np.dot(axis, helper))) > 0.92:
            helper = np.array((1.0, 0.0, 0.0), dtype=np.float32)
        side = np.cross(axis, helper)
        side /= np.linalg.norm(side)
        up = np.cross(side, axis)
        vertices: list[np.ndarray] = []
        for origin in (start_array, end_array):
            for index in range(segments):
                angle = 2.0 * math.pi * index / segments
                vertices.append(origin + radius * (math.cos(angle) * side + math.sin(angle) * up))
        faces: list[tuple[int, int, int]] = []
        for index in range(segments):
            nxt = (index + 1) % segments
            faces.extend(((index, nxt, segments + index), (nxt, segments + nxt, segments + index)))
        self.add_mesh(vertices, faces, color)

    def add_sphere(
        self,
        center: Sequence[float],
        radius: float,
        color: Color,
        rings: int = 5,
        segments: int = 10,
    ) -> None:
        center_array = np.asarray(center, dtype=np.float32)
        vertices: list[np.ndarray] = []
        for ring in range(rings + 1):
            phi = math.pi * ring / rings
            for segment in range(segments):
                theta = 2.0 * math.pi * segment / segments
                offset = np.array(
                    (math.sin(phi) * math.cos(theta), math.cos(phi), math.sin(phi) * math.sin(theta)),
                    dtype=np.float32,
                )
                vertices.append(center_array + radius * offset)
        faces: list[tuple[int, int, int]] = []
        for ring in range(rings):
            for segment in range(segments):
                nxt = (segment + 1) % segments
                a = ring * segments + segment
                b = ring * segments + nxt
                c = (ring + 1) * segments + segment
                d = (ring + 1) * segments + nxt
                faces.extend(((a, c, b), (b, c, d)))
        self.add_mesh(vertices, faces, color)


def _add_terrain(scene: TriangleScene) -> None:
    x_values = np.linspace(-14.5, 14.5, 38, dtype=np.float32)
    z_values = np.linspace(-11.5, 12.0, 32, dtype=np.float32)
    vertices: list[tuple[float, float, float]] = []
    for z in z_values:
        for x in x_values:
            y = 0.10 * math.sin(float(x) * 0.34) * math.cos(float(z) * 0.28)
            y += 0.28 * math.exp(-((float(x) + 11.0) ** 2 + (float(z) - 7.0) ** 2) / 18.0)
            vertices.append((float(x), y, float(z)))
    faces: list[tuple[int, int, int]] = []
    colors: list[Color] = []
    columns = len(x_values)
    for row in range(len(z_values) - 1):
        for column in range(columns - 1):
            a = row * columns + column
            b = a + 1
            c = a + columns
            d = c + 1
            shade = 8 * ((row + column) % 3)
            ground = (78 + shade, 93 + shade, 70 + shade // 2)
            faces.extend(((a, c, b), (b, c, d)))
            colors.extend((ground, ground))
    scene.add_mesh(vertices, faces, colors)


def _add_road(scene: TriangleScene, points: Iterable[Sequence[float]], width: float) -> None:
    points_array = [np.asarray(point, dtype=np.float32) for point in points]
    left: list[np.ndarray] = []
    right: list[np.ndarray] = []
    for index, point in enumerate(points_array):
        previous = points_array[max(0, index - 1)]
        nxt = points_array[min(len(points_array) - 1, index + 1)]
        tangent = nxt - previous
        tangent[1] = 0.0
        tangent /= max(float(np.linalg.norm(tangent)), 1e-6)
        side = np.array((-tangent[2], 0.0, tangent[0]), dtype=np.float32) * (width * 0.5)
        left.append(point - side)
        right.append(point + side)
    vertices: list[np.ndarray] = []
    for a, b in zip(left, right):
        vertices.extend((a, b))
    faces: list[tuple[int, int, int]] = []
    for index in range(len(points_array) - 1):
        a = index * 2
        faces.extend(((a, a + 2, a + 1), (a + 1, a + 2, a + 3)))
    scene.add_mesh(vertices, faces, (54, 59, 62))


def build_industrial_site() -> TriangleScene:
    scene = TriangleScene()
    _add_terrain(scene)
    scene.add_box((0.0, -0.22, 0.4), (29.0, 0.25, 2.1), (34, 77, 91))
    _add_road(scene, ((-13.0, 0.18, -4.8), (-6.5, 0.18, -1.8), (0.0, 0.18, 0.6),
                      (6.0, 0.18, 2.2), (13.0, 0.18, 5.4)), 2.45)
    _add_road(scene, ((-7.4, 0.19, -1.4), (-7.0, 0.19, 3.5), (-3.0, 0.19, 7.5),
                      (5.0, 0.19, 8.0), (10.5, 0.19, 7.2)), 1.65)

    buildings = (
        ((-7.8, 0.15, -5.8), (4.7, 2.9, 3.7), (157, 165, 159), (72, 84, 92)),
        ((-2.4, 0.15, -6.8), (3.9, 2.2, 2.9), (168, 151, 123), (100, 69, 54)),
        ((4.2, 0.15, -6.0), (5.5, 3.4, 4.1), (141, 154, 165), (63, 73, 82)),
        ((9.8, 0.15, -3.4), (3.6, 2.5, 3.0), (177, 172, 151), (83, 72, 66)),
    )
    for center, size, wall, roof in buildings:
        scene.add_box((center[0], center[1] + size[1] * 0.5, center[2]), size, wall)
        scene.add_gabled_roof((center[0], center[1] + size[1], center[2]), size[0] + 0.3,
                              size[2] + 0.3, 0.7, roof)
        scene.add_box((center[0], center[1] + 0.75, center[2] - size[2] * 0.51),
                      (0.9, 1.4, 0.08), (42, 52, 58))
        for offset in np.linspace(-size[0] * 0.32, size[0] * 0.32, 3):
            scene.add_box((center[0] + float(offset), center[1] + size[1] * 0.62,
                           center[2] - size[2] * 0.515), (0.58, 0.62, 0.06), (64, 112, 134))

    for tank_center in ((7.2, 1.3, 5.8), (9.4, 1.3, 5.8), (7.2, 1.3, 8.0)):
        scene.add_cylinder(tank_center, 0.78, 2.3, (133, 145, 151), segments=18)
        scene.add_cylinder((tank_center[0], tank_center[1] + 0.25, tank_center[2]),
                           0.81, 0.18, (221, 174, 42), segments=18)
    scene.add_box((8.25, 0.38, 6.9), (5.0, 0.55, 0.22), (89, 96, 98))
    scene.add_box((5.7, 0.38, 6.9), (0.22, 0.55, 4.4), (89, 96, 98))
    scene.add_cylinder_between((5.8, 1.15, 2.2), (9.2, 1.15, 4.8), 0.16, (151, 158, 159))
    scene.add_cylinder_between((9.2, 1.15, 4.8), (9.2, 1.15, 8.6), 0.16, (151, 158, 159))

    scene.add_box((-0.5, 0.6, 0.35), (6.6, 0.48, 2.8), (118, 121, 119))
    scene.add_box((-0.5, 1.0, -0.85), (6.8, 0.75, 0.18), (91, 95, 94))
    scene.add_box((-0.5, 1.0, 1.55), (6.8, 0.75, 0.18), (91, 95, 94))

    random = np.random.default_rng(49031)
    for index in range(54):
        x = -7.8 + float(random.uniform(-3.0, 3.0))
        z = 6.8 + float(random.uniform(-2.0, 2.0))
        size = random.uniform((0.18, 0.12, 0.16), (0.9, 0.55, 0.85))
        color = (132, 123, 109) if index % 4 else (121, 77, 60)
        scene.add_box((x, float(size[1]) * 0.5 + 0.12, z), size, color,
                      yaw_degrees=float(random.uniform(0.0, 180.0)))
    scene.add_cylinder_between((-10.0, 0.5, 5.5), (-5.2, 0.85, 8.1), 0.16, (84, 91, 94))

    tree_positions = (
        (-12.0, -9.0), (-10.0, -7.5), (-12.4, -4.5), (-11.8, 1.2),
        (-12.7, 4.0), (-9.6, 10.1), (-5.0, 10.8), (0.0, 11.0),
        (5.0, 10.8), (12.2, 10.0), (12.7, 3.6), (12.4, -4.2),
    )
    for index, (x, z) in enumerate(tree_positions):
        height = 1.7 + 0.18 * (index % 4)
        scene.add_cylinder((x, height * 0.27, z), 0.13, height * 0.54, (91, 65, 42), segments=9)
        scene.add_sphere((x, height * 0.82, z), 0.72 + 0.08 * (index % 3), (52, 94, 54))

    scene.add_box((1.8, 0.5, 4.7), (1.4, 0.5, 0.82), (199, 166, 39), yaw_degrees=-22.0)
    scene.add_box((1.8, 0.98, 4.7), (0.7, 0.45, 0.68), (81, 91, 98), yaw_degrees=-22.0)
    scene.add_cylinder((1.8, 1.35, 4.7), 0.19, 0.18, (47, 74, 88), segments=12)
    scene.add_cylinder_between((-1.0, 0.25, 6.8), (-1.0, 2.25, 6.8), 0.09, (72, 77, 80))
    scene.add_cylinder_between((-1.0, 2.1, 6.8), (0.6, 2.1, 6.8), 0.07, (232, 169, 37))
    return scene


def render_scene(
    scene: TriangleScene,
    width: int,
    height: int,
    near_m: float = DEPTH_NEAR_METERS,
    far_m: float = DEPTH_FAR_METERS,
    vertical_fov_deg: float = VERTICAL_FOV_DEGREES,
) -> tuple[np.ndarray, np.ndarray]:
    vertices, faces, base_colors = scene.arrays()
    pitch = math.radians(CAMERA_PITCH_DEGREES)
    camera_to_world = np.array(
        ((1.0, 0.0, 0.0), (0.0, math.cos(pitch), -math.sin(pitch)),
         (0.0, math.sin(pitch), math.cos(pitch))),
        dtype=np.float32,
    )
    camera_vertices = (vertices - CAMERA_POSITION) @ camera_to_world
    z_values = camera_vertices[:, 2]
    focal = height * 0.5 / math.tan(math.radians(vertical_fov_deg) * 0.5)
    screen = np.empty((vertices.shape[0], 2), dtype=np.float32)
    screen[:, 0] = width * 0.5 + focal * camera_vertices[:, 0] / z_values
    screen[:, 1] = height * 0.5 - focal * camera_vertices[:, 1] / z_values

    color = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.full((height, width), np.inf, dtype=np.float32)
    light = np.array((-0.38, 0.82, -0.43), dtype=np.float32)
    light /= np.linalg.norm(light)

    for face, base_color in zip(faces, base_colors):
        triangle_z = z_values[face]
        if np.any(triangle_z <= near_m) or np.any(triangle_z >= far_m):
            continue
        triangle = screen[face]
        min_x = max(0, int(math.floor(float(np.min(triangle[:, 0])))))
        max_x = min(width - 1, int(math.ceil(float(np.max(triangle[:, 0])))))
        min_y = max(0, int(math.floor(float(np.min(triangle[:, 1])))))
        max_y = min(height - 1, int(math.ceil(float(np.max(triangle[:, 1])))))
        if min_x > max_x or min_y > max_y:
            continue

        x0, y0 = triangle[0]
        x1, y1 = triangle[1]
        x2, y2 = triangle[2]
        denominator = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
        if abs(float(denominator)) < 1e-6:
            continue
        yy, xx = np.mgrid[min_y:max_y + 1, min_x:max_x + 1]
        sample_x = xx.astype(np.float32) + 0.5
        sample_y = yy.astype(np.float32) + 0.5
        w0 = ((y1 - y2) * (sample_x - x2) + (x2 - x1) * (sample_y - y2)) / denominator
        w1 = ((y2 - y0) * (sample_x - x2) + (x0 - x2) * (sample_y - y2)) / denominator
        w2 = 1.0 - w0 - w1
        inside = (w0 >= -1e-4) & (w1 >= -1e-4) & (w2 >= -1e-4)
        if not np.any(inside):
            continue
        inverse_depth = w0 / triangle_z[0] + w1 / triangle_z[1] + w2 / triangle_z[2]
        pixel_depth = np.divide(1.0, inverse_depth, out=np.full_like(inverse_depth, np.inf),
                                where=inverse_depth > 1e-8)
        depth_region = depth[min_y:max_y + 1, min_x:max_x + 1]
        visible = inside & (pixel_depth < depth_region)
        if not np.any(visible):
            continue

        world_triangle = vertices[face]
        normal = np.cross(world_triangle[1] - world_triangle[0], world_triangle[2] - world_triangle[0])
        normal_length = float(np.linalg.norm(normal))
        if normal_length > 1e-6:
            normal /= normal_length
        intensity = 0.42 + 0.58 * abs(float(np.dot(normal, light)))
        shaded = np.clip(base_color * intensity, 0.0, 255.0).astype(np.uint8)
        depth_region[visible] = pixel_depth[visible]
        color_region = color[min_y:max_y + 1, min_x:max_x + 1]
        color_region[visible] = shaded

    return color, depth


DEPTH_LUMA_MIN = 32
DEPTH_LUMA_MAX = 255
REMOTE_RGBD_FORMAT_VERSION = "rgbd_multiview_luma_nibbles_v2"
_DEPTH_NIBBLE_TO_LUMA = np.rint(
    np.linspace(DEPTH_LUMA_MIN, DEPTH_LUMA_MAX, 16, dtype=np.float32)
).astype(np.uint8)


def encode_luma_depth(
    depth: np.ndarray,
    near_m: float = DEPTH_NEAR_METERS,
    far_m: float = DEPTH_FAR_METERS,
) -> np.ndarray:
    """Encode one depth byte as two H.264-resistant grayscale nibbles.

    Even columns carry the high nibble and odd columns the low nibble. One
    encoded pair represents two neighboring source pixels, which still exceeds
    the Quest reprojection-grid resolution while giving each nibble fifteen
    luma levels of separation.
    """
    if depth.ndim != 2 or depth.shape[1] % 2:
        raise ValueError("depth image width must be even")
    encoded = np.zeros((*depth.shape, 3), dtype=np.uint8)
    sampled = depth[:, 0::2]
    valid = np.isfinite(sampled) & (sampled >= near_m) & (sampled <= far_m)
    scaled = np.nan_to_num(
        (sampled - near_m) * (255.0 / (far_m - near_m)),
        nan=0.0,
        posinf=0.0,
        neginf=0.0,
    )
    code = np.rint(np.clip(scaled, 0.0, 255.0)).astype(np.uint8)
    high_luma = _DEPTH_NIBBLE_TO_LUMA[code >> 4]
    low_luma = _DEPTH_NIBBLE_TO_LUMA[code & 0x0F]
    high_luma[~valid] = 0
    low_luma[~valid] = 0
    encoded[:, 0::2, :] = high_luma[..., None]
    encoded[:, 1::2, :] = low_luma[..., None]
    return encoded


def decode_luma_depth(
    encoded: np.ndarray,
    near_m: float = DEPTH_NEAR_METERS,
    far_m: float = DEPTH_FAR_METERS,
) -> np.ndarray:
    if encoded.ndim != 3 or encoded.shape[1] % 2:
        raise ValueError("encoded depth image width must be even")
    high_luma = encoded[:, 0::2].astype(np.float32).mean(axis=-1)
    low_luma = encoded[:, 1::2].astype(np.float32).mean(axis=-1)
    valid = high_luma >= DEPTH_LUMA_MIN * 0.5
    level_span = DEPTH_LUMA_MAX - DEPTH_LUMA_MIN
    high = np.rint(np.clip((high_luma - DEPTH_LUMA_MIN) * 15.0 / level_span, 0.0, 15.0))
    low = np.rint(np.clip((low_luma - DEPTH_LUMA_MIN) * 15.0 / level_span, 0.0, 15.0))
    code = high * 16.0 + low
    pair_depth = near_m + (far_m - near_m) * code / 255.0
    pair_depth[~valid] = np.inf
    return np.repeat(pair_depth, 2, axis=1).astype(np.float32)


def build_remote_map_rgbd(width: int, height: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if width < 160 or height < 90:
        raise ValueError("remote map view must be at least 160x90")
    color, depth = render_scene(build_industrial_site(), width, height)
    encoded_depth = encode_luma_depth(depth)
    packed = np.ascontiguousarray(np.concatenate((color, encoded_depth), axis=1))
    return color, depth, packed
