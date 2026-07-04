#!/usr/bin/env python3
"""Publish a colored PLY point cloud as a HORUS-compatible ROS 2 mesh marker.

This is the ROS 2-native fallback for datasets such as ETHZ ASL Cow & Lady,
where the available ground-truth file is a colored PLY point cloud rather than
a triangle mesh. The tool turns sampled points into small colored triangle
shells and publishes them as chunked visualization_msgs/Marker TRIANGLE_LIST
messages on /map_3d_mesh.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


DEFAULT_PLY = (
    Path("~/.cache/horus/voxblox_cow_lady/extras/cow_and_lady_gt.ply").expanduser()
)
SDK_PYTHON_DIR = Path(__file__).resolve().parents[2]

PLY_SCALARS = {
    "char": ("b", 1),
    "int8": ("b", 1),
    "uchar": ("B", 1),
    "uint8": ("B", 1),
    "short": ("h", 2),
    "int16": ("h", 2),
    "ushort": ("H", 2),
    "uint16": ("H", 2),
    "int": ("i", 4),
    "int32": ("i", 4),
    "uint": ("I", 4),
    "uint32": ("I", 4),
    "float": ("f", 4),
    "float32": ("f", 4),
    "double": ("d", 8),
    "float64": ("d", 8),
}


@dataclass(frozen=True)
class PlyLayout:
    vertex_count: int
    encoding: str
    properties: list[tuple[str, str]]
    header_bytes: int


@dataclass(frozen=True)
class Vertex:
    x: float
    y: float
    z: float
    r: float
    g: float
    b: float
    a: float


@dataclass(frozen=True)
class MeshGeometry:
    points: list[tuple[float, float, float]]
    colors: list[tuple[float, float, float, float]]
    triangle_count: int
    source_vertex_count: int
    occupied_voxel_count: int = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ply", type=Path, default=DEFAULT_PLY)
    parser.add_argument("--topic", default="/map_3d_mesh")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--namespace", default="cow_lady_ply_chunks")
    parser.add_argument(
        "--mode",
        choices=("voxel_surface", "triangle_shell"),
        default="voxel_surface",
        help="voxel_surface merges nearby points into exterior voxel faces; triangle_shell draws one shell per point.",
    )
    parser.add_argument("--max-triangles", type=int, default=0, help="0 means no cap.")
    parser.add_argument("--chunk-max-triangles", type=int, default=5000)
    parser.add_argument("--triangle-size", type=float, default=0.025)
    parser.add_argument("--shape", choices=("triangle", "triad"), default="triangle")
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.02,
        help="Voxel edge size for --mode voxel_surface, in meters.",
    )
    parser.add_argument(
        "--voxel-color-quant-step",
        type=int,
        default=12,
        help="Color quantization step used for greedy face merging in voxel_surface mode.",
    )
    parser.add_argument("--origin-mode", choices=("raw", "center_floor"), default="center_floor")
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument(
        "--republish-interval",
        type=float,
        default=0.0,
        help="Seconds between full snapshot republishes. 0 disables periodic replay.",
    )
    parser.add_argument(
        "--qos-depth",
        type=int,
        default=2048,
        help="Transient-local history depth. Must exceed chunk count for late subscribers.",
    )
    parser.add_argument(
        "--chunk-publish-delay",
        type=float,
        default=0.005,
        help="Small delay between chunk publishes to avoid bridge/headset burst overload.",
    )
    parser.add_argument(
        "--no-wait-for-subscriber",
        action="store_true",
        help="Publish immediately instead of waiting for a ROS subscriber.",
    )
    parser.add_argument(
        "--subscriber-wait-timeout",
        type=float,
        default=0.0,
        help="Seconds to wait for a subscriber before publishing anyway. 0 waits forever.",
    )
    parser.add_argument("--no-transient-local", action="store_true")
    parser.add_argument(
        "--publish-delete-all",
        action="store_true",
        help=(
            "Publish a DELETEALL marker before each full snapshot. Disabled by default "
            "because a delayed DELETEALL can erase already loaded dense-map chunks over "
            "the HORUS bridge."
        ),
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def read_header(path: Path) -> PlyLayout:
    with path.open("rb") as handle:
        first = handle.readline()
        if first.strip() != b"ply":
            raise ValueError(f"{path} is not a PLY file")

        encoding = ""
        vertex_count = -1
        properties: list[tuple[str, str]] = []
        in_vertex = False

        while True:
            raw = handle.readline()
            if not raw:
                raise ValueError("PLY header ended before end_header")
            text = raw.decode("ascii", "replace").strip()
            parts = text.split()
            if not parts:
                continue
            if parts[0] == "format":
                encoding = parts[1]
            elif parts[:2] == ["element", "vertex"]:
                vertex_count = int(parts[2])
                in_vertex = True
            elif parts[0] == "element":
                in_vertex = False
            elif in_vertex and parts[0] == "property":
                if parts[1] == "list":
                    raise ValueError("List properties are not supported on PLY vertices")
                properties.append((parts[1], parts[2]))
            elif parts[0] == "end_header":
                break

        if vertex_count < 0:
            raise ValueError("PLY has no vertex element")
        if encoding not in {"ascii", "binary_little_endian"}:
            raise ValueError(f"Unsupported PLY encoding: {encoding}")
        return PlyLayout(
            vertex_count=vertex_count,
            encoding=encoding,
            properties=properties,
            header_bytes=handle.tell(),
        )


def property_indices(layout: PlyLayout) -> dict[str, int]:
    return {name: index for index, (_, name) in enumerate(layout.properties)}


def binary_struct(layout: PlyLayout) -> struct.Struct:
    try:
        fmt = "<" + "".join(PLY_SCALARS[data_type][0] for data_type, _ in layout.properties)
    except KeyError as exc:
        raise ValueError(f"Unsupported PLY property type: {exc.args[0]}") from exc
    return struct.Struct(fmt)


def vertex_stride(layout: PlyLayout) -> int:
    try:
        return sum(PLY_SCALARS[data_type][1] for data_type, _ in layout.properties)
    except KeyError as exc:
        raise ValueError(f"Unsupported PLY property type: {exc.args[0]}") from exc


def selected_vertex_indices(vertex_count: int, max_vertices: int) -> set[int]:
    if max_vertices <= 0 or vertex_count <= max_vertices:
        return set(range(vertex_count))
    step = vertex_count / float(max_vertices)
    return {min(vertex_count - 1, int(i * step)) for i in range(max_vertices)}


def read_vertices(path: Path, layout: PlyLayout, max_vertices: int) -> list[Vertex]:
    indices = property_indices(layout)
    required = ("x", "y", "z")
    if any(name not in indices for name in required):
        raise ValueError("PLY vertices must include x, y and z properties")

    selected = selected_vertex_indices(layout.vertex_count, max_vertices)
    vertices: list[Vertex] = []
    with path.open("rb") as handle:
        handle.seek(layout.header_bytes)
        if layout.encoding == "binary_little_endian":
            read_binary_vertices(handle, layout, selected, indices, vertices)
        else:
            read_ascii_vertices(handle, layout, selected, indices, vertices)
    return vertices


def color_value(values: tuple[object, ...], indices: dict[str, int], name: str, fallback: float) -> float:
    if name not in indices:
        return fallback
    value = float(values[indices[name]])
    if value > 1.0:
        value /= 255.0
    return max(0.0, min(1.0, value))


def make_vertex(values: tuple[object, ...], indices: dict[str, int]) -> Vertex:
    return Vertex(
        x=float(values[indices["x"]]),
        y=float(values[indices["y"]]),
        z=float(values[indices["z"]]),
        r=color_value(values, indices, "red", 0.70),
        g=color_value(values, indices, "green", 0.72),
        b=color_value(values, indices, "blue", 0.78),
        a=color_value(values, indices, "alpha", 1.0),
    )


def read_binary_vertices(
    handle: object,
    layout: PlyLayout,
    selected: set[int],
    indices: dict[str, int],
    vertices: list[Vertex],
) -> None:
    unpacker = binary_struct(layout)
    stride = vertex_stride(layout)
    for index in range(layout.vertex_count):
        raw = handle.read(stride)
        if len(raw) != stride:
            raise ValueError(f"PLY ended early at vertex {index}")
        if index in selected:
            vertices.append(make_vertex(unpacker.unpack(raw), indices))


def read_ascii_vertices(
    handle: object,
    layout: PlyLayout,
    selected: set[int],
    indices: dict[str, int],
    vertices: list[Vertex],
) -> None:
    for index in range(layout.vertex_count):
        raw = handle.readline()
        if not raw:
            raise ValueError(f"PLY ended early at vertex {index}")
        if index in selected:
            values = tuple(float(v) for v in raw.decode("ascii", "replace").split())
            vertices.append(make_vertex(values, indices))


def bounds(vertices: list[Vertex]) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    mins = (
        min(v.x for v in vertices),
        min(v.y for v in vertices),
        min(v.z for v in vertices),
    )
    maxs = (
        max(v.x for v in vertices),
        max(v.y for v in vertices),
        max(v.z for v in vertices),
    )
    return mins, maxs


def transform_vertices(vertices: list[Vertex], origin_mode: str, scale: float) -> list[Vertex]:
    if not vertices:
        return []
    mins, maxs = bounds(vertices)
    if origin_mode == "center_floor":
        ox = (mins[0] + maxs[0]) * 0.5
        oy = (mins[1] + maxs[1]) * 0.5
        oz = mins[2]
    else:
        ox = oy = oz = 0.0
    return [
        Vertex(
            x=(v.x - ox) * scale,
            y=(v.y - oy) * scale,
            z=(v.z - oz) * scale,
            r=v.r,
            g=v.g,
            b=v.b,
            a=v.a,
        )
        for v in vertices
    ]


def triangles_per_vertex(shape: str) -> int:
    return 3 if shape == "triad" else 1


def iter_triangle_shell(
    vertices: list[Vertex],
    size: float,
    shape: str,
) -> Iterable[tuple[list[tuple[float, float, float]], tuple[float, float, float, float]]]:
    s = max(1e-6, size)
    for v in vertices:
        color = (v.r, v.g, v.b, v.a)
        yield (
            [
                (v.x - s, v.y - s, v.z),
                (v.x + s, v.y - s, v.z),
                (v.x, v.y + s, v.z),
            ],
            color,
        )
        if shape == "triad":
            yield (
                [
                    (v.x - s, v.y, v.z - s),
                    (v.x + s, v.y, v.z - s),
                    (v.x, v.y, v.z + s),
                ],
                color,
            )
            yield (
                [
                    (v.x, v.y - s, v.z - s),
                    (v.x, v.y + s, v.z - s),
                    (v.x, v.y, v.z + s),
                ],
                color,
            )


def chunk_points(
    points: list[tuple[float, float, float]],
    colors: list[tuple[float, float, float, float]],
    chunk_max_triangles: int,
) -> Iterable[tuple[int, list[tuple[float, float, float]], list[tuple[float, float, float, float]]]]:
    points_per_chunk = max(1, int(chunk_max_triangles)) * 3
    marker_id = 0
    for start in range(0, len(points), points_per_chunk):
        end = min(len(points), start + points_per_chunk)
        end -= (end - start) % 3
        if end > start:
            yield marker_id, points[start:end], colors[start:end]
            marker_id += 1


def build_triangle_shell_geometry(args: argparse.Namespace, vertices: list[Vertex]) -> MeshGeometry:
    points: list[tuple[float, float, float]] = []
    colors: list[tuple[float, float, float, float]] = []
    for tri_points, color in iter_triangle_shell(vertices, args.triangle_size, args.shape):
        points.extend(tri_points)
        colors.extend([color, color, color])
    return MeshGeometry(
        points=points,
        colors=colors,
        triangle_count=len(points) // 3,
        source_vertex_count=len(vertices),
    )


def build_voxel_surface_geometry(args: argparse.Namespace, vertices: list[Vertex]) -> MeshGeometry:
    import numpy as np

    voxel_mesh_path = SDK_PYTHON_DIR / "horus" / "utils" / "voxel_mesh.py"
    spec = importlib.util.spec_from_file_location("_horus_voxel_mesh", voxel_mesh_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load HORUS voxel mesher: {voxel_mesh_path}")
    voxel_mesh_module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = voxel_mesh_module
    spec.loader.exec_module(voxel_mesh_module)
    build_greedy_surface_mesh = voxel_mesh_module.build_greedy_surface_mesh

    voxel_size = max(1e-6, float(args.voxel_size))
    xyz = np.asarray([(v.x, v.y, v.z) for v in vertices], dtype=np.float64)
    colors = np.asarray(
        [
            (
                int(round(v.r * 255.0)),
                int(round(v.g * 255.0)),
                int(round(v.b * 255.0)),
            )
            for v in vertices
        ],
        dtype=np.float64,
    )
    voxels = np.floor(xyz / voxel_size).astype(np.int32)
    unique_voxels, inverse = np.unique(voxels, axis=0, return_inverse=True)

    color_sums = np.zeros((len(unique_voxels), 3), dtype=np.float64)
    counts = np.zeros(len(unique_voxels), dtype=np.float64)
    np.add.at(color_sums, inverse, colors)
    np.add.at(counts, inverse, 1.0)
    voxel_colors = np.rint(color_sums / np.maximum(counts[:, None], 1.0)).astype(np.uint8)

    result = build_greedy_surface_mesh(
        unique_voxels,
        voxel_size,
        voxel_colors=voxel_colors,
        max_triangles=max(0, int(args.max_triangles)),
        color_quant_step=max(1, int(args.voxel_color_quant_step)),
    )
    result_colors = result.colors or [(180, 184, 198)] * len(result.vertices)
    rgba = [
        (
            max(0.0, min(1.0, float(r) / 255.0)),
            max(0.0, min(1.0, float(g) / 255.0)),
            max(0.0, min(1.0, float(b) / 255.0)),
            1.0,
        )
        for r, g, b in result_colors
    ]
    return MeshGeometry(
        points=result.vertices,
        colors=rgba,
        triangle_count=result.triangle_count,
        source_vertex_count=len(vertices),
        occupied_voxel_count=len(unique_voxels),
    )


def load_geometry(args: argparse.Namespace) -> MeshGeometry:
    path = args.ply.expanduser()
    if not path.exists():
        raise SystemExit(f"Missing PLY file: {path}")
    layout = read_header(path)
    if args.mode == "triangle_shell":
        per_vertex = triangles_per_vertex(args.shape)
        max_vertices = 0 if args.max_triangles <= 0 else max(1, args.max_triangles // per_vertex)
    else:
        max_vertices = 0
    vertices = read_vertices(path, layout, max_vertices)
    vertices = transform_vertices(vertices, args.origin_mode, args.scale)
    geometry = (
        build_voxel_surface_geometry(args, vertices)
        if args.mode == "voxel_surface"
        else build_triangle_shell_geometry(args, vertices)
    )
    mins, maxs = bounds_points(geometry.points)
    voxel_text = (
        f" voxels={geometry.occupied_voxel_count}"
        if geometry.occupied_voxel_count > 0
        else ""
    )
    print(
        f"[ply] mode={args.mode} vertices={geometry.source_vertex_count}/{layout.vertex_count}"
        f"{voxel_text} triangles={geometry.triangle_count} "
        f"bbox_min=({mins[0]:.2f}, {mins[1]:.2f}, {mins[2]:.2f}) "
        f"bbox_max=({maxs[0]:.2f}, {maxs[1]:.2f}, {maxs[2]:.2f})"
    )
    return geometry


def bounds_points(
    points: list[tuple[float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if not points:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    mins = (
        min(p[0] for p in points),
        min(p[1] for p in points),
        min(p[2] for p in points),
    )
    maxs = (
        max(p[0] for p in points),
        max(p[1] for p in points),
        max(p[2] for p in points),
    )
    return mins, maxs


def run_ros2(args: argparse.Namespace, geometry: MeshGeometry) -> None:
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker

    class PlyMeshPublisher(Node):
        def __init__(self) -> None:
            super().__init__("ply_to_horus_mesh_marker")
            qos = QoSProfile(depth=max(1, int(args.qos_depth)))
            qos.reliability = ReliabilityPolicy.RELIABLE
            if not args.no_transient_local:
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.pub = self.create_publisher(Marker, args.topic, qos)
            self.markers = self.build_markers()
            if args.no_wait_for_subscriber:
                self.publish_all()
                self.has_published_initial_snapshot = True
            else:
                self.has_published_initial_snapshot = False
                self.wait_start_monotonic = time.monotonic()
            self.timer = (
                self.create_timer(max(0.2, args.republish_interval), self.publish_periodic)
                if args.republish_interval > 0
                else self.create_timer(0.5, self.publish_when_subscribed)
            )
            self.get_logger().info(
                f"Publishing {len(self.markers)} chunks on {args.topic} frame={args.frame_id}"
            )
            if not args.no_wait_for_subscriber:
                self.get_logger().info(
                    "Waiting for a subscriber before publishing the static mesh snapshot"
                )

        def build_markers(self) -> list[Marker]:
            markers: list[Marker] = []
            stamp = self.get_clock().now().to_msg()
            for marker_id, points, colors in chunk_points(
                geometry.points,
                geometry.colors,
                args.chunk_max_triangles,
            ):
                marker = Marker()
                marker.header.stamp = stamp
                marker.header.frame_id = args.frame_id
                marker.ns = args.namespace
                marker.id = int(marker_id)
                marker.type = Marker.TRIANGLE_LIST
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 1.0
                marker.color = ColorRGBA(r=0.72, g=0.74, b=0.80, a=1.0)
                marker.points = [Point(x=x, y=y, z=z) for x, y, z in points]
                marker.colors = [ColorRGBA(r=r, g=g, b=b, a=a) for r, g, b, a in colors]
                markers.append(marker)
            return markers

        def publish_delete_all(self) -> None:
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = args.frame_id
            marker.ns = args.namespace
            marker.id = 0
            marker.type = Marker.TRIANGLE_LIST
            marker.action = Marker.DELETEALL
            self.pub.publish(marker)

        def publish_when_subscribed(self) -> None:
            if self.has_published_initial_snapshot:
                return
            subscriber_count = self.pub.get_subscription_count()
            timed_out = (
                args.subscriber_wait_timeout > 0 and
                (time.monotonic() - self.wait_start_monotonic) >= args.subscriber_wait_timeout
            )
            if subscriber_count <= 0 and not timed_out:
                return
            if timed_out:
                self.get_logger().warning(
                    "No subscriber appeared before timeout; publishing static mesh snapshot anyway"
                )
            else:
                self.get_logger().info(
                    f"Subscriber detected ({subscriber_count}); publishing static mesh snapshot"
                )
            self.publish_all()
            self.has_published_initial_snapshot = True

        def publish_periodic(self) -> None:
            if not self.has_published_initial_snapshot:
                self.publish_when_subscribed()
                return
            self.publish_all()

        def publish_all(self) -> None:
            stamp = self.get_clock().now().to_msg()
            delay = max(0.0, float(args.chunk_publish_delay))
            if args.publish_delete_all:
                self.publish_delete_all()
            for marker in self.markers:
                marker.header.stamp = stamp
                self.pub.publish(marker)
                if delay > 0:
                    time.sleep(delay)

    rclpy.init()
    node = PlyMeshPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    args = parse_args()
    if not math.isfinite(args.triangle_size) or args.triangle_size <= 0:
        raise SystemExit("--triangle-size must be > 0")
    if not math.isfinite(args.scale) or args.scale <= 0:
        raise SystemExit("--scale must be > 0")
    if not math.isfinite(args.republish_interval) or args.republish_interval < 0:
        raise SystemExit("--republish-interval must be >= 0")
    if args.qos_depth < 1:
        raise SystemExit("--qos-depth must be >= 1")
    if not math.isfinite(args.chunk_publish_delay) or args.chunk_publish_delay < 0:
        raise SystemExit("--chunk-publish-delay must be >= 0")
    if not math.isfinite(args.subscriber_wait_timeout) or args.subscriber_wait_timeout < 0:
        raise SystemExit("--subscriber-wait-timeout must be >= 0")
    geometry = load_geometry(args)
    if args.dry_run:
        return
    run_ros2(args, geometry)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
