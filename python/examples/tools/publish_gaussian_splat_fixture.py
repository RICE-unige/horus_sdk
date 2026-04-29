#!/usr/bin/env python3
"""Publish a Gaussian Splat fixture as ROS2 test data for HORUS.

This publishes two complementary streams:
  * a sampled PointCloud2 preview that current HORUS builds can visualize
  * a small JSON manifest with the source .ply path for the future splat renderer

Fetch data first with:
    python3 python/examples/tools/fetch_gaussian_splat_fixtures.py --bundle prebuilt
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


DEFAULT_MANIFEST = Path("~/.cache/horus/gaussian_splatting/manifest.json").expanduser()
DEFAULT_ROBOT_NAMES = ("splat_rover_1", "splat_rover_2")
SH_C0 = 0.28209479177387814


PLY_STRUCT_TYPES: Dict[str, str] = {
    "char": "b",
    "int8": "b",
    "uchar": "B",
    "uint8": "B",
    "short": "h",
    "int16": "h",
    "ushort": "H",
    "uint16": "H",
    "int": "i",
    "int32": "i",
    "uint": "I",
    "uint32": "I",
    "float": "f",
    "float32": "f",
    "double": "d",
    "float64": "d",
}


@dataclass(frozen=True)
class PlyProperty:
    name: str
    data_type: str


@dataclass(frozen=True)
class PlyHeader:
    format_name: str
    vertex_count: int
    vertex_properties: Sequence[PlyProperty]
    data_offset: int


@dataclass(frozen=True)
class PreviewData:
    points: List[Tuple[float, float, float, float]]
    original_vertex_count: int
    published_vertex_count: int
    center_offset: Tuple[float, float, float]
    scale: float
    ply_path: Path


@dataclass
class FakeRobotState:
    name: str
    base_frame: str
    phase: float
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=DEFAULT_MANIFEST,
        help=f"Fixture manifest written by fetch_gaussian_splat_fixtures.py. Default: {DEFAULT_MANIFEST}",
    )
    parser.add_argument(
        "--ply",
        type=Path,
        default=None,
        help="Override the Gaussian Splat .ply path instead of reading it from the manifest.",
    )
    parser.add_argument(
        "--preview-topic",
        default="/map_gaussian_splat_preview",
        help="PointCloud2 topic used by current HORUS point-cloud visualization.",
    )
    parser.add_argument(
        "--manifest-topic",
        default="/horus/gaussian_splat/manifest",
        help="std_msgs/String topic carrying future Gaussian Splat renderer metadata.",
    )
    parser.add_argument("--frame-id", default="map", help="ROS frame for the preview point cloud.")
    parser.add_argument(
        "--sample-count",
        type=int,
        default=120000,
        help="Maximum vertices to publish in the PointCloud2 preview. Use 0 for all vertices.",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Scale applied to preview point positions before publication.",
    )
    parser.add_argument(
        "--no-center",
        dest="center",
        action="store_false",
        help="Do not center the sampled preview around the map origin.",
    )
    parser.set_defaults(center=True)
    parser.add_argument(
        "--publish-rate",
        type=float,
        default=1.0,
        help="Republish rate in Hz. Transient-local QoS also keeps the latest message for late subscribers.",
    )
    parser.add_argument(
        "--no-fake-robots",
        dest="publish_fake_robots",
        action="store_false",
        help="Disable the default fake robot TF/odom publishers.",
    )
    parser.set_defaults(publish_fake_robots=True)
    parser.add_argument(
        "--robot-names",
        default=",".join(DEFAULT_ROBOT_NAMES),
        help="Comma-separated fake robots to publish for HORUS registration/visualization.",
    )
    parser.add_argument(
        "--robot-base-frame",
        default="base_link",
        help="Base frame leaf published under each robot prefix, e.g. splat_rover_1/base_link.",
    )
    parser.add_argument(
        "--robot-rate",
        type=float,
        default=20.0,
        help="Fake robot TF/odom publish rate in Hz.",
    )
    parser.add_argument(
        "--robot-motion",
        choices=("circle", "static"),
        default="circle",
        help="Fake robot motion profile.",
    )
    parser.add_argument(
        "--robot-radius",
        type=float,
        default=1.4,
        help="Circle radius in map-frame meters for fake robot motion.",
    )
    parser.add_argument(
        "--robot-height",
        type=float,
        default=0.0,
        help="Fake robot base-frame height above map.",
    )
    parser.add_argument(
        "--robot-angular-speed",
        type=float,
        default=0.22,
        help="Circle angular speed in radians/sec for fake robot motion.",
    )
    return parser.parse_args()


def parse_robot_names(raw_value: str) -> List[str]:
    names = []
    for token in str(raw_value or "").split(","):
        name = token.strip().strip("/")
        if name and name not in names:
            names.append(name)
    return names or list(DEFAULT_ROBOT_NAMES)


def normalize_frame_leaf(raw_value: str, default_value: str = "base_link") -> str:
    value = str(raw_value or "").strip().strip("/")
    return value or default_value


def quaternion_from_yaw(yaw_rad: float) -> Tuple[float, float, float, float]:
    half = yaw_rad * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def build_fake_robot_states(args: argparse.Namespace) -> List[FakeRobotState]:
    if not args.publish_fake_robots:
        return []

    names = parse_robot_names(args.robot_names)
    base_frame = normalize_frame_leaf(args.robot_base_frame)
    count = max(len(names), 1)
    return [
        FakeRobotState(
            name=name,
            base_frame=base_frame,
            phase=(math.tau * index) / count,
        )
        for index, name in enumerate(names)
    ]


def update_fake_robot_states(states: Sequence[FakeRobotState], args: argparse.Namespace, elapsed: float) -> None:
    count = max(len(states), 1)
    radius = max(0.0, float(args.robot_radius))
    omega = float(args.robot_angular_speed)
    z = float(args.robot_height)

    for index, state in enumerate(states):
        state.z = z
        if args.robot_motion == "static" or radius <= 0.0 or abs(omega) <= 1e-6:
            offset = index - ((count - 1) * 0.5)
            state.x = offset * 1.2
            state.y = -1.0
            state.yaw = 0.0
            state.vx = 0.0
            state.vy = 0.0
            continue

        angle = state.phase + (elapsed * omega)
        state.x = math.cos(angle) * radius
        state.y = math.sin(angle) * radius
        state.yaw = angle + (math.pi * 0.5)
        state.vx = -math.sin(angle) * radius * omega
        state.vy = math.cos(angle) * radius * omega


def read_manifest(path: Path) -> Dict:
    if not path.exists():
        raise SystemExit(
            f"Missing fixture manifest: {path}\n"
            "Run: python3 python/examples/tools/fetch_gaussian_splat_fixtures.py --bundle prebuilt"
        )
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def choose_ply_path(args: argparse.Namespace, manifest: Dict) -> Path:
    if args.ply is not None:
        path = args.ply.expanduser()
    else:
        prebuilt = manifest.get("prebuilt") or {}
        path_value = prebuilt.get("splat_ply")
        if not path_value:
            raise SystemExit(
                f"No prebuilt splat_ply entry found in {args.manifest}.\n"
                "Run the fetch tool with: --bundle prebuilt"
            )
        path = Path(path_value).expanduser()

    if not path.exists():
        raise SystemExit(f"Gaussian Splat PLY does not exist: {path}")
    return path


def read_ply_header(path: Path) -> PlyHeader:
    with path.open("rb") as handle:
        first_line = handle.readline()
        if first_line.strip() != b"ply":
            raise ValueError(f"{path} is not a PLY file")

        format_name = ""
        vertex_count: Optional[int] = None
        vertex_properties: List[PlyProperty] = []
        current_element: Optional[str] = None

        while True:
            line_bytes = handle.readline()
            if not line_bytes:
                raise ValueError(f"PLY header ended before end_header in {path}")
            try:
                line = line_bytes.decode("ascii").strip()
            except UnicodeDecodeError as exc:
                raise ValueError(f"PLY header is not ASCII in {path}") from exc

            if line == "end_header":
                return PlyHeader(
                    format_name=format_name,
                    vertex_count=int(vertex_count or 0),
                    vertex_properties=tuple(vertex_properties),
                    data_offset=handle.tell(),
                )

            if not line or line.startswith("comment"):
                continue

            parts = line.split()
            if parts[0] == "format" and len(parts) >= 2:
                format_name = parts[1]
            elif parts[0] == "element" and len(parts) >= 3:
                current_element = parts[1]
                if current_element == "vertex":
                    vertex_count = int(parts[2])
            elif parts[0] == "property" and current_element == "vertex":
                if len(parts) >= 2 and parts[1] == "list":
                    raise ValueError("PLY vertex list properties are not supported for this fixture publisher")
                if len(parts) < 3:
                    raise ValueError(f"Malformed PLY property line: {line}")
                vertex_properties.append(PlyProperty(name=parts[2], data_type=parts[1]))


def selected_indices(vertex_count: int, sample_count: int) -> List[int]:
    if sample_count <= 0 or sample_count >= vertex_count:
        return list(range(vertex_count))
    if sample_count == 1:
        return [0]
    last = vertex_count - 1
    return [round((last * i) / (sample_count - 1)) for i in range(sample_count)]


def clamp_u8(value: float) -> int:
    if not math.isfinite(value):
        return 0
    return max(0, min(255, int(round(value))))


def sh_dc_to_u8(value: float) -> int:
    return clamp_u8((0.5 + SH_C0 * value) * 255.0)


def rgb_as_float(red: int, green: int, blue: int) -> float:
    packed = (clamp_u8(red) << 16) | (clamp_u8(green) << 8) | clamp_u8(blue)
    return struct.unpack("<f", struct.pack("<I", packed))[0]


def vertex_to_point(record: Dict[str, float]) -> Tuple[float, float, float, float]:
    try:
        x = float(record["x"])
        y = float(record["y"])
        z = float(record["z"])
    except KeyError as exc:
        raise ValueError("PLY must contain x, y, and z vertex properties") from exc

    if {"red", "green", "blue"}.issubset(record):
        red = clamp_u8(float(record["red"]))
        green = clamp_u8(float(record["green"]))
        blue = clamp_u8(float(record["blue"]))
    elif {"r", "g", "b"}.issubset(record):
        red = clamp_u8(float(record["r"]))
        green = clamp_u8(float(record["g"]))
        blue = clamp_u8(float(record["b"]))
    elif {"f_dc_0", "f_dc_1", "f_dc_2"}.issubset(record):
        red = sh_dc_to_u8(float(record["f_dc_0"]))
        green = sh_dc_to_u8(float(record["f_dc_1"]))
        blue = sh_dc_to_u8(float(record["f_dc_2"]))
    else:
        red, green, blue = 110, 215, 255

    return x, y, z, rgb_as_float(red, green, blue)


def parse_ascii_vertices(path: Path, header: PlyHeader, indices: Sequence[int]) -> List[Tuple[float, float, float, float]]:
    wanted = set(indices)
    points: List[Tuple[float, float, float, float]] = []
    property_names = [prop.name for prop in header.vertex_properties]

    with path.open("rb") as handle:
        handle.seek(header.data_offset)
        for vertex_index in range(header.vertex_count):
            line = handle.readline().decode("ascii").strip()
            if vertex_index not in wanted:
                continue
            values = line.split()
            record = {
                name: float(value)
                for name, value in zip(property_names, values)
            }
            points.append(vertex_to_point(record))
    return points


def binary_vertex_struct(header: PlyHeader) -> struct.Struct:
    formats = []
    for prop in header.vertex_properties:
        try:
            formats.append(PLY_STRUCT_TYPES[prop.data_type])
        except KeyError as exc:
            raise ValueError(f"Unsupported PLY vertex property type: {prop.data_type}") from exc
    return struct.Struct("<" + "".join(formats))


def parse_binary_vertices(path: Path, header: PlyHeader, indices: Sequence[int]) -> List[Tuple[float, float, float, float]]:
    vertex_struct = binary_vertex_struct(header)
    property_names = [prop.name for prop in header.vertex_properties]
    points: List[Tuple[float, float, float, float]] = []

    with path.open("rb") as handle:
        for vertex_index in indices:
            handle.seek(header.data_offset + vertex_index * vertex_struct.size)
            data = handle.read(vertex_struct.size)
            if len(data) != vertex_struct.size:
                raise ValueError(f"Unexpected end of PLY vertex data at index {vertex_index}")
            values = vertex_struct.unpack(data)
            points.append(vertex_to_point(dict(zip(property_names, values))))
    return points


def load_preview_points(path: Path, sample_count: int, scale: float, center: bool) -> PreviewData:
    header = read_ply_header(path)
    if header.vertex_count <= 0:
        raise ValueError(f"No vertices found in {path}")
    if header.format_name not in ("ascii", "binary_little_endian"):
        raise ValueError(f"Unsupported PLY format: {header.format_name}")

    indices = selected_indices(header.vertex_count, sample_count)
    if header.format_name == "ascii":
        points = parse_ascii_vertices(path, header, indices)
    else:
        points = parse_binary_vertices(path, header, indices)

    if not points:
        raise ValueError(f"No preview points were sampled from {path}")

    center_offset = (0.0, 0.0, 0.0)
    if center:
        center_offset = (
            sum(point[0] for point in points) / len(points),
            sum(point[1] for point in points) / len(points),
            sum(point[2] for point in points) / len(points),
        )

    transformed = [
        (
            (x - center_offset[0]) * scale,
            (y - center_offset[1]) * scale,
            (z - center_offset[2]) * scale,
            rgb,
        )
        for x, y, z, rgb in points
    ]

    return PreviewData(
        points=transformed,
        original_vertex_count=header.vertex_count,
        published_vertex_count=len(transformed),
        center_offset=center_offset,
        scale=scale,
        ply_path=path,
    )


def build_runtime_manifest(source_manifest: Dict, preview: PreviewData, args: argparse.Namespace) -> Dict:
    prebuilt = source_manifest.get("prebuilt") or {}
    return {
        "schema": "horus.gaussian_splat.fixture.v1",
        "visualization_type": "gaussian_splat",
        "source_format": "3dgs_ply",
        "splat_ply": str(preview.ply_path),
        "dataset": prebuilt.get("dataset", "custom"),
        "scene": prebuilt.get("scene", ""),
        "iteration": prebuilt.get("iteration", ""),
        "source_url": prebuilt.get("source_url", ""),
        "frame_id": args.frame_id,
        "preview_topic": args.preview_topic,
        "manifest_topic": args.manifest_topic,
        "original_vertex_count": preview.original_vertex_count,
        "published_preview_vertex_count": preview.published_vertex_count,
        "preview_center_offset": preview.center_offset,
        "preview_scale": preview.scale,
        "fake_robots": [
            {
                "name": state.name,
                "base_frame": state.base_frame,
                "tf_frame": f"{state.name}/{state.base_frame}",
                "odom_topic": f"/{state.name}/odom",
            }
            for state in build_fake_robot_states(args)
        ],
    }


def publish_ros(preview: PreviewData, runtime_manifest: Dict, args: argparse.Namespace) -> None:
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2, PointField
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header, String
        from tf2_msgs.msg import TFMessage
    except ImportError as exc:
        raise SystemExit(
            "ROS2 Python packages are not available in this shell.\n"
            "Run from WSL after sourcing ROS2, for example:\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            "  source ~/horus_ws/install/setup.bash"
        ) from exc

    rclpy.init()
    node = rclpy.create_node("horus_gaussian_splat_fixture_publisher")

    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    point_pub = node.create_publisher(PointCloud2, args.preview_topic, qos)
    manifest_pub = node.create_publisher(String, args.manifest_topic, qos)

    fake_robot_states = build_fake_robot_states(args)
    tf_pub = None
    tf_static_pub = None
    odom_pubs = {}
    if fake_robot_states:
        tf_pub = node.create_publisher(TFMessage, "/tf", 10)

        static_qos = QoSProfile(depth=1)
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        tf_static_pub = node.create_publisher(TFMessage, "/tf_static", static_qos)

        odom_qos = QoSProfile(depth=10)
        odom_qos.reliability = ReliabilityPolicy.RELIABLE
        for state in fake_robot_states:
            odom_pubs[state.name] = node.create_publisher(Odometry, f"/{state.name}/odom", odom_qos)

    header = Header()
    header.frame_id = args.frame_id
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    cloud_msg = point_cloud2.create_cloud(header, fields, preview.points)
    manifest_msg = String()
    manifest_msg.data = json.dumps(runtime_manifest, separators=(",", ":"))

    period = 1.0 / args.publish_rate if args.publish_rate > 0 else 5.0
    start_time = time.monotonic()

    def make_transform(
        stamp,
        parent_frame: str,
        child_frame: str,
        xyz: Tuple[float, float, float],
        yaw_rad: float,
    ):
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame
        msg.transform.translation.x = float(xyz[0])
        msg.transform.translation.y = float(xyz[1])
        msg.transform.translation.z = float(xyz[2])
        qx, qy, qz, qw = quaternion_from_yaw(yaw_rad)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg

    def publish_static_robot_frames() -> None:
        if tf_static_pub is None:
            return

        stamp = node.get_clock().now().to_msg()
        transforms = []
        static_children = [
            ("base_footprint", (0.0, 0.0, -0.08), 0.0),
            ("left_wheel", (0.0, 0.24, -0.08), 0.0),
            ("right_wheel", (0.0, -0.24, -0.08), 0.0),
            ("lidar_link", (0.22, 0.0, 0.18), 0.0),
            ("camera_link", (0.28, 0.0, 0.22), 0.0),
        ]
        for state in fake_robot_states:
            parent = f"{state.name}/{state.base_frame}"
            for child, xyz, yaw in static_children:
                transforms.append(
                    make_transform(stamp, parent, f"{state.name}/{child}", xyz, yaw)
                )

        if transforms:
            tf_static_pub.publish(TFMessage(transforms=transforms))

    def publish_fake_robots() -> None:
        if tf_pub is None:
            return

        stamp = node.get_clock().now().to_msg()
        elapsed = time.monotonic() - start_time
        update_fake_robot_states(fake_robot_states, args, elapsed)

        transforms = []
        for state in fake_robot_states:
            child_frame = f"{state.name}/{state.base_frame}"
            transforms.append(
                make_transform(
                    stamp,
                    args.frame_id,
                    child_frame,
                    (state.x, state.y, state.z),
                    state.yaw,
                )
            )

            odom_pub = odom_pubs.get(state.name)
            if odom_pub is not None:
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = args.frame_id
                odom.child_frame_id = child_frame
                odom.pose.pose.position.x = state.x
                odom.pose.pose.position.y = state.y
                odom.pose.pose.position.z = state.z
                qx, qy, qz, qw = quaternion_from_yaw(state.yaw)
                odom.pose.pose.orientation.x = qx
                odom.pose.pose.orientation.y = qy
                odom.pose.pose.orientation.z = qz
                odom.pose.pose.orientation.w = qw
                odom.twist.twist.linear.x = state.vx
                odom.twist.twist.linear.y = state.vy
                odom_pub.publish(odom)

        if transforms:
            tf_pub.publish(TFMessage(transforms=transforms))

    def publish_once() -> None:
        cloud_msg.header.stamp = node.get_clock().now().to_msg()
        point_pub.publish(cloud_msg)
        manifest_pub.publish(manifest_msg)

    node.create_timer(period, publish_once)
    publish_once()
    if fake_robot_states:
        robot_period = 1.0 / max(0.1, float(args.robot_rate))
        node.create_timer(robot_period, publish_fake_robots)
        publish_static_robot_frames()
        publish_fake_robots()

    node.get_logger().info(
        f"Publishing {preview.published_vertex_count}/{preview.original_vertex_count} vertices "
        f"from {preview.ply_path} on {args.preview_topic}"
    )
    node.get_logger().info(f"Publishing Gaussian Splat manifest on {args.manifest_topic}")
    if fake_robot_states:
        robot_summary = ", ".join(f"{state.name}/{state.base_frame}" for state in fake_robot_states)
        node.get_logger().info(
            f"Publishing fake robot TF on /tf, static frames on /tf_static, and odom for {robot_summary}"
        )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    args = parse_args()
    source_manifest = read_manifest(args.manifest.expanduser()) if args.ply is None else {}
    ply_path = choose_ply_path(args, source_manifest)
    preview = load_preview_points(ply_path, args.sample_count, args.scale, args.center)
    runtime_manifest = build_runtime_manifest(source_manifest, preview, args)

    print(
        f"Loaded {preview.published_vertex_count}/{preview.original_vertex_count} preview vertices "
        f"from {ply_path}"
    )
    print(f"Preview topic:  {args.preview_topic}")
    print(f"Manifest topic: {args.manifest_topic}")
    publish_ros(preview, runtime_manifest, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
