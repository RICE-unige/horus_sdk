#!/usr/bin/env python3
"""Register live Isaac hospital Carter robots with compressed camera + prefixed TF.

Expected live topics per robot by default:
- `/<robot>/tf`
- `/<robot>/front_stereo_camera/left/image_raw`
- `/<robot>/chassis/odom`
- `/<robot>/front_2d_lidar/scan`
- `/<robot>/cmd_vel`

This demo:
1. Probes required live topics and samples first messages.
2. Starts `image_transport` republishers (raw -> compressed).
3. Waits for compressed image topics to publish.
4. Relays `/<robot>/tf` into a shared TF topic (default: `/tf`) with prefixed frame IDs.
5. Registers robots in HORUS using compressed camera topics.
"""

import argparse
import os
import subprocess
import sys
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.join(SCRIPT_DIR, "..")
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
GLOBAL_ROOT_FRAMES = {"map", "world"}


@dataclass(frozen=True)
class CameraProbeResult:
    topic: str
    frame_id: str
    width: int
    height: int
    encoding: str


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
    tf_source_topic: str
    tf_output_topic: str
    cmd_vel_topic: str
    camera_raw_topic: str
    camera_compressed_topic: str
    camera_frame: str
    camera: CameraProbeResult
    odom: OdomProbeResult
    scan: Optional[ScanProbeResult]


def _tf_source_topic(robot_name: str) -> str:
    return f"/{robot_name}/tf"


def _camera_raw_topic(robot_name: str) -> str:
    return f"/{robot_name}/front_stereo_camera/left/image_raw"


def _camera_compressed_topic(robot_name: str) -> str:
    return f"/{robot_name}/front_stereo_camera/left/image_raw/compressed"


def _odom_topic(robot_name: str) -> str:
    return f"/{robot_name}/chassis/odom"


def _scan_topic(robot_name: str) -> str:
    return f"/{robot_name}/front_2d_lidar/scan"


def _cmd_vel_topic(robot_name: str) -> str:
    return f"/{robot_name}/cmd_vel"


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


def _topic_types_by_name(node) -> Dict[str, Tuple[str, ...]]:
    return {
        str(topic): tuple(str(item) for item in types)
        for topic, types in node.get_topic_names_and_types()
    }


def _topic_types_from_ros2_cli(timeout_sec: float) -> Dict[str, Tuple[str, ...]]:
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


def resolve_camera_frame_id(
    robot_name: str,
    camera_header_frame: str,
    available_tf_frames: Sequence[str],
) -> str:
    normalized_frames = {
        str(frame).strip().lstrip("/")
        for frame in available_tf_frames
        if str(frame).strip()
    }
    header_frame = str(camera_header_frame or "").strip().lstrip("/")

    candidates: List[str] = []
    if header_frame:
        candidates.append(header_frame)
        if header_frame.endswith("_optical"):
            candidates.append(header_frame[: -len("_optical")] + "_rgb")
        if header_frame.endswith("_rgb"):
            candidates.append(header_frame[: -len("_rgb")] + "_optical")

    candidates.extend(
        [
            "front_stereo_camera_left_rgb",
            "front_stereo_camera_left_optical",
            "base_link",
        ]
    )

    for candidate in candidates:
        normalized_candidate = str(candidate or "").strip().lstrip("/")
        if normalized_candidate and normalized_candidate in normalized_frames:
            return prefix_frame_id(normalized_candidate, robot_name)

    if header_frame:
        return prefix_frame_id(header_frame, robot_name)
    return prefix_frame_id("base_link", robot_name)


def probe_live_robots(
    robot_names: Sequence[str],
    tf_output_topic: str,
    include_scan: bool,
    timeout_sec: float,
) -> List[RobotProbeResult]:
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, LaserScan
    from tf2_msgs.msg import TFMessage

    timeout_sec = max(0.5, float(timeout_sec))
    expected_types: Dict[str, str] = {}

    for robot_name in robot_names:
        expected_types[_tf_source_topic(robot_name)] = "tf2_msgs/msg/TFMessage"
        expected_types[_camera_raw_topic(robot_name)] = "sensor_msgs/msg/Image"
        expected_types[_odom_topic(robot_name)] = "nav_msgs/msg/Odometry"
        expected_types[_cmd_vel_topic(robot_name)] = "geometry_msgs/msg/Twist"
        if include_scan:
            expected_types[_scan_topic(robot_name)] = "sensor_msgs/msg/LaserScan"

    node_name = f"horus_hospital_probe_{uuid.uuid4().hex[:8]}"
    node = rclpy.create_node(node_name)

    camera_results: Dict[str, Optional[CameraProbeResult]] = {
        name: None for name in robot_names
    }
    odom_results: Dict[str, Optional[OdomProbeResult]] = {name: None for name in robot_names}
    scan_results: Dict[str, Optional[ScanProbeResult]] = {name: None for name in robot_names}
    tf_frames: Dict[str, set[str]] = {name: set() for name in robot_names}
    tf_seen: Dict[str, bool] = {name: False for name in robot_names}
    subscriptions = []

    def make_camera_callback(name: str):
        def _callback(msg: Image):
            if camera_results[name] is not None:
                return
            camera_results[name] = CameraProbeResult(
                topic=_camera_raw_topic(name),
                frame_id=str(msg.header.frame_id or "").strip(),
                width=int(max(1, msg.width)),
                height=int(max(1, msg.height)),
                encoding=str(msg.encoding or "").strip() or "rgb8",
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

    def make_tf_callback(name: str):
        def _callback(msg: TFMessage):
            transforms = list(getattr(msg, "transforms", []))
            if not transforms:
                return
            tf_seen[name] = True
            for transform in transforms:
                parent_frame = str(transform.header.frame_id or "").strip().lstrip("/")
                child_frame = str(transform.child_frame_id or "").strip().lstrip("/")
                if parent_frame:
                    tf_frames[name].add(parent_frame)
                if child_frame:
                    tf_frames[name].add(child_frame)

        return _callback

    try:
        for robot_name in robot_names:
            subscriptions.append(
                node.create_subscription(
                    Image,
                    _camera_raw_topic(robot_name),
                    make_camera_callback(robot_name),
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
            subscriptions.append(
                node.create_subscription(
                    TFMessage,
                    _tf_source_topic(robot_name),
                    make_tf_callback(robot_name),
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

        last_topic_types = _topic_types_from_ros2_cli(timeout_sec=min(2.0, timeout_sec))
        last_topic_types.update(_topic_types_by_name(node))

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            topic_problems = _missing_or_wrong_topics(last_topic_types, expected_types)
            all_cameras = all(camera_results[name] is not None for name in robot_names)
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
                    f"Timed out waiting for first image on {_camera_raw_topic(robot_name)}."
                )
            if odom_results[robot_name] is None:
                raise RuntimeError(
                    f"Timed out waiting for first odometry message on {_odom_topic(robot_name)}."
                )
            if not tf_seen[robot_name]:
                raise RuntimeError(
                    f"Timed out waiting for TF messages on {_tf_source_topic(robot_name)}."
                )
            if include_scan and scan_results[robot_name] is None:
                raise RuntimeError(
                    f"Timed out waiting for LaserScan message on {_scan_topic(robot_name)}."
                )

        probes: List[RobotProbeResult] = []
        for robot_name in robot_names:
            resolved_camera_frame = resolve_camera_frame_id(
                robot_name=robot_name,
                camera_header_frame=camera_results[robot_name].frame_id,  # type: ignore[union-attr]
                available_tf_frames=sorted(tf_frames[robot_name]),
            )
            probes.append(
                RobotProbeResult(
                    robot_name=robot_name,
                    tf_source_topic=_tf_source_topic(robot_name),
                    tf_output_topic=tf_output_topic,
                    cmd_vel_topic=_cmd_vel_topic(robot_name),
                    camera_raw_topic=_camera_raw_topic(robot_name),
                    camera_compressed_topic=_camera_compressed_topic(robot_name),
                    camera_frame=resolved_camera_frame,
                    camera=camera_results[robot_name],  # type: ignore[arg-type]
                    odom=odom_results[robot_name],  # type: ignore[arg-type]
                    scan=scan_results[robot_name] if include_scan else None,
                )
            )
        return probes
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


def start_image_transport_republishers(
    probes: Sequence[RobotProbeResult],
) -> List[subprocess.Popen]:
    processes: List[subprocess.Popen] = []
    for probe in probes:
        command = [
            "ros2",
            "run",
            "image_transport",
            "republish",
            "--ros-args",
            "-r",
            f"__node:={probe.robot_name}_image_republisher",
            "-p",
            "in_transport:=raw",
            "-p",
            "out_transport:=compressed",
            "-r",
            f"in:={probe.camera_raw_topic}",
            "-r",
            f"out/compressed:={probe.camera_compressed_topic}",
        ]
        cli.print_info(
            f"Starting image republisher for {probe.robot_name}: "
            f"{probe.camera_raw_topic} -> {probe.camera_compressed_topic}"
        )
        process = subprocess.Popen(command)
        processes.append(process)

    time.sleep(0.6)
    for process in processes:
        if process.poll() is not None:
            raise RuntimeError(
                f"image_transport republisher exited early with code {process.returncode}."
            )
    return processes


def stop_processes(processes: Sequence[subprocess.Popen]) -> None:
    for process in processes:
        if process.poll() is None:
            process.terminate()
    deadline = time.monotonic() + 5.0
    for process in processes:
        if process.poll() is not None:
            continue
        remaining = max(0.0, deadline - time.monotonic())
        try:
            process.wait(timeout=max(0.1, remaining))
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)


def wait_for_compressed_topics(
    probes: Sequence[RobotProbeResult],
    timeout_sec: float,
    processes: Sequence[subprocess.Popen],
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
            for process in processes:
                if process.poll() is not None:
                    raise RuntimeError(
                        "image_transport republisher exited unexpectedly while waiting "
                        "for compressed images."
                    )
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


class PrefixedTfRelay:
    """Relay one robot TF stream to prefixed frame IDs."""

    def __init__(
        self,
        robot_name: str,
        source_topic: str,
        output_topic: str,
        odom_frame: str,
        map_frame: str = "map",
    ) -> None:
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from tf2_msgs.msg import TFMessage

        self._transform_type = TransformStamped
        self._tf_message_type = TFMessage
        self.robot_name = str(robot_name or "").strip()
        self.source_topic = str(source_topic or "").strip()
        self.output_topic = str(output_topic or "").strip()
        self.map_frame = str(map_frame or "").strip() or "map"
        self.odom_frame = (
            prefix_frame_id(odom_frame, self.robot_name) or f"{self.robot_name}/odom"
        )
        self.active = threading.Event()

        node_name = f"horus_hospital_tf_relay_{self.robot_name}_{uuid.uuid4().hex[:8]}"
        self.node = rclpy.create_node(node_name)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=100,
        )
        self.publisher = self.node.create_publisher(
            self._tf_message_type, self.output_topic, qos
        )
        self.subscription = self.node.create_subscription(
            self._tf_message_type, self.source_topic, self._on_message, qos
        )

    def _on_message(self, msg) -> None:
        transforms = []
        has_root_transform = False
        for transform in getattr(msg, "transforms", []):
            parent_frame = prefix_frame_id(transform.header.frame_id, self.robot_name)
            child_frame = prefix_frame_id(transform.child_frame_id, self.robot_name)
            if not parent_frame or not child_frame:
                continue

            if parent_frame == self.map_frame and child_frame == self.odom_frame:
                has_root_transform = True

            rewritten = self._transform_type()
            rewritten.header.stamp = transform.header.stamp
            rewritten.header.frame_id = parent_frame
            rewritten.child_frame_id = child_frame
            rewritten.transform.translation = transform.transform.translation
            rewritten.transform.rotation = transform.transform.rotation
            transforms.append(rewritten)

        if not transforms:
            return

        if not has_root_transform and self.odom_frame != self.map_frame:
            root_tf = self._transform_type()
            root_tf.header.stamp = transforms[0].header.stamp
            root_tf.header.frame_id = self.map_frame
            root_tf.child_frame_id = self.odom_frame
            root_tf.transform.translation.x = 0.0
            root_tf.transform.translation.y = 0.0
            root_tf.transform.translation.z = 0.0
            root_tf.transform.rotation.x = 0.0
            root_tf.transform.rotation.y = 0.0
            root_tf.transform.rotation.z = 0.0
            root_tf.transform.rotation.w = 1.0
            transforms.insert(0, root_tf)

        self.publisher.publish(self._tf_message_type(transforms=transforms))
        self.active.set()

    def destroy(self) -> None:
        try:
            self.node.destroy_node()
        except Exception:
            pass


class TfRelayRunner:
    """Run multiple TF relay nodes on one executor thread."""

    def __init__(self, relays: Sequence[PrefixedTfRelay]) -> None:
        from rclpy.executors import MultiThreadedExecutor

        self.relays = list(relays)
        self.executor = MultiThreadedExecutor(num_threads=max(2, len(self.relays)))
        for relay in self.relays:
            self.executor.add_node(relay.node)
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

    def wait_until_active(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.5, float(timeout_sec))
        while time.monotonic() < deadline:
            if all(relay.active.is_set() for relay in self.relays):
                return True
            time.sleep(0.05)
        return all(relay.active.is_set() for relay in self.relays)

    def stop(self) -> None:
        self._stop_requested.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        for relay in self.relays:
            try:
                self.executor.remove_node(relay.node)
            except Exception:
                pass
            relay.destroy()
        try:
            self.executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass


def build_robot_and_dataviz(
    probe: RobotProbeResult,
    include_scan: bool,
    minimap_streaming_type: str,
    teleop_streaming_type: str,
) -> Tuple[Robot, DataViz]:
    robot = Robot(
        name=probe.robot_name,
        robot_type=RobotType.WHEELED,
        dimensions=RobotDimensions(length=0.75, width=0.50, height=0.55),
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
            "go_to_point": {"enabled": False},
            "waypoint": {"enabled": False},
        },
    )

    robot_tf_frame = prefix_frame_id(probe.odom.child_frame_id, probe.robot_name)
    if not robot_tf_frame:
        robot_tf_frame = f"{probe.robot_name}/base_link"

    camera = Camera(
        name="front_camera",
        frame_id=robot_tf_frame,
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
    camera.add_metadata("image_scale", 0.072)
    camera.add_metadata("focal_length_scale", 0.108)
    camera.add_metadata("camera_source_frame_hint", probe.camera_frame)
    camera.add_metadata("view_position_offset", [0.0, 0.0, 0.0])
    camera.add_metadata("view_rotation_offset", [0.0, 0.0, 0.0])
    camera.add_metadata("show_frustum", True)
    camera.add_metadata("frustum_color", "#FFFF00")
    camera.add_metadata("overhead_size", 1.0)
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

    dataviz = DataViz(name=f"{probe.robot_name}_hospital_live_viz")
    dataviz.add_sensor_visualization(camera, robot_name=probe.robot_name)
    if scan_sensor is not None:
        dataviz.add_sensor_visualization(scan_sensor, robot_name=probe.robot_name)

    dataviz.add_robot_transform(
        robot_name=probe.robot_name,
        topic=probe.tf_output_topic,
        frame_id=robot_tf_frame,
    )
    dataviz.add_robot_velocity_data(
        robot_name=probe.robot_name,
        topic=probe.odom.topic,
        frame_id="map",
    )
    dataviz.add_robot_odometry_trail(
        robot_name=probe.robot_name,
        topic=probe.odom.topic,
        frame_id="map",
    )

    return robot, dataviz


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Register live hospital Carter robots with compressed camera transport "
            "and prefixed TF relay."
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
        help=f"Shared prefixed TF output topic (default: {DEFAULT_TF_TOPIC}).",
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
    return parser


def _print_probe_summary(probes: Sequence[RobotProbeResult], include_scan: bool) -> None:
    tf_output_topic = probes[0].tf_output_topic if probes else DEFAULT_TF_TOPIC
    cli.print_info(f"shared_tf_topic={tf_output_topic}")
    for probe in probes:
        cli.print_info(
            f"[{probe.robot_name}] camera_raw={probe.camera_raw_topic} "
            f"({probe.camera.width}x{probe.camera.height}, {probe.camera.encoding})"
        )
        cli.print_info(
            f"[{probe.robot_name}] camera_frame={probe.camera_frame}"
        )
        cli.print_info(
            f"[{probe.robot_name}] tf_source={probe.tf_source_topic} "
            f"odom={probe.odom.topic} cmd_vel={probe.cmd_vel_topic}"
        )
        if include_scan and probe.scan is not None:
            cli.print_info(
                f"[{probe.robot_name}] scan={probe.scan.topic} frame={probe.scan.frame_id}"
            )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    robot_names = _parse_robot_names(args.robot_names)
    tf_output_topic = str(args.tf_topic or "").strip() or DEFAULT_TF_TOPIC
    if not tf_output_topic.startswith("/"):
        tf_output_topic = "/" + tf_output_topic.lstrip("/")
    include_scan = bool(args.with_scan)
    timeout_sec = max(0.5, float(args.probe_timeout_sec))
    minimap_streaming_type = _normalize_transport(
        args.camera_minimap_streaming_type, "ros"
    )
    teleop_streaming_type = _normalize_transport(
        args.camera_teleop_streaming_type, "webrtc"
    )

    compression_processes: List[subprocess.Popen] = []
    relay_runner: Optional[TfRelayRunner] = None
    rclpy_initialized = False

    try:
        import rclpy

        if not rclpy.ok():
            rclpy.init()
            rclpy_initialized = True

        cli.print_step("Probing live hospital Carter topics...")
        probes = probe_live_robots(
            robot_names=robot_names,
            tf_output_topic=tf_output_topic,
            include_scan=include_scan,
            timeout_sec=timeout_sec,
        )
        _print_probe_summary(probes, include_scan)

        cli.print_step("Starting image_transport republishers (raw -> compressed)...")
        compression_processes = start_image_transport_republishers(probes)

        cli.print_step("Waiting for compressed image topics...")
        wait_for_compressed_topics(
            probes=probes,
            timeout_sec=timeout_sec,
            processes=compression_processes,
        )
        cli.print_success("Compressed camera topics are active.")

        cli.print_step("Starting per-robot prefixed TF relays...")
        relays = [
            PrefixedTfRelay(
                robot_name=probe.robot_name,
                source_topic=probe.tf_source_topic,
                output_topic=probe.tf_output_topic,
                odom_frame=probe.odom.frame_id or "odom",
                map_frame="map",
            )
            for probe in probes
        ]
        relay_runner = TfRelayRunner(relays)
        relay_runner.start()
        if not relay_runner.wait_until_active(timeout_sec=timeout_sec):
            raise RuntimeError(
                "Timed out waiting for TF relay activation on topics: "
                + ", ".join(probe.tf_output_topic for probe in probes)
            )
        cli.print_success("Prefixed TF relays are active.")

        robots: List[Robot] = []
        datavizs: List[DataViz] = []
        for probe in probes:
            robot, dataviz = build_robot_and_dataviz(
                probe=probe,
                include_scan=include_scan,
                minimap_streaming_type=minimap_streaming_type,
                teleop_streaming_type=teleop_streaming_type,
            )
            robots.append(robot)
            datavizs.append(dataviz)

        cli.print_step(f"Registering {len(robots)} robot(s) with HORUS...")
        success, result = register_robots(
            robots,
            datavizs=datavizs,
            keep_alive=bool(args.keep_alive),
            show_dashboard=True,
            workspace_scale=float(args.workspace_scale),
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
        if relay_runner is not None:
            cli.print_info("Stopping TF relays...")
            relay_runner.stop()
        if compression_processes:
            cli.print_info("Stopping image_transport republishers...")
            stop_processes(compression_processes)

        if rclpy_initialized:
            try:
                import rclpy

                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
