#!/usr/bin/env python3
"""Render a workspace map on the PC and publish pose-tagged RGB-D frames.

Real scenes use one headset-driven server camera. The Quest warps each RGB-D
frame from its embedded render pose and retains a bounded set of spatial
keyframes, so newly revealed surfaces do not destructively replace valid map
content from earlier viewpoints.
"""

from __future__ import annotations

import argparse
from array import array
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from horus.remote_rendering import (
    COW_LADY_DEPTH_FAR_BASE,
    COW_LADY_DEPTH_NEAR_BASE,
    DEFAULT_VIEWER_POSE_TOPIC,
    DEPTH_FAR_METERS,
    DEPTH_NEAR_METERS,
    DYNAMIC_RGBD_FORMAT_VERSION,
    ETH3D_COURTYARD_DEPTH_FAR_BASE,
    ETH3D_COURTYARD_DEPTH_NEAR_BASE,
    ETH3D_COURTYARD_POINT_COUNT,
    REMOTE_RGBD_FORMAT_VERSION,
    VIEWER_POSE_VERSION,
    VERTICAL_FOV_DEGREES,
    CudaPointRenderer,
    DynamicCameraPose,
    build_dynamic_camera_poses,
    build_remote_map_rgbd,
    encode_luma_depth,
    pack_dynamic_rgbd,
)
from horus.remote_rendering.real_scene import (
    COW_LADY_VIEW_SPECS,
    ETH3D_COURTYARD_VIEW_SPECS,
    fill_small_depth_holes,
    iter_eth3d_courtyard_chunks,
    load_colored_vertex_ply,
    prepare_cow_lady_points,
    resolve_cow_lady_ply,
    view_spec_camera_pose,
)


STREAM_TOPIC = "/horus/remote_render/map_portal"
ROS_STREAM_TOPIC = "/horus/remote_render/map_rgbd/compressed"
AGENT_STATUS_TOPIC = "/horus/remote_render/agent_status"


class RemoteMapRenderSource(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("horus_remote_map_render_source")
        self.args = args
        self.frame_index = 0
        self.rendered_frame_count = 0
        self.last_render_ms = 0.0
        self.valid_depth_fraction = 0.0
        self.latest_pose_sequence = 0
        self.latest_render_sequence = 0
        self.last_published_render_sequence = -1
        self.last_pose_time = 0.0
        self._frame_lock = threading.Lock()
        self._pose_lock = threading.Lock()
        self._pose_event = threading.Event()
        self._stop_event = threading.Event()
        self._render_thread: threading.Thread | None = None
        self._cuda_renderer: CudaPointRenderer | None = None
        self.encoded_frame: bytes | None = None

        if args.dynamic_view:
            self._initialize_dynamic_source()
        else:
            self._initialize_static_source()
        if args.transport == "ros_compressed" and self.encoded_frame is None:
            self.encoded_frame = self._encode_ros_frame(self.packed)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=(
                ReliabilityPolicy.RELIABLE
                if args.transport == "ros_compressed"
                else ReliabilityPolicy.BEST_EFFORT
            ),
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=4,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=4,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        if args.transport == "ros_compressed":
            self.image_publisher = self.create_publisher(
                CompressedImage, ROS_STREAM_TOPIC, image_qos
            )
        else:
            self.image_publisher = self.create_publisher(Image, STREAM_TOPIC, image_qos)
        self.status_publisher = self.create_publisher(String, AGENT_STATUS_TOPIC, status_qos)
        self.pose_subscription = None
        if args.dynamic_view:
            self.pose_subscription = self.create_subscription(
                String,
                args.viewer_pose_topic,
                self._handle_viewer_pose,
                pose_qos,
            )
            self._render_thread = threading.Thread(
                target=self._render_loop,
                name="horus-remote-map-cuda",
                daemon=True,
            )
            self._render_thread.start()
        self.create_timer(1.0 / args.fps, self._publish_frame)
        self.create_timer(1.0, self._publish_status)

    @property
    def format_version(self) -> str:
        return DYNAMIC_RGBD_FORMAT_VERSION if self.args.dynamic_view else REMOTE_RGBD_FORMAT_VERSION

    @property
    def renderer_name(self) -> str:
        if self.args.dynamic_view:
            return "cuda_points"
        if self.args.scene != "synthetic":
            return "cuda_static_proxy"
        return "native_pc"

    def _initialize_static_source(self) -> None:
        args = self.args
        if args.scene in ("cow_lady", "eth3d_courtyard"):
            self._initialize_static_complete_source()
            return

        self.color, self.depth, packed = build_remote_map_rgbd(args.width, args.height)
        self.depth_near_m = DEPTH_NEAR_METERS
        self.depth_far_m = DEPTH_FAR_METERS
        self.source_point_count = 0
        self.valid_depth_fraction = float(np.isfinite(self.depth).mean())
        self.packed_width = int(packed.shape[1])
        self.packed_height = int(packed.shape[0])
        self.packed = packed
        self.frames = self._build_static_frames(packed)

    def _initialize_static_complete_source(self) -> None:
        args = self.args
        if args.width % 3 or args.height % 3:
            raise ValueError("complete static atlases require dimensions divisible by three")

        renderer: CudaPointRenderer | None = None
        try:
            if args.scene == "cow_lady":
                points, colors = load_colored_vertex_ply(
                    resolve_cow_lady_ply(args.dataset_path or None)
                )
                points = prepare_cow_lady_points(points, world_scale=args.dataset_scale)
                self.source_point_count = len(points)
                self.depth_near_m = COW_LADY_DEPTH_NEAR_BASE * args.dataset_scale
                self.depth_far_m = COW_LADY_DEPTH_FAR_BASE * args.dataset_scale
                views = COW_LADY_VIEW_SPECS
                renderer = CudaPointRenderer(self.source_point_count)
                renderer.upload(points, colors)
            else:
                self.source_point_count = ETH3D_COURTYARD_POINT_COUNT
                self.depth_near_m = ETH3D_COURTYARD_DEPTH_NEAR_BASE * args.dataset_scale
                self.depth_far_m = ETH3D_COURTYARD_DEPTH_FAR_BASE * args.dataset_scale
                views = ETH3D_COURTYARD_VIEW_SPECS
                renderer = CudaPointRenderer(self.source_point_count)
                offset = 0
                for points, colors in iter_eth3d_courtyard_chunks(
                    args.dataset_path or None,
                    world_scale=args.dataset_scale,
                    chunk_size=1_000_000,
                ):
                    renderer.upload(points, colors, offset=offset)
                    offset += len(points)
                    if offset % 5_000_000 < len(points):
                        print(
                            f"[remote-map-source] uploaded {offset:,}/"
                            f"{self.source_point_count:,} ETH3D points to CUDA",
                            flush=True,
                        )
                if offset != self.source_point_count:
                    raise RuntimeError(
                        f"ETH3D source contained {offset:,} points, "
                        f"expected {self.source_point_count:,}"
                    )

            camera_poses = [
                view_spec_camera_pose(view, world_scale=args.dataset_scale)
                for view in views
            ]
            tile_width = args.width // 3
            tile_height = args.height // 3
            colors, depths, cuda_ms = renderer.render_views(
                [position for position, _ in camera_poses],
                [rotation for _, rotation in camera_poses],
                tile_width,
                tile_height,
                vertical_fov_deg=VERTICAL_FOV_DEGREES,
                near_m=self.depth_near_m,
                far_m=self.depth_far_m,
                point_radius=args.point_splat_radius,
            )

            color_atlas = np.zeros((args.height, args.width, 3), dtype=np.uint8)
            depth_atlas = np.full((args.height, args.width), np.inf, dtype=np.float32)
            for index, view in enumerate(views):
                view_color, view_depth = fill_small_depth_holes(
                    colors[index],
                    depths[index],
                    iterations=2,
                )
                x0 = int(round(view.atlas_x * args.width))
                y0 = args.height - int(
                    round((view.atlas_y + view.atlas_height) * args.height)
                )
                color_atlas[y0:y0 + tile_height, x0:x0 + tile_width] = view_color
                depth_atlas[y0:y0 + tile_height, x0:x0 + tile_width] = view_depth

            encoded_depth = encode_luma_depth(
                depth_atlas,
                near_m=self.depth_near_m,
                far_m=self.depth_far_m,
            )
            packed = np.ascontiguousarray(
                np.concatenate((color_atlas, encoded_depth), axis=1)
            )
            self.color = color_atlas
            self.depth = depth_atlas
            self.valid_depth_fraction = float(np.isfinite(depth_atlas).mean())
            self.packed_width = int(packed.shape[1])
            self.packed_height = int(packed.shape[0])
            self.packed = packed
            self.frames = self._build_static_frames(packed)
            self.rendered_frame_count = 1
            self.last_cuda_ms = cuda_ms
            self.last_render_ms = cuda_ms
        finally:
            if renderer is not None:
                renderer.close()

    def _initialize_dynamic_source(self) -> None:
        args = self.args
        if args.scene == "cow_lady":
            points, colors = load_colored_vertex_ply(resolve_cow_lady_ply(args.dataset_path or None))
            points = prepare_cow_lady_points(points, world_scale=args.dataset_scale)
            self.source_point_count = len(points)
            self.depth_near_m = args.depth_near_m or max(0.1, 0.1 * args.dataset_scale)
            self.depth_far_m = args.depth_far_m or 40.0 * args.dataset_scale
            initial_pose = DynamicCameraPose(
                (0.0, 2.0 * args.dataset_scale, 3.0 * args.dataset_scale),
                (0.0, 1.0, 0.0, 0.0),
            )
            self._cuda_renderer = CudaPointRenderer(self.source_point_count)
            self._cuda_renderer.upload(points, colors)
        elif args.scene == "eth3d_courtyard":
            self.source_point_count = ETH3D_COURTYARD_POINT_COUNT
            self.depth_near_m = args.depth_near_m or max(0.25, 0.5 * args.dataset_scale)
            self.depth_far_m = args.depth_far_m or 60.0 * args.dataset_scale
            pitch = np.deg2rad(18.0) * 0.5
            initial_pose = DynamicCameraPose(
                (0.0, 10.0 * args.dataset_scale, -24.0 * args.dataset_scale),
                (float(np.sin(pitch)), 0.0, 0.0, float(np.cos(pitch))),
            )
            self._cuda_renderer = CudaPointRenderer(self.source_point_count)
            offset = 0
            for points, colors in iter_eth3d_courtyard_chunks(
                args.dataset_path or None,
                world_scale=args.dataset_scale,
                chunk_size=1_000_000,
            ):
                self._cuda_renderer.upload(points, colors, offset=offset)
                offset += len(points)
                if offset % 5_000_000 < len(points):
                    print(
                        f"[remote-map-source] uploaded {offset:,}/{self.source_point_count:,} "
                        "ETH3D points to CUDA",
                        flush=True,
                    )
            if offset != self.source_point_count:
                raise RuntimeError(
                    f"ETH3D source contained {offset:,} points, expected {self.source_point_count:,}"
                )
        else:
            raise ValueError("dynamic view rendering requires a real point-map scene")

        self._latest_pose = initial_pose
        self.latest_pose_sequence = 0
        self.packed_width = args.width * 2
        self.packed_height = args.height
        self._render_dynamic_frame(initial_pose, sequence=0)

    @staticmethod
    def _build_static_frames(packed: np.ndarray) -> tuple[array, array]:
        frame = array("B", packed.tobytes())
        return frame, frame

    def _encode_ros_frame(self, packed: np.ndarray) -> bytes:
        ok, encoded = cv2.imencode(
            ".jpg",
            cv2.cvtColor(packed, cv2.COLOR_RGB2BGR),
            [cv2.IMWRITE_JPEG_QUALITY, self.args.jpeg_quality],
        )
        if not ok:
            raise RuntimeError("failed to encode pose-tagged RGB-D frame")
        return encoded.tobytes()

    @staticmethod
    def _read_vector(payload, key: str, length: int) -> tuple[float, ...]:
        raw = payload.get(key)
        names = ("x", "y", "z", "w")[:length]
        if isinstance(raw, dict):
            return tuple(float(raw[name]) for name in names)
        if isinstance(raw, (list, tuple)) and len(raw) == length:
            return tuple(float(value) for value in raw)
        raise ValueError(f"{key} must contain {', '.join(names)}")

    def _handle_viewer_pose(self, message: String) -> None:
        try:
            payload = json.loads((message.data or "").replace("\0", "").strip())
            if payload.get("version") != VIEWER_POSE_VERSION:
                return
            sequence = int(payload.get("sequence", 0))
            use_prediction = (
                isinstance(payload.get("predicted_position"), (dict, list, tuple))
                and isinstance(payload.get("predicted_rotation"), (dict, list, tuple))
            )
            pose = DynamicCameraPose(
                self._read_vector(
                    payload,
                    "predicted_position" if use_prediction else "position",
                    3,
                ),
                self._read_vector(
                    payload,
                    "predicted_rotation" if use_prediction else "rotation",
                    4,
                ),
            )
            if not np.isfinite(np.asarray((*pose.position, *pose.rotation))).all():
                return
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        with self._pose_lock:
            self._latest_pose = pose
            self.latest_pose_sequence = sequence
            self.last_pose_time = time.monotonic()
        self._pose_event.set()

    def _render_loop(self) -> None:
        period = 1.0 / self.args.render_update_fps
        next_render = time.monotonic()
        while not self._stop_event.is_set():
            self._pose_event.wait(timeout=0.5)
            self._pose_event.clear()
            if self._stop_event.is_set():
                return
            now = time.monotonic()
            if now < next_render and self._stop_event.wait(next_render - now):
                return
            with self._pose_lock:
                pose = self._latest_pose
                sequence = self.latest_pose_sequence
            if sequence == self.latest_render_sequence and self.last_pose_time > 0.0:
                next_render = time.monotonic() + period
                continue
            try:
                started = time.monotonic()
                self._render_dynamic_frame(pose, sequence=sequence)
                next_render = max(started + period, time.monotonic())
            except Exception as exception:
                self.get_logger().error(f"dynamic CUDA render failed: {exception}")
                self._stop_event.wait(0.5)

    def _render_dynamic_frame(self, center_pose: DynamicCameraPose, *, sequence: int) -> None:
        poses = build_dynamic_camera_poses(
            center_pose.position,
            center_pose.rotation,
            baseline_m=self.args.dynamic_view_baseline * self.args.dataset_scale,
        )
        started = time.monotonic()
        color, depth, cuda_ms = self._cuda_renderer.render(
            poses[0].position,
            poses[0].rotation,
            self.args.width,
            self.args.height,
            vertical_fov_deg=self.args.vertical_fov_deg,
            near_m=self.depth_near_m,
            far_m=self.depth_far_m,
            point_radius=self.args.point_splat_radius,
        )
        packed = pack_dynamic_rgbd(
            color,
            depth,
            poses,
            sequence=sequence,
            near_m=self.depth_near_m,
            far_m=self.depth_far_m,
            position_range_m=self.args.pose_position_range,
        )
        frame = array("B", packed.tobytes())
        encoded_frame = (
            self._encode_ros_frame(packed)
            if self.args.transport == "ros_compressed"
            else None
        )
        with self._frame_lock:
            self.frames = (frame, frame)
            self.packed = packed
            self.encoded_frame = encoded_frame
            self.color = color
            self.depth = depth
            self.valid_depth_fraction = float(np.isfinite(depth).mean())
            self.rendered_frame_count += 1
            self.latest_render_sequence = sequence
            self.last_render_ms = (time.monotonic() - started) * 1000.0
            self.last_cuda_ms = cuda_ms

    def _publish_frame(self) -> None:
        if self.args.transport == "ros_compressed":
            with self._frame_lock:
                encoded = self.encoded_frame
                render_sequence = self.latest_render_sequence
            if encoded is None:
                return
            if self.args.dynamic_view and render_sequence == self.last_published_render_sequence:
                return
            message = CompressedImage()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "remote_map_render_camera"
            message.format = f"jpeg; {self.format_version}"
            message.data = encoded
            self.image_publisher.publish(message)
            self.last_published_render_sequence = render_sequence
            self.frame_index += 1
            return

        phase = (self.frame_index // max(1, self.args.fps // 2)) % 2
        with self._frame_lock:
            frame = self.frames[phase]
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "remote_map_render_camera"
        message.height = self.packed_height
        message.width = self.packed_width
        message.encoding = "rgb8"
        message.is_bigendian = 0
        message.step = self.packed_width * 3
        message.data = frame
        self.image_publisher.publish(message)
        self.frame_index += 1

    def _publish_status(self) -> None:
        pose_age_ms = None
        if self.last_pose_time > 0.0:
            pose_age_ms = max(0.0, (time.monotonic() - self.last_pose_time) * 1000.0)
        status = String()
        status.data = json.dumps(
            {
                "state": "streaming_source",
                "renderer": self.renderer_name,
                "scene": self.args.scene,
                "transport_owner": self.args.transport,
                "format_version": self.format_version,
                "update_mode": "refresh" if self.args.dynamic_view else "static",
                "dynamic_view": self.args.dynamic_view,
                "stream_topic": (
                    ROS_STREAM_TOPIC
                    if self.args.transport == "ros_compressed"
                    else STREAM_TOPIC
                ),
                "status_topic": AGENT_STATUS_TOPIC,
                "viewer_pose_topic": self.args.viewer_pose_topic if self.args.dynamic_view else "",
                "width": self.packed_width,
                "height": self.packed_height,
                "view_width": (
                    self.args.width
                    if self.args.dynamic_view
                    else self.args.width // 3
                    if self.args.scene != "synthetic"
                    else self.args.width
                ),
                "view_height": (
                    self.args.height
                    if self.args.dynamic_view
                    else self.args.height // 3
                    if self.args.scene != "synthetic"
                    else self.args.height
                ),
                "fps": self.args.fps,
                "render_update_fps": self.args.render_update_fps if self.args.dynamic_view else 0,
                "frames_published": self.frame_index,
                "frames_rendered": self.rendered_frame_count,
                "latest_pose_sequence": self.latest_pose_sequence,
                "latest_render_sequence": self.latest_render_sequence,
                "pose_age_ms": pose_age_ms,
                "render_ms": self.last_render_ms,
                "cuda_ms": getattr(self, "last_cuda_ms", 0.0),
                "valid_depth_fraction": self.valid_depth_fraction,
                "source_point_count": self.source_point_count,
                "depth_near_m": self.depth_near_m,
                "depth_far_m": self.depth_far_m,
                "vertical_fov_deg": self.args.vertical_fov_deg if self.args.dynamic_view else VERTICAL_FOV_DEGREES,
            },
            separators=(",", ":"),
        )
        self.status_publisher.publish(status)

    def close(self) -> None:
        self._stop_event.set()
        self._pose_event.set()
        if self._render_thread is not None:
            self._render_thread.join(timeout=10.0)
            if self._render_thread.is_alive():
                self.get_logger().warning(
                    "CUDA render thread did not stop; native resources will be released by process exit"
                )
                return
            self._render_thread = None
        if self._cuda_renderer is not None:
            self._cuda_renderer.close()
            self._cuda_renderer = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish PC-rendered packed RGB-D frames for the HORUS WebRTC sender."
    )
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--transport",
        choices=("webrtc_source", "ros_compressed"),
        default="webrtc_source",
    )
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument(
        "--scene",
        choices=("synthetic", "cow_lady", "eth3d_courtyard"),
        default="synthetic",
    )
    parser.add_argument("--dataset-path", default="")
    parser.add_argument("--dataset-scale", type=float, default=1.0)
    parser.add_argument("--point-splat-radius", type=int, default=1)
    parser.add_argument("--dynamic-view", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--viewer-pose-topic", default=DEFAULT_VIEWER_POSE_TOPIC)
    parser.add_argument("--dynamic-view-baseline", type=float, default=0.75)
    parser.add_argument("--render-update-fps", type=float, default=0.0)
    parser.add_argument("--vertical-fov-deg", type=float, default=90.0)
    parser.add_argument("--pose-position-range", type=float, default=128.0)
    parser.add_argument("--depth-near-m", type=float, default=0.0)
    parser.add_argument("--depth-far-m", type=float, default=0.0)
    parser.add_argument("--ready-file", default="")
    parser.add_argument("--preview-path", default="")
    args = parser.parse_args()
    if not 320 <= args.width <= 1920 or not 180 <= args.height <= 1080:
        parser.error("view dimensions must be within 320x180 and 1920x1080")
    if args.width % 2 or args.height % 2:
        parser.error("view width and height must be even for RGB-D packing")
    if args.dynamic_view and args.width % 4:
        parser.error("dynamic view width must be divisible by four for 16-bit depth packing")
    if not args.dynamic_view and args.scene != "synthetic" and (
        args.width % 3 or args.height % 3
    ):
        parser.error("complete static atlas width and height must be divisible by three")
    if not 1 <= args.fps <= 60:
        parser.error("--fps must be between 1 and 60")
    if args.transport == "ros_compressed" and args.fps > 15:
        parser.error("ROS compressed diagnostics are capped at 15 FPS")
    if not 80 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be between 80 and 100")
    if args.dynamic_view and args.scene == "synthetic":
        parser.error("--dynamic-view requires cow_lady or eth3d_courtyard")
    if not args.viewer_pose_topic.startswith("/"):
        parser.error("--viewer-pose-topic must be an absolute ROS topic")
    if args.render_update_fps <= 0.0:
        args.render_update_fps = min(args.fps, 30 if args.scene == "eth3d_courtyard" else 60)
    if not 1.0 <= args.render_update_fps <= 60.0:
        parser.error("--render-update-fps must be between 1 and 60")
    if not 0.0 <= args.dynamic_view_baseline <= 5.0:
        parser.error("--dynamic-view-baseline must be between 0 and 5 metres")
    if not 40.0 <= args.vertical_fov_deg <= 140.0:
        parser.error("--vertical-fov-deg must be between 40 and 140")
    if not 8.0 <= args.pose_position_range <= 2048.0:
        parser.error("--pose-position-range must be between 8 and 2048 metres")
    if args.depth_near_m < 0.0 or args.depth_far_m < 0.0:
        parser.error("depth overrides cannot be negative")
    if args.depth_far_m > 0.0 and args.depth_near_m > 0.0 and args.depth_far_m <= args.depth_near_m:
        parser.error("--depth-far-m must be greater than --depth-near-m")
    if not 0.1 <= args.dataset_scale <= 10.0:
        parser.error("--dataset-scale must be between 0.1 and 10")
    if not 0 <= args.point_splat_radius <= 4:
        parser.error("--point-splat-radius must be between 0 and 4")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init(args=None)
    node = RemoteMapRenderSource(args)
    ready_path = Path(args.ready_file).expanduser() if args.ready_file else None
    try:
        if args.preview_path:
            preview_path = Path(args.preview_path).expanduser()
            preview_path.parent.mkdir(parents=True, exist_ok=True)
            with node._frame_lock:
                preview = node.color.copy()
            cv2.imwrite(str(preview_path), cv2.cvtColor(preview, cv2.COLOR_RGB2BGR))
        if ready_path is not None:
            ready_path.parent.mkdir(parents=True, exist_ok=True)
            ready_path.write_text(
                json.dumps(
                    {
                        "state": "ready",
                        "renderer": node.renderer_name,
                        "scene": args.scene,
                        "encoder": "horus_ros2",
                        "transport": args.transport,
                        "transport_owner": args.transport,
                        "format_version": node.format_version,
                        "update_mode": "refresh" if args.dynamic_view else "static",
                        "dynamic_view": args.dynamic_view,
                        "stream_topic": (
                            ROS_STREAM_TOPIC
                            if args.transport == "ros_compressed"
                            else STREAM_TOPIC
                        ),
                        "viewer_pose_topic": args.viewer_pose_topic if args.dynamic_view else "",
                        "width": node.packed_width,
                        "height": node.packed_height,
                        "fps": args.fps,
                        "render_update_fps": args.render_update_fps,
                        "valid_depth_fraction": node.valid_depth_fraction,
                    },
                    indent=2,
                ),
                encoding="utf-8",
            )
        print(
            "[remote-map-source] READY "
            f"topic={ROS_STREAM_TOPIC if args.transport == 'ros_compressed' else STREAM_TOPIC} "
            f"packed={node.packed_width}x{node.packed_height} "
            f"fps={args.fps} dynamic={args.dynamic_view} transport={args.transport}",
            flush=True,
        )
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if ready_path is not None:
            ready_path.unlink(missing_ok=True)
        print("[remote-map-source] stopped", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
