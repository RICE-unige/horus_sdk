#!/usr/bin/env python3
"""Render a dense map on the PC for synchronized HORUS XR composition."""

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
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from horus.remote_rendering import (
    ADVANCED_REMOTE_SCENE_IDS,
    COW_LADY_DEPTH_FAR_BASE,
    DEFAULT_VIEWER_POSE_TOPIC,
    ETH3D_COURTYARD_DEPTH_FAR_BASE,
    ETH3D_COURTYARD_POINT_COUNT,
    ETH3D_REMOTE_SCENE_IDS,
    FRAME_MARKER_WIDTH,
    REMOTE_FRAME_DATA_TOPIC,
    REMOTE_FRAME_FORMAT_VERSION,
    ROS_DEBUG_FRAME_FORMAT_VERSION,
    ROS_DEBUG_FRAME_TOPIC,
    VIEWER_POSE_VERSION,
    CudaMeshRenderer,
    CudaPointRenderer,
    GaussianSplatRenderer,
    GpuH264Encoder,
    GAUSSIAN_SCENES,
    MESH_SCENES,
    NvdiffrastMeshRenderer,
    DynamicCameraPose,
    downsample_depth_for_surfels,
    embed_frame_marker,
    expand_projection,
    pack_stereo_remote_frame,
    pack_ros_debug_frame,
    projection_from_vertical_fov,
    load_gaussian_splat_ply,
    load_textured_mesh_scene,
    resolve_gaussian_splat,
    V3_DEFAULT_ROS_DIAGNOSTIC_TOPIC,
    V3_DEFAULT_STATUS_TOPIC,
    V3_DEFAULT_VIEW_STATE_TOPIC,
    V3_PROTOCOL_VERSION,
    V3_ROS_DIAGNOSTIC_FORMAT_VERSION,
    V3_WEBRTC_FORMAT_VERSION,
    ViewState,
    render_stereo_diagnostic,
    render_stereo_frame,
    render_stereo_frame_cuda,
)
from horus.remote_rendering.v3_protocol import ViewStreamSequenceGate
from horus.remote_rendering.real_scene import (
    iter_eth3d_courtyard_chunks,
    iter_eth3d_remote_scene_chunks,
    iter_eth3d_remote_scene_lod_chunks,
    load_colored_vertex_ply,
    prepare_cow_lady_points,
    prepare_eth3d_remote_scene,
    prepare_eth3d_remote_scene_lod,
    resolve_cow_lady_ply,
)
from horus.remote_rendering.synthetic_scene import build_industrial_site


STREAM_TOPIC = "/horus/remote_render/map_portal"
AGENT_STATUS_TOPIC = "/horus/remote_render/agent_status"


def _shade_mesh_faces(
    vertices: np.ndarray,
    faces: np.ndarray,
    colors: np.ndarray,
) -> np.ndarray:
    """Apply the stable directional shading used by the original static renderer."""
    triangles = vertices[faces]
    normals = np.cross(
        triangles[:, 1] - triangles[:, 0],
        triangles[:, 2] - triangles[:, 0],
    )
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(
        normals,
        np.maximum(lengths, 1e-8),
        out=np.zeros_like(normals),
    )
    light = np.asarray((-0.38, 0.82, -0.43), dtype=np.float32)
    light /= np.linalg.norm(light)
    intensity = 0.42 + 0.58 * np.abs(normals @ light)
    return np.ascontiguousarray(
        np.clip(colors.astype(np.float32) * intensity[:, None], 0.0, 255.0),
        dtype=np.uint8,
    )


def _preview_pose_for_bounds(
    bounds_min: np.ndarray,
    bounds_max: np.ndarray,
) -> DynamicCameraPose:
    """Build an exterior startup view without changing live alignment."""
    minimum = np.asarray(bounds_min, dtype=np.float32)
    maximum = np.asarray(bounds_max, dtype=np.float32)
    extent = np.maximum(maximum - minimum, 0.01)
    center = (minimum + maximum) * 0.5
    eye_height = float(
        minimum[1] + max(1.5, min(2.5, float(extent[1]) * 0.35))
    )
    target = np.asarray((center[0], eye_height, center[2]), dtype=np.float32)
    position = target.copy()
    if extent[0] >= extent[2]:
        position[0] = center[0] - float(extent[0]) * 0.3
    else:
        position[2] = center[2] - float(extent[2]) * 0.3
    direction = target - position
    horizontal = max(1e-6, float(np.hypot(direction[0], direction[2])))
    yaw = float(np.arctan2(direction[0], direction[2]))
    pitch = float(np.arctan2(-direction[1], horizontal))
    half_yaw = yaw * 0.5
    half_pitch = pitch * 0.5
    rotation = (
        float(np.cos(half_yaw) * np.sin(half_pitch)),
        float(np.sin(half_yaw) * np.cos(half_pitch)),
        float(-np.sin(half_yaw) * np.sin(half_pitch)),
        float(np.cos(half_yaw) * np.cos(half_pitch)),
    )
    return DynamicCameraPose(
        tuple(float(value) for value in position),
        rotation,
    )


class RemoteMapRenderSource(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("horus_remote_map_render_source")
        self.args = args
        self.frame_index = 0
        self.rendered_frame_count = 0
        self.last_render_ms = 0.0
        self.last_cuda_ms = 0.0
        self.last_encode_ms = 0.0
        self.last_pipeline_ms = 0.0
        self.last_publish_ms = 0.0
        self._pipeline_window_count = 0
        self._pipeline_window_total_ms = 0.0
        self._pipeline_window_max_ms = 0.0
        self.valid_depth_fraction = 0.0
        self.latest_pose_sequence = 0
        self.latest_render_sequence = -1
        self._last_submitted_sequence = -1
        self.last_published_render_sequence = -1
        self.last_pose_time = 0.0
        self.current_render_fov_deg = args.minimum_vertical_fov_deg
        self.last_ros_debug_payload_bytes = 0
        self._status_publish_count = 0
        self._last_ros_debug_subscription_count = -1
        self._rate_window_time = time.monotonic()
        self._rate_window_views = 0
        self._rate_window_rendered = 0
        self._rate_window_published = 0
        self._rate_window_encoded = 0

        self._frame_lock = threading.Lock()
        self._pose_lock = threading.Lock()
        self._pose_event = threading.Event()
        self._stop_event = threading.Event()
        self._render_thread: threading.Thread | None = None
        self._encoded_publish_thread: threading.Thread | None = None
        self._cuda_renderer: (
            CudaPointRenderer
            | CudaMeshRenderer
            | NvdiffrastMeshRenderer
            | GaussianSplatRenderer
            | None
        ) = None
        self._renderer_name = "uninitialized"
        self._color_frame: array | None = None
        self._frame_data: array | None = None
        self._ros_debug_frame: array | None = None
        self._v3_ros_frame: array | None = None
        self._v3_metadata_json = ""
        self._v3_video_height = 0
        self._encoded_frame: bytes | None = None
        self._h264_encoder: GpuH264Encoder | None = None
        self.webrtc_image_type = "raw"
        self._viewer_pose_received = False
        self._latest_view_state: ViewState | None = None
        self._view_stream_gate = ViewStreamSequenceGate()
        self._latest_predicted_display_time_ns = 0
        self._latest_workspace_revision = 0
        self._v3_view_received_count = 0
        self._v3_view_rejected_count = 0
        self._v3_last_view_reject_reason = "none"

        self._initialize_source()
        if args.protocol == "v3" and args.transport == "webrtc":
            try:
                self._h264_encoder = GpuH264Encoder(
                    args.width,
                    args.height,
                    args.visibility_mask_height,
                    fps=args.fps,
                    bitrate_kbps=args.bitrate_kbps,
                )
                self._h264_encoder.start_async()
                self.webrtc_image_type = "h264"
                self.get_logger().info(
                    "HORUS remote rendering is using direct CUDA-to-NVENC H.264"
                )
            except RuntimeError as exception:
                self.get_logger().warning(
                    "Direct GPU H.264 is unavailable; retaining the raw WebRTC "
                    f"fallback: {exception}"
                )

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=32 if self.webrtc_image_type == "h264" else 2,
            reliability=(ReliabilityPolicy.RELIABLE
                         if self.webrtc_image_type == "h264"
                         else ReliabilityPolicy.BEST_EFFORT),
            durability=DurabilityPolicy.VOLATILE,
        )
        metadata_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        ros_debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
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
        self.image_publisher = None
        self.frame_data_publisher = None
        self.ros_debug_publisher = None
        if args.transport == "webrtc":
            image_message_type = (
                CompressedImage if self.webrtc_image_type == "h264" else Image
            )
            self.image_publisher = self.create_publisher(
                image_message_type,
                STREAM_TOPIC,
                image_qos,
            )
            if args.protocol != "v3":
                self.frame_data_publisher = self.create_publisher(
                    CompressedImage,
                    REMOTE_FRAME_DATA_TOPIC,
                    metadata_qos,
                )
        else:
            self.ros_debug_publisher = self.create_publisher(
                CompressedImage,
                (
                    V3_DEFAULT_ROS_DIAGNOSTIC_TOPIC
                    if args.protocol == "v3"
                    else ROS_DEBUG_FRAME_TOPIC
                ),
                ros_debug_qos,
            )
        self.status_publisher = self.create_publisher(
            String,
            V3_DEFAULT_STATUS_TOPIC if args.protocol == "v3" else AGENT_STATUS_TOPIC,
            status_qos,
        )
        self.pose_subscription = self.create_subscription(
            String,
            args.viewer_pose_topic,
            self._handle_viewer_pose,
            pose_qos,
        )
        self._uses_async_encoder = self._h264_encoder is not None and callable(
            getattr(self._cuda_renderer, "render_video_views_cuda", None)
        )
        self.keyframe_subscription = None
        if self._h264_encoder is not None:
            self.keyframe_subscription = self.create_subscription(
                String, STREAM_TOPIC + "/request_keyframe",
                self._handle_keyframe_request, 1,
            )
        if self._uses_async_encoder:
            self._encoded_publish_thread = threading.Thread(
                target=self._publish_encoded_loop,
                name="horus-remote-map-video-publisher",
                daemon=True,
            )
            self._encoded_publish_thread.start()
        self._render_thread = threading.Thread(
            target=self._render_loop,
            name="horus-remote-map-cuda",
            daemon=True,
        )
        self._render_thread.start()
        # Frames are published by the render thread as soon as they exist. A
        # timer here would quantize publication to its own period, adding up to
        # a full frame of latency to every frame and re-introducing jitter that
        # the receiver then has to absorb.
        self.create_timer(1.0, self._publish_status)

    @property
    def renderer_name(self) -> str:
        return self._renderer_name

    def _handle_keyframe_request(self, message: String) -> None:
        if message.data == "idr" and self._h264_encoder is not None:
            self._h264_encoder.request_keyframe()

    def _create_mesh_renderer(
        self,
        vertices: np.ndarray,
        faces: np.ndarray,
        colors: np.ndarray,
        *,
        uvs: np.ndarray | None = None,
        texture_atlas: np.ndarray | None = None,
        texture_rects: np.ndarray | None = None,
    ) -> tuple[NvdiffrastMeshRenderer | CudaMeshRenderer, str]:
        """Prefer the hardware rasterizer and retain the CUDA fallback."""
        try:
            renderer = NvdiffrastMeshRenderer(
                vertices,
                faces,
                colors,
                uvs=uvs,
                texture_atlas=texture_atlas,
                texture_rects=texture_rects,
            )
            return renderer, "nvdiffrast_hardware_triangles"
        except RuntimeError as exc:
            self.get_logger().warning(
                f"nvdiffrast unavailable; using CUDA mesh fallback: {exc}"
            )
            renderer = CudaMeshRenderer(
                vertices,
                faces,
                colors,
                uvs=uvs,
                texture_atlas=texture_atlas,
                texture_rects=texture_rects,
            )
            return renderer, "cuda_pose_adaptive_triangles"

    def _initialize_source(self) -> None:
        args = self.args
        if args.scene in MESH_SCENES:
            mesh = load_textured_mesh_scene(
                args.scene,
                args.dataset_path or None,
                world_scale=args.dataset_scale,
            )
            self.source_point_count = len(mesh.faces)
            bounds_min = mesh.vertices.min(axis=0)
            bounds_max = mesh.vertices.max(axis=0)
            diagonal = float(np.linalg.norm(bounds_max - bounds_min))
            self.depth_near_m = args.depth_near_m or max(
                0.03,
                diagonal * 0.001,
            )
            self.depth_far_m = args.depth_far_m or max(10.0, diagonal * 2.0)
            preview_pose = _preview_pose_for_bounds(bounds_min, bounds_max)
            self._cuda_renderer, backend = self._create_mesh_renderer(
                mesh.vertices,
                mesh.faces,
                mesh.face_tints,
                uvs=mesh.uvs,
                texture_atlas=mesh.texture_atlas,
                texture_rects=mesh.face_texture_rects,
            )
            self._renderer_name = f"{backend}_textured"
        elif args.scene in GAUSSIAN_SCENES:
            scene_spec = GAUSSIAN_SCENES[args.scene]
            source_path = resolve_gaussian_splat(
                args.scene,
                args.dataset_path or None,
            )
            scene = load_gaussian_splat_ply(
                source_path,
                world_scale=args.dataset_scale,
                canonicalize_y_down=scene_spec.canonicalize_y_down,
            )
            self.source_point_count = scene.gaussian_count
            bounds_min = np.asarray(scene.world_bounds_min, dtype=np.float32)
            bounds_max = np.asarray(scene.world_bounds_max, dtype=np.float32)
            diagonal = float(np.linalg.norm(bounds_max - bounds_min))
            self.depth_near_m = args.depth_near_m or max(
                0.02,
                diagonal * 0.001,
            )
            self.depth_far_m = args.depth_far_m or max(10.0, diagonal * 2.0)
            preview_pose = _preview_pose_for_bounds(bounds_min, bounds_max)
            self._cuda_renderer = GaussianSplatRenderer(scene)
            self._renderer_name = (
                f"gsplat_cuda_anisotropic_sh{scene.sh_degree}_full"
            )
        elif args.scene in ("recorded_mesh", "recorded_points"):
            from real_mapping_data import arrays
            if not args.dataset_path:
                raise ValueError("Recorded scenes require --dataset-path; no generated fallback")
            source_path = Path(args.dataset_path).expanduser()
            source, colors, faces = arrays(source_path, mesh=args.scene == "recorded_mesh")
            # ROS FLU to Unity: (x,y,z) -> (-y,z,x). Do not recenter separate
            # layers independently; all retain their original common map origin.
            points = np.ascontiguousarray(source[:, (1, 2, 0)] * (-1, 1, 1) * args.dataset_scale, dtype=np.float32)
            bounds_min, bounds_max = points.min(axis=0), points.max(axis=0)
            self.depth_near_m = args.depth_near_m or 0.05
            self.depth_far_m = args.depth_far_m or max(10., float(np.linalg.norm(bounds_max - bounds_min)) * 2)
            preview_pose = _preview_pose_for_bounds(bounds_min, bounds_max)
            if faces is not None:
                self.source_point_count = len(faces)
                self._cuda_renderer = NvdiffrastMeshRenderer(
                    points, np.ascontiguousarray(faces[:, (0, 2, 1)], dtype=np.int32),
                    np.full((len(faces), 3), 255, dtype=np.uint8), vertex_colors=colors)
                self._renderer_name = "nvdiffrast_recorded_mesh"
            else:
                self.source_point_count = len(points)
                self._cuda_renderer = CudaPointRenderer(len(points))
                self._cuda_renderer.upload(points, colors)
                self._renderer_name = "cuda_recorded_points"
            print(f"[remote-map-source] {args.scene}: {source_path}; all {self.source_point_count:,} primitives", flush=True)
        elif args.scene in ("cow_lady", "cow_lady_points"):
            source_path = resolve_cow_lady_ply(args.dataset_path or None)
            surface_path = source_path.with_name("cow_and_lady_surface.npz")
            self.depth_near_m = args.depth_near_m or max(0.1, 0.1 * args.dataset_scale)
            self.depth_far_m = (
                args.depth_far_m or COW_LADY_DEPTH_FAR_BASE * args.dataset_scale
            )
            pitch = np.deg2rad(22.0) * 0.5
            preview_pose = DynamicCameraPose(
                (0.0, 5.0 * args.dataset_scale, -8.0 * args.dataset_scale),
                (float(np.sin(pitch)), 0.0, 0.0, float(np.cos(pitch))),
            )
            if args.scene == "cow_lady" and surface_path.is_file():
                with np.load(surface_path, allow_pickle=False) as surface:
                    vertices = prepare_cow_lady_points(
                        surface["vertices"],
                        world_scale=args.dataset_scale,
                    )
                    faces = np.ascontiguousarray(surface["faces"], dtype=np.uint32)
                    face_colors = np.ascontiguousarray(
                        surface["face_colors"],
                        dtype=np.uint8,
                    )
                self.source_point_count = len(faces)
                self._cuda_renderer, self._renderer_name = (
                    self._create_mesh_renderer(
                        vertices,
                        faces,
                        face_colors,
                    )
                )
                print(
                    f"[remote-map-source] using cached Cow and Lady surface "
                    f"{surface_path}",
                    flush=True,
                )
            else:
                points, colors = load_colored_vertex_ply(source_path)
                points = prepare_cow_lady_points(
                    points,
                    world_scale=args.dataset_scale,
                )
                self.source_point_count = len(points)
                self._cuda_renderer = CudaPointRenderer(self.source_point_count)
                self._cuda_renderer.upload(points, colors)
                self._renderer_name = "cuda_pose_adaptive_points"
                print(f"[remote-map-source] loaded all {len(points):,} source points "
                      f"from {source_path}; no surface-cache substitution", flush=True)
        elif args.scene == "eth3d_courtyard":
            self.source_point_count = ETH3D_COURTYARD_POINT_COUNT
            self.depth_near_m = args.depth_near_m or max(0.25, 0.5 * args.dataset_scale)
            self.depth_far_m = (
                args.depth_far_m or ETH3D_COURTYARD_DEPTH_FAR_BASE * args.dataset_scale
            )
            pitch = np.deg2rad(18.0) * 0.5
            preview_pose = DynamicCameraPose(
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
                        f"[remote-map-source] uploaded {offset:,}/"
                        f"{self.source_point_count:,} ETH3D points to CUDA",
                        flush=True,
                    )
            if offset != self.source_point_count:
                raise RuntimeError(
                    f"ETH3D source contained {offset:,} points, "
                    f"expected {self.source_point_count:,}"
                )
            self._renderer_name = "cuda_pose_adaptive_points"
        elif args.scene in ETH3D_REMOTE_SCENE_IDS:
            scene = prepare_eth3d_remote_scene(
                args.scene,
                args.dataset_path or None,
                dataset_scale=args.dataset_scale,
            )
            use_full_source = args.source_voxel_size == 0.0
            if use_full_source:
                self.source_point_count = scene.point_count
                point_chunks = iter_eth3d_remote_scene_chunks(scene)
            else:
                lod_path, lod_point_count = prepare_eth3d_remote_scene_lod(
                    scene,
                    voxel_size_m=args.source_voxel_size,
                )
                self.source_point_count = lod_point_count
                point_chunks = iter_eth3d_remote_scene_lod_chunks(
                    lod_path,
                    chunk_size=1_000_000,
                )
            self.depth_near_m = args.depth_near_m or scene.depth_near_m
            self.depth_far_m = args.depth_far_m or scene.depth_far_m
            pitch = np.deg2rad(scene.initial_pitch_degrees) * 0.5
            preview_pose = DynamicCameraPose(
                scene.initial_position,
                (float(np.sin(pitch)), 0.0, 0.0, float(np.cos(pitch))),
            )
            self._cuda_renderer = CudaPointRenderer(self.source_point_count)
            offset = 0
            for points, colors in point_chunks:
                self._cuda_renderer.upload(points, colors, offset=offset)
                offset += len(points)
                if offset % 5_000_000 < len(points):
                    print(
                        f"[remote-map-source] uploaded {offset:,}/"
                        f"{self.source_point_count:,} {args.scene} points to CUDA",
                        flush=True,
                    )
            if offset != self.source_point_count:
                raise RuntimeError(
                    f"ETH3D {args.scene} contained {offset:,} finite points, "
                    f"expected {self.source_point_count:,}"
                )
            self._renderer_name = (
                "cuda_pose_adaptive_points_full_source"
                if use_full_source
                else f"cuda_pose_adaptive_points_rust_lod_{args.source_voxel_size:.4f}m"
            )
        else:
            vertices, faces, face_colors = build_industrial_site().arrays()
            vertices = np.ascontiguousarray(
                vertices * args.dataset_scale,
                dtype=np.float32,
            )
            faces = np.ascontiguousarray(faces, dtype=np.uint32)
            face_colors = _shade_mesh_faces(vertices, faces, face_colors)
            self.source_point_count = len(faces)
            self.depth_near_m = args.depth_near_m or max(0.1, 0.2 * args.dataset_scale)
            self.depth_far_m = args.depth_far_m or 80.0 * args.dataset_scale
            pitch = np.deg2rad(38.0) * 0.5
            preview_pose = DynamicCameraPose(
                (0.0, 15.0 * args.dataset_scale, -19.0 * args.dataset_scale),
                (float(np.sin(pitch)), 0.0, 0.0, float(np.cos(pitch))),
            )
            self._cuda_renderer, self._renderer_name = (
                self._create_mesh_renderer(vertices, faces, face_colors)
            )

        # Dataset-specific camera poses are only for the startup render used to
        # validate the renderer. Live frames use Quest-local eye poses directly.
        # Reusing this pitched preview as a scene anchor tilts the map floor and
        # makes source geometry move in a camera-relative coordinate system.
        preview_eyes = self._build_eye_poses(preview_pose, 0.064)
        self._latest_poses = preview_eyes
        self._latest_workspace_pose = preview_pose
        self._latest_viewer_fov_deg = args.minimum_vertical_fov_deg
        initial_projection = projection_from_vertical_fov(
            args.minimum_vertical_fov_deg + args.guard_band_degrees * 2.0,
            args.width / args.height,
        )
        self._latest_projections = (initial_projection, initial_projection)
        self._render_frame(
            preview_eyes,
            sequence=0,
            render_projections=self._latest_projections,
        )

    @staticmethod
    def _build_eye_poses(
        center_pose: DynamicCameraPose,
        ipd_m: float,
    ) -> tuple[DynamicCameraPose, DynamicCameraPose]:
        from horus.remote_rendering import quaternion_to_matrix

        rotation = np.asarray(center_pose.rotation, dtype=np.float32)
        basis = quaternion_to_matrix(rotation)
        center = np.asarray(center_pose.position, dtype=np.float32)
        half_offset = basis @ np.asarray((ipd_m * 0.5, 0.0, 0.0), dtype=np.float32)
        return (
            DynamicCameraPose(
                tuple(float(value) for value in center - half_offset),
                center_pose.rotation,
            ),
            DynamicCameraPose(
                tuple(float(value) for value in center + half_offset),
                center_pose.rotation,
            ),
        )

    @staticmethod
    def _read_vector(payload, key: str, length: int) -> tuple[float, ...]:
        raw = payload.get(key)
        names = ("x", "y", "z", "w")[:length]
        if isinstance(raw, dict):
            return tuple(float(raw[name]) for name in names)
        if isinstance(raw, (list, tuple)) and len(raw) == length:
            return tuple(float(value) for value in raw)
        raise ValueError(f"{key} must contain {', '.join(names)}")

    def _expanded_projection(
        self,
        projection: tuple[float, ...],
    ) -> tuple[float, float, float, float]:
        return expand_projection(
            projection,
            guard_band_degrees=self.args.guard_band_degrees,
            minimum_vertical_fov_deg=self.args.minimum_vertical_fov_deg,
        )

    def _handle_viewer_pose(self, message: String) -> None:
        if self.args.protocol == "v3":
            self._handle_v3_view_state(message)
            return
        try:
            payload = json.loads((message.data or "").replace("\0", "").strip())
            if payload.get("version") != VIEWER_POSE_VERSION:
                return
            sequence = int(payload.get("sequence", 0))
            use_prediction = (
                isinstance(payload.get("predicted_position"), (dict, list, tuple))
                and isinstance(payload.get("predicted_rotation"), (dict, list, tuple))
            )
            viewer_pose = DynamicCameraPose(
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
            viewer_fov = float(
                payload.get("vertical_fov_deg", self.args.minimum_vertical_fov_deg)
            )
            left_pose = DynamicCameraPose(
                self._read_vector(
                    payload,
                    "predicted_left_position"
                    if use_prediction
                    else "left_position",
                    3,
                ),
                self._read_vector(
                    payload,
                    "predicted_left_rotation"
                    if use_prediction
                    else "left_rotation",
                    4,
                ),
            )
            right_pose = DynamicCameraPose(
                self._read_vector(
                    payload,
                    "predicted_right_position"
                    if use_prediction
                    else "right_position",
                    3,
                ),
                self._read_vector(
                    payload,
                    "predicted_right_rotation"
                    if use_prediction
                    else "right_rotation",
                    4,
                ),
            )
            left_projection = self._expanded_projection(
                self._read_vector(payload, "left_projection", 4)
            )
            right_projection = self._expanded_projection(
                self._read_vector(payload, "right_projection", 4)
            )
            if not np.isfinite(
                np.asarray(
                    (
                        *viewer_pose.position,
                        *viewer_pose.rotation,
                        *left_pose.position,
                        *left_pose.rotation,
                        *right_pose.position,
                        *right_pose.rotation,
                        *left_projection,
                        *right_projection,
                        viewer_fov,
                    )
                )
            ).all():
                return
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return

        with self._pose_lock:
            self._latest_poses = (left_pose, right_pose)
            self._latest_workspace_pose = viewer_pose
            self._latest_viewer_fov_deg = float(np.clip(viewer_fov, 40.0, 140.0))
            self._latest_projections = (left_projection, right_projection)
            self.latest_pose_sequence = sequence
            self.last_pose_time = time.monotonic()
            self._viewer_pose_received = True
        self._pose_event.set()

    def _handle_v3_view_state(self, message: String) -> None:
        self._v3_view_received_count += 1
        try:
            state = ViewState.from_json((message.data or "").replace("\0", "").strip())
            if self.args.session_id and state.session_id != self.args.session_id:
                self._reject_v3_view("session_id")
                return
            left_pose = DynamicCameraPose(
                (
                    state.left_eye.position.x,
                    state.left_eye.position.y,
                    state.left_eye.position.z,
                ),
                (
                    state.left_eye.rotation.x,
                    state.left_eye.rotation.y,
                    state.left_eye.rotation.z,
                    state.left_eye.rotation.w,
                ),
            )
            right_pose = DynamicCameraPose(
                (
                    state.right_eye.position.x,
                    state.right_eye.position.y,
                    state.right_eye.position.z,
                ),
                (
                    state.right_eye.rotation.x,
                    state.right_eye.rotation.y,
                    state.right_eye.rotation.z,
                    state.right_eye.rotation.w,
                ),
            )
            left_projection = state.left_eye.fov.renderer_projection()
            right_projection = state.right_eye.fov.renderer_projection()
            left_vertical_fov = np.degrees(
                np.arctan(state.left_eye.fov.up)
                - np.arctan(state.left_eye.fov.down)
            )
            right_vertical_fov = np.degrees(
                np.arctan(state.right_eye.fov.up)
                - np.arctan(state.right_eye.fov.down)
            )
            center_position = tuple(
                float(value)
                for value in (
                    np.asarray(left_pose.position, dtype=np.float32)
                    + np.asarray(right_pose.position, dtype=np.float32)
                )
                * 0.5
            )
            left_rotation = np.asarray(left_pose.rotation, dtype=np.float32)
            right_rotation = np.asarray(right_pose.rotation, dtype=np.float32)
            if float(np.dot(left_rotation, right_rotation)) < 0.0:
                right_rotation = -right_rotation
            center_rotation = left_rotation + right_rotation
            center_rotation /= max(float(np.linalg.norm(center_rotation)), 1e-8)
            center_pose = DynamicCameraPose(
                center_position,
                tuple(float(value) for value in center_rotation),
            )
        except (TypeError, ValueError, json.JSONDecodeError) as exception:
            self._reject_v3_view(f"contract:{type(exception).__name__}")
            return

        with self._pose_lock:
            reject_reason = self._view_stream_gate.accept(state, time.monotonic())
            if reject_reason is not None:
                self._reject_v3_view(reject_reason)
                return
            self._latest_poses = (left_pose, right_pose)
            self._latest_workspace_pose = center_pose
            self._latest_viewer_fov_deg = float(
                np.clip(max(left_vertical_fov, right_vertical_fov), 40.0, 140.0)
            )
            self._latest_projections = (left_projection, right_projection)
            self._latest_view_state = state
            self._latest_predicted_display_time_ns = state.predicted_display_time_ns
            self._latest_workspace_revision = state.workspace.revision
            self.latest_pose_sequence = state.view_sequence
            self.last_pose_time = time.monotonic()
            self._viewer_pose_received = True
        self._pose_event.set()

    def _reject_v3_view(self, reason: str) -> None:
        self._v3_view_rejected_count += 1
        self._v3_last_view_reject_reason = reason

    def _render_loop(self) -> None:
        period = 1.0 / self.args.render_update_fps
        next_render = time.monotonic()
        while not self._stop_event.is_set():
            with self._pose_lock:
                pending_sequence = self._pending_render_sequence()
            if pending_sequence == self._last_submitted_sequence:
                # Clear before rechecking. Clearing after wait can erase a pose
                # notification that races the timeout and stall the renderer
                # until a later sample arrives.
                self._pose_event.clear()
                with self._pose_lock:
                    pending_sequence = self._pending_render_sequence()
                if pending_sequence != self._last_submitted_sequence:
                    continue
                self._pose_event.wait(timeout=min(period, 0.05))
                continue

            now = time.monotonic()
            if now < next_render and self._stop_event.wait(next_render - now):
                return
            with self._pose_lock:
                poses = self._latest_poses
                workspace_pose = self._latest_workspace_pose
                viewer_fov = self._latest_viewer_fov_deg
                projections = self._latest_projections
                sequence = self._pending_render_sequence()
                view_state = self._latest_view_state
                predicted_display_time_ns = self._latest_predicted_display_time_ns
                workspace_revision = self._latest_workspace_revision
            if sequence == self._last_submitted_sequence:
                continue
            try:
                started = time.monotonic()
                self._render_frame(
                    poses,
                    sequence=sequence,
                    workspace_pose=workspace_pose,
                    viewer_fov_deg=viewer_fov,
                    render_projections=projections,
                    view_state=view_state,
                    predicted_display_time_ns=predicted_display_time_ns,
                    workspace_revision=workspace_revision,
                )
                self._last_submitted_sequence = sequence
                publish_started = time.monotonic()
                self._publish_frame()
                completed = time.monotonic()
                self.last_publish_ms = (completed - publish_started) * 1000.0
                self.last_pipeline_ms = (completed - started) * 1000.0
                self._pipeline_window_count += 1
                self._pipeline_window_total_ms += self.last_pipeline_ms
                self._pipeline_window_max_ms = max(
                    self._pipeline_window_max_ms,
                    self.last_pipeline_ms,
                )
                # Keep a stable server cadence and sample the latest available
                # pose at each deadline. Anchoring to the previous deadline
                # avoids notification coalescing and render-time jitter from
                # progressively lowering the output rate.
                next_render = max(next_render + period, completed)
            except Exception as exception:
                self.get_logger().error(f"pose-adaptive CUDA render failed: {exception}")
                self._stop_event.wait(0.5)

    def _pending_render_sequence(self) -> int:
        # Frame IDs must stay monotonic even when a new viewer starts at pose 1.
        return (self._view_stream_gate.revision if self.args.protocol == "v3"
                else self.latest_pose_sequence)

    def _render_frame(
        self,
        poses: tuple[DynamicCameraPose, DynamicCameraPose],
        *,
        sequence: int,
        workspace_pose: DynamicCameraPose | None = None,
        viewer_fov_deg: float | None = None,
        render_projections: tuple[
            tuple[float, float, float, float],
            tuple[float, float, float, float],
        ] | None = None,
        view_state: ViewState | None = None,
        predicted_display_time_ns: int = 0,
        workspace_revision: int = 0,
    ) -> None:
        if self.args.protocol == "v3":
            self._render_v3_stereo_frame(
                poses,
                sequence=sequence,
                render_projections=render_projections,
                view_state=view_state,
                predicted_display_time_ns=predicted_display_time_ns,
                workspace_revision=workspace_revision,
            )
            return
        started = time.monotonic()
        render_fov = float(
            np.clip(
                max(
                    self.args.minimum_vertical_fov_deg,
                    (viewer_fov_deg or self.args.minimum_vertical_fov_deg)
                    + self.args.guard_band_degrees * 2.0,
                ),
                40.0,
                140.0,
            )
        )
        left_rotation = np.asarray(poses[0].rotation, dtype=np.float32)
        right_rotation = np.asarray(poses[1].rotation, dtype=np.float32)
        if float(np.dot(left_rotation, right_rotation)) < 0.0:
            right_rotation = -right_rotation
        center_rotation = left_rotation + right_rotation
        center_rotation /= max(float(np.linalg.norm(center_rotation)), 1e-8)
        center_pose = DynamicCameraPose(
            tuple(
                float(value)
                for value in (
                    np.asarray(poses[0].position, dtype=np.float32)
                    + np.asarray(poses[1].position, dtype=np.float32)
                )
                * 0.5
            ),
            tuple(float(value) for value in center_rotation),
        )
        if render_projections:
            center_projection = tuple(
                float(value)
                for value in (
                    np.asarray(render_projections[0], dtype=np.float32)
                    + np.asarray(render_projections[1], dtype=np.float32)
                )
                * 0.5
            )
        else:
            center_projection = projection_from_vertical_fov(
                render_fov,
                self.args.width / self.args.height,
            )

        # Render one predicted center-eye view. The Quest reprojects its
        # synchronized metric depth independently into both display eyes,
        # which restores stereo while halving the expensive server render.
        # The small source-to-display eye baseline is covered by the guard
        # band and independent surfels.
        colors, depths, cuda_ms = self._cuda_renderer.render_views(
            [center_pose.position],
            [center_pose.rotation],
            self.args.width,
            self.args.height,
            vertical_fov_deg=render_fov,
            near_m=self.depth_near_m,
            far_m=self.depth_far_m,
            point_radius=self.args.point_splat_radius,
            projections=(center_projection,),
        )
        center_depth = downsample_depth_for_surfels(
            depths[0],
            width=self.args.depth_width,
            height=self.args.depth_height,
        )
        reduced_depth = np.stack((center_depth, center_depth), axis=0)
        # One view on the wire. Both eyes reproject the same server render, so
        # a side-by-side atlas would carry the identical image twice and double
        # the cost of every frame for nothing.
        color = np.ascontiguousarray(colors[0], dtype=np.uint8)
        if self.args.transport == "webrtc":
            embed_frame_marker(color, sequence)
        projections = (center_projection, center_projection)
        frame_data = pack_stereo_remote_frame(
            reduced_depth,
            sequence=sequence,
            color_width=self.args.width,
            color_height=self.args.height,
            near_m=self.depth_near_m,
            far_m=self.depth_far_m,
            projections=projections,
            # Depth is camera-local. Carry the exact Quest-local camera pose
            # that requested this frame so the headset can place every
            # reconstructed point directly into its stable workspace. The
            # server's arbitrary scene anchor must never enter Quest-side
            # placement.
            positions=(
                (workspace_pose or center_pose).position,
                (workspace_pose or center_pose).position,
            ),
            rotations=(
                (workspace_pose or center_pose).rotation,
                (workspace_pose or center_pose).rotation,
            ),
            mono_atlas=True,
        )
        ros_debug_frame = None
        if self.args.transport == "ros_compressed":
            success, encoded = cv2.imencode(
                ".jpg",
                cv2.cvtColor(color, cv2.COLOR_RGB2BGR),
                (cv2.IMWRITE_JPEG_QUALITY, self.args.ros_jpeg_quality),
            )
            if not success:
                raise RuntimeError("failed to encode ROS diagnostic JPEG")
            ros_debug_frame = pack_ros_debug_frame(encoded.tobytes(), frame_data)
        with self._frame_lock:
            self._color_frame = array("B", color.tobytes())
            self._frame_data = array("B", frame_data)
            self._ros_debug_frame = (
                array("B", ros_debug_frame) if ros_debug_frame is not None else None
            )
            self.last_ros_debug_payload_bytes = (
                len(ros_debug_frame) if ros_debug_frame is not None else 0
            )
            self.color = color
            self.depth = depths
            self.valid_depth_fraction = float(np.isfinite(reduced_depth).mean())
            self.rendered_frame_count += 1
            self.latest_render_sequence = sequence
            self.current_render_fov_deg = render_fov
            self.last_render_ms = (time.monotonic() - started) * 1000.0
            self.last_cuda_ms = cuda_ms

    def _render_v3_stereo_frame(
        self,
        poses: tuple[DynamicCameraPose, DynamicCameraPose],
        *,
        sequence: int,
        render_projections: tuple[
            tuple[float, float, float, float],
            tuple[float, float, float, float],
        ] | None,
        view_state: ViewState | None,
        predicted_display_time_ns: int,
        workspace_revision: int,
    ) -> None:
        projections = render_projections or (
            projection_from_vertical_fov(
                self.args.minimum_vertical_fov_deg,
                self.args.width / self.args.height,
            ),
        ) * 2
        render_arguments = dict(
            state=view_state,
            sequence=sequence,
            session_id=self.args.session_id,
            workspace_revision=workspace_revision,
            predicted_display_time_ns=predicted_display_time_ns,
            width=self.args.width,
            height=self.args.height,
            vertical_fov_deg=self.args.minimum_vertical_fov_deg,
            near_m=self.depth_near_m,
            far_m=self.depth_far_m,
            point_radius=self.args.point_splat_radius,
        )
        if self.args.transport == "webrtc":
            cuda_video = getattr(
                self._cuda_renderer,
                "render_video_views_cuda",
                None,
            )
            if self._h264_encoder is not None and callable(cuda_video):
                result = render_stereo_frame_cuda(
                    self._cuda_renderer,
                    poses,
                    projections,
                    visibility_mask_height=self.args.visibility_mask_height,
                    frame_marker_height=self._h264_encoder.frame_marker_height,
                    **render_arguments,
                )
                self._h264_encoder.submit_cuda_views(
                    result.colors,
                    result.depth_codes,
                    result.metadata.to_json(),
                    result.metadata.frame_id,
                )
                encoded = []  # The independent publisher drains every access unit.
            else:
                result = render_stereo_frame(
                    self._cuda_renderer,
                    poses,
                    projections,
                    codec="h264",
                    visibility_mask_height=self.args.visibility_mask_height,
                    frame_marker_height=16,
                    **render_arguments,
                )
                encoded = (
                    self._h264_encoder.encode_rgb_numpy(
                        result.video_rgb,
                        result.metadata.to_json(),
                        result.metadata.frame_id,
                    )
                    if self._h264_encoder is not None
                    else []
                )
        else:
            result = render_stereo_diagnostic(
                self._cuda_renderer,
                poses,
                projections,
                png_compression=self.args.ros_png_compression,
                **render_arguments,
            )

        with self._frame_lock:
            self._v3_video_height = result.metadata.video_height
            if self.args.transport == "webrtc":
                self._v3_ros_frame = None
                if self._h264_encoder is not None:
                    self._color_frame = None
                    # Synchronous encoders may return multiple access units.
                    # Forward every one, in order; never select encoded[-1].
                    for packet in encoded:
                        self._publish_encoded_packet(packet)
                    self.last_encode_ms = self._h264_encoder.last_encode_ms
                else:
                    self._encoded_frame = None
                    self._color_frame = array(
                        "B",
                        result.video_rgb.tobytes(order="C"),
                    )
                    self._v3_metadata_json = result.metadata.to_json()
                    self.latest_render_sequence = sequence
                self.last_ros_debug_payload_bytes = 0
            else:
                self._v3_ros_frame = array("B", result.payload)
                self._color_frame = None
                self._encoded_frame = None
                self._v3_metadata_json = ""
                self.last_ros_debug_payload_bytes = len(result.payload)
            self._ros_debug_frame = None
            self._frame_data = None
            if hasattr(result, "stereo_rgba"):
                self.color = result.stereo_rgba[..., :3]
            self.depth = getattr(result, "depths", None)
            self.valid_depth_fraction = result.valid_depth_fraction
            self.rendered_frame_count += 1
            if self.args.transport != "webrtc":
                self.latest_render_sequence = sequence
            self.current_render_fov_deg = self.args.minimum_vertical_fov_deg
            self.last_render_ms = (
                result.metadata.capture_time_ns - result.metadata.render_start_ns
            ) / 1_000_000.0
            self.last_cuda_ms = result.cuda_ms

    def _publish_encoded_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                for packet in self._h264_encoder.poll_encoded(timeout=0.05):
                    if self._stop_event.is_set():
                        return
                    self._publish_encoded_packet(packet)
        except Exception as exception:
            if not self._stop_event.is_set():
                self.get_logger().error(f"encoded video publisher failed: {exception}")
                self._stop_event.set()

    def _publish_encoded_packet(self, packet) -> None:
        image = CompressedImage()
        image.header.stamp = self.get_clock().now().to_msg()
        metadata = json.loads(packet.metadata_json)
        metadata["encoder_frame_index"] = packet.frame_index
        image.header.frame_id = "horus.remote_render.v3:" + json.dumps(
            metadata, separators=(",", ":")
        )
        image.format = "h264; stream-format=byte-stream; alignment=au"
        image.data = packet.data
        self.image_publisher.publish(image)
        self.latest_render_sequence = packet.source_sequence
        self.last_published_render_sequence = packet.source_sequence
        self.frame_index += 1

    def _publish_frame(self) -> None:
        if self.webrtc_image_type == "h264":
            return
        with self._frame_lock:
            color_frame = self._color_frame
            frame_data = self._frame_data
            ros_debug_frame = self._ros_debug_frame
            v3_ros_frame = self._v3_ros_frame
            v3_metadata_json = self._v3_metadata_json
            v3_video_height = self._v3_video_height
            render_sequence = self.latest_render_sequence
        if self.args.protocol == "v3":
            if self.args.transport == "webrtc":
                if (
                    color_frame is None
                    or not v3_metadata_json
                    or v3_video_height <= 0
                    or render_sequence <= 0
                    or render_sequence == self.last_published_render_sequence
                ):
                    return
                stamp = self.get_clock().now().to_msg()
                image = Image()
                image.header.stamp = stamp
                image.header.frame_id = "horus.remote_render.v3:" + v3_metadata_json
                image.height = v3_video_height
                image.width = self.args.width * 2
                image.encoding = "rgb8"
                image.is_bigendian = False
                image.step = image.width * 3
                image.data = color_frame
                self.image_publisher.publish(image)
                self.last_published_render_sequence = render_sequence
                self.frame_index += 1
                return
            if (
                v3_ros_frame is None
                or render_sequence <= 0
                or render_sequence == self.last_published_render_sequence
            ):
                return
            stamp = self.get_clock().now().to_msg()
            diagnostic = CompressedImage()
            diagnostic.header.stamp = stamp
            diagnostic.header.frame_id = f"remote_render_v3:{render_sequence}"
            diagnostic.format = V3_ROS_DIAGNOSTIC_FORMAT_VERSION
            diagnostic.data = v3_ros_frame
            self.ros_debug_publisher.publish(diagnostic)
            self.last_published_render_sequence = render_sequence
            self.frame_index += 1
            return
        if (
            color_frame is None
            or frame_data is None
            or render_sequence == self.last_published_render_sequence
        ):
            return

        stamp = self.get_clock().now().to_msg()
        frame_id = f"remote_map_render_camera:{render_sequence & 0xFFFF}"
        if self.args.transport == "webrtc":
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = frame_id
            image.height = self.args.height
            image.width = self.args.width
            image.encoding = "rgb8"
            image.is_bigendian = 0
            image.step = self.args.width * 3
            image.data = color_frame
            self.image_publisher.publish(image)

            auxiliary = CompressedImage()
            auxiliary.header.stamp = stamp
            auxiliary.header.frame_id = frame_id
            auxiliary.format = REMOTE_FRAME_FORMAT_VERSION
            auxiliary.data = frame_data
            self.frame_data_publisher.publish(auxiliary)
        elif ros_debug_frame is not None:
            diagnostic = CompressedImage()
            diagnostic.header.stamp = stamp
            diagnostic.header.frame_id = frame_id
            diagnostic.format = ROS_DEBUG_FRAME_FORMAT_VERSION
            diagnostic.data = ros_debug_frame
            self.ros_debug_publisher.publish(diagnostic)

        self.last_published_render_sequence = render_sequence
        self.frame_index += 1

    def _publish_status(self) -> None:
        self._status_publish_count += 1
        now = time.monotonic()
        rate_elapsed = max(now - self._rate_window_time, 1e-6)
        encoded_frames = (
            self._h264_encoder.encoded_frames
            if self._h264_encoder is not None
            else self.rendered_frame_count
        )
        view_rate_hz = (
            self._v3_view_received_count - self._rate_window_views
        ) / rate_elapsed
        render_rate_hz = (
            self.rendered_frame_count - self._rate_window_rendered
        ) / rate_elapsed
        publish_rate_hz = (
            self.frame_index - self._rate_window_published
        ) / rate_elapsed
        encode_rate_hz = (
            encoded_frames - self._rate_window_encoded
        ) / rate_elapsed
        self._rate_window_time = now
        self._rate_window_views = self._v3_view_received_count
        self._rate_window_rendered = self.rendered_frame_count
        self._rate_window_published = self.frame_index
        self._rate_window_encoded = encoded_frames
        pose_age_ms = None
        if self.last_pose_time > 0.0:
            pose_age_ms = max(0.0, (time.monotonic() - self.last_pose_time) * 1000.0)
        ros_debug_subscription_count = (
            self.ros_debug_publisher.get_subscription_count()
            if self.ros_debug_publisher is not None
            else 0
        )
        pose_publisher_count = self.pose_subscription.get_publisher_count()
        if (
            ros_debug_subscription_count != self._last_ros_debug_subscription_count
            or self._status_publish_count % 5 == 0
        ):
            pipeline_count = self._pipeline_window_count
            pipeline_average_ms = (
                self._pipeline_window_total_ms / pipeline_count
                if pipeline_count
                else 0.0
            )
            pipeline_max_ms = self._pipeline_window_max_ms
            self._pipeline_window_count = 0
            self._pipeline_window_total_ms = 0.0
            self._pipeline_window_max_ms = 0.0
            self.get_logger().info(
                "HORUS_REMOTE_RENDER_DIAG "
                f"transport={self.args.transport} "
                f"ros_debug_qos=reliable "
                f"subscriptions={ros_debug_subscription_count} "
                f"pose_publishers={pose_publisher_count} "
                f"v3_views={self._v3_view_received_count} "
                f"v3_rejected={self._v3_view_rejected_count} "
                f"v3_last_reject={self._v3_last_view_reject_reason} "
                f"published={self.frame_index} "
                f"rendered={self.rendered_frame_count} "
                f"view_fps={view_rate_hz:.2f} "
                f"render_fps={render_rate_hz:.2f} "
                f"encode_fps={encode_rate_hz:.2f} "
                f"publish_fps={publish_rate_hz:.2f} "
                f"latest_pose={self.latest_pose_sequence} "
                f"latest_render={self.latest_render_sequence} "
                f"payload_bytes={self.last_ros_debug_payload_bytes} "
                f"pose_age_ms={pose_age_ms} "
                f"renderer={self.renderer_name} "
                f"primitives={self.source_point_count} "
                f"render_ms={self.last_render_ms:.2f} "
                f"gpu_ms={self.last_cuda_ms:.2f} "
                f"encode_ms={self.last_encode_ms:.2f} "
                f"pipeline_ms={self.last_pipeline_ms:.2f} "
                f"pipeline_avg_ms={pipeline_average_ms:.2f} "
                f"pipeline_max_ms={pipeline_max_ms:.2f} "
                f"publish_ms={self.last_publish_ms:.2f} "
                f"encode_dropped={getattr(self._h264_encoder, 'dropped_pending_frames', 0)} "
                f"webrtc_source={self.webrtc_image_type} "
                f"visible={self.valid_depth_fraction:.4f}"
            )
            self._last_ros_debug_subscription_count = ros_debug_subscription_count
        status = String()
        status.data = json.dumps(
            {
                "state": "streaming_source",
                "renderer": self.renderer_name,
                "scene": self.args.scene,
                "transport": self.args.transport,
                "protocol_version": (
                    V3_PROTOCOL_VERSION
                    if self.args.protocol == "v3"
                    else "remote_stereo_rgbd_v2"
                ),
                "format_version": (
                    (
                        V3_WEBRTC_FORMAT_VERSION
                        if self.args.transport == "webrtc"
                        else V3_ROS_DIAGNOSTIC_FORMAT_VERSION
                    )
                    if self.args.protocol == "v3"
                    else (
                        REMOTE_FRAME_FORMAT_VERSION
                        if self.args.transport == "webrtc"
                        else ROS_DEBUG_FRAME_FORMAT_VERSION
                    )
                ),
                "stream_topic": (
                    (
                        STREAM_TOPIC
                        if self.args.transport == "webrtc"
                        else V3_DEFAULT_ROS_DIAGNOSTIC_TOPIC
                    )
                    if self.args.protocol == "v3"
                    else (
                        STREAM_TOPIC
                        if self.args.transport == "webrtc"
                        else ROS_DEBUG_FRAME_TOPIC
                    )
                ),
                "frame_data_topic": (
                    ""
                    if self.args.protocol == "v3" and self.args.transport == "webrtc"
                    else REMOTE_FRAME_DATA_TOPIC
                ),
                "ros_compressed_topic": (
                    V3_DEFAULT_ROS_DIAGNOSTIC_TOPIC
                    if self.args.protocol == "v3"
                    else ROS_DEBUG_FRAME_TOPIC
                ),
                "status_topic": (
                    V3_DEFAULT_STATUS_TOPIC
                    if self.args.protocol == "v3"
                    else AGENT_STATUS_TOPIC
                ),
                "viewer_pose_topic": self.args.viewer_pose_topic,
                "color_width": (
                    self.args.width * 2
                    if self.args.protocol == "v3"
                    else self.args.width
                ),
                "color_width_per_eye": self.args.width,
                "color_height": (
                    self._v3_video_height
                    if self.args.protocol == "v3" and self._v3_video_height > 0
                    else self.args.height
                ),
                "depth_width": (
                    self.args.width // 4
                    if self.args.protocol == "v3"
                    else self.args.depth_width
                ),
                "depth_height": (
                    self.args.visibility_mask_height
                    if self.args.protocol == "v3"
                    else self.args.depth_height
                ),
                "fps": self.args.fps,
                "render_update_fps": self.args.render_update_fps,
                "frames_published": self.frame_index,
                "frames_rendered": self.rendered_frame_count,
                "view_fps": view_rate_hz,
                "render_fps": render_rate_hz,
                "encode_fps": encode_rate_hz,
                "publish_fps": publish_rate_hz,
                "ros_debug_qos": "reliable",
                "ros_debug_subscription_count": ros_debug_subscription_count,
                "viewer_pose_publisher_count": pose_publisher_count,
                "v3_view_received_count": self._v3_view_received_count,
                "v3_view_rejected_count": self._v3_view_rejected_count,
                "v3_last_view_reject_reason": self._v3_last_view_reject_reason,
                "ros_debug_payload_bytes": self.last_ros_debug_payload_bytes,
                "latest_pose_sequence": self.latest_pose_sequence,
                "latest_render_sequence": self.latest_render_sequence,
                "pose_age_ms": pose_age_ms,
                "render_ms": self.last_render_ms,
                "cuda_ms": self.last_cuda_ms,
                "encode_ms": self.last_encode_ms,
                "pipeline_ms": self.last_pipeline_ms,
                "publish_ms": self.last_publish_ms,
                "webrtc_image_type": self.webrtc_image_type,
                "encoded_frames": (
                    self._h264_encoder.encoded_frames
                    if self._h264_encoder is not None
                    else 0
                ),
                "encoded_bytes": (
                    self._h264_encoder.encoded_bytes
                    if self._h264_encoder is not None
                    else 0
                ),
                "valid_depth_fraction": self.valid_depth_fraction,
                "source_primitive_count": self.source_point_count,
                "depth_near_m": self.depth_near_m,
                "depth_far_m": self.depth_far_m,
                "render_vertical_fov_deg": self.current_render_fov_deg,
                "guard_band_degrees": self.args.guard_band_degrees,
                "viewer_pose_received": self._viewer_pose_received,
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
                raise RuntimeError("render worker did not stop; GPU resources remain in use")
            self._render_thread = None
        if self._encoded_publish_thread is not None:
            self._encoded_publish_thread.join(timeout=5.0)
            if self._encoded_publish_thread.is_alive():
                raise RuntimeError("encoded publisher did not stop")
            self._encoded_publish_thread = None
        if self._cuda_renderer is not None:
            self._cuda_renderer.close()
            self._cuda_renderer = None
        if self._h264_encoder is not None:
            self._h264_encoder.close()
            self._h264_encoder = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Render a pose-adaptive 3D map on the PC and publish "
            "synchronized WebRTC color plus lossless depth metadata."
        )
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--depth-width", type=int, default=480)
    parser.add_argument("--depth-height", type=int, default=270)
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--bitrate-kbps", type=int, default=35_000)
    parser.add_argument(
        "--protocol",
        choices=("v2", "v3"),
        default="v3",
        help="Use exact per-eye V3; V2 is retained only for rollback.",
    )
    parser.add_argument("--session-id", default="")
    parser.add_argument(
        "--transport",
        choices=("webrtc", "ros_compressed"),
        default="webrtc",
    )
    parser.add_argument("--ros-jpeg-quality", type=int, default=92)
    parser.add_argument("--ros-png-compression", type=int, default=1)
    parser.add_argument("--visibility-mask-height", type=int, default=128)
    parser.add_argument(
        "--scene",
        choices=(
            "synthetic",
            *ADVANCED_REMOTE_SCENE_IDS,
            "cow_lady",
            "cow_lady_points",
            "recorded_mesh",
            "recorded_points",
            "eth3d_courtyard",
            *ETH3D_REMOTE_SCENE_IDS,
        ),
        default="cow_lady",
    )
    parser.add_argument("--dataset-path", default="")
    parser.add_argument("--dataset-scale", type=float, default=1.0)
    parser.add_argument(
        "--source-voxel-size",
        type=float,
        default=0.0,
        help=(
            "Optional PC render LOD voxel size in scene metres. Zero preserves "
            "every source point and is the default."
        ),
    )
    parser.add_argument("--point-splat-radius", type=int, default=2)
    parser.add_argument(
        "--synthetic-point-spacing",
        type=float,
        default=0.04,
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--viewer-pose-topic", default="")
    parser.add_argument("--render-update-fps", type=float, default=60.0)
    parser.add_argument("--minimum-vertical-fov-deg", type=float, default=90.0)
    parser.add_argument("--guard-band-degrees", type=float, default=6.0)
    parser.add_argument("--depth-near-m", type=float, default=0.0)
    parser.add_argument("--depth-far-m", type=float, default=0.0)
    parser.add_argument("--ready-file", default="")
    parser.add_argument("--preview-path", default="")
    args = parser.parse_args()

    if not args.viewer_pose_topic:
        args.viewer_pose_topic = (
            V3_DEFAULT_VIEW_STATE_TOPIC
            if args.protocol == "v3"
            else DEFAULT_VIEWER_POSE_TOPIC
        )

    if not FRAME_MARKER_WIDTH <= args.width <= 1920 or not 180 <= args.height <= 1920:
        parser.error(
            f"color dimensions must be within {FRAME_MARKER_WIDTH}x180 and 1920x1920"
        )
    if (
        not 1 <= args.depth_width <= args.width
        or not 1 <= args.depth_height <= args.height
    ):
        parser.error("depth dimensions must be positive and no larger than color")
    if not 1 <= args.fps <= 90 or not 1.0 <= args.render_update_fps <= 90.0:
        parser.error("publish and render rates must be between 1 and 90 FPS")
    if not 1000 <= args.bitrate_kbps <= 200_000:
        parser.error("--bitrate-kbps must be between 1000 and 200000")
    if not 20 <= args.ros_jpeg_quality <= 95:
        parser.error("--ros-jpeg-quality must be between 20 and 95")
    if not 0 <= args.ros_png_compression <= 9:
        parser.error("--ros-png-compression must be between 0 and 9")
    if not 16 <= args.visibility_mask_height <= min(512, args.height):
        parser.error("--visibility-mask-height must be between 16 and the frame height")
    if not args.viewer_pose_topic.startswith("/"):
        parser.error("--viewer-pose-topic must be an absolute ROS topic")
    if not 40.0 <= args.minimum_vertical_fov_deg <= 130.0:
        parser.error("--minimum-vertical-fov-deg must be between 40 and 130")
    if not 0.0 <= args.guard_band_degrees <= 25.0:
        parser.error("--guard-band-degrees must be between 0 and 25")
    if not 0 <= args.point_splat_radius <= 4:
        parser.error("--point-splat-radius must be between 0 and 4")
    if not 0.1 <= args.dataset_scale <= 10.0:
        parser.error("--dataset-scale must be between 0.1 and 10")
    if not 0.0 <= args.source_voxel_size <= 0.25:
        parser.error("--source-voxel-size must be zero or between 0.001 and 0.25 metres")
    if 0.0 < args.source_voxel_size < 0.001:
        parser.error("--source-voxel-size must be zero or between 0.001 and 0.25 metres")
    if not 0.01 <= args.synthetic_point_spacing <= 0.25:
        parser.error("--synthetic-point-spacing must be between 0.01 and 0.25 metres")
    if args.depth_near_m < 0.0 or args.depth_far_m < 0.0:
        parser.error("depth overrides cannot be negative")
    if (
        args.depth_near_m > 0.0
        and args.depth_far_m > 0.0
        and args.depth_far_m <= args.depth_near_m
    ):
        parser.error("--depth-far-m must be greater than --depth-near-m")
    # Rendering faster than the carrier can deliver only builds stale work.
    args.render_update_fps = min(args.render_update_fps, float(args.fps))
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
                        "webrtc_image_type": node.webrtc_image_type,
                        "transport": args.transport,
                        "protocol_version": (
                            V3_PROTOCOL_VERSION
                            if args.protocol == "v3"
                            else "remote_stereo_rgbd_v2"
                        ),
                        "format_version": (
                            (
                                V3_WEBRTC_FORMAT_VERSION
                                if args.transport == "webrtc"
                                else V3_ROS_DIAGNOSTIC_FORMAT_VERSION
                            )
                            if args.protocol == "v3"
                            else (
                                REMOTE_FRAME_FORMAT_VERSION
                                if args.transport == "webrtc"
                                else ROS_DEBUG_FRAME_FORMAT_VERSION
                            )
                        ),
                        "stream_topic": (
                            (
                                STREAM_TOPIC
                                if args.transport == "webrtc"
                                else V3_DEFAULT_ROS_DIAGNOSTIC_TOPIC
                            )
                            if args.protocol == "v3"
                            else (
                                STREAM_TOPIC
                                if args.transport == "webrtc"
                                else ROS_DEBUG_FRAME_TOPIC
                            )
                        ),
                        "frame_data_topic": (
                            ""
                            if args.protocol == "v3" and args.transport == "webrtc"
                            else REMOTE_FRAME_DATA_TOPIC
                        ),
                        "ros_compressed_topic": (
                            V3_DEFAULT_ROS_DIAGNOSTIC_TOPIC
                            if args.protocol == "v3"
                            else ROS_DEBUG_FRAME_TOPIC
                        ),
                        "viewer_pose_topic": args.viewer_pose_topic,
                        "color_width": args.width * 2 if args.protocol == "v3" else args.width,
                        "color_width_per_eye": args.width,
                        "color_height": (
                            args.height
                            + args.visibility_mask_height
                            + (
                                node._h264_encoder.frame_marker_height
                                if node._h264_encoder is not None
                                else 0
                            )
                            if args.protocol == "v3"
                            else args.height
                        ),
                        "depth_width": (
                            args.width // 4
                            if args.protocol == "v3"
                            else args.depth_width
                        ),
                        "depth_height": (
                            args.visibility_mask_height
                            if args.protocol == "v3"
                            else args.depth_height
                        ),
                        "fps": args.fps,
                        "render_update_fps": args.render_update_fps,
                        "valid_depth_fraction": node.valid_depth_fraction,
                    },
                    indent=2,
                ),
                encoding="utf-8",
            )
        color_topic = (
            STREAM_TOPIC
            if args.transport == "webrtc"
            else (
                V3_DEFAULT_ROS_DIAGNOSTIC_TOPIC
                if args.protocol == "v3"
                else ROS_DEBUG_FRAME_TOPIC
            )
        )
        frame_data_topic = (
            ""
            if args.protocol == "v3" and args.transport == "webrtc"
            else REMOTE_FRAME_DATA_TOPIC
        )
        depth_width = args.width // 4 if args.protocol == "v3" else args.depth_width
        depth_height = (
            args.visibility_mask_height
            if args.protocol == "v3"
            else args.depth_height
        )
        print(
            "[remote-map-source] READY "
            f"protocol={args.protocol} transport={args.transport} "
            f"color_topic={color_topic} "
            f"frame_data_topic={frame_data_topic or 'integrated'} "
            f"color={args.width}x{args.height} "
            f"(per_eye={args.width}x{args.height}) "
            f"depth_per_eye={depth_width}x{depth_height} "
            f"fps={args.fps} scene={args.scene}",
            flush=True,
        )
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():
            raise
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
