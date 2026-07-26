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
)
from horus.remote_rendering.real_scene import (
    iter_eth3d_courtyard_chunks,
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
        self.valid_depth_fraction = 0.0
        self.latest_pose_sequence = 0
        self.latest_render_sequence = -1
        self.last_published_render_sequence = -1
        self.last_pose_time = 0.0
        self.current_render_fov_deg = args.minimum_vertical_fov_deg
        self.last_ros_debug_payload_bytes = 0
        self._status_publish_count = 0
        self._last_ros_debug_subscription_count = -1

        self._frame_lock = threading.Lock()
        self._pose_lock = threading.Lock()
        self._pose_event = threading.Event()
        self._stop_event = threading.Event()
        self._render_thread: threading.Thread | None = None
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
        self._viewer_pose_received = False

        self._initialize_source()

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
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
            self.image_publisher = self.create_publisher(Image, STREAM_TOPIC, image_qos)
            self.frame_data_publisher = self.create_publisher(
                CompressedImage,
                REMOTE_FRAME_DATA_TOPIC,
                metadata_qos,
            )
        else:
            self.ros_debug_publisher = self.create_publisher(
                CompressedImage,
                ROS_DEBUG_FRAME_TOPIC,
                ros_debug_qos,
            )
        self.status_publisher = self.create_publisher(String, AGENT_STATUS_TOPIC, status_qos)
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
        # Frames are published by the render thread as soon as they exist. A
        # timer here would quantize publication to its own period, adding up to
        # a full frame of latency to every frame and re-introducing jitter that
        # the receiver then has to absorb.
        self.create_timer(1.0, self._publish_status)

    @property
    def renderer_name(self) -> str:
        return self._renderer_name

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
        elif args.scene == "cow_lady":
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
            if surface_path.is_file():
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
                print(
                    "[remote-map-source] WARNING Cow and Lady surface cache is "
                    "missing; point splats can shimmer during head motion. Run "
                    "prepare_cow_lady_surface.py once.",
                    flush=True,
                )
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
            lod_path, lod_point_count = prepare_eth3d_remote_scene_lod(
                scene,
                voxel_size_m=args.source_voxel_size,
            )
            self.source_point_count = lod_point_count
            self.depth_near_m = args.depth_near_m or scene.depth_near_m
            self.depth_far_m = args.depth_far_m or scene.depth_far_m
            pitch = np.deg2rad(scene.initial_pitch_degrees) * 0.5
            preview_pose = DynamicCameraPose(
                scene.initial_position,
                (float(np.sin(pitch)), 0.0, 0.0, float(np.cos(pitch))),
            )
            self._cuda_renderer = CudaPointRenderer(self.source_point_count)
            offset = 0
            for points, colors in iter_eth3d_remote_scene_lod_chunks(
                lod_path,
                chunk_size=1_000_000,
            ):
                self._cuda_renderer.upload(points, colors, offset=offset)
                offset += len(points)
                if offset % 5_000_000 < len(points):
                    print(
                        f"[remote-map-source] uploaded {offset:,}/"
                        f"{self.source_point_count:,} {args.scene} LOD points to CUDA",
                        flush=True,
                    )
            if offset != self.source_point_count:
                raise RuntimeError(
                    f"ETH3D {args.scene} contained {offset:,} finite points, "
                    f"expected {self.source_point_count:,}"
                )
            self._renderer_name = (
                f"cuda_pose_adaptive_points_rust_lod_{args.source_voxel_size:.4f}m"
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
                poses = self._latest_poses
                workspace_pose = self._latest_workspace_pose
                viewer_fov = self._latest_viewer_fov_deg
                projections = self._latest_projections
                sequence = self.latest_pose_sequence
            if sequence == self.latest_render_sequence:
                next_render = time.monotonic() + period
                continue
            try:
                started = time.monotonic()
                self._render_frame(
                    poses,
                    sequence=sequence,
                    workspace_pose=workspace_pose,
                    viewer_fov_deg=viewer_fov,
                    render_projections=projections,
                )
                self._publish_frame()
                next_render = max(started + period, time.monotonic())
            except Exception as exception:
                self.get_logger().error(f"pose-adaptive CUDA render failed: {exception}")
                self._stop_event.wait(0.5)

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
    ) -> None:
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

    def _publish_frame(self) -> None:
        with self._frame_lock:
            color_frame = self._color_frame
            frame_data = self._frame_data
            ros_debug_frame = self._ros_debug_frame
            render_sequence = self.latest_render_sequence
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
        pose_age_ms = None
        if self.last_pose_time > 0.0:
            pose_age_ms = max(0.0, (time.monotonic() - self.last_pose_time) * 1000.0)
        ros_debug_subscription_count = (
            self.ros_debug_publisher.get_subscription_count()
            if self.ros_debug_publisher is not None
            else 0
        )
        if (
            ros_debug_subscription_count != self._last_ros_debug_subscription_count
            or self._status_publish_count % 5 == 0
        ):
            self.get_logger().info(
                "HORUS_REMOTE_RENDER_DIAG "
                f"transport={self.args.transport} "
                f"ros_debug_qos=reliable "
                f"subscriptions={ros_debug_subscription_count} "
                f"published={self.frame_index} "
                f"rendered={self.rendered_frame_count} "
                f"latest_pose={self.latest_pose_sequence} "
                f"latest_render={self.latest_render_sequence} "
                f"payload_bytes={self.last_ros_debug_payload_bytes} "
                f"pose_age_ms={pose_age_ms}"
            )
            self._last_ros_debug_subscription_count = ros_debug_subscription_count
        status = String()
        status.data = json.dumps(
            {
                "state": "streaming_source",
                "renderer": self.renderer_name,
                "scene": self.args.scene,
                "transport": self.args.transport,
                "format_version": (
                    REMOTE_FRAME_FORMAT_VERSION
                    if self.args.transport == "webrtc"
                    else ROS_DEBUG_FRAME_FORMAT_VERSION
                ),
                "stream_topic": (
                    STREAM_TOPIC
                    if self.args.transport == "webrtc"
                    else ROS_DEBUG_FRAME_TOPIC
                ),
                "frame_data_topic": REMOTE_FRAME_DATA_TOPIC,
                "ros_compressed_topic": ROS_DEBUG_FRAME_TOPIC,
                "status_topic": AGENT_STATUS_TOPIC,
                "viewer_pose_topic": self.args.viewer_pose_topic,
                "color_width": self.args.width,
                "color_width_per_eye": self.args.width,
                "color_height": self.args.height,
                "depth_width": self.args.depth_width,
                "depth_height": self.args.depth_height,
                "fps": self.args.fps,
                "render_update_fps": self.args.render_update_fps,
                "frames_published": self.frame_index,
                "frames_rendered": self.rendered_frame_count,
                "ros_debug_qos": "reliable",
                "ros_debug_subscription_count": ros_debug_subscription_count,
                "ros_debug_payload_bytes": self.last_ros_debug_payload_bytes,
                "latest_pose_sequence": self.latest_pose_sequence,
                "latest_render_sequence": self.latest_render_sequence,
                "pose_age_ms": pose_age_ms,
                "render_ms": self.last_render_ms,
                "cuda_ms": self.last_cuda_ms,
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
            self._render_thread = None
        if self._cuda_renderer is not None:
            self._cuda_renderer.close()
            self._cuda_renderer = None


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
    parser.add_argument(
        "--transport",
        choices=("webrtc", "ros_compressed"),
        default="ros_compressed",
    )
    parser.add_argument("--ros-jpeg-quality", type=int, default=92)
    parser.add_argument(
        "--scene",
        choices=(
            "synthetic",
            *ADVANCED_REMOTE_SCENE_IDS,
            "cow_lady",
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
        default=0.0125,
        help=(
            "PC render LOD voxel size in scene metres. The complete downloaded "
            "source remains unchanged."
        ),
    )
    parser.add_argument("--point-splat-radius", type=int, default=2)
    parser.add_argument(
        "--synthetic-point-spacing",
        type=float,
        default=0.04,
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--viewer-pose-topic", default=DEFAULT_VIEWER_POSE_TOPIC)
    parser.add_argument("--render-update-fps", type=float, default=60.0)
    parser.add_argument("--minimum-vertical-fov-deg", type=float, default=90.0)
    parser.add_argument("--guard-band-degrees", type=float, default=6.0)
    parser.add_argument("--depth-near-m", type=float, default=0.0)
    parser.add_argument("--depth-far-m", type=float, default=0.0)
    parser.add_argument("--ready-file", default="")
    parser.add_argument("--preview-path", default="")
    args = parser.parse_args()

    if not FRAME_MARKER_WIDTH <= args.width <= 1920 or not 180 <= args.height <= 1920:
        parser.error(
            f"color dimensions must be within {FRAME_MARKER_WIDTH}x180 and 1920x1920"
        )
    if (
        not 1 <= args.depth_width <= args.width
        or not 1 <= args.depth_height <= args.height
    ):
        parser.error("depth dimensions must be positive and no larger than color")
    if not 1 <= args.fps <= 60 or not 1.0 <= args.render_update_fps <= 60.0:
        parser.error("publish and render rates must be between 1 and 60 FPS")
    if not 20 <= args.ros_jpeg_quality <= 95:
        parser.error("--ros-jpeg-quality must be between 20 and 95")
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
    if not 0.001 <= args.source_voxel_size <= 0.25:
        parser.error("--source-voxel-size must be between 0.001 and 0.25 metres")
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
                        "format_version": (
                            REMOTE_FRAME_FORMAT_VERSION
                            if args.transport == "webrtc"
                            else ROS_DEBUG_FRAME_FORMAT_VERSION
                        ),
                        "stream_topic": (
                            STREAM_TOPIC
                            if args.transport == "webrtc"
                            else ROS_DEBUG_FRAME_TOPIC
                        ),
                        "frame_data_topic": REMOTE_FRAME_DATA_TOPIC,
                        "ros_compressed_topic": ROS_DEBUG_FRAME_TOPIC,
                        "viewer_pose_topic": args.viewer_pose_topic,
                        "color_width": args.width,
                        "color_width_per_eye": args.width,
                        "color_height": args.height,
                        "depth_width": args.depth_width,
                        "depth_height": args.depth_height,
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
            f"transport={args.transport} "
            f"color_topic={STREAM_TOPIC if args.transport == 'webrtc' else ROS_DEBUG_FRAME_TOPIC} "
            f"frame_data_topic={REMOTE_FRAME_DATA_TOPIC} "
            f"color={args.width}x{args.height} "
            f"(per_eye={args.width}x{args.height}) "
            f"depth_per_eye={args.depth_width}x{args.depth_height} "
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
