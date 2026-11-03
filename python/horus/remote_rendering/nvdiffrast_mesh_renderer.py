"""Hardware-rasterized textured mesh rendering with NVIDIA nvdiffrast."""

from __future__ import annotations

import numpy as np


class NvdiffrastMeshRenderer:
    """Render indexed meshes through the GPU rasterizer instead of CUDA loops."""

    def __init__(
        self,
        vertices: np.ndarray,
        faces: np.ndarray,
        colors: np.ndarray,
        *,
        vertex_colors: np.ndarray | None = None,
        uvs: np.ndarray | None = None,
        texture_atlas: np.ndarray | None = None,
        texture_rects: np.ndarray | None = None,
    ) -> None:
        try:
            import torch
            import nvdiffrast.torch as dr
        except ImportError as exc:
            raise RuntimeError(
                "Hardware mesh rendering requires nvdiffrast. Run "
                "'python3 python/examples/tools/"
                "install_remote_render_dependencies.py'."
            ) from exc
        if not torch.cuda.is_available():
            raise RuntimeError(
                "Hardware mesh rendering requires a CUDA-capable NVIDIA GPU"
            )

        vertices = np.ascontiguousarray(vertices, dtype=np.float32)
        faces = np.ascontiguousarray(faces, dtype=np.int32)
        colors = np.ascontiguousarray(colors, dtype=np.uint8)
        if vertices.ndim != 2 or vertices.shape[1] != 3:
            raise ValueError("vertices must have shape (N, 3)")
        if faces.ndim != 2 or faces.shape[1] != 3:
            raise ValueError("faces must have shape (M, 3)")
        if colors.shape != (len(faces), 3):
            raise ValueError("colors must have shape (M, 3)")
        if len(vertices) == 0 or len(faces) == 0:
            raise ValueError("mesh must contain vertices and faces")
        if int(faces.min()) < 0 or int(faces.max()) >= len(vertices):
            raise ValueError("face index exceeds the vertex buffer")
        if vertex_colors is not None:
            vertex_colors = np.ascontiguousarray(vertex_colors, dtype=np.float32) / 255.0
            if vertex_colors.shape != vertices.shape or not np.isfinite(vertex_colors).all():
                raise ValueError("vertex_colors must have shape (N, 3) with finite values")

        if uvs is None:
            uvs = np.zeros((len(vertices), 2), dtype=np.float32)
        uvs = np.ascontiguousarray(uvs, dtype=np.float32)
        if uvs.shape != (len(vertices), 2):
            raise ValueError("uvs must have shape (N, 2)")
        if texture_atlas is None:
            texture_atlas = np.full((1, 1, 3), 255, dtype=np.uint8)
        texture_atlas = np.ascontiguousarray(texture_atlas, dtype=np.uint8)
        if texture_atlas.ndim != 3 or texture_atlas.shape[2] != 3:
            raise ValueError("texture_atlas must have shape (H, W, 3)")
        if texture_rects is None:
            texture_rects = np.repeat(
                np.asarray(((0, 0, 1, 1),), dtype=np.uint32),
                len(faces),
                axis=0,
            )
        texture_rects = np.ascontiguousarray(texture_rects, dtype=np.float32)
        if texture_rects.shape != (len(faces), 4):
            raise ValueError("texture_rects must have shape (M, 4)")

        atlas_height, atlas_width = texture_atlas.shape[:2]

        self._torch = torch
        self._dr = dr
        self._context = dr.RasterizeCudaContext()
        device = torch.device("cuda")
        self._vertices = torch.from_numpy(
            np.ascontiguousarray(vertices)
        ).to(device)
        self._triangles = torch.from_numpy(faces).to(device)
        self._uvs = torch.from_numpy(np.ascontiguousarray(uvs)).to(device)
        texture_extents = np.maximum(texture_rects[:, 2:] - 1.0, 0.0)
        uv_offsets = np.column_stack(
            (
                (texture_rects[:, 0] + 0.5) / float(atlas_width),
                1.0 - (texture_rects[:, 1] + 0.5) / float(atlas_height),
            )
        )
        uv_scales = np.column_stack(
            (
                texture_extents[:, 0] / float(atlas_width),
                -texture_extents[:, 1] / float(atlas_height),
            )
        )
        face_materials = np.column_stack(
            (
                uv_offsets,
                uv_scales,
                np.ascontiguousarray(colors, dtype=np.float32) / 255.0,
            )
        ).astype(np.float32, copy=False)
        # One indexed load now supplies atlas offset, atlas scale, and tint.
        # The previous path gathered two separate face tables and recomputed
        # normalized atlas transforms for every visible output pixel.
        self._face_materials = torch.from_numpy(
            np.ascontiguousarray(face_materials)
        ).to(device)
        atlas = texture_atlas.astype(np.float32) / 255.0
        self._texture = torch.from_numpy(atlas[None]).to(device)
        # Interpolation uses resident UV batches; texture sampling broadcasts a
        # single atlas across eyes, so stereo does not duplicate the whole atlas.
        self._uv_batches = {
            1: self._uvs[None],
            2: self._uvs[None].expand(2, -1, -1).contiguous(),
        }
        self._color_batches = None
        if vertex_colors is not None:
            resident_colors = torch.from_numpy(vertex_colors).to(device)
            self._color_batches = {
                1: resident_colors[None],
                2: resident_colors[None].expand(2, -1, -1).contiguous(),
            }
        self._visibility_rows: dict[tuple[int, int], object] = {}
        self._prepare_fragments_eager = self._build_prepare_fragments(torch)
        self._finish_fragments_eager = self._build_finish_fragments(torch)
        self._prepare_fragments = self._compile_helper(
            torch,
            self._prepare_fragments_eager,
        )
        self._finish_fragments = self._compile_helper(
            torch,
            self._finish_fragments_eager,
        )
        # Never rasterize on CUDA's default stream. The legacy default stream
        # implicitly serializes with the independent NVENC staging stream,
        # turning a pipelined render/encode path back into render + encode.
        self._render_stream = torch.cuda.Stream(device=device)
        self._render_start_event = torch.cuda.Event(enable_timing=True)
        self._render_end_event = torch.cuda.Event(enable_timing=True)
        mip_width = int(atlas_width)
        mip_height = int(atlas_height)
        self._max_mip_level = 0
        while (
            mip_width > 1
            and mip_height > 1
            and mip_width % 2 == 0
            and mip_height % 2 == 0
        ):
            mip_width //= 2
            mip_height //= 2
            self._max_mip_level += 1
        self._texture_mip = dr.texture_construct_mip(
            self._texture, max_mip_level=self._max_mip_level
        )
        self._topology_hash = dr.antialias_construct_topology_hash(self._triangles)
        # Scene tensors and caches were created on the loading stream. Establish
        # an explicit dependency before the independent render stream reads them.
        self._render_stream.wait_stream(torch.cuda.current_stream(device))
        self.vertex_count = int(len(vertices))
        self.face_count = int(len(faces))

    @staticmethod
    def _build_prepare_fragments(torch):
        def prepare_fragments(
            local_uv,
            local_uv_derivatives,
            raster,
            face_materials,
        ):
            primitive_ids = torch.clamp(
                raster[..., 3].to(torch.int64) - 1,
                min=0,
                max=face_materials.shape[0] - 1,
            )
            material = face_materials[primitive_ids]
            uv_scale = material[..., 2:4]
            atlas_uv = torch.remainder(local_uv, 1.0).mul(uv_scale).add_(
                material[..., 0:2]
            )
            atlas_derivatives = local_uv_derivatives.mul(
                uv_scale.repeat_interleave(2, dim=-1)
            )
            return (
                atlas_uv,
                atlas_derivatives,
                material[..., 4:7],
                raster[..., 3:] > 0.0,
            )

        return prepare_fragments

    @staticmethod
    def _build_finish_fragments(torch):
        def finish_fragments(
            image,
            visible,
            metric_depth,
            source_rows,
            near_m,
            far_m,
        ):
            color = (
                image.flip(1)
                .clamp(0.0, 1.0)
                .mul(255.0)
                .round()
                .to(torch.uint8)
            )
            sampled_depth = metric_depth.flip(1)[:, source_rows, :, 0]
            sample_valid = (
                torch.isfinite(sampled_depth)
                & (sampled_depth >= near_m)
                & (sampled_depth <= far_m)
            )
            groups = sampled_depth.reshape(
                sampled_depth.shape[0],
                sampled_depth.shape[1],
                sampled_depth.shape[2] // 4,
                4,
            )
            valid_groups = sample_valid.reshape_as(groups)
            sampled = torch.where(
                valid_groups,
                groups,
                torch.full_like(groups, float("inf")),
            ).amin(dim=3)
            valid = torch.isfinite(sampled)
            normalized = torch.nan_to_num(
                (sampled - near_m) / (far_m - near_m),
                nan=0.0,
                posinf=0.0,
                neginf=0.0,
            ).clamp_(0.0, 1.0)
            code = normalized.mul(65535.0).round().to(torch.int32)
            planes = []
            for shift in (12, 8, 4, 0):
                nibble = torch.bitwise_and(
                    torch.bitwise_right_shift(code, shift),
                    0x0F,
                )
                luma = nibble.mul(192.0 / 15.0).add(32.0).round()
                planes.append(
                    torch.where(valid, luma, torch.zeros_like(luma)).to(
                        torch.uint8
                    )
                )
            return color, torch.cat(planes, dim=2)

        return finish_fragments

    @staticmethod
    def _compile_helper(torch, function):
        compile_function = getattr(torch, "compile", None)
        if not callable(compile_function):
            return function
        return compile_function(
            function,
            fullgraph=True,
            dynamic=False,
            mode="max-autotune-no-cudagraphs",
        )

    def render_views(
        self,
        camera_positions,
        camera_rotations,
        width: int,
        height: int,
        *,
        vertical_fov_deg: float,
        projections=None,
        near_m: float,
        far_m: float,
        point_radius: int = 0,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        del point_radius
        positions, rotations, projections = self._normalize_views(
            camera_positions,
            camera_rotations,
            width,
            height,
            vertical_fov_deg=vertical_fov_deg,
            projections=projections,
        )

        colors, depths, _, elapsed_ms = self._render_gpu(
            positions,
            rotations,
            projections,
            width=int(width),
            height=int(height),
            near_m=float(near_m),
            far_m=float(far_m),
            visibility_mask_height=0,
            include_depth=True,
        )
        return colors, depths, elapsed_ms

    def render_video_views(
        self,
        camera_positions,
        camera_rotations,
        width: int,
        height: int,
        *,
        vertical_fov_deg: float,
        projections=None,
        near_m: float,
        far_m: float,
        visibility_mask_height: int,
        point_radius: int = 0,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """Render video color and a compact metric-depth strip."""
        del point_radius
        if int(width) % 4:
            raise ValueError("video depth width must be divisible by four")
        positions, rotations, projections = self._normalize_views(
            camera_positions,
            camera_rotations,
            width,
            height,
            vertical_fov_deg=vertical_fov_deg,
            projections=projections,
        )
        if visibility_mask_height <= 0:
            raise ValueError("visibility_mask_height must be positive")
        colors, _, masks, elapsed_ms = self._render_gpu(
            positions,
            rotations,
            projections,
            width=int(width),
            height=int(height),
            near_m=float(near_m),
            far_m=float(far_m),
            visibility_mask_height=int(visibility_mask_height),
            include_depth=False,
        )
        return colors, masks, elapsed_ms

    def render_video_views_cuda(
        self,
        camera_positions,
        camera_rotations,
        width: int,
        height: int,
        *,
        vertical_fov_deg: float,
        projections=None,
        near_m: float,
        far_m: float,
        visibility_mask_height: int,
        point_radius: int = 0,
    ):
        """Render video color and robust 16-bit depth into CUDA tensors."""
        del point_radius
        if int(width) % 4:
            raise ValueError("video depth width must be divisible by four")
        positions, rotations, projections = self._normalize_views(
            camera_positions,
            camera_rotations,
            width,
            height,
            vertical_fov_deg=vertical_fov_deg,
            projections=projections,
        )
        if visibility_mask_height <= 0:
            raise ValueError("visibility_mask_height must be positive")
        colors, _, masks, elapsed_ms = self._render_gpu_tensors(
            positions,
            rotations,
            projections,
            width=int(width),
            height=int(height),
            near_m=float(near_m),
            far_m=float(far_m),
            visibility_mask_height=int(visibility_mask_height),
            include_depth=False,
        )
        return colors, masks, elapsed_ms

    @staticmethod
    def _normalize_views(
        camera_positions,
        camera_rotations,
        width: int,
        height: int,
        *,
        vertical_fov_deg: float,
        projections,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        positions = np.ascontiguousarray(camera_positions, dtype=np.float32)
        rotations = np.ascontiguousarray(camera_rotations, dtype=np.float32)
        if positions.ndim != 2 or positions.shape[1] != 3:
            raise ValueError("camera_positions must have shape (N, 3)")
        if rotations.shape != (len(positions), 4):
            raise ValueError("camera_rotations must have shape (N, 4)")
        if projections is None:
            projection_y = 1.0 / np.tan(np.deg2rad(vertical_fov_deg) * 0.5)
            projection_x = projection_y / (float(width) / float(height))
            projections = [(projection_x, projection_y, 0.0, 0.0)] * len(
                positions
            )
        projections = np.ascontiguousarray(projections, dtype=np.float32)
        if projections.shape != (len(positions), 4):
            raise ValueError("projections must have shape (N, 4)")
        return positions, rotations, projections

    def _render_gpu(
        self,
        positions: np.ndarray,
        rotations: np.ndarray,
        projections: np.ndarray,
        *,
        width: int,
        height: int,
        near_m: float,
        far_m: float,
        visibility_mask_height: int,
        include_depth: bool,
    ) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None, float]:
        color_tensor, depth_tensor, mask_tensor, elapsed_ms = (
            self._render_gpu_tensors(
                positions,
                rotations,
                projections,
                width=width,
                height=height,
                near_m=near_m,
                far_m=far_m,
                visibility_mask_height=visibility_mask_height,
                include_depth=include_depth,
            )
        )
        return (
            color_tensor.cpu().numpy(),
            depth_tensor.float().cpu().numpy() if depth_tensor is not None else None,
            mask_tensor.cpu().numpy() if mask_tensor is not None else None,
            elapsed_ms,
        )

    def _render_gpu_tensors(
        self,
        positions: np.ndarray,
        rotations: np.ndarray,
        projections: np.ndarray,
        *,
        width: int,
        height: int,
        near_m: float,
        far_m: float,
        visibility_mask_height: int,
        include_depth: bool,
    ):
        from .dynamic_stream import quaternion_to_matrix

        torch = self._torch
        dr = self._dr
        device = self._vertices.device
        view_count = len(positions)
        if view_count not in self._uv_batches:
            self._uv_batches[view_count] = (
                self._uvs[None].expand(view_count, -1, -1).contiguous()
            )
            self._render_stream.wait_stream(torch.cuda.current_stream(device))
        camera_to_world = np.stack(
            [quaternion_to_matrix(rotation) for rotation in rotations],
            axis=0,
        ).astype(np.float32, copy=False)
        with torch.inference_mode():
            with torch.cuda.stream(self._render_stream):
                self._render_start_event.record(self._render_stream)
                position_tensor = torch.from_numpy(positions).to(device)
                rotation_tensor = torch.from_numpy(camera_to_world).to(device)
                projection_tensor = torch.from_numpy(projections).to(device)
                camera_vertices = torch.matmul(
                    self._vertices[None] - position_tensor[:, None, :],
                    rotation_tensor,
                )
                z = camera_vertices[..., 2]
                clip_z_scale = (far_m + near_m) / (far_m - near_m)
                clip_z_offset = -2.0 * far_m * near_m / (far_m - near_m)
                clip = torch.stack(
                    (
                        projection_tensor[:, None, 0] * camera_vertices[..., 0]
                        + projection_tensor[:, None, 2] * z,
                        projection_tensor[:, None, 1] * camera_vertices[..., 1]
                        + projection_tensor[:, None, 3] * z,
                        clip_z_scale * z + clip_z_offset,
                        z,
                    ),
                    dim=2,
                )
                raster, derivatives = dr.rasterize(
                    self._context,
                    clip,
                    self._triangles,
                    resolution=(height, width),
                )
                local_uv, local_uv_derivatives = dr.interpolate(
                    self._uv_batches[view_count],
                    raster,
                    self._triangles,
                    rast_db=derivatives,
                    diff_attrs="all",
                )
                try:
                    (
                        atlas_uv,
                        atlas_derivatives,
                        face_tints,
                        visible,
                    ) = self._prepare_fragments(
                        local_uv,
                        local_uv_derivatives,
                        raster,
                        self._face_materials,
                    )
                except Exception:
                    # TorchInductor is an optimization, not a runtime
                    # dependency. Preserve the exact eager implementation on
                    # systems where its CUDA compiler is unavailable.
                    self._prepare_fragments = self._prepare_fragments_eager
                    (
                        atlas_uv,
                        atlas_derivatives,
                        face_tints,
                        visible,
                    ) = self._prepare_fragments(
                        local_uv,
                        local_uv_derivatives,
                        raster,
                        self._face_materials,
                    )
                sampled = dr.texture(
                    self._texture,
                    atlas_uv,
                    atlas_derivatives,
                    filter_mode="linear-mipmap-linear",
                    boundary_mode="clamp",
                    max_mip_level=self._max_mip_level,
                    mip=self._texture_mip,
                )
                image = sampled.mul(face_tints)
                if self._color_batches is not None:
                    source_color, _ = dr.interpolate(
                        self._color_batches[view_count], raster, self._triangles
                    )
                    image.mul_(source_color)
                image.masked_fill_(~visible, 0.0)
                image = dr.antialias(
                    image, raster, clip, self._triangles, topology_hash=self._topology_hash
                )

                metric_depth = None
                depth_tensor = None
                if include_depth or visibility_mask_height > 0:
                    metric_depth, _ = dr.interpolate(
                        camera_vertices[..., 2:3].contiguous(),
                        raster,
                        self._triangles,
                    )
                    metric_depth = torch.where(
                        visible,
                        metric_depth,
                        torch.full_like(metric_depth, float("inf")),
                    )
                if include_depth:
                    depth_tensor = metric_depth[..., 0].flip(1)

                # Encode robust 16-bit depth into a narrow video strip. The PC
                # still renders the complete mesh; Quest receives only enough
                # geometry information to reproject that image into the fixed
                # workspace as the operator translates their head.
                mask_key = (height, visibility_mask_height)
                source_rows = self._visibility_rows.get(mask_key)
                if source_rows is None:
                    source_rows = (
                        ((torch.arange(visibility_mask_height, device=device) + 0.5)
                         * height / visibility_mask_height).floor().to(torch.int64)
                        if visibility_mask_height > 0
                        else torch.empty(0, dtype=torch.int64, device=device)
                    )
                    self._visibility_rows[mask_key] = source_rows
                try:
                    color_tensor, sampled_depth = self._finish_fragments(
                        image,
                        visible,
                        metric_depth,
                        source_rows,
                        near_m,
                        far_m,
                    )
                except Exception:
                    self._finish_fragments = self._finish_fragments_eager
                    color_tensor, sampled_depth = self._finish_fragments(
                        image,
                        visible,
                        metric_depth,
                        source_rows,
                        near_m,
                        far_m,
                    )
                mask_tensor = (
                    sampled_depth if visibility_mask_height > 0 else None
                )
                self._render_end_event.record(self._render_stream)
            # Wait only for this renderer's stream. A device-wide synchronize
            # would also wait for the independent NVENC staging stream and
            # eliminate the intended render/encode overlap.
            self._render_end_event.synchronize()

        elapsed_ms = self._render_start_event.elapsed_time(self._render_end_event)
        return color_tensor, depth_tensor, mask_tensor, elapsed_ms

    def close(self) -> None:
        for name in (
            "_vertices",
            "_triangles",
            "_uvs",
            "_face_materials",
            "_texture",
            "_uv_batches",
            "_color_batches",
            "_texture_mip",
            "_topology_hash",
            "_visibility_rows",
            "_render_stream",
            "_render_start_event",
            "_render_end_event",
            "_context",
        ):
            if hasattr(self, name):
                delattr(self, name)

    def __enter__(self) -> "NvdiffrastMeshRenderer":
        return self

    def __exit__(self, *_args) -> None:
        self.close()
