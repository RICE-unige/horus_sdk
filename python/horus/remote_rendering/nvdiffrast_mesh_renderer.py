"""Hardware-rasterized textured mesh rendering with NVIDIA nvdiffrast."""

from __future__ import annotations

import time

import numpy as np


class NvdiffrastMeshRenderer:
    """Render indexed meshes through the GPU rasterizer instead of CUDA loops."""

    def __init__(
        self,
        vertices: np.ndarray,
        faces: np.ndarray,
        colors: np.ndarray,
        *,
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

        # Material and UV seams require face-corner attributes. Unrolling them
        # once keeps the render loop entirely on the GPU and avoids per-frame
        # material batches.
        corner_vertices = vertices[faces].reshape(-1, 3)
        corner_uvs = uvs[faces].reshape(-1, 2)
        rectangles = np.repeat(texture_rects, 3, axis=0)
        atlas_height, atlas_width = texture_atlas.shape[:2]
        corner_tints = np.repeat(colors, 3, axis=0).astype(np.float32) / 255.0
        triangles = np.arange(len(corner_vertices), dtype=np.int32).reshape(-1, 3)

        self._torch = torch
        self._dr = dr
        self._context = dr.RasterizeCudaContext()
        device = torch.device("cuda")
        self._vertices = torch.from_numpy(
            np.ascontiguousarray(corner_vertices)
        ).to(device)
        self._triangles = torch.from_numpy(triangles).to(device)
        self._uvs = torch.from_numpy(np.ascontiguousarray(corner_uvs)).to(device)
        self._texture_rects = torch.from_numpy(
            np.ascontiguousarray(rectangles)
        ).to(device)
        self._tints = torch.from_numpy(
            np.ascontiguousarray(corner_tints)
        ).to(device)
        atlas = texture_atlas.astype(np.float32) / 255.0
        self._texture = torch.from_numpy(atlas[None]).to(device)
        self._atlas_width = float(atlas_width)
        self._atlas_height = float(atlas_height)
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
        self.vertex_count = int(len(corner_vertices))
        self.face_count = int(len(faces))

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
        from .dynamic_stream import quaternion_to_matrix

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

        torch = self._torch
        dr = self._dr
        colors = []
        depths = []
        started = time.perf_counter()
        with torch.inference_mode():
            for position, rotation, projection in zip(
                positions,
                rotations,
                projections,
            ):
                world_to_camera = torch.from_numpy(
                    np.ascontiguousarray(
                        quaternion_to_matrix(rotation).T,
                        dtype=np.float32,
                    )
                ).to(self._vertices.device)
                camera_vertices = (
                    self._vertices
                    - torch.from_numpy(position).to(self._vertices.device)
                ) @ world_to_camera.T
                projection_x, projection_y, offset_x, offset_y = (
                    float(value) for value in projection
                )
                z = camera_vertices[:, 2]
                clip_z_scale = (far_m + near_m) / (far_m - near_m)
                clip_z_offset = -2.0 * far_m * near_m / (far_m - near_m)
                clip = torch.stack(
                    (
                        projection_x * camera_vertices[:, 0] + offset_x * z,
                        projection_y * camera_vertices[:, 1] + offset_y * z,
                        clip_z_scale * z + clip_z_offset,
                        z,
                    ),
                    dim=1,
                )
                raster, derivatives = dr.rasterize(
                    self._context,
                    clip[None],
                    self._triangles,
                    resolution=(int(height), int(width)),
                )
                local_uv, local_uv_derivatives = dr.interpolate(
                    self._uvs[None],
                    raster,
                    self._triangles,
                    rast_db=derivatives,
                    diff_attrs="all",
                )
                texture_rect, _ = dr.interpolate(
                    self._texture_rects[None],
                    raster,
                    self._triangles,
                )
                wrapped_uv = torch.remainder(local_uv, 1.0)
                texture_extent = torch.clamp(
                    texture_rect[..., 2:] - 1.0,
                    min=0.0,
                )
                atlas_uv = torch.empty_like(wrapped_uv)
                atlas_uv[..., 0] = (
                    texture_rect[..., 0]
                    + wrapped_uv[..., 0] * texture_extent[..., 0]
                    + 0.5
                ) / self._atlas_width
                # Source UVs and the atlas are top-down; nvdiffrast texture
                # coordinates are bottom-up.
                atlas_uv[..., 1] = 1.0 - (
                    texture_rect[..., 1]
                    + wrapped_uv[..., 1] * texture_extent[..., 1]
                    + 0.5
                ) / self._atlas_height
                derivative_scale = torch.stack(
                    (
                        texture_extent[..., 0] / self._atlas_width,
                        texture_extent[..., 0] / self._atlas_width,
                        -texture_extent[..., 1] / self._atlas_height,
                        -texture_extent[..., 1] / self._atlas_height,
                    ),
                    dim=-1,
                )
                atlas_uv_derivatives = (
                    local_uv_derivatives * derivative_scale
                )
                sampled = dr.texture(
                    self._texture,
                    atlas_uv,
                    atlas_uv_derivatives,
                    filter_mode="linear-mipmap-linear",
                    boundary_mode="clamp",
                    max_mip_level=self._max_mip_level,
                )
                tint, _ = dr.interpolate(
                    self._tints[None],
                    raster,
                    self._triangles,
                )
                image = sampled * tint
                visible = raster[..., 3:] > 0.0
                image = torch.where(visible, image, torch.zeros_like(image))
                image = dr.antialias(
                    image,
                    raster,
                    clip[None],
                    self._triangles,
                )
                metric_depth, _ = dr.interpolate(
                    camera_vertices[:, 2:3].contiguous()[None],
                    raster,
                    self._triangles,
                )
                metric_depth = torch.where(
                    visible,
                    metric_depth,
                    torch.full_like(metric_depth, float("inf")),
                )
                colors.append(
                    image[0]
                    .flip(0)
                    .clamp(0.0, 1.0)
                    .mul(255.0)
                    .round()
                    .to(torch.uint8)
                )
                # nvdiffrast exposes OpenGL's bottom-left framebuffer origin.
                # Every other remote renderer and the frame protocol use
                # top-left NumPy images, so normalize both attachments here.
                depths.append(metric_depth[0, ..., 0].flip(0))
            torch.cuda.synchronize()

        elapsed_ms = (time.perf_counter() - started) * 1000.0
        return (
            torch.stack(colors).cpu().numpy(),
            torch.stack(depths).float().cpu().numpy(),
            elapsed_ms,
        )

    def close(self) -> None:
        for name in (
            "_vertices",
            "_triangles",
            "_uvs",
            "_texture_rects",
            "_tints",
            "_texture",
            "_context",
        ):
            if hasattr(self, name):
                delattr(self, name)

    def __enter__(self) -> "NvdiffrastMeshRenderer":
        return self

    def __exit__(self, *_args) -> None:
        self.close()
