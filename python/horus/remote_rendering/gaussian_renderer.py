"""CUDA Gaussian-splat rendering adapter for the HORUS remote-map agent."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np

from .scene_catalog import GAUSSIAN_SCENES


DEFAULT_PLAYROOM_SPLAT = (
    Path.home()
    / ".cache"
    / "horus"
    / "gaussian_splatting"
    / "prebuilt"
    / "playroom"
    / "playroom_30000.ply"
)


@dataclass(frozen=True)
class GaussianSceneTransform:
    """Fixed mapping from HORUS workspace coordinates into 3DGS coordinates."""

    source_from_world_rotation: tuple[float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        1.0,
    )
    source_from_world_translation: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def world_pose_to_source(self, position, rotation) -> tuple[np.ndarray, np.ndarray]:
        from .dynamic_stream import quaternion_multiply, quaternion_to_matrix

        source_rotation = quaternion_multiply(
            self.source_from_world_rotation,
            rotation,
        )
        source_position = (
            quaternion_to_matrix(self.source_from_world_rotation)
            @ np.asarray(position, dtype=np.float32)
            + np.asarray(self.source_from_world_translation, dtype=np.float32)
        )
        return source_position, source_rotation

    def source_pose_to_world(self, position, rotation) -> tuple[np.ndarray, np.ndarray]:
        from .dynamic_stream import (
            quaternion_inverse,
            quaternion_multiply,
            quaternion_to_matrix,
        )

        inverse_rotation = quaternion_inverse(self.source_from_world_rotation)
        source_position = np.asarray(position, dtype=np.float32)
        translation = np.asarray(
            self.source_from_world_translation,
            dtype=np.float32,
        )
        world_position = quaternion_to_matrix(inverse_rotation) @ (
            source_position - translation
        )
        world_rotation = quaternion_multiply(inverse_rotation, rotation)
        return world_position, world_rotation


@dataclass(frozen=True)
class GaussianSplatScene:
    means: np.ndarray
    quaternions: np.ndarray
    scales: np.ndarray
    opacities: np.ndarray
    spherical_harmonics: np.ndarray
    sh_degree: int
    transform: GaussianSceneTransform = GaussianSceneTransform()
    world_bounds_min: tuple[float, float, float] = (0.0, 0.0, 0.0)
    world_bounds_max: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def gaussian_count(self) -> int:
        return int(len(self.means))


def resolve_playroom_splat(path: str | Path | None = None) -> Path:
    return resolve_gaussian_splat("gaussian_playroom", path)


def resolve_gaussian_splat(
    scene_id: str,
    path: str | Path | None = None,
) -> Path:
    if scene_id not in GAUSSIAN_SCENES:
        raise ValueError(f"unknown Gaussian Splat scene: {scene_id}")
    candidate = (
        Path(path).expanduser()
        if path
        else GAUSSIAN_SCENES[scene_id].cache_path
    )
    if candidate.is_dir():
        expected = GAUSSIAN_SCENES[scene_id].cache_path.name
        candidate = candidate / expected
    if not candidate.is_file():
        raise FileNotFoundError(
            f"{GAUSSIAN_SCENES[scene_id].label} is missing: {candidate}. Run "
            "'python3 python/examples/tools/fetch_remote_render_advanced_maps.py "
            f"--scene {scene_id}' first."
        )
    return candidate


def load_gaussian_splat_ply(
    path: str | Path,
    *,
    world_scale: float = 1.0,
    canonicalize_y_down: bool = False,
    floor_quantile: float = 0.995,
) -> GaussianSplatScene:
    """Load an original 3DGS PLY without reducing or converting its Gaussians."""
    from .dynamic_stream import quaternion_inverse, quaternion_to_matrix

    try:
        from plyfile import PlyData
    except ImportError as exc:
        raise RuntimeError(
            "Gaussian rendering requires plyfile. Run "
            "'python3 python/examples/tools/install_remote_render_dependencies.py'."
        ) from exc

    vertex = PlyData.read(Path(path), mmap=True)["vertex"].data
    names = set(vertex.dtype.names or ())
    required = {
        "x",
        "y",
        "z",
        "f_dc_0",
        "f_dc_1",
        "f_dc_2",
        "opacity",
        "scale_0",
        "scale_1",
        "scale_2",
        "rot_0",
        "rot_1",
        "rot_2",
        "rot_3",
    }
    missing = sorted(required - names)
    if missing:
        raise ValueError(f"Gaussian PLY is missing properties: {', '.join(missing)}")

    means = np.stack([vertex[name] for name in ("x", "y", "z")], axis=1).astype(
        np.float32
    )
    dc = np.stack([vertex[f"f_dc_{index}"] for index in range(3)], axis=1).astype(
        np.float32
    )
    rest_names = sorted(
        (name for name in names if name.startswith("f_rest_")),
        key=lambda name: int(name.rsplit("_", 1)[1]),
    )
    if len(rest_names) % 3:
        raise ValueError("Gaussian PLY has an invalid spherical-harmonic layout")
    coefficients = 1 + len(rest_names) // 3
    sh_degree = int(round(np.sqrt(coefficients) - 1))
    if (sh_degree + 1) ** 2 != coefficients:
        raise ValueError(
            f"Gaussian PLY has {coefficients} SH coefficients, which is not square"
        )
    if rest_names:
        rest = np.stack([vertex[name] for name in rest_names], axis=1).astype(
            np.float32
        )
        rest = rest.reshape(-1, 3, coefficients - 1).transpose(0, 2, 1)
        spherical_harmonics = np.concatenate((dc[:, None, :], rest), axis=1)
    else:
        spherical_harmonics = dc[:, None, :]

    raw_opacity = np.asarray(vertex["opacity"], dtype=np.float32)
    opacities = 1.0 / (1.0 + np.exp(-np.clip(raw_opacity, -30.0, 30.0)))
    raw_scales = np.stack(
        [vertex[f"scale_{index}"] for index in range(3)], axis=1
    ).astype(np.float32)
    scales = np.exp(np.clip(raw_scales, -20.0, 10.0))
    quaternions = np.stack(
        [vertex[f"rot_{index}"] for index in range(4)], axis=1
    ).astype(np.float32)
    quaternion_norms = np.linalg.norm(quaternions, axis=1, keepdims=True)
    quaternions = np.divide(
        quaternions,
        np.maximum(quaternion_norms, 1e-8),
        out=np.zeros_like(quaternions),
    )
    zero_rotation = quaternion_norms[:, 0] < 1e-8
    quaternions[zero_rotation, 0] = 1.0

    finite = (
        np.isfinite(means).all(axis=1)
        & np.isfinite(scales).all(axis=1)
        & np.isfinite(quaternions).all(axis=1)
        & np.isfinite(opacities)
        & np.isfinite(spherical_harmonics).all(axis=(1, 2))
    )
    if not finite.all():
        means = means[finite]
        scales = scales[finite]
        quaternions = quaternions[finite]
        opacities = opacities[finite]
        spherical_harmonics = spherical_harmonics[finite]

    scale = float(world_scale)
    means *= scale
    scales *= scale
    transform = GaussianSceneTransform()
    if canonicalize_y_down:
        if not 0.9 <= float(floor_quantile) <= 1.0:
            raise ValueError("floor_quantile must be between 0.9 and 1.0")
        # Graphdeco's Deep Blending models use a y-down world. Keep the
        # trained Gaussians and SH coefficients in that source space, and map
        # HORUS' stable Y-up workspace camera into it at render time. This
        # avoids rotating millions of anisotropic covariances or directional
        # SH coefficients while still giving the workspace a horizontal floor.
        transform = GaussianSceneTransform(
            source_from_world_rotation=(0.0, 0.0, 1.0, 0.0),
            source_from_world_translation=(
                float(np.median(means[:, 0])),
                float(np.quantile(means[:, 1], floor_quantile)),
                float(np.median(means[:, 2])),
            ),
        )
    inverse_rotation = quaternion_to_matrix(
        quaternion_inverse(transform.source_from_world_rotation)
    )
    translation = np.asarray(
        transform.source_from_world_translation,
        dtype=np.float32,
    )
    # Quantiles reject isolated training outliers that would otherwise place
    # the startup validation camera far outside the reconstructed scene.
    source_bounds = np.stack(
        (
            np.quantile(means, 0.005, axis=0),
            np.quantile(means, 0.995, axis=0),
        ),
        axis=0,
    ).astype(np.float32)
    source_corners = np.asarray(
        [
            (x, y, z)
            for x in source_bounds[:, 0]
            for y in source_bounds[:, 1]
            for z in source_bounds[:, 2]
        ],
        dtype=np.float32,
    )
    world_corners = (source_corners - translation) @ inverse_rotation.T
    world_bounds_min = tuple(float(value) for value in world_corners.min(axis=0))
    world_bounds_max = tuple(float(value) for value in world_corners.max(axis=0))
    return GaussianSplatScene(
        means=np.ascontiguousarray(means),
        quaternions=np.ascontiguousarray(quaternions),
        scales=np.ascontiguousarray(scales),
        opacities=np.ascontiguousarray(opacities),
        spherical_harmonics=np.ascontiguousarray(spherical_harmonics),
        sh_degree=sh_degree,
        transform=transform,
        world_bounds_min=world_bounds_min,
        world_bounds_max=world_bounds_max,
    )


class GaussianSplatRenderer:
    """Render full anisotropic 3D Gaussians with SH color and metric depth."""

    def __init__(self, scene: GaussianSplatScene) -> None:
        try:
            import torch
            from gsplat.rendering import rasterization
        except ImportError as exc:
            raise RuntimeError(
                "Gaussian rendering requires CUDA PyTorch and gsplat. Run "
                "'python3 python/examples/tools/install_remote_render_dependencies.py'."
            ) from exc
        if not torch.cuda.is_available():
            raise RuntimeError("Gaussian rendering requires a CUDA-capable NVIDIA GPU")
        self._torch = torch
        self._rasterization = rasterization
        self.gaussian_count = scene.gaussian_count
        self.sh_degree = scene.sh_degree
        device = torch.device("cuda")
        self._means = torch.from_numpy(scene.means).to(device)
        self._quaternions = torch.from_numpy(scene.quaternions).to(device)
        self._scales = torch.from_numpy(scene.scales).to(device)
        self._opacities = torch.from_numpy(scene.opacities).to(device)
        self._spherical_harmonics = torch.from_numpy(
            scene.spherical_harmonics
        ).to(device)
        self._source_from_world_rotation = (
            scene.transform.source_from_world_rotation
        )
        self._source_from_world_translation = np.asarray(
            scene.transform.source_from_world_translation,
            dtype=np.float32,
        )

    def render_views(
        self,
        camera_positions,
        camera_rotations,
        width: int,
        height: int,
        *,
        vertical_fov_deg: float,
        near_m: float,
        far_m: float,
        point_radius: int = 0,
        projections=None,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        del point_radius
        from .dynamic_stream import quaternion_multiply, quaternion_to_matrix

        positions = np.ascontiguousarray(camera_positions, dtype=np.float32)
        rotations = np.ascontiguousarray(camera_rotations, dtype=np.float32)
        if positions.ndim != 2 or positions.shape[1] != 3:
            raise ValueError("camera_positions must have shape (N, 3)")
        if rotations.shape != (len(positions), 4):
            raise ValueError("camera_rotations must have shape (N, 4)")
        transformed_poses = [
            (
                quaternion_to_matrix(self._source_from_world_rotation) @ position
                + self._source_from_world_translation,
                quaternion_multiply(self._source_from_world_rotation, rotation),
            )
            for position, rotation in zip(positions, rotations)
        ]
        positions = np.ascontiguousarray(
            [pose[0] for pose in transformed_poses],
            dtype=np.float32,
        )
        rotations = np.ascontiguousarray(
            [pose[1] for pose in transformed_poses],
            dtype=np.float32,
        )
        if projections is None:
            projection_y = 1.0 / np.tan(np.deg2rad(vertical_fov_deg) * 0.5)
            projection_x = projection_y / (float(width) / float(height))
            projections = [(projection_x, projection_y, 0.0, 0.0)] * len(positions)
        projections = np.ascontiguousarray(projections, dtype=np.float32)
        if projections.shape != (len(positions), 4):
            raise ValueError("projections must have shape (N, 4)")

        view_matrices = np.repeat(
            np.eye(4, dtype=np.float32)[None, :, :],
            len(positions),
            axis=0,
        )
        intrinsics = np.zeros((len(positions), 3, 3), dtype=np.float32)
        for index, (position, rotation, projection) in enumerate(
            zip(positions, rotations, projections)
        ):
            camera_to_world = quaternion_to_matrix(rotation)
            world_to_camera = camera_to_world.T
            # gsplat follows OpenCV's y-down camera coordinates.
            world_to_camera[1] *= -1.0
            view_matrices[index, :3, :3] = world_to_camera
            view_matrices[index, :3, 3] = -world_to_camera @ position
            projection_x, projection_y, offset_x, offset_y = projection
            intrinsics[index] = (
                (0.5 * width * projection_x, 0.0, 0.5 * width * (offset_x + 1.0)),
                (0.0, 0.5 * height * projection_y, 0.5 * height * (1.0 - offset_y)),
                (0.0, 0.0, 1.0),
            )

        torch = self._torch
        started = time.perf_counter()
        with torch.inference_mode():
            rendered, alpha, _ = self._rasterization(
                self._means,
                self._quaternions,
                self._scales,
                self._opacities,
                self._spherical_harmonics,
                torch.from_numpy(view_matrices).to(self._means.device),
                torch.from_numpy(intrinsics).to(self._means.device),
                int(width),
                int(height),
                near_plane=float(near_m),
                far_plane=float(far_m),
                sh_degree=self.sh_degree,
                packed=True,
                render_mode="RGB+ED",
                rasterize_mode="antialiased",
            )
            colors = (
                rendered[..., :3]
                .clamp(0.0, 1.0)
                .mul(255.0)
                .round()
                .to(torch.uint8)
                .cpu()
                .numpy()
            )
            depths = rendered[..., 3].float()
            depths = torch.where(
                (alpha[..., 0] >= 0.01) & torch.isfinite(depths),
                depths,
                torch.full_like(depths, float("inf")),
            )
            depth_array = depths.cpu().numpy()
            torch.cuda.synchronize()
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        return (
            np.ascontiguousarray(colors),
            np.ascontiguousarray(depth_array, dtype=np.float32),
            elapsed_ms,
        )

    def close(self) -> None:
        for name in (
            "_means",
            "_quaternions",
            "_scales",
            "_opacities",
            "_spherical_harmonics",
        ):
            if hasattr(self, name):
                delattr(self, name)

    def __enter__(self) -> "GaussianSplatRenderer":
        return self

    def __exit__(self, *_args) -> None:
        self.close()
