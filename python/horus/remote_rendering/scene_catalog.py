"""Canonical scene metadata for HORUS PC-side remote rendering."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class RemoteRenderScene:
    scene_id: str
    label: str
    kind: str
    cache_path: Path
    source_url: str
    license_name: str
    canonicalize_y_down: bool = False


_CACHE_ROOT = Path.home() / ".cache" / "horus"
_VOXEL51_SOURCE = "https://huggingface.co/datasets/Voxel51/gaussian_splatting"
_VOXEL51_ROOT = _CACHE_ROOT / "gaussian_splatting" / "prebuilt"

GAUSSIAN_SCENES = {
    scene.scene_id: scene
    for scene in (
        RemoteRenderScene(
            "gaussian_drjohnson",
            "Deep Blending Dr Johnson",
            "gaussian_splat",
            _VOXEL51_ROOT / "drjohnson" / "drjohnson_30000.ply",
            _VOXEL51_SOURCE,
            "Apache-2.0",
            True,
        ),
        RemoteRenderScene(
            "gaussian_playroom",
            "Deep Blending Playroom",
            "gaussian_splat",
            _VOXEL51_ROOT / "playroom" / "playroom_30000.ply",
            _VOXEL51_SOURCE,
            "Apache-2.0",
            True,
        ),
        RemoteRenderScene(
            "gaussian_train",
            "Tanks and Temples Train",
            "gaussian_splat",
            _VOXEL51_ROOT / "train" / "train_30000.ply",
            _VOXEL51_SOURCE,
            "Apache-2.0",
            True,
        ),
        RemoteRenderScene(
            "gaussian_truck",
            "Tanks and Temples Truck",
            "gaussian_splat",
            _VOXEL51_ROOT / "truck" / "truck_30000.ply",
            _VOXEL51_SOURCE,
            "Apache-2.0",
            True,
        ),
    )
}

MESH_SCENES = {
    scene.scene_id: scene
    for scene in (
        RemoteRenderScene(
            "sponza_mesh",
            "Sponza",
            "textured_triangle_mesh",
            _CACHE_ROOT
            / "remote_render_assets"
            / "mesh"
            / "sponza"
            / "Sponza.gltf",
            "https://github.com/KhronosGroup/glTF-Sample-Assets/tree/main/Models/Sponza",
            "See the asset's bundled LICENSE.md",
        ),
        RemoteRenderScene(
            "san_miguel_mesh",
            "San Miguel 2.0",
            "textured_triangle_mesh",
            _CACHE_ROOT / "remote_render_assets" / "mesh" / "san_miguel",
            "https://casual-effects.com/data",
            "See the asset's bundled license",
        ),
    )
}

ADVANCED_REMOTE_SCENES = {**MESH_SCENES, **GAUSSIAN_SCENES}
ADVANCED_REMOTE_SCENE_IDS = tuple(ADVANCED_REMOTE_SCENES)
GAUSSIAN_SCENE_IDS = tuple(GAUSSIAN_SCENES)
MESH_SCENE_IDS = tuple(MESH_SCENES)


def get_advanced_scene(scene_id: str) -> RemoteRenderScene:
    try:
        return ADVANCED_REMOTE_SCENES[scene_id]
    except KeyError as exception:
        choices = ", ".join(ADVANCED_REMOTE_SCENE_IDS)
        raise ValueError(
            f"unknown advanced remote-render scene '{scene_id}'; choose {choices}"
        ) from exception
