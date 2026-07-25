from pathlib import Path

import numpy as np
import pytest

from horus.remote_rendering.gaussian_renderer import (
    load_gaussian_splat_ply,
    resolve_gaussian_splat,
    resolve_playroom_splat,
)
from horus.remote_rendering.scene_catalog import (
    ADVANCED_REMOTE_SCENE_IDS,
    GAUSSIAN_SCENE_IDS,
    MESH_SCENE_IDS,
)
from horus.remote_rendering.textured_mesh_scene import (
    _build_texture_atlas,
    _material_texture_and_tint,
    resolve_sponza_gltf,
    resolve_textured_mesh,
)


def test_texture_atlas_preserves_images_and_pads_their_edges():
    red = np.full((3, 5, 3), (255, 0, 0), dtype=np.uint8)
    green = np.full((2, 4, 3), (0, 255, 0), dtype=np.uint8)

    atlas, rectangles = _build_texture_atlas({"red": red, "green": green})

    for key, source in (("red", red), ("green", green)):
        x, y, width, height = rectangles[key]
        assert np.array_equal(atlas[y : y + height, x : x + width], source)
        assert np.array_equal(atlas[y - 1, x : x + width], source[0])
        assert np.array_equal(atlas[y + height, x : x + width], source[-1])
    assert atlas.shape[0] % 16 == 0
    assert atlas.shape[1] % 16 == 0


def test_texture_atlas_does_not_allocate_every_cell_at_largest_texture_size():
    textures = {
        "large": np.zeros((256, 256, 3), dtype=np.uint8),
        **{
            f"small_{index}": np.zeros((16, 16, 3), dtype=np.uint8)
            for index in range(15)
        },
    }

    atlas, _ = _build_texture_atlas(textures)
    fixed_grid_area = 4 * 4 * (256 + 4) * (256 + 4)

    assert atlas.shape[0] * atlas.shape[1] < fixed_grid_area * 0.4


def test_obj_material_image_and_diffuse_are_preserved():
    class ObjMaterial:
        image = np.full((8, 6, 3), (80, 120, 160), dtype=np.uint8)
        diffuse = (128, 64, 255, 255)

    pixels, tint = _material_texture_and_tint(ObjMaterial())

    assert np.array_equal(pixels, ObjMaterial.image)
    assert np.array_equal(tint, (128, 64, 255))


def test_gaussian_loader_preserves_primitives_and_decodes_3dgs_fields(tmp_path):
    plyfile = pytest.importorskip("plyfile")
    vertex = np.zeros(
        2,
        dtype=[
            ("x", "f4"),
            ("y", "f4"),
            ("z", "f4"),
            ("f_dc_0", "f4"),
            ("f_dc_1", "f4"),
            ("f_dc_2", "f4"),
            ("opacity", "f4"),
            ("scale_0", "f4"),
            ("scale_1", "f4"),
            ("scale_2", "f4"),
            ("rot_0", "f4"),
            ("rot_1", "f4"),
            ("rot_2", "f4"),
            ("rot_3", "f4"),
        ],
    )
    vertex["x"] = (1.0, 2.0)
    vertex["y"] = (2.0, 3.0)
    vertex["z"] = (3.0, 4.0)
    vertex["opacity"] = (0.0, np.log(3.0))
    vertex["scale_0"] = np.log((0.5, 1.0))
    vertex["scale_1"] = np.log((0.25, 2.0))
    vertex["scale_2"] = np.log((0.125, 3.0))
    vertex["rot_0"] = (2.0, 1.0)
    vertex["rot_1"] = (0.0, 1.0)

    path = tmp_path / "fixture.ply"
    plyfile.PlyData([plyfile.PlyElement.describe(vertex, "vertex")]).write(path)
    scene = load_gaussian_splat_ply(path, world_scale=2.0)

    assert scene.gaussian_count == 2
    assert scene.sh_degree == 0
    assert scene.spherical_harmonics.shape == (2, 1, 3)
    assert np.allclose(scene.means[0], (2.0, 4.0, 6.0))
    assert np.allclose(scene.scales[0], (1.0, 0.5, 0.25))
    assert np.allclose(scene.opacities, (0.5, 0.75))
    assert np.allclose(np.linalg.norm(scene.quaternions, axis=1), 1.0)
    assert np.all(np.isfinite(scene.world_bounds_min))
    assert np.all(np.isfinite(scene.world_bounds_max))


def test_gaussian_y_down_scene_has_a_stable_y_up_workspace_transform(tmp_path):
    plyfile = pytest.importorskip("plyfile")
    vertex = np.zeros(
        4,
        dtype=[
            ("x", "f4"),
            ("y", "f4"),
            ("z", "f4"),
            ("f_dc_0", "f4"),
            ("f_dc_1", "f4"),
            ("f_dc_2", "f4"),
            ("opacity", "f4"),
            ("scale_0", "f4"),
            ("scale_1", "f4"),
            ("scale_2", "f4"),
            ("rot_0", "f4"),
            ("rot_1", "f4"),
            ("rot_2", "f4"),
            ("rot_3", "f4"),
        ],
    )
    vertex["x"] = (-2.0, -1.0, 1.0, 2.0)
    vertex["y"] = (-3.0, 1.0, 4.0, 5.0)
    vertex["z"] = (-4.0, -1.0, 1.0, 4.0)
    vertex["rot_0"] = 1.0

    path = tmp_path / "y_down_fixture.ply"
    plyfile.PlyData([plyfile.PlyElement.describe(vertex, "vertex")]).write(path)
    scene = load_gaussian_splat_ply(
        path,
        world_scale=2.0,
        canonicalize_y_down=True,
        floor_quantile=1.0,
    )

    source_floor_origin = scene.transform.source_from_world_translation
    world_position, world_rotation = scene.transform.source_pose_to_world(
        source_floor_origin,
        (0.0, 0.0, 1.0, 0.0),
    )
    assert np.allclose(world_position, (0.0, 0.0, 0.0), atol=1e-6)
    assert abs(float(np.dot(world_rotation, (0.0, 0.0, 0.0, 1.0)))) > 0.99999

    source_position, source_rotation = scene.transform.world_pose_to_source(
        (1.5, 2.0, -0.75),
        (0.0, 0.0, 0.0, 1.0),
    )
    round_trip_position, round_trip_rotation = (
        scene.transform.source_pose_to_world(source_position, source_rotation)
    )
    assert np.allclose(round_trip_position, (1.5, 2.0, -0.75), atol=1e-6)
    assert abs(float(np.dot(round_trip_rotation, (0.0, 0.0, 0.0, 1.0)))) > 0.99999


def test_advanced_scene_catalog_has_unique_mesh_and_gaussian_ids():
    assert set(MESH_SCENE_IDS).isdisjoint(GAUSSIAN_SCENE_IDS)
    assert set(ADVANCED_REMOTE_SCENE_IDS) == {
        *MESH_SCENE_IDS,
        *GAUSSIAN_SCENE_IDS,
    }


@pytest.mark.parametrize("scene_id", MESH_SCENE_IDS)
def test_mesh_resolver_reports_the_scene_fetch_command(
    tmp_path: Path,
    scene_id: str,
):
    with pytest.raises(FileNotFoundError, match=f"--scene {scene_id}"):
        resolve_textured_mesh(scene_id, tmp_path / "missing")


@pytest.mark.parametrize("scene_id", GAUSSIAN_SCENE_IDS)
def test_gaussian_resolver_reports_the_scene_fetch_command(
    tmp_path: Path,
    scene_id: str,
):
    with pytest.raises(FileNotFoundError, match=f"--scene {scene_id}"):
        resolve_gaussian_splat(scene_id, tmp_path / "missing")


def test_legacy_advanced_resolvers_keep_existing_examples_compatible(tmp_path: Path):
    with pytest.raises(FileNotFoundError, match="--scene sponza_mesh"):
        resolve_sponza_gltf(tmp_path / "missing")
    with pytest.raises(FileNotFoundError, match="--scene gaussian_playroom"):
        resolve_playroom_splat(tmp_path / "missing")
