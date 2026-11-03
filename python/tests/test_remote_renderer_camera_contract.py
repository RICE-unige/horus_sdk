import numpy as np
import pytest

from horus.remote_rendering import quaternion_to_matrix
from horus.remote_rendering.cuda_renderer import CudaPointRenderer
from horus.remote_rendering.nvdiffrast_mesh_renderer import (
    NvdiffrastMeshRenderer,
)


WIDTH = 320
HEIGHT = 240
PROJECTION = (1.25, 1.65, 0.08, -0.05)
CAMERA_POSES = (
    (
        np.asarray((0.0, 0.0, 0.0), dtype=np.float32),
        np.asarray((0.0, 0.0, 0.0, 1.0), dtype=np.float32),
    ),
    (
        np.asarray((0.25, -0.12, 0.08), dtype=np.float32),
        np.asarray((0.025, -0.065, 0.015, 0.9974), dtype=np.float32),
    ),
)


def _plane_depth(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    return 5.0 + 0.18 * x - 0.27 * y


def _reconstruct_scene_points(
    depth: np.ndarray,
    position: np.ndarray,
    rotation: np.ndarray,
) -> np.ndarray:
    rows, columns = np.nonzero(np.isfinite(depth))
    metric_depth = depth[rows, columns]
    ndc_x = 2.0 * (columns.astype(np.float32) + 0.5) / WIDTH - 1.0
    # NumPy render targets are top-left origin. The frame protocol reverses
    # their rows before Unity uploads the raw depth texture.
    ndc_y = 1.0 - 2.0 * (rows.astype(np.float32) + 0.5) / HEIGHT
    camera_points = np.column_stack(
        (
            (ndc_x - PROJECTION[2]) * metric_depth / PROJECTION[0],
            (ndc_y - PROJECTION[3]) * metric_depth / PROJECTION[1],
            metric_depth,
        )
    )
    return camera_points @ quaternion_to_matrix(rotation).T + position


def _assert_plane_is_world_locked(
    depth_views: np.ndarray,
) -> None:
    reconstructed = []
    for depth, (position, rotation) in zip(depth_views, CAMERA_POSES):
        points = _reconstruct_scene_points(depth, position, rotation)
        assert len(points) > 500
        residual = points[:, 2] - _plane_depth(points[:, 0], points[:, 1])
        assert float(np.quantile(np.abs(residual), 0.95)) < 0.035
        reconstructed.append(points)

    for points in reconstructed:
        center = np.median(points, axis=0)
        assert np.all(np.isfinite(center))


def _surface_mesh() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    xy = np.asarray(
        (
            (-2.5, -2.0),
            (2.5, -2.0),
            (-2.5, 2.0),
            (2.5, 2.0),
        ),
        dtype=np.float32,
    )
    vertices = np.column_stack((xy, _plane_depth(xy[:, 0], xy[:, 1])))
    faces = np.asarray(((0, 1, 2), (2, 1, 3)), dtype=np.int32)
    colors = np.asarray(((220, 90, 50), (50, 170, 220)), dtype=np.uint8)
    return vertices, faces, colors


def _surface_points() -> tuple[np.ndarray, np.ndarray]:
    x, y = np.meshgrid(
        np.linspace(-2.5, 2.5, 260, dtype=np.float32),
        np.linspace(-2.0, 2.0, 210, dtype=np.float32),
    )
    points = np.column_stack((x.ravel(), y.ravel(), _plane_depth(x, y).ravel()))
    colors = np.empty_like(points, dtype=np.uint8)
    colors[:, 0] = np.clip((points[:, 0] + 2.5) * 45.0, 0.0, 255.0)
    colors[:, 1] = np.clip((points[:, 1] + 2.0) * 55.0, 0.0, 255.0)
    colors[:, 2] = 180
    return np.ascontiguousarray(points), np.ascontiguousarray(colors)


def _render(renderer) -> np.ndarray:
    positions = [pose[0] for pose in CAMERA_POSES]
    rotations = [pose[1] for pose in CAMERA_POSES]
    _, depth, _ = renderer.render_views(
        positions,
        rotations,
        WIDTH,
        HEIGHT,
        vertical_fov_deg=62.0,
        projections=(PROJECTION, PROJECTION),
        near_m=0.1,
        far_m=20.0,
        point_radius=1,
    )
    return depth


def test_nvdiffrast_mesh_depth_matches_quest_camera_contract():
    torch = pytest.importorskip("torch")
    pytest.importorskip("nvdiffrast.torch")
    if not torch.cuda.is_available():
        pytest.skip("CUDA is unavailable")
    vertices, faces, colors = _surface_mesh()
    with NvdiffrastMeshRenderer(vertices, faces, colors) as renderer:
        _assert_plane_is_world_locked(_render(renderer))


def test_stereo_texture_and_topology_cache_matches_uncached_reference(monkeypatch):
    torch = pytest.importorskip("torch")
    dr = pytest.importorskip("nvdiffrast.torch")
    if not torch.cuda.is_available():
        pytest.skip("CUDA is unavailable")
    # Test rendering, not TorchInductor compilation or its filesystem cache.
    monkeypatch.setattr(NvdiffrastMeshRenderer, "_compile_helper", staticmethod(lambda _torch, f: f))
    vertices, faces, colors = _surface_mesh()
    uvs = np.asarray(((0, 0), (1, 0), (0, 1), (1, 1)), dtype=np.float32)
    atlas = np.random.default_rng(7).integers(0, 256, (256, 256, 3), dtype=np.uint8)
    rects = np.tile(np.array((0, 0, 256, 256)), (2, 1))
    mip_calls = []
    hash_calls = []
    make_mip, make_hash = dr.texture_construct_mip, dr.antialias_construct_topology_hash

    def mip(*args, **kwargs):
        mip_calls.append(args[0].shape)
        return make_mip(*args, **kwargs)

    def topology(*args, **kwargs):
        hash_calls.append(args[0].shape)
        return make_hash(*args, **kwargs)

    monkeypatch.setattr(dr, "texture_construct_mip", mip)
    monkeypatch.setattr(dr, "antialias_construct_topology_hash", topology)
    with NvdiffrastMeshRenderer(vertices, faces, colors, uvs=uvs,
                                texture_atlas=atlas, texture_rects=rects) as renderer:
        params = dict(vertical_fov_deg=62, projections=(PROJECTION, PROJECTION), near_m=0.1, far_m=20)
        positions, rotations = zip(*CAMERA_POSES)
        first, depth, _ = renderer.render_views(positions, rotations, WIDTH, HEIGHT, **params)
        second, _, _ = renderer.render_views(positions, rotations, WIDTH, HEIGHT, **params)
        assert renderer._texture.shape[0] == 1
        assert len(mip_calls) == len(hash_calls) == 1
        assert np.array_equal(first, second)
        original_texture, original_antialias = dr.texture, dr.antialias

        def uncached_texture(texture, uv, *args, **kwargs):
            kwargs.pop("mip", None)
            return original_texture(texture.expand(uv.shape[0], -1, -1, -1).contiguous(), uv, *args, **kwargs)

        def uncached_antialias(*args, **kwargs):
            kwargs.pop("topology_hash", None)
            return original_antialias(*args, **kwargs)

        monkeypatch.setattr(dr, "texture", uncached_texture)
        monkeypatch.setattr(dr, "antialias", uncached_antialias)
        reference, reference_depth, _ = renderer.render_views(positions, rotations, WIDTH, HEIGHT, **params)
        assert np.array_equal(first, reference)
        assert np.array_equal(depth, reference_depth)


def test_cuda_point_depth_matches_quest_camera_contract():
    torch = pytest.importorskip("torch")
    if not torch.cuda.is_available():
        pytest.skip("CUDA is unavailable")
    points, colors = _surface_points()
    with CudaPointRenderer(len(points)) as renderer:
        renderer.upload(points, colors)
        _assert_plane_is_world_locked(_render(renderer))


def test_cuda_point_device_outputs_match_host_and_survive_later_frames():
    torch = pytest.importorskip("torch")
    if not torch.cuda.is_available():
        pytest.skip("CUDA is unavailable")
    from horus.remote_rendering.v3_renderer import encode_robust_depth_tensor

    points, colors = _surface_points()
    positions, rotations = zip(*CAMERA_POSES)
    params = dict(vertical_fov_deg=62, projections=(PROJECTION, PROJECTION),
                  near_m=0.1, far_m=20, point_radius=1)
    stream = torch.cuda.Stream()
    with CudaPointRenderer(len(points)) as renderer:
        renderer.upload(points, colors)
        host_color, host_depth, _ = renderer.render_views(
            positions, rotations, WIDTH, HEIGHT, **params)
        with torch.cuda.stream(stream):
            device_color, device_depth, _ = renderer._render_views(
                positions, rotations, WIDTH, HEIGHT, device_outputs=True, **params)
            video_color, video_depth, _ = renderer.render_video_views_cuda(
                positions, rotations, WIDTH, HEIGHT, visibility_mask_height=60, **params)
            expected_depth = encode_robust_depth_tensor(
                torch, torch.as_tensor(host_depth, device="cuda"), 60, 0.1, 20)
        stream.synchronize()
        assert device_color.is_cuda and device_depth.is_cuda
        assert video_color.is_cuda and video_depth.is_cuda
        assert np.array_equal(device_color.cpu().numpy(), host_color)
        assert np.array_equal(device_depth.cpu().numpy(), host_depth)
        assert np.array_equal(video_color.cpu().numpy(), host_color)
        assert torch.equal(video_depth, expected_depth)
        saved_color, saved_depth = video_color.clone(), video_depth.clone()
        # Reallocate native scratch at a different size, then destroy it. Outputs
        # already submitted to an asynchronous encoder must remain owned and intact.
        renderer.render_views(positions, rotations, WIDTH * 2, HEIGHT * 2, **params)
    assert torch.equal(video_color, saved_color)
    assert torch.equal(video_depth, saved_depth)


FLOOR_POSITION = np.asarray((0.0, 2.0, -3.0), dtype=np.float32)
_FLOOR_HALF_PITCH = np.deg2rad(15.0)
FLOOR_ROTATION = np.asarray(
    (
        np.sin(_FLOOR_HALF_PITCH),
        0.0,
        0.0,
        np.cos(_FLOOR_HALF_PITCH),
    ),
    dtype=np.float32,
)
FLOOR_PROJECTION = (1.15, 1.55, 0.0, 0.0)


def _assert_reconstructed_floor_is_horizontal(renderer) -> None:
    _, depth_views, _ = renderer.render_views(
        [FLOOR_POSITION],
        [FLOOR_ROTATION],
        WIDTH,
        HEIGHT,
        vertical_fov_deg=70.0,
        projections=(FLOOR_PROJECTION,),
        near_m=0.1,
        far_m=30.0,
        point_radius=1,
    )
    depth = depth_views[0]
    rows, columns = np.nonzero(np.isfinite(depth))
    assert len(rows) > 500
    metric_depth = depth[rows, columns]
    ndc_x = 2.0 * (columns.astype(np.float32) + 0.5) / WIDTH - 1.0
    ndc_y = 1.0 - 2.0 * (rows.astype(np.float32) + 0.5) / HEIGHT
    camera_points = np.column_stack(
        (
            (ndc_x - FLOOR_PROJECTION[2])
            * metric_depth
            / FLOOR_PROJECTION[0],
            (ndc_y - FLOOR_PROJECTION[3])
            * metric_depth
            / FLOOR_PROJECTION[1],
            metric_depth,
        )
    )
    reconstructed = (
        camera_points @ quaternion_to_matrix(FLOOR_ROTATION).T
        + FLOOR_POSITION
    )
    design = np.column_stack(
        (
            reconstructed[:, 0],
            reconstructed[:, 2],
            np.ones(len(reconstructed), dtype=np.float32),
        )
    )
    slope_x, slope_z, offset = np.linalg.lstsq(
        design,
        reconstructed[:, 1],
        rcond=None,
    )[0]
    residual = reconstructed[:, 1] - design @ (slope_x, slope_z, offset)
    # Point splats choose one source sample for several neighbouring pixels,
    # which introduces bounded depth quantization. The fitted floor must still
    # be horizontal and tightly planar.
    assert abs(float(slope_x)) < 0.01
    assert abs(float(slope_z)) < 0.01
    assert abs(float(offset)) < 0.1
    assert float(np.quantile(np.abs(residual), 0.95)) < 0.03


def _horizontal_floor_mesh() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    vertices = np.asarray(
        (
            (-6.0, 0.0, -1.0),
            (6.0, 0.0, -1.0),
            (-6.0, 0.0, 12.0),
            (6.0, 0.0, 12.0),
        ),
        dtype=np.float32,
    )
    faces = np.asarray(((0, 2, 1), (1, 2, 3)), dtype=np.int32)
    colors = np.asarray(((180, 180, 180), (180, 180, 180)), dtype=np.uint8)
    return vertices, faces, colors


def _horizontal_floor_points() -> tuple[np.ndarray, np.ndarray]:
    x, z = np.meshgrid(
        np.linspace(-6.0, 6.0, 420, dtype=np.float32),
        np.linspace(-1.0, 12.0, 420, dtype=np.float32),
    )
    points = np.column_stack(
        (x.ravel(), np.zeros(x.size, dtype=np.float32), z.ravel())
    )
    colors = np.full(points.shape, 180, dtype=np.uint8)
    return np.ascontiguousarray(points), colors


def test_nvdiffrast_keeps_map_floor_horizontal_under_tilted_viewer_pose():
    torch = pytest.importorskip("torch")
    pytest.importorskip("nvdiffrast.torch")
    if not torch.cuda.is_available():
        pytest.skip("CUDA is unavailable")
    vertices, faces, colors = _horizontal_floor_mesh()
    with NvdiffrastMeshRenderer(vertices, faces, colors) as renderer:
        _assert_reconstructed_floor_is_horizontal(renderer)


def test_cuda_points_keep_map_floor_horizontal_under_tilted_viewer_pose():
    torch = pytest.importorskip("torch")
    if not torch.cuda.is_available():
        pytest.skip("CUDA is unavailable")
    points, colors = _horizontal_floor_points()
    with CudaPointRenderer(len(points)) as renderer:
        renderer.upload(points, colors)
        _assert_reconstructed_floor_is_horizontal(renderer)
