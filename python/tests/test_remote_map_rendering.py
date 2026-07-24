import numpy as np
import struct
import cv2

from horus.remote_rendering.synthetic_scene import (
    DEPTH_FAR_METERS,
    DEPTH_NEAR_METERS,
    build_remote_map_rgbd,
    decode_luma_depth,
    encode_luma_depth,
)
from horus.remote_rendering.real_scene import (
    COW_LADY_VIEW_SPECS,
    ETH3D_COURTYARD_VIEW_SPECS,
    RemoteRenderViewSpec,
    load_colored_vertex_ply,
    prepare_cow_lady_points,
    render_colored_points,
    unproject_multiview_surfels,
    view_spec_camera_pose,
)
from horus.remote_rendering.dynamic_stream import (
    build_dynamic_camera_poses,
    decode_dynamic_depth,
    decode_dynamic_pose_metadata,
    encode_dynamic_depth,
    encode_dynamic_pose_metadata,
    metadata_rows,
    pack_dynamic_rgbd,
)


def test_dynamic_depth_round_trip_uses_16_bit_precision():
    near_m, far_m = 0.2, 60.0
    values = np.linspace(near_m, far_m, 320, dtype=np.float32).reshape(1, 320)
    source = np.repeat(values, 4, axis=1)
    encoded = encode_dynamic_depth(source, near_m=near_m, far_m=far_m)
    decoded = decode_dynamic_depth(encoded, near_m=near_m, far_m=far_m)
    quantization_step = (far_m - near_m) / 65535.0
    assert np.max(np.abs(source - decoded)) <= quantization_step * 1.1
    assert np.array_equal(encoded[..., 0], encoded[..., 1])
    assert np.array_equal(encoded[..., 1], encoded[..., 2])


def test_dynamic_depth_uses_contiguous_codec_friendly_nibble_planes():
    near_m, far_m = 0.2, 60.0
    normalized = 0x1234 / 65535.0
    source = np.full((2, 16), near_m + (far_m - near_m) * normalized, dtype=np.float32)

    encoded = encode_dynamic_depth(source, near_m=near_m, far_m=far_m)
    plane_width = encoded.shape[1] // 4
    expected = np.rint(
        32.0 + np.asarray((1, 2, 3, 4), dtype=np.float32) * (255.0 - 32.0) / 15.0
    ).astype(np.uint8)

    for index, level in enumerate(expected):
        plane = encoded[:, index * plane_width:(index + 1) * plane_width]
        assert np.all(plane == level)


def test_luma_depth_round_trip_is_bounded():
    values = np.linspace(DEPTH_NEAR_METERS, DEPTH_FAR_METERS, 256, dtype=np.float32).reshape(16, 16)
    source = np.repeat(values, 2, axis=1)
    encoded = encode_luma_depth(source)
    decoded = decode_luma_depth(encoded)
    quantization_step = (DEPTH_FAR_METERS - DEPTH_NEAR_METERS) / 255.0
    assert np.max(np.abs(source - decoded)) <= quantization_step * 1.1


def test_luma_depth_uses_equal_channels_for_h264_stability():
    source = np.array([[DEPTH_NEAR_METERS, DEPTH_NEAR_METERS, np.inf, np.inf]], dtype=np.float32)
    encoded = encode_luma_depth(source)
    assert np.array_equal(encoded[..., 0], encoded[..., 1])
    assert np.array_equal(encoded[..., 1], encoded[..., 2])
    assert np.all(encoded[0, 2:] == 0)


def test_native_synthetic_map_has_spatial_content():
    color, depth, packed = build_remote_map_rgbd(320, 180)
    valid = np.isfinite(depth)
    assert color.shape == (180, 320, 3)
    assert depth.shape == (180, 320)
    assert packed.shape == (180, 640, 3)
    assert 0.35 < float(valid.mean()) < 0.9
    assert float(depth[valid].min()) >= DEPTH_NEAR_METERS
    assert float(depth[valid].max()) <= DEPTH_FAR_METERS
    assert np.unique(color[valid].reshape(-1, 3), axis=0).shape[0] > 40
    assert np.count_nonzero(packed[:, 320:]) > 0


def test_binary_colored_ply_loader_preserves_every_vertex(tmp_path):
    path = tmp_path / "scan.ply"
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "element vertex 2\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "property uchar alpha\n"
        "element face 0\n"
        "property list uchar int vertex_indices\n"
        "end_header\n"
    ).encode("ascii")
    path.write_bytes(
        header
        + struct.pack("<fffBBBB", 1.0, 2.0, 3.0, 10, 20, 30, 255)
        + struct.pack("<fffBBBB", -1.0, -2.0, 0.0, 40, 50, 60, 255)
    )
    points, colors = load_colored_vertex_ply(path)
    assert points.shape == (2, 3)
    assert colors.shape == (2, 3)
    assert np.allclose(points[0], (1.0, 2.0, 3.0))
    assert np.array_equal(colors[1], (40, 50, 60))


def test_real_point_cloud_raster_uses_metric_camera_z():
    source = np.array(
        [
            (-1.0, -1.0, 0.0),
            (1.0, -1.0, 0.0),
            (-1.0, 1.0, 2.0),
            (1.0, 1.0, 2.0),
        ],
        dtype=np.float32,
    )
    colors = np.array(
        [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255)],
        dtype=np.uint8,
    )
    prepared = prepare_cow_lady_points(source, world_scale=1.0)
    color, depth = render_colored_points(prepared, colors, 160, 90, point_radius=1)
    assert np.isfinite(depth).any()
    assert np.count_nonzero(color) > 0
    assert float(depth[np.isfinite(depth)].min()) >= DEPTH_NEAR_METERS


def test_multiview_unprojection_uses_unity_bottom_origin_atlas_rects():
    color = np.zeros((4, 4, 3), dtype=np.uint8)
    color[:2] = (220, 20, 10)
    color[2:] = (10, 40, 230)
    depth = np.full((4, 4), np.inf, dtype=np.float32)
    depth[:2] = 8.0
    depth[2:] = 2.0
    views = (
        RemoteRenderViewSpec("bottom", (0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 0.0, 1.0, 0.5),
        RemoteRenderViewSpec("top", (100.0, 0.0, 0.0), 0.0, 0.0, 0.0, 0.5, 1.0, 0.5),
    )

    points, colors = unproject_multiview_surfels(
        color,
        depth,
        views,
        columns=1,
        rows=1,
    )

    assert points.shape == (2, 3)
    assert np.allclose(points[0], (0.0, 0.0, 2.0))
    assert np.allclose(points[1], (100.0, 0.0, 8.0))
    assert np.array_equal(colors[0], (10, 40, 230))
    assert np.array_equal(colors[1], (220, 20, 10))


def test_complete_static_scene_specs_cover_the_full_three_by_three_atlas():
    for views in (COW_LADY_VIEW_SPECS, ETH3D_COURTYARD_VIEW_SPECS):
        assert len(views) == 9
        cells = {
            (
                round(view.atlas_x * 3),
                round(view.atlas_y * 3),
            )
            for view in views
        }
        assert cells == {(column, row) for column in range(3) for row in range(3)}
        assert any(view.name in ("overhead", "interior_high") for view in views)
        assert all(np.isclose(view.atlas_width, 1.0 / 3.0) for view in views)
        assert all(np.isclose(view.atlas_height, 1.0 / 3.0) for view in views)


def test_static_view_pose_faces_toward_the_map_center():
    view = COW_LADY_VIEW_SPECS[0]
    position, rotation = view_spec_camera_pose(view)
    x, y, z, w = rotation
    camera_forward = np.array(
        (
            2.0 * (x * z + y * w),
            2.0 * (y * z - x * w),
            1.0 - 2.0 * (x * x + y * y),
        )
    )
    toward_center = -np.asarray(position)

    assert np.dot(camera_forward, toward_center) > 0.0
    assert camera_forward[1] < 0.0


def test_dynamic_pose_metadata_round_trip_is_precise():
    poses = build_dynamic_camera_poses(
        (12.25, -3.5, 6.75),
        (0.1, -0.2, 0.3, 0.92),
        baseline_m=0.8,
    )
    encoded = np.zeros((180, 960, 3), dtype=np.uint8)

    used_rows = encode_dynamic_pose_metadata(
        encoded,
        poses,
        sequence=70001,
        position_range_m=128.0,
    )
    sequence, decoded = decode_dynamic_pose_metadata(encoded, position_range_m=128.0)

    assert used_rows == metadata_rows(960)
    assert sequence == (70001 & 0xFFFF)
    assert int(encoded[4, 4, 0]) >= 32
    assert len(decoded) == 1
    for source, restored in zip(poses, decoded):
        assert np.allclose(source.position, restored.position, atol=0.004)
        assert abs(float(np.dot(source.rotation, restored.rotation))) > 0.99999


def test_pose_timewarp_contract_has_one_unmodified_server_camera():
    half = np.sqrt(0.5)
    poses = build_dynamic_camera_poses(
        (0.0, 0.0, 0.0),
        (0.0, half, 0.0, half),
        baseline_m=2.0,
    )

    assert np.allclose(poses[0].position, (0.0, 0.0, 0.0), atol=1e-5)
    assert len(poses) == 1
    assert np.allclose(poses[0].rotation, (0.0, half, 0.0, half), atol=1e-5)


def test_pose_metadata_survives_ros_jpeg_transport():
    width, height = 320, 180
    pose = build_dynamic_camera_poses(
        (1.25, 1.7, -2.5),
        (0.0, np.sqrt(0.5), 0.0, np.sqrt(0.5)),
        baseline_m=0.0,
    )
    color = np.full((height, width, 3), (72, 118, 156), dtype=np.uint8)
    depth = np.full((height, width), 4.5, dtype=np.float32)
    packed = pack_dynamic_rgbd(
        color,
        depth,
        pose,
        sequence=418,
        near_m=0.3,
        far_m=20.0,
        position_range_m=128.0,
    )

    ok, encoded = cv2.imencode(
        ".jpg",
        cv2.cvtColor(packed, cv2.COLOR_RGB2BGR),
        [cv2.IMWRITE_JPEG_QUALITY, 95],
    )
    assert ok
    decoded = cv2.cvtColor(cv2.imdecode(encoded, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    sequence, restored = decode_dynamic_pose_metadata(
        decoded[:, width:],
        position_range_m=128.0,
    )

    assert sequence == 418
    assert np.allclose(restored[0].position, pose[0].position, atol=0.01)
    assert abs(float(np.dot(restored[0].rotation, pose[0].rotation))) > 0.9999
