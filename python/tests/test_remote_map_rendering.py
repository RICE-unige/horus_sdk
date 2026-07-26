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
    iter_eth3d_remote_scene_lod_chunks,
    load_colored_vertex_ply,
    iter_eth3d_remote_scene_chunks,
    prepare_eth3d_remote_scene,
    prepare_eth3d_remote_scene_lod,
    prepare_cow_lady_points,
    render_colored_points,
    unproject_multiview_surfels,
    view_spec_camera_pose,
)
from horus.remote_rendering.dynamic_stream import (
    DynamicCameraPose,
    build_dynamic_camera_poses,
    decode_dynamic_depth,
    decode_dynamic_pose_metadata,
    encode_dynamic_depth,
    encode_dynamic_pose_metadata,
    metadata_rows,
    pack_dynamic_rgbd,
)
from horus.remote_rendering.frame_protocol import (
    FRAME_MARKER_HEIGHT,
    FRAME_MARKER_WIDTH,
    decode_frame_marker,
    dequantize_depth,
    downsample_depth_for_reprojection,
    downsample_depth_for_surfels,
    downsample_depth_min,
    embed_frame_marker,
    expand_projection,
    pack_remote_frame,
    pack_ros_debug_frame,
    pack_stereo_remote_frame,
    projection_from_vertical_fov,
    unpack_remote_frame,
    unpack_ros_debug_frame,
    unpack_stereo_remote_frame,
)


def _projection_tangents(projection):
    projection_x, projection_y, offset_x, offset_y = projection
    return (
        (offset_x - 1.0) / projection_x,
        (offset_x + 1.0) / projection_x,
        (offset_y - 1.0) / projection_y,
        (offset_y + 1.0) / projection_y,
    )


def test_projection_guard_band_expands_each_asymmetric_frustum_edge():
    source = (1.42, 1.73, 0.08, -0.04)
    expanded = expand_projection(
        source,
        guard_band_degrees=4.0,
        minimum_vertical_fov_deg=45.0,
    )
    source_left, source_right, source_bottom, source_top = _projection_tangents(
        source
    )
    left, right, bottom, top = _projection_tangents(expanded)

    assert np.arctan(left) < np.arctan(source_left)
    assert np.arctan(right) > np.arctan(source_right)
    assert np.arctan(bottom) < np.arctan(source_bottom)
    assert np.arctan(top) > np.arctan(source_top)
    assert not np.isclose(expanded[2], source[2] * expanded[0] / source[0])


def test_projection_guard_band_preserves_zero_guard_projection():
    source = (1.42, 1.73, 0.08, -0.04)
    vertical_fov = np.rad2deg(
        np.arctan(_projection_tangents(source)[3])
        - np.arctan(_projection_tangents(source)[2])
    )
    expanded = expand_projection(
        source,
        guard_band_degrees=0.0,
        minimum_vertical_fov_deg=vertical_fov,
    )

    assert np.allclose(expanded, source, atol=1e-6)


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


def test_eth3d_remote_scene_uses_alignment_and_centers_full_scan(tmp_path):
    scan_path = tmp_path / "scan.ply"
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
        "end_header\n"
    ).encode("ascii")
    scan_path.write_bytes(
        header
        + struct.pack("<fffBBB", 0.0, 0.0, 0.0, 10, 20, 30)
        + struct.pack("<fffBBB", 2.0, 4.0, 1.0, 40, 50, 60)
    )
    (tmp_path / "scan_alignment.mlp").write_text(
        """<MeshLabProject><MeshGroup>
<MLMesh label="scan.ply" filename="scan.ply"><MLMatrix44>
1 0 0 10
0 1 0 -2
0 0 1 3
0 0 0 1
</MLMatrix44></MLMesh>
</MeshGroup></MeshLabProject>""",
        encoding="utf-8",
    )

    scene = prepare_eth3d_remote_scene("delivery_area", tmp_path)
    chunks = list(iter_eth3d_remote_scene_chunks(scene, chunk_size=10_000))
    points, colors = chunks[0]

    assert scene.point_count == 2
    assert np.allclose(points.mean(axis=0), (0.0, 3.5, 0.0))
    assert np.isclose(points[:, 1].min(), 0.0)
    assert np.isclose(np.ptp(points[:, 2]), 28.0)
    assert np.array_equal(colors, ((10, 20, 30), (40, 50, 60)))

    lod_path, lod_count = prepare_eth3d_remote_scene_lod(
        scene,
        voxel_size_m=0.25,
    )
    cached_path, cached_count = prepare_eth3d_remote_scene_lod(
        scene,
        voxel_size_m=0.25,
    )
    lod_chunks = list(iter_eth3d_remote_scene_lod_chunks(lod_path))
    assert lod_count == 2
    assert cached_path == lod_path
    assert cached_count == lod_count
    assert np.allclose(lod_chunks[0][0], points)
    assert np.array_equal(lod_chunks[0][1], colors)


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


def test_remote_frame_contract_preserves_lossless_depth_codes_and_metadata():
    near_m, far_m = 0.2, 80.0
    depth = np.linspace(near_m, far_m, 320 * 180, dtype=np.float32).reshape(180, 320)
    depth[0, 0] = np.inf
    projection = projection_from_vertical_fov(104.0, 16.0 / 9.0)

    encoded = pack_remote_frame(
        depth,
        sequence=0x12345678,
        color_width=960,
        color_height=540,
        near_m=near_m,
        far_m=far_m,
        projection=projection,
        position=(1.25, -2.5, 3.75),
        rotation=(0.0, np.sqrt(0.5), 0.0, np.sqrt(0.5)),
        capture_time_ns=123456789,
    )
    restored = unpack_remote_frame(encoded)
    restored_depth = dequantize_depth(
        restored.depth_codes,
        near_m=restored.near_m,
        far_m=restored.far_m,
    )

    assert encoded[80] == 2
    assert len(encoded) < depth.nbytes
    assert restored.sequence == 0x12345678
    assert restored.capture_time_ns == 123456789
    assert (restored.color_width, restored.color_height) == (960, 540)
    assert (restored.depth_width, restored.depth_height) == (320, 180)
    assert np.isinf(restored_depth[0, 0])
    valid = np.isfinite(depth)
    quantization_step = (far_m - near_m) / 65534.0
    assert np.max(np.abs(depth[valid] - restored_depth[valid])) <= quantization_step * 1.1
    assert np.allclose(restored.position, (1.25, -2.5, 3.75))
    assert np.allclose(restored.rotation, (0.0, np.sqrt(0.5), 0.0, np.sqrt(0.5)))
    assert np.allclose(
        (
            restored.projection_x,
            restored.projection_y,
            restored.projection_offset_x,
            restored.projection_offset_y,
        ),
        projection,
    )


def test_remote_frame_contract_rejects_corrupted_depth():
    depth = np.full((18, 32), 3.0, dtype=np.float32)
    encoded = bytearray(
        pack_remote_frame(
            depth,
            sequence=7,
            color_width=320,
            color_height=180,
            near_m=0.2,
            far_m=10.0,
            projection=projection_from_vertical_fov(100.0, 16.0 / 9.0),
            position=(0.0, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
        )
    )
    encoded[-1] ^= 0x01

    with np.testing.assert_raises_regex(ValueError, "checksum"):
        unpack_remote_frame(encoded)


def test_stereo_remote_frame_preserves_two_eye_depth_and_metadata():
    near_m, far_m = 0.2, 40.0
    left = np.linspace(near_m, 10.0, 24, dtype=np.float32).reshape(4, 6)
    right = np.linspace(12.0, far_m, 24, dtype=np.float32).reshape(4, 6)
    left[0, 0] = np.inf
    depth = np.stack((left, right))
    projections = (
        (0.91, 1.62, 0.03, -0.01),
        (0.92, 1.63, -0.03, -0.01),
    )
    positions = ((-0.032, 1.7, 0.0), (0.032, 1.7, 0.0))
    rotations = (
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0, 1.0),
    )

    restored = unpack_stereo_remote_frame(
        pack_stereo_remote_frame(
            depth,
            sequence=73,
            color_width=640,
            color_height=360,
            near_m=near_m,
            far_m=far_m,
            projections=projections,
            positions=positions,
            rotations=rotations,
            capture_time_ns=987654321,
        )
    )
    restored_depth = dequantize_depth(
        restored.depth_codes,
        near_m=restored.near_m,
        far_m=restored.far_m,
    )

    assert restored.sequence == 73
    assert restored.capture_time_ns == 987654321
    assert restored.depth_codes.shape == (2, 4, 6)
    assert np.isinf(restored_depth[0, 0, 0])
    valid = np.isfinite(depth)
    quantization_step = (far_m - near_m) / 65534.0
    assert np.max(np.abs(depth[valid] - restored_depth[valid])) <= quantization_step * 1.1
    assert np.allclose(restored.projections, projections)
    assert np.allclose(restored.positions, positions)
    assert np.allclose(restored.rotations, rotations)


def test_ros_debug_frame_bundles_matching_jpeg_and_depth_metadata():
    jpeg = b"\xff\xd8diagnostic-jpeg\xff\xd9"
    frame_data = pack_remote_frame(
        np.full((18, 32), 3.0, dtype=np.float32),
        sequence=41,
        color_width=640,
        color_height=360,
        near_m=0.2,
        far_m=10.0,
        projection=projection_from_vertical_fov(100.0, 16.0 / 9.0),
        position=(1.0, 2.0, 3.0),
        rotation=(0.0, 0.0, 0.0, 1.0),
    )

    restored_jpeg, restored_frame_data = unpack_ros_debug_frame(
        pack_ros_debug_frame(jpeg, frame_data)
    )

    assert restored_jpeg == jpeg
    assert unpack_remote_frame(restored_frame_data).sequence == 41


def test_depth_downsampling_uses_nearest_finite_foreground():
    depth = np.full((6, 6), np.inf, dtype=np.float32)
    depth[0:3, 0:3] = 4.0
    depth[1, 1] = 1.5
    depth[3:6, 3:6] = 7.0

    reduced = downsample_depth_min(depth, width=2, height=2)

    assert reduced.shape == (2, 2)
    assert reduced[0, 0] == 1.5
    assert np.isinf(reduced[0, 1])
    assert np.isinf(reduced[1, 0])
    assert reduced[1, 1] == 7.0


def test_reprojection_depth_invalidates_far_side_of_surface_discontinuity():
    depth = np.full((6, 8), 4.0, dtype=np.float32)
    depth[:, 4:] = 12.0

    reduced = downsample_depth_for_reprojection(depth, width=4, height=3)

    assert np.isfinite(reduced[:, :2]).all()
    assert np.isinf(reduced[:, 2]).all()
    assert np.all(reduced[:, 3] == 12.0)


def test_reprojection_depth_supports_non_integer_scaling_without_false_edges():
    rows = np.linspace(2.0, 2.05, 7, dtype=np.float32)[:, None]
    columns = np.linspace(0.0, 0.05, 9, dtype=np.float32)[None, :]
    depth = rows + columns

    reduced = downsample_depth_for_reprojection(depth, width=4, height=3)

    assert reduced.shape == (3, 4)
    assert np.isfinite(reduced).all()


def test_surfel_depth_preserves_both_sides_of_surface_discontinuity():
    depth = np.full((6, 8), 4.0, dtype=np.float32)
    depth[:, 4:] = 12.0

    reduced = downsample_depth_for_surfels(depth, width=4, height=3)

    assert reduced.shape == (3, 4)
    assert np.all(reduced[:, :2] == 4.0)
    assert np.all(reduced[:, 2:] == 12.0)


def test_frame_marker_survives_jpeg_like_color_loss():
    color = np.full((180, 640, 3), (84, 128, 172), dtype=np.uint8)
    embed_frame_marker(color, 0xBEEF)
    assert color.shape[1] >= FRAME_MARKER_WIDTH
    assert color.shape[0] >= FRAME_MARKER_HEIGHT
    np.testing.assert_array_equal(
        color[:FRAME_MARKER_HEIGHT, :FRAME_MARKER_WIDTH],
        color[-FRAME_MARKER_HEIGHT:, :FRAME_MARKER_WIDTH],
    )

    ok, encoded = cv2.imencode(
        ".jpg",
        cv2.cvtColor(color, cv2.COLOR_RGB2BGR),
        [cv2.IMWRITE_JPEG_QUALITY, 35],
    )
    assert ok
    decoded = cv2.cvtColor(cv2.imdecode(encoded, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)

    assert decode_frame_marker(decoded) == 0xBEEF
