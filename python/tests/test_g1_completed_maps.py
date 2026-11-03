"""Finished-map fidelity, alignment and registration without live hardware."""

import importlib
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parents[1] / 'examples'))
data = importlib.import_module('tools.g1_recording_data')
maps = importlib.import_module('tools.g1_completed_maps')
registration = importlib.import_module('g1_recording_registration')


def test_cache_digest_is_streamed_and_python310_compatible(tmp_path):
    import hashlib
    path = tmp_path / 'mesh.bin'
    content = b'original mesh bytes' * 100000
    path.write_bytes(content)
    assert maps.digest(path) == hashlib.sha256(content).hexdigest()


def test_pages_retain_all_faces_positions_colors_and_winding():
    from horus.mapping.mesh_protocol import decode_live_header, HMSH_V2_HEADER
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [-3, 1, 1]], dtype=np.float32)
    colors = np.array([[255, 0, 3], [0, 255, 4], [0, 0, 255], [8, 9, 10]], dtype=np.uint8)
    faces = np.array([[0, 1, 2], [2, 1, 3], [0, 0, 1]], dtype=np.int32)
    state = maps.mesh_state(vertices, colors, faces, page_triangles=1)
    assert state.manifest()['triangles'] == 3  # Even the source's degenerate face is retained.
    actual = []
    for update in state.snapshot_updates():
        header = decode_live_header(update.payload)
        count = header.vertex_count
        packed = np.frombuffer(update.payload, dtype=[('xyz', '<f4', (3,)), ('rgba', 'u1', (4,))],
                               count=count, offset=HMSH_V2_HEADER.size)
        points, rgba = packed['xyz'], packed['rgba']
        indices = np.frombuffer(update.payload, '<u4', offset=HMSH_V2_HEADER.size+count*16).reshape(-1, 3)
        for face in indices:
            actual.append(tuple((tuple(points[i]), tuple(rgba[i, :3])) for i in face))
    expected = [tuple((tuple(vertices[i]), tuple(colors[i])) for i in face) for face in faces]
    assert sorted(actual) == sorted(expected)
    assert state.repair_updates({'request': 'snapshot'}) == state.snapshot_updates()


def test_occupancy_retains_ros_rows_unknown_cells_and_composes_origin_once(tmp_path):
    pytest.importorskip('nav_msgs.msg')
    cells = np.array([[-1, 0, 100], [25, 50, 75]], dtype=np.int16)
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('xyz', [0.1, -0.2, 0.7]).as_matrix()
    transform[:3, 3] = [3, 4, 5]
    origin = np.r_[[-2, 1, 0], Rotation.from_euler('z', 0.3).as_quat()]
    path = tmp_path/'grid.npz'
    np.savez(path, occupancy=cells, resolution=0.01, origin=origin, world_from_map=transform)
    message = data.load_occupancy(path)
    assert (message.info.width, message.info.height) == (3, 2)
    assert list(message.data) == [-1, 0, 100, 25, 50, 75]
    p, q = message.info.origin.position, message.info.origin.orientation
    actual = np.eye(4)
    actual[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    actual[:3, 3] = [p.x, p.y, p.z]
    local = np.eye(4)
    local[:3, :3] = Rotation.from_quat(origin[3:]).as_matrix()
    local[:3, 3] = origin[:3]
    assert np.allclose(actual, transform @ local)


@pytest.mark.parametrize('bad', [np.full((4, 4), np.nan), np.zeros((4, 4)), np.diag([2, 1, 1, 1])])
def test_invalid_placement_is_rejected(bad):
    with pytest.raises(ValueError, match='transform'):
        data.checked_matrix(bad)


def test_static_registration_has_both_maps_without_control():
    from horus.bridge.robot_registry import RobotRegistryClient
    info = dict(session='first', width=1280, height=720, k=[900, 0, 640, 0, 900, 360, 0, 0, 1],
                map_mode='static', occupancy=True)
    robot, viz = registration.make_registration(info)
    assert robot.get_capabilities()['controllable'] is False
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.node, client.ros_initialized = None, False
    payload = client._build_global_visualizations_payload([viz])
    assert len(payload) == 2
    mesh = next(p for p in payload if p.get('mesh'))
    assert mesh['mesh']['indexed_live'] is True
    assert mesh['mesh']['static_mesh'] is True
    occupancy = next(v for v in viz.visualizations if v.data_source.topic.endswith('/occupancy'))
    assert occupancy.render_options['transport_lane'] == 'bulk_replaceable'


def test_first_live_is_explicitly_unavailable():
    from tools.g1_recording_demo import check
    with pytest.raises(ValueError, match='no saved incremental'):
        check('first', 'live')
