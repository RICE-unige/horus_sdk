"""Opt-in exact geometry and recorded-frame checks on the local G1 bundles."""

from collections import Counter
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

pytestmark = pytest.mark.skipif(os.environ.get('HORUS_G1_RECORDING_INTEGRATION') != '1',
    reason='Requires the two local physical G1 recording bundles')
sys.path.insert(0, str(Path(__file__).parents[2] / 'examples'))


def test_fixed_camera_mount_matches_every_original_first_walk_pose():
    from g1_recording_registration import URDF
    from tools.g1_recording_data import bag_path, camera_mount, first_display
    rows = first_display(bag_path('first'), URDF)
    parent, mount, _ = camera_mount(bag_path('first'), 'first', URDF)
    _, later_mount, _ = camera_mount(bag_path('later'), 'later', URDF)
    np.testing.assert_allclose(later_mount, mount, atol=1e-12)
    joints = {j.find('child').get('link'): j for j in ET.parse(URDF).getroot().findall('joint')}
    chain = []
    while parent != 'pelvis':
        joint = joints[parent]
        chain.insert(0, joint)
        parent = joint.find('parent').get('link')
    for index, positions in enumerate(rows['joint_positions']):
        angles = dict(zip(rows['joint_names'], positions))
        result = rows['world_from_pelvis'][index].copy()
        for joint in chain:
            origin = joint.find('origin')
            local = np.eye(4)
            if origin is not None:
                local[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                local[:3, :3] = Rotation.from_euler('xyz',
                    np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            if joint.get('type') != 'fixed':
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                local[:3, :3] = local[:3, :3] @ Rotation.from_rotvec(
                    axis * angles[joint.get('name')]).as_matrix()
            result = result @ local
        np.testing.assert_allclose(result @ mount, rows['world_from_camera'][index], atol=1e-6)


@pytest.mark.parametrize('dataset', ['first', 'later'])
def test_every_saved_face_retains_position_winding_and_rgb(dataset):
    from tools.g1_completed_maps import mesh_arrays, mesh_state
    from horus.mapping.mesh_protocol import decode_live_header, HMSH_V2_HEADER

    vertices, colors, faces = mesh_arrays(dataset)
    vertex_type = np.dtype([('xyz', '<f4', (3,)), ('rgba', 'u1', (4,))])
    original = np.empty(len(vertices), dtype=vertex_type)
    original['xyz'], original['rgba'][:, :3], original['rgba'][:, 3] = vertices, colors, 255
    expected = Counter(row.tobytes() for row in original[faces])
    actual = Counter()
    state = mesh_state(vertices, colors, faces)
    for update in state.snapshot_updates():
        header = decode_live_header(update.payload)
        packed = np.frombuffer(update.payload, vertex_type, count=header.vertex_count,
                               offset=HMSH_V2_HEADER.size)
        indices = np.frombuffer(update.payload, '<u4',
            offset=HMSH_V2_HEADER.size+header.vertex_count*16).reshape(-1, 3)
        actual.update(row.tobytes() for row in packed[indices])
    assert actual == expected
    assert state.manifest()['triangles'] == len(faces)


def test_first_scan_robot_and_camera_use_the_recorded_map_gauge():
    from g1_recording_registration import URDF
    from tools.g1_recording_data import bag_path, first_display, read_first
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from tf2_msgs.msg import TFMessage

    path = bag_path('first')
    rows = first_display(path, URDF)
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(path), storage_id='mcap'), ConverterOptions('', ''))
    reader.set_filter(StorageFilter(topics=['/pointcloud', '/tf', '/tf_static']))
    transforms, source_scan = {}, None
    while reader.has_next() and (source_scan is None or len(transforms) < 2):
        topic, payload, _ = reader.read_next()
        if topic == '/pointcloud':
            if source_scan is None:
                source_scan = deserialize_message(payload, PointCloud2)
        else:
            for item in deserialize_message(payload, TFMessage).transforms:
                t, q = item.transform.translation, item.transform.rotation
                matrix = np.eye(4)
                matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                matrix[:3, 3] = [t.x, t.y, t.z]
                transforms.setdefault((item.header.frame_id, item.child_frame_id), matrix)
    sensor = transforms[('g1_map', 'g1_sensor')]
    camera = sensor @ transforms[('g1_sensor', 'g1_rgb')]
    np.testing.assert_allclose(sensor, rows['world_from_sensor'][0], atol=1e-6)
    np.testing.assert_allclose(camera, rows['world_from_camera'][0], atol=1e-6)

    generated = {}
    for key, message, _ in read_first(path, URDF):
        generated.setdefault(key, message)
        if {'pelvis', 'camera_pose', 'scan'} <= generated.keys():
            break
    for key, expected in [('pelvis', rows['world_from_pelvis'][0]), ('camera_pose', camera)]:
        pose = generated[key].pose
        p, q = pose.position, pose.orientation
        np.testing.assert_allclose([p.x, p.y, p.z], expected[:3, 3], atol=1e-6)
        np.testing.assert_allclose(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
                                   expected[:3, :3], atol=1e-6)
    xyz = point_cloud2.read_points_numpy(source_scan, field_names=('x', 'y', 'z'))
    expected = (xyz @ sensor[:3, :3].T + sensor[:3, 3]).astype(np.float32)
    actual = point_cloud2.read_points_numpy(generated['scan'], field_names=('x', 'y', 'z'))
    np.testing.assert_allclose(actual, expected, atol=1e-6)
    assert len(actual) == source_scan.width*source_scan.height
