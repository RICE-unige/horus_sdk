"""Matched, read-only G1 recording bundles and first-walk observation playback."""

from copy import deepcopy
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

FIRST = Path('/media/omotoye/New Volume/g1_spatial/runtime/results/20261003T185621Z_fast-livo_3a2478')
LATER = Path('/media/omotoye/New Volume/g1_operations/artifacts/spatial/20261006_live_like_walk/20261006T011621Z_fast-livo_7d69e5')
CACHE = Path.home() / '.cache/horus/g1-recording-demo/assets'


def bundle(dataset):
    if dataset not in ('first', 'later'):
        raise ValueError('Choose first or later recording')
    return Path(os.environ.get('HORUS_G1_' + dataset.upper() + '_RESULT',
                               str(FIRST if dataset == 'first' else LATER))).expanduser()


def bag_path(dataset):
    return bundle(dataset) / ('map_input/bag' if dataset == 'first' else 'outputs')


def occupancy_path(dataset):
    folder = 'mapping2d_20261004T031657Z_e07b35' if dataset == 'first' else 'mapping2d'
    return bundle(dataset) / folder / 'layers.npz'


def checked_matrix(matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-5)):
        raise ValueError('Invalid rigid map/robot transform')
    return matrix


def matrix_pose(matrix):
    from geometry_msgs.msg import Pose
    matrix = checked_matrix(matrix)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    return pose


def camera_mount(path, dataset, urdf):
    """Use the recording's nominal optical rig, rigidly attached to its LiDAR link."""
    root = Path(path).parents[1] if dataset == 'first' else Path(path).parent
    if dataset == 'first':
        config = root / 'input/manifest.json'
        matrix = np.linalg.inv(checked_matrix(json.loads(config.read_text())['nominal_camera_from_lidar']))
    else:
        config = root / 'configuration.json'
        matrix = checked_matrix(json.loads(config.read_text())['sensor_from_camera'])
    if ET.parse(urdf).find("link[@name='mid360_link']") is None:
        raise ValueError('Recorded camera mounting requires the G1 mid360_link')
    return 'mid360_link', checked_matrix(matrix), str(config)


def load_occupancy(path):
    from nav_msgs.msg import OccupancyGrid
    with np.load(path, allow_pickle=False) as data:
        cells = data['occupancy']
        resolution = float(data['resolution'])
        origin = np.asarray(data['origin'], dtype=np.float64)
        world = checked_matrix(data['world_from_map'])
        if (cells.ndim != 2 or not cells.size or not np.issubdtype(cells.dtype, np.integer)
                or cells.min() < -1 or cells.max() > 100
                or not np.isfinite(resolution) or resolution <= 0
                or origin.shape != (7,) or not np.isfinite(origin).all()
                or not np.isclose(np.linalg.norm(origin[3:]), 1, atol=1e-5)):
            raise ValueError('Invalid saved occupancy grid')
        local = np.eye(4)
        local[:3, :3] = Rotation.from_quat(origin[3:]).as_matrix()
        local[:3, 3] = origin[:3]
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.info.resolution = resolution
        message.info.height, message.info.width = cells.shape
        message.info.origin = matrix_pose(world @ local)
        # The NPZ contains ROS row order, unlike a top-down PGM image.
        message.data = np.ascontiguousarray(cells, dtype=np.int8).ravel().tolist()
    return message


def first_display(path, urdf):
    with np.load(Path(path).parents[1] / 'odometry/display.npz', allow_pickle=False) as data:
        rows = {key: data[key] for key in ('stamps_s', 'world_from_pelvis',
            'world_from_sensor', 'world_from_camera', 'joint_names', 'joint_positions')}
    stamps = rows['stamps_s']
    names = rows['joint_names'].tolist()
    movable = {j.attrib['name'] for j in ET.parse(urdf).getroot().findall('joint') if j.get('type') != 'fixed'}
    if set(names) != movable or len(names) != len(movable):
        raise ValueError('First recording joints do not match the G1 URDF')
    if (stamps.ndim != 1 or not len(stamps) or not np.isfinite(stamps).all()
            or np.any(np.diff(stamps) <= 0)
            or rows['joint_positions'].shape != (len(stamps), len(names))
            or not np.isfinite(rows['joint_positions']).all()):
        raise ValueError('Invalid first-recording pose/joint timeline')
    for key in ('world_from_pelvis', 'world_from_sensor', 'world_from_camera'):
        if rows[key].shape != (len(stamps), 4, 4):
            raise ValueError('Incomplete first-recording pose array: ' + key)
        for matrix in rows[key]:
            checked_matrix(matrix)
    return rows


def read_first(path, urdf):
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from sensor_msgs_py import point_cloud2
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from tools.g1_recording_replay import MAP_FRAME, OPTICAL_FRAME

    rows = first_display(path, urdf)
    topics = {'/pointcloud': ('scan', 'sensor_msgs/msg/PointCloud2', 'g1_sensor'),
              '/camera_0/color/image': ('rgb', 'sensor_msgs/msg/Image', 'g1_rgb'),
              '/camera_0/color/camera_info': ('camera_info', 'sensor_msgs/msg/CameraInfo', 'g1_rgb')}
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(path), storage_id='mcap'), ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic, (_, expected, _) in topics.items():
        if types.get(topic) != expected:
            raise ValueError(f'{topic}: expected {expected}, got {types.get(topic)}')
    reader.set_filter(StorageFilter(topics=list(topics)))
    classes = {t: get_message(spec[1]) for t, spec in topics.items()}
    previous, pose_index = None, None
    while reader.has_next():
        topic, payload, receipt = reader.read_next()
        if previous is not None and receipt < previous:
            raise ValueError('First-recording receipt clock goes backwards')
        previous = receipt
        key, _, expected_frame = topics[topic]
        msg = deserialize_message(payload, classes[topic])
        if msg.header.frame_id != expected_frame:
            raise ValueError('Unexpected first-recording frame: ' + msg.header.frame_id)
        seconds = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        insertion = int(np.searchsorted(rows['stamps_s'], seconds))
        candidates = {max(0, min(len(rows['stamps_s']) - 1, i)) for i in (insertion - 1, insertion)}
        index = min(candidates, key=lambda i: abs(rows['stamps_s'][i] - seconds))
        if abs(rows['stamps_s'][index] - seconds) > 0.002:
            raise ValueError('Mapper input and display pose timestamps do not match')
        if index != pose_index:
            pose_index = index
            for pose_key, matrix_key in (('pelvis', 'world_from_pelvis'), ('camera_pose', 'world_from_camera')):
                pose = PoseStamped(header=deepcopy(msg.header), pose=matrix_pose(rows[matrix_key][index]))
                pose.header.frame_id = MAP_FRAME
                yield pose_key, pose, receipt
            joints = JointState(header=deepcopy(msg.header), name=rows['joint_names'].tolist(),
                                position=rows['joint_positions'][index].tolist())
            joints.header.frame_id = MAP_FRAME
            yield 'joints', joints, receipt
        if key == 'scan':
            xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=False)
            transform = rows['world_from_sensor'][index]
            xyz = (xyz @ transform[:3, :3].T + transform[:3, 3]).astype(np.float32)
            msg = point_cloud2.create_cloud_xyz32(deepcopy(msg.header), xyz)
        msg.header.frame_id = OPTICAL_FRAME if key in ('rgb', 'camera_info') else MAP_FRAME
        yield key, msg, receipt


def read_source(path, urdf, dataset='later', map_mode='live'):
    from tools.g1_recording_replay import read_bag
    if dataset == 'first':
        if map_mode != 'static':
            raise ValueError('The first recording has no archived incremental mesh; use --map static')
        yield from read_first(path, urdf)
    else:
        yield from read_bag(path, include_mesh=map_mode == 'live')
