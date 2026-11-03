"""Opt-in real recording playback over isolated local DDS; no headset/robot control."""

from collections import Counter
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('HORUS_G1_RECORDING_INTEGRATION') != '1',
    reason='Opt-in full recorded-data DDS test (approximately seven minutes)')
EXAMPLES = Path(__file__).parents[2] / 'examples'
sys.path.insert(0, str(EXAMPLES))


@pytest.mark.parametrize('dataset,mode,expected', [
    ('first', 'static', 115205), ('later', 'static', 227954), ('later', 'live', 177090)])
def test_full_recording_to_dds_with_both_maps(tmp_path, dataset, mode, expected):
    import numpy as np
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage, PointCloud2, JointState
    from nav_msgs.msg import OccupancyGrid, Path as RosPath
    from std_msgs.msg import String, UInt8MultiArray
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    import yaml
    from g1_recording_registration import CAMERA_FRAME, ROBOT, TOPIC, URDF
    from tools.g1_recording_data import bag_path, occupancy_path, load_occupancy
    from horus.mapping.mesh_protocol import decode_live_header
    from mapping_visualization_demo import stop_owned

    assert os.environ.get('ROS_DOMAIN_ID') == '227', 'Use the dedicated local test domain, never the live demo domain'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    node = rclpy.create_node('horus_g1_recording_test')
    processes, logs = [], []
    counts, received, latest = Counter(), {}, {}
    transforms = set()
    dynamic_counts, dynamic_times, static_parents = Counter(), {}, {}
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    params = tmp_path/'rsp.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'robot_description': URDF.read_text(), 'frame_prefix': ROBOT+'/', 'publish_frequency': 50.0}}}))

    def capture(key, message):
        counts[key] += 1
        latest[key] = message

    def page(message):
        header = decode_live_header(bytes(message.data))
        if header.operation == 2:
            received.clear()
        elif header.operation == 1:
            received.pop(header.chunk_id, None)
        else:
            received[header.chunk_id] = header

    def manifest(message):
        latest['manifest'] = json.loads(message.data)

    def tf(message, static=False):
        transforms.update(t.child_frame_id for t in message.transforms)
        for transform in message.transforms:
            if static:
                static_parents[transform.child_frame_id] = transform.header.frame_id
            else:
                dynamic_counts[transform.child_frame_id] += 1
                if latest.get('status', {}).get('phase') == 'playing':
                    dynamic_times.setdefault(transform.child_frame_id, []).append(time.monotonic())

    for key, type_, suffix in [('rgb', CompressedImage, '/camera/compressed'),
                             ('scan', PointCloud2, '/scan'), ('joints', JointState, '/joint_states'),
                             ('path', RosPath, '/walked_path'), ('grid', OccupancyGrid, '/occupancy')]:
        node.create_subscription(type_, TOPIC+suffix, lambda m, k=key: capture(k, m),
                                 retained if key in ('path', 'grid') else 100)
    node.create_subscription(String, TOPIC+'/replay_status', lambda m: capture('status', json.loads(m.data)), retained)
    node.create_subscription(String, TOPIC+'/mesh/indexed_manifest', manifest, retained)
    node.create_subscription(UInt8MultiArray, TOPIC+'/mesh/indexed', page, 512)
    node.create_subscription(TFMessage, '/tf', tf, 100)
    node.create_subscription(TFMessage, '/tf_static', lambda m: tf(m, static=True), retained)
    ack = node.create_publisher(String, '/horus/registration_ack', 10)
    repair = node.create_publisher(String, TOPIC+'/mesh/indexed_request', 16)
    start = node.create_client(Trigger, TOPIC+'/start')

    def spin_until(condition, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            assert all(p.poll() is None for p in processes), 'Demo process exited; inspect its log'
            rclpy.spin_once(node, timeout_sec=0.02)
            if condition():
                return
        pytest.fail(f'Timed out: {counts}, status={latest.get("status")}')

    try:
        commands = [
            ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args',
             '--params-file', str(params), '-r', 'joint_states:='+TOPIC+'/joint_states'],
            [sys.executable, '-m', 'tools.g1_completed_maps', '--dataset', dataset, '--map', mode,
             '--ready-file', str(tmp_path/'maps.json')],
            [sys.executable, '-m', 'tools.g1_recording_replay', '--dataset', dataset, '--map', mode,
             '--bag', str(bag_path(dataset)), '--urdf', str(URDF), '--ready-file', str(tmp_path/'ready.json')],
        ]
        if mode == 'live':
            commands.insert(1, [sys.executable, '-m', 'tools.nvblox_mesh_to_horus',
                '--input-topic', TOPIC+'/mesh/source', '--output-topic', TOPIC+'/mesh', '--color-mode', 'source'])
        for i, command in enumerate(commands):
            log = (tmp_path/f'child-{i}.log').open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        spin_until(lambda: (tmp_path/'ready.json').exists() and (tmp_path/'maps.json').exists()
                   and start.service_is_ready() and ack.get_subscription_count() > 0
                   and 'grid' in latest, 60)
        # Real start gate rejects a client that has not received a registration ACK.
        rejected = start.call_async(Trigger.Request())
        spin_until(rejected.done, 5)
        assert not rejected.result().success
        ack.publish(String(data=json.dumps({'robot_name': ROBOT, 'success': True, 'robot_id': 'test-recorded'})))
        spin_until(lambda: latest.get('status', {}).get('registered'), 5)
        future = start.call_async(Trigger.Request())
        spin_until(future.done, 5)
        assert future.result().success, future.result().message
        next_repair = time.monotonic()
        deadline = time.monotonic()+190
        while time.monotonic() < deadline:
            assert all(p.poll() is None for p in processes)
            rclpy.spin_once(node, timeout_sec=0.01)
            if time.monotonic() > next_repair:
                repair.publish(String(data='{"request":"snapshot"}'))
                next_repair = time.monotonic()+5
            if (latest.get('status', {}).get('phase') == 'complete'
                    and sum(p.index_count//3 for p in received.values()) == expected):
                break
        else:
            pytest.fail(f'Incomplete replay/map: {latest.get("status")}, pages={len(received)}')
        spin_until(lambda: 'path' in latest and len(latest['path'].poses) == latest['status']['path_samples'], 5)
        grid = load_occupancy(occupancy_path(dataset))
        assert latest['grid'].info == grid.info
        assert latest['grid'].data == grid.data
        expected_samples = 1852 if dataset == 'first' else 1723
        assert len(latest['path'].poses) == expected_samples
        assert counts['rgb'] >= expected_samples and counts['scan'] >= expected_samples
        assert len(transforms) >= 54
        assert static_parents[CAMERA_FRAME] == ROBOT + '/mid360_link'
        assert dynamic_counts[CAMERA_FRAME] == 0, 'No competing map->camera dynamic publisher'
        assert dynamic_counts[ROBOT + '/pelvis'] >= expected_samples
        assert dynamic_counts[ROBOT + '/floor_anchor'] >= expected_samples
        intervals = np.diff(dynamic_times[ROBOT + '/pelvis'])
        assert len(intervals) > expected_samples * 0.9
        assert np.percentile(intervals, 95) < 0.2, 'Replay itself is starving root TF'
        summary = json.loads((tmp_path/'replay-summary.json').read_text())
        summary.update(received=dict(counts), triangles=expected, pages=len(received), tf_children=len(transforms),
                       dynamic_tf_counts=dict(dynamic_counts), camera_parent=static_parents[CAMERA_FRAME],
                       pelvis_interval_ms_p50_p95_max=(np.array([
                           np.median(intervals), np.percentile(intervals, 95), max(intervals)]) * 1000).tolist())
        (tmp_path/'verification.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(json.dumps(summary), flush=True)
    finally:
        stop_owned(processes)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
