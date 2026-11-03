#!/usr/bin/env python3
"""Isolated real DDS sensor-to-risk checks, including padded big-endian clouds."""
import json
import struct
import subprocess
import sys
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String

def check(binary, source):
    node = rclpy.create_node('collision_peer')
    result = []
    kind = LaserScan if source == 'laser_scan' else PointCloud2
    publisher = node.create_publisher(kind, '/parity/source', qos_profile_sensor_data)
    node.create_subscription(String, '/parity/risk', lambda m: result.append(json.loads(m.data)), 10)
    process = subprocess.Popen([binary, source], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        message = kind()
        message.header.frame_id = 'sensor_frame'
        if source == 'laser_scan':
            message.range_min = 0.0
            message.range_max = 20.0
            message.angle_increment = 0.5
            message.ranges = [float('nan'), 2.0, 0.5, float('inf')]
        else:
            message.width, message.height, message.point_step, message.row_step = 1, 2, 24, 32
            message.is_bigendian = True
            message.fields = [PointField(name=name, offset=i*8, datatype=8, count=1) for i, name in enumerate('xyz')]
            message.data = struct.pack('>ddd8xddd8x', .1, 0, 2, .3, .4, 0)
        deadline = time.monotonic() + 10
        while process.poll() is None and time.monotonic() < deadline:
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=.05)
        stdout, stderr = process.communicate(timeout=3)
        assert process.returncode == 0, stdout + stderr
        samples = [m for m in result if m['frame'] == 'sensor_frame']
        assert len(samples) > 10, result
        for m in samples:
            assert m['min_distance_m'] == .5 and m['risk'] == .5833, m
            assert m['direction']['x'] == (.5403 if source == 'laser_scan' else .6), m
            assert abs(m['stamp_ms'] / 1000 - time.time()) < 20, m
        print(json.dumps(dict(binary=binary, source=source, messages=len(samples), passed=True)), flush=True)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        node.destroy_node()

if __name__ == '__main__':
    rclpy.init()
    try:
        for binary in sys.argv[1:]:
            for source in ('laser_scan', 'point_cloud'):
                check(binary, source)
    finally:
        rclpy.shutdown()
