#!/usr/bin/env python3
"""Exercise C++/Rust publishers over isolated ROS DDS, without Quest/robot commands."""
import argparse
import json
import struct
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, UInt8MultiArray


def header(data):
    magic, version, chunk, x, y, z, epoch, revision, op, color, reserved, vertices, indices = struct.unpack_from("<IIIiiiQQBBHII", data)
    assert magic == 0x484D5348 and version == 2 and reserved == 0
    assert len(data) == 52 + vertices * (16 if color else 12) + indices * 4
    return chunk, epoch, revision, op, vertices, indices


def check(executable, index):
    node = rclpy.create_node(f"native_mesh_peer_{index}")
    topic = f"/native_mesh_contract_{index}"
    observed, late, manifests = [], [], []
    subscription = node.create_subscription(UInt8MultiArray, topic + "/indexed", lambda msg: observed.append(bytes(msg.data)), 16)
    request = node.create_publisher(String, topic + "/indexed_request", 16)
    process = None
    try:
        process = subprocess.Popen([executable, topic], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        start = time.monotonic()
        while not observed and time.monotonic() - start < 3:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert observed, "No native map delivered"
        first = observed[0]
        assert header(first) == (1, 11, 1, 0, 3, 3)
        expected_body = b"".join(struct.pack("<fffBBBB", *vertex, *color) for vertex, color in [
            ((0, 0, 0), (255, 0, 0, 255)), ((1, 0, 0), (0, 255, 0, 255)), ((0, 1, 0), (0, 0, 255, 255))]) + struct.pack("<III", 0, 1, 2)
        assert first[52:] == expected_body, "Geometry/color changed in native delivery"
        # A subscriber created after the map was sent must retrieve the inventory
        # and repair the map, not depend on history in the ordinary page topic.
        late_subscription = node.create_subscription(UInt8MultiArray, topic + "/indexed", lambda msg: late.append(bytes(msg.data)), 16)
        inventory = node.create_subscription(String, topic + "/indexed_manifest", lambda msg: manifests.append(json.loads(msg.data)),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        repaired = False
        sent_request = False
        while process.poll() is None and time.monotonic() - start < 12:
            elapsed = time.monotonic() - start
            if elapsed >= 2.5 and not sent_request:
                request.publish(String(data=json.dumps({"request":"snapshot"})))
                sent_request = True
            rclpy.spin_once(node, timeout_sec=0.02)
            repaired |= first in late
        stdout, stderr = process.communicate(timeout=2)
        assert process.returncode == 0, stdout + stderr
        assert repaired, "Late subscriber did not receive the retained geometry"
        assert any(m["epoch"] == "11" and m["triangles"] == 1 for m in manifests), manifests
        values = [header(data) for data in observed]
        assert (1, 11, 2, 0, 3, 3) in values, values
        assert (1, 11, 3, 1, 0, 0) in values, values
        assert (0, 12, 0, 2, 0, 0) in values, values
        assert any(m["epoch"] == "12" and m["pages"] == [] for m in manifests), manifests
        print(json.dumps({"executable":executable,"result":"passed","pages_received":len(observed),
            "late_join_repaired":repaired,"geometry_preserved":True}))
    finally:
        if process and process.poll() is None:
            process.terminate()
            try:
                process.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("executables", nargs="+")
    args = parser.parse_args()
    rclpy.init()
    try:
        for index, executable in enumerate(args.executables):
            check(executable, index)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
