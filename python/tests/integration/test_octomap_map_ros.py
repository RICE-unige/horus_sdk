"""Actual ColorOcTree relay check on an explicitly isolated local ROS graph."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import pytest

if os.environ.get("HORUS_MAPPING_ROS_TESTS") != "1":
    pytest.skip("explicit isolated ROS integration test opt-in required", allow_module_level=True)
if os.environ.get("ROS_DOMAIN_ID") != "231" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise RuntimeError("map integration test requires localhost-only ROS domain 231")

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from octomap_msgs.msg import Octomap
from visualization_msgs.msg import Marker


def test_real_octomap_source_colors_dimensions_and_empty_map(tmp_path):
    root = Path(__file__).resolve().parents[2]
    fixture = tmp_path / "octomap_fixture"
    subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    str(root / "tests/cpp/octomap_ros_fixture.cpp"), "-loctomap", "-loctomath",
                    "-o", str(fixture)], check=True, capture_output=True, timeout=60)
    wrapper = root / "examples/tools/uav_sim_octomap_marker_relay.py"
    subprocess.run([sys.executable, str(wrapper), "--build-only"], check=True,
                   capture_output=True, timeout=120)
    topic = "/horus_octomap_test_" + uuid.uuid4().hex
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node("horus_octomap_relay_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    publisher = node.create_publisher(Octomap, topic + "/source",
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    markers = []
    node.create_subscription(Marker, topic + "/mesh", markers.append, 64)

    def wait_for(condition, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if condition():
                return
        raise AssertionError("OctoMap relay condition timed out")

    def publish(kind):
        wire = bytes.fromhex(subprocess.check_output([str(fixture), kind], text=True, timeout=10).strip())
        message = Octomap(binary=False, id="ColorOcTree", resolution=0.1)
        message.header.frame_id = "map"
        message.data = [value if value < 128 else value - 256 for value in wire]
        publisher.publish(message)

    with (tmp_path / "relay.log").open("w") as log:
        process = subprocess.Popen([sys.executable, str(wrapper), "--input-topic", topic + "/source",
                                    "--output-topic", topic + "/mesh", "--min-update-interval-sec", "0.05"],
                                   stdout=log, stderr=subprocess.STDOUT)
        try:
            wait_for(lambda: publisher.get_subscription_count() == 1)
            publish("occupied")
            wait_for(lambda: any(marker.action == Marker.ADD and marker.points for marker in markers))
            mesh = next(marker for marker in markers if marker.action == Marker.ADD and marker.points)
            colors = {(round(c.r * 255), round(c.g * 255), round(c.b * 255)) for c in mesh.colors}
            assert colors == {(255, 0, 0), (0, 255, 0)}
            assert len(mesh.colors) == len(mesh.points)
            assert min(p.x for p in mesh.points) == pytest.approx(0.0, abs=1e-6)
            assert max(p.x for p in mesh.points) == pytest.approx(0.2, abs=1e-6)
            assert max(p.y for p in mesh.points) - min(p.y for p in mesh.points) == pytest.approx(0.1)
            previous = len(markers)
            publish("empty")
            wait_for(lambda: any(marker.action == Marker.DELETEALL for marker in markers[previous:]))
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            executor.shutdown()
            node.destroy_node()
            context.shutdown()
