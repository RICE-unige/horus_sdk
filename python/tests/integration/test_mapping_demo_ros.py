"""Actual demo publisher/adapter checks, isolated from robot and Quest traffic."""

import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

if os.environ.get("HORUS_MAPPING_ROS_TESTS") != "1":
    pytest.skip("explicit isolated ROS integration opt-in required", allow_module_level=True)
if os.environ.get("ROS_DOMAIN_ID") != "231" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise RuntimeError("mapping demo integration requires local-only domain 231")

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, UInt8MultiArray
from visualization_msgs.msg import Marker


@pytest.mark.parametrize("mode", ["mesh", "indexed", "points", "points-refresh", "nvblox", "octomap", "voxblox-marker"])
def test_generated_fixture_delivers_colored_geometry(tmp_path, mode):
    import importlib.util
    examples = Path(__file__).parents[2] / "examples"
    spec = importlib.util.spec_from_file_location("mapping_demo_ros", examples / "mapping_visualization_demo.py")
    demo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(demo)
    args = demo.arguments([mode, "--synthetic"])
    ready = tmp_path / "ready.json"
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node("mapping_demo_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    messages = []
    indexed = mode in ("nvblox", "indexed")
    msg_type = UInt8MultiArray if indexed else PointCloud2 if mode.startswith("points") else Marker
    topic = demo.TOPIC + ("/indexed" if indexed else "")
    qos = QoSProfile(depth=128)
    if mode not in ("nvblox", "octomap"):
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    node.create_subscription(msg_type, topic, messages.append, qos)
    processes, logs = [], []
    try:
        for index, command in enumerate(demo.child_commands(args, ready)):
            log = (tmp_path / f"{index}.log").open("w")
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            assert all(p.poll() is None for p in processes), "source/adapter exited; inspect source logs"
            if not ready.exists():
                continue
            info = json.loads(ready.read_text())
            if mode in ("mesh", "indexed") and len(messages) >= info["messages_per_snapshot"]:
                break
            if mode.startswith("points") and len(messages) >= 2:
                break
            if mode in ("nvblox", "octomap", "voxblox-marker") and any(
                len(m.data) > 56 if indexed else m.action == Marker.ADD and len(m.points) > 0 for m in messages):
                break
        assert messages, f"no {mode} output received"
        if mode.startswith("points"):
            assert len(messages) >= 2
            assert messages[0].width == 100000
            assert len(messages[0].data) == 100000 * 16
            assert (messages[0].data != messages[1].data) == (mode == "points-refresh")
        elif indexed:
            assert any(bytes(m.data[:4]) == b"HSMH" for m in messages)
        else:
            additions = [m for m in messages if m.action == Marker.ADD and m.points]
            assert additions and all(len(m.colors) == len(m.points) for m in additions)
            assert all(m.header.frame_id == "map" for m in additions)
            assert all(len(m.points) % 3 == 0 for m in additions)
    finally:
        for p in processes:
            p.send_signal(signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(timeout=5)
        for log in logs:
            log.close()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def test_gaussian_binary_transfer_reassembles_exact_source_file(tmp_path):
    import importlib.util
    examples = Path(__file__).parents[2] / "examples"
    spec = importlib.util.spec_from_file_location("gaussian_demo_ros", examples / "mapping_visualization_demo.py")
    demo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(demo)
    assert demo.SPLAT.is_file(), "native Gaussian demo fixture must be installed"
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node("gaussian_transfer_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    manifests, begins, ends, chunks = [], [], [], {}
    qos = QoSProfile(depth=512)

    def receive_chunk(message):
        dimensions = message.layout.dim
        assert len(dimensions) == 2
        assert message.layout.data_offset == dimensions[0].size
        assert len(message.data) == dimensions[1].size
        chunks[dimensions[0].size] = (dimensions[0].label, dimensions[0].stride, bytes(message.data))

    node.create_subscription(String, "/horus/gaussian_splat/manifest",
                             lambda m: manifests.append(json.loads(m.data)), qos)
    node.create_subscription(String, "/horus/gaussian_splat/chunk_begin",
                             lambda m: begins.append(json.loads(m.data)), qos)
    node.create_subscription(String, "/horus/gaussian_splat/chunk_end",
                             lambda m: ends.append(json.loads(m.data)), qos)
    node.create_subscription(UInt8MultiArray, "/horus/gaussian_splat/chunk_item", receive_chunk, qos)
    command = demo.child_commands(demo.arguments(["gaussian"]), tmp_path / "ready.json")[0]
    process = None
    try:
        with (tmp_path / "gaussian.log").open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
                assert process.poll() is None, "Gaussian publisher exited"
                if manifests and begins and ends and len(chunks) == manifests[-1]["chunk_count"]:
                    break
            assert manifests and begins and ends
            manifest = manifests[-1]
            count = manifest["chunk_count"]
            assert sorted(chunks) == list(range(count))
            assert all(asset == manifest["asset_id"] and total == count for asset, total, _ in chunks.values())
            payload = b"".join(chunks[index][2] for index in range(count))
            assert manifest["chunk_encoding"] == "raw+uint8multiarray"
            assert len(payload) == manifest["content_length"]
            assert hashlib.sha256(payload).hexdigest() == manifest["sha256"]
            assert payload == demo.SPLAT.read_bytes()
            assert begins[-1]["asset_id"] == ends[-1]["asset_id"] == manifest["asset_id"]
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
