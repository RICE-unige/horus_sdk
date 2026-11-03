"""Opt-in delivery tests using physical captures, never robot command topics."""

import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

if os.environ.get("HORUS_MAPPING_REAL_TESTS") != "1":
    pytest.skip("requires prepared real mapping assets", allow_module_level=True)
if os.environ.get("ROS_DOMAIN_ID") != "231" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise RuntimeError("real mapping checks require local-only domain 231")

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import UInt8MultiArray, String
from visualization_msgs.msg import Marker


@pytest.mark.parametrize("mode,dataset", [
    ("mesh", "spot"), ("indexed", "g1"), ("points", "cow-lady"),
    ("points-refresh", "g1"), ("octomap", "freiburg"),
    ("voxblox-marker", "g1"), ("nvblox", "g1"),
])
def test_measured_geometry_reaches_native_transport(tmp_path, mode, dataset):
    examples = Path(__file__).parents[2] / "examples"
    spec = importlib.util.spec_from_file_location("real_demo_ros", examples / "mapping_visualization_demo.py")
    demo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(demo)
    args = demo.arguments([mode, "--dataset", dataset])
    entry = demo.real_record(args)
    rclpy.init()
    node = rclpy.create_node("real_mapping_probe")
    indexed = mode in ("indexed", "nvblox")
    points = mode.startswith("points")
    received, manifest, pages = {}, [], {}
    source_done = False
    processes, logs = [], []
    ready = tmp_path / "ready.json"

    def receive(message):
        if points:
            assert message.header.frame_id == "map"
            assert len(message.data) == message.row_step * message.height
            received[0] = message.width * message.height
        elif indexed:
            if mode == "nvblox":
                from horus.mapping.nvblox_mesh import (
                    decode_live_header, HMSH_OPERATION_CLEAR, HMSH_OPERATION_DELETE)
                header = decode_live_header(bytes(message.data))
                received[len(received)] = header
                if header.operation == HMSH_OPERATION_CLEAR:
                    pages.clear()
                elif header.operation == HMSH_OPERATION_DELETE:
                    pages.pop(header.chunk_id, None)
                else:
                    pages[header.chunk_id] = header.index_count // 3
            else:
                assert bytes(message.data[:4]) == b"HSMH"
                received[len(received)] = len(message.data)
        elif message.action == Marker.ADD and message.points:
            assert message.header.frame_id == "map"
            assert len(message.colors) == len(message.points)
            received[(message.ns, message.id)] = len(message.points) // 3

    qos = QoSProfile(depth=1024)
    if mode not in ("nvblox", "octomap"):
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    node.create_subscription(UInt8MultiArray if indexed else PointCloud2 if points else Marker,
                             demo.TOPIC + ("/indexed" if indexed else ""), receive, qos)
    if mode == "nvblox":
        node.create_subscription(String, demo.TOPIC + "/indexed_manifest",
                                 lambda m: manifest.append(json.loads(m.data)),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        for i, command in enumerate(demo.child_commands(args, ready)):
            log = (tmp_path / f"source-{i}.log").open("w")
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + (360 if mode == "nvblox" else 75)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            assert all(p.poll() is None for p in processes), [
                p.read_text()[-3000:] for p in tmp_path.glob("*.log")]
            if not received:
                continue
            if mode == "nvblox":
                source_done = "Replay complete: 1151 messages" in (tmp_path / "source-0.log").read_text()
                if (source_done and manifest and manifest[-1].get("triangles") == 177090
                        and sum(pages.values()) == 177090):
                    break
            elif mode == "indexed" and ready.exists():
                if len(received) >= json.loads(ready.read_text())["messages_per_snapshot"]:
                    break
            elif mode in ("mesh", "voxblox-marker"):
                if sum(received.values()) == entry["triangles"]:
                    break
            elif mode == "octomap":
                if sum(received.values()) == 2125042:
                    break
            else:
                break
        assert received
        if mode == "points":
            assert received[0] == entry["points"]
        if mode in ("mesh", "voxblox-marker"):
            assert sum(received.values()) == entry["triangles"]
        if mode == "octomap":
            assert sum(received.values()) == 2125042
        if mode == "indexed":
            assert len(received) == json.loads(ready.read_text())["messages_per_snapshot"]
        if mode == "nvblox":
            # Independently decoded bag: 177,664 index triples, of which
            # 574 repeat vertex indices and have no drawable surface.
            assert source_done and manifest[-1]["triangles"] == 177090
            assert sum(pages.values()) == 177090
        print(f"REAL_DATA mode={mode} dataset={dataset} messages={len(received)} "
              f"manifest={manifest[-1] if manifest else None}")
    finally:
        demo.stop_owned(processes)
        for log in logs:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
