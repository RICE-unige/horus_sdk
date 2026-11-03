"""Opt-in local DDS test of the actual adapter, not a mocked ROS publisher.

Run only in isolated domain 231 with ROS_LOCALHOST_ONLY=1 and
HORUS_MAPPING_ROS_TESTS=1. No robot command topics or services are used.
"""

import json
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
from geometry_msgs.msg import Point32
from nvblox_msgs.msg import Index3D, Mesh, MeshBlock
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String, UInt8MultiArray
from horus.mapping.nvblox_mesh import decode_live_header


@pytest.mark.parametrize("reliable", [True, False])
@pytest.mark.parametrize("encoder_delay", [0.0, 1.5])
def test_real_adapter_replay_preserves_existing_viewer_and_colors(tmp_path, reliable, encoder_delay):
    root = Path(__file__).resolve().parents[2]
    topic = "/horus_map_test_" + uuid.uuid4().hex
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node("horus_map_repair_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    input_pub = node.create_publisher(Mesh, topic + "/source", QoSProfile(
        depth=8, reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT))
    request_pub = node.create_publisher(String, topic + "/indexed_request", 8)
    pages, late_pages, wire, manifests = {}, {}, [], []

    def consume(msg, cache):
        payload = bytes(msg.data)
        header = decode_live_header(payload)
        wire.append(header)
        if header.operation == 2:
            cache.clear()
        elif header.operation == 1:
            cache.pop(header.chunk_id, None)
        else:
            cache[header.chunk_id] = payload

    node.create_subscription(UInt8MultiArray, topic + "/indexed", lambda msg: consume(msg, pages), 64)
    node.create_subscription(String, topic + "/indexed_manifest", lambda msg: manifests.append(json.loads(msg.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def wait_for(predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError("ROS adapter condition timed out")

    def spin_for(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)

    def block(x):
        return MeshBlock(vertices=[Point32(x=float(x), y=0.0, z=0.0), Point32(x=float(x)+1, y=0.0, z=0.0),
                                   Point32(x=float(x), y=1.0, z=0.0)], triangles=[0, 1, 2],
                         colors=[ColorRGBA(r=1.0, g=0.25, b=0.0, a=1.0)] * 3)

    with (tmp_path / "adapter.log").open("w") as log:
        script = str(root / "examples/tools/nvblox_mesh_to_horus.py")
        command = [sys.executable, script]
        if encoder_delay:
            # Inject test-only CPU work into the real adapter process, not a
            # production delay knob. Its ROS publish/repair timer must stay live.
            wrapper = ("import sys,time,runpy\n"
                "from horus.mapping.nvblox_mesh import LiveNvbloxMeshState\n"
                "original=LiveNvbloxMeshState.apply_message\n"
                "def slow(self,message):\n"
                f" time.sleep({encoder_delay})\n"
                " return original(self,message)\n"
                "LiveNvbloxMeshState.apply_message=slow\n"
                "script=sys.argv.pop(1)\nsys.argv[0]=script\n"
                "runpy.run_path(script,run_name='__main__')\n")
            command = [sys.executable, "-c", wrapper, script]
        process = subprocess.Popen(command + ["--input-topic", topic + "/source", "--output-topic", topic,
                                             "--blocks-per-axis", "1"],
            stdout=log, stderr=subprocess.STDOUT)
        try:
            wait_for(lambda: input_pub.get_subscription_count() == 1 and request_pub.get_subscription_count() == 1)
            message = Mesh(block_size_m=1.0, block_indices=[Index3D(x=0), Index3D(x=1)], blocks=[block(0), block(1)], clear=True)
            message.header.frame_id = "map"
            input_pub.publish(message)
            wait_for(lambda: len(pages) == 2)
            snapshot = dict(pages)
            for payload in pages.values():
                assert payload[64:68] == bytes([255, 64, 0, 255])
            before = len(wire)
            message.clear = False
            input_pub.publish(message)
            spin_for(1.0)
            assert len(wire) == before, "unchanged geometry must not be republished"

            if encoder_delay:
                input_pub.publish(message)
                spin_for(.2)
                before = len(wire)
                request_pub.publish(String(data='{"request":"snapshot"}'))
                wait_for(lambda: len(wire) > before, timeout=1.0)
                assert pages == snapshot, "a repair during encoding must expose only the last complete state"

            deletion = Mesh(block_size_m=1.0, block_indices=[Index3D(x=0)], blocks=[MeshBlock()])
            deletion.header.frame_id = "map"
            input_pub.publish(deletion)
            wait_for(lambda: len(pages) == 1)
            current = dict(pages)
            node.create_subscription(UInt8MultiArray, topic + "/indexed", lambda msg: consume(msg, late_pages), 64)
            spin_for(0.4)
            before = len(wire)
            request_pub.publish(String(data='{"request":"snapshot"}'))
            wait_for(lambda: late_pages == current)
            assert pages == current
            assert all(header.operation != 2 for header in wire[before:]), "replay must not broadcast CLEAR"
            wait_for(lambda: manifests and manifests[-1]["triangles"] == 1)
            manifest = manifests[-1]
            assert len(manifest["pages"]) == 2, "inventory must retain the deletion tombstone"

            # Drop the retained page at one viewer and recover only its requested ID.
            late_pages.clear()
            request_pub.publish(String(data=json.dumps({"request": "pages", "epoch": manifest["epoch"],
                                                       "ids": list(current)})))
            wait_for(lambda: late_pages == current)
            assert pages == current and len(snapshot) == 2
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
        assert process.returncode == 0, (tmp_path / "adapter.log").read_text()
    assert f"Source delivery: {'reliable' if reliable else 'best_effort'}" in (tmp_path / "adapter.log").read_text()
