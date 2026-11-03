"""Opt-in real ROS wire test using immutable captured Field map pages."""
from array import array
import json
import os
from pathlib import Path
import struct
import signal
import subprocess
import sys
import time

import pytest

from horus.mapping.mesh_protocol import decode_live_header, encode_live_payload


@pytest.mark.skipif(os.environ.get("HORUS_ADAPTIVE_ROS_TEST") != "1", reason="isolated native ROS test")
def test_real_pages_independent_operators_repair_delete_reset_and_late_packets():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String, UInt8MultiArray
    from horus.mapping.adaptive_mesh import ensure_native
    from horus.mapping.adaptive_mesh_ros import arguments, client_topic_token, create_node

    ensure_native()
    folder = Path(os.environ["HORUS_ADAPTIVE_FIXTURE"])
    files = sorted(folder.glob("*.hmsh")) or sorted(folder.glob("*.bin"))
    assert len(files) >= 4, "real saved pages are required, not a synthetic replacement"
    payloads = [path.read_bytes() for path in files[:4]]
    headers = [decode_live_header(payload) for payload in payloads]
    assert len({header.epoch for header in headers}) == 1
    epoch = headers[0].epoch
    records = {header.chunk_id: {"chunk_id": header.chunk_id, "revision": str(header.revision), "operation": 0}
               for header in headers}
    args = arguments(["--input-topic", "/adaptive_wire/source", "--output-topic", "/adaptive_wire/view",
                      "--frame-id", "field_source"])
    rclpy.init(domain_id=218)
    server = create_node(args)
    probe = rclpy.create_node("adaptive_wire_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(probe)
    source = probe.create_publisher(UInt8MultiArray, args.input_topic + "/indexed", 16)
    inventory = probe.create_publisher(String, args.input_topic + "/indexed_manifest",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    request_publisher = probe.create_publisher(String, args.output_topic + "/adaptive_request", 16)
    # Reproduce the actual Quest crash, while keeping the old alphabetic route compatible.
    client_ids = ["3910882bdc0c4681b7d8dd0d87114200", "b" * 32]
    received = [{}, {}]
    manifests = [None, None]
    ignored = [False]
    subscriptions = []
    delivery_status = []
    subscriptions.append(probe.create_subscription(String, args.output_topic + "/status",
        lambda msg: delivery_status.append(json.loads(msg.data)), 1))
    repairs = []
    dropped_id = headers[0].chunk_id

    def receive(index, message):
        payload = bytes(message.data)
        header = decode_live_header(payload)
        if header.operation == 2:
            received[index].clear()
        else:
            if index == 0 and header.chunk_id == dropped_id and not ignored[0]:
                ignored[0] = True
                return
            old = received[index].get(header.chunk_id)
            if old is None or (header.epoch, header.revision) > (old[0].epoch, old[0].revision):
                received[index][header.chunk_id] = (header, payload)

    for index, client in enumerate(client_ids):
        base = args.output_topic + "/clients/" + client_topic_token(client)
        subscriptions.append(probe.create_subscription(UInt8MultiArray, base + "/indexed",
            lambda msg, i=index: receive(i, msg), 32))
        subscriptions.append(probe.create_subscription(String, base + "/indexed_manifest",
            lambda msg, i=index: manifests.__setitem__(i, json.loads(msg.data)),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        repairs.append(probe.create_publisher(String, base + "/indexed_request", 4))
    sequences = [0, 0]
    last_source = last_request = last_repair = -100.

    def pump(predicate, timeout=30):
        nonlocal last_source, last_request, last_repair
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_source > 1:
                inventory.publish(String(data=json.dumps({"version": 1, "epoch": str(epoch),
                    "frame_id": "field_source", "pages": list(records.values())})))
                for payload in payloads:
                    source.publish(UInt8MultiArray(data=array("B", payload)))
                last_source = now
            if now - last_request > .5:
                for i, client in enumerate(client_ids):
                    sequences[i] += 1
                    request_publisher.publish(String(data=json.dumps({"version": 1, "client_id": client,
                        "seq": sequences[i], "frame_id": "field_source", "triangle_budget": 2000000,
                        "memory_budget": 192 << 20, "regions": [] if i else [{"id": "inspection", "shape": "box",
                        "center": [0, 0, 0], "size": [30, 30, 30], "rotation": [0, 0, 0, 1], "priority": 1}]})))
                last_request = now
            if ignored[0] and manifests[0] is not None and now - last_repair > 1:
                repairs[0].publish(String(data=json.dumps({"request": "snapshot"})))
                last_repair = now
            executor.spin_once(timeout_sec=.02)
            if predicate():
                return
        pytest.fail(f"adaptive ROS timeout: pages={[list(r) for r in received]}, manifests={manifests}")

    try:
        pump(lambda: all(len(entries) == 4 for entries in received) and all(manifests))
        assert ignored[0]
        assert manifests[0]["adaptive"]["selected_triangles"] >= manifests[1]["adaptive"]["selected_triangles"]
        first_epochs = [int(manifest["epoch"]) for manifest in manifests]
        # Original fine payloads retain every position, colour, index and winding.
        for original, header in zip(payloads, headers):
            actual = received[0][header.chunk_id][1]
            assert actual[52:] == original[52:]
        target = headers[0]
        deletion = encode_live_payload(chunk_id=target.chunk_id, page_key=target.page_key, epoch=epoch,
            revision=target.revision + 1, operation=1, geometry=None, max_payload_bytes=16 << 20)
        payloads = [deletion, *payloads[1:]]
        records[target.chunk_id] = {"chunk_id": target.chunk_id, "revision": str(target.revision + 1), "operation": 1}
        pump(lambda: all(entries.get(target.chunk_id, (None,))[0] is not None and
                        entries[target.chunk_id][0].operation == 1 for entries in received))
        # Reset and one fresh real page must replace all old coverage coherently.
        epoch += 1
        restored = bytearray(files[0].read_bytes())
        struct.pack_into("<QQ", restored, 24, epoch, target.revision + 2)
        payloads = [encode_live_payload(chunk_id=0, page_key=(0, 0, 0), epoch=epoch,
            revision=0, operation=2, geometry=None, max_payload_bytes=1024), bytes(restored)]
        records = {target.chunk_id: {"chunk_id": target.chunk_id, "revision": str(target.revision + 2), "operation": 0}}
        pump(lambda: all(int(manifests[i]["epoch"]) > first_epochs[i] and len(received[i]) == 1
                        and received[i][target.chunk_id][0].operation == 0 for i in range(2)))
        source.publish(UInt8MultiArray(data=array("B", files[1].read_bytes())))
        until = time.monotonic() + .5
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.02)
        assert all(len(entries) == 1 for entries in received)
        assert server.rejected == 0
        pump(lambda: bool(delivery_status) and delivery_status[-1]["source_pages"] == 1)
        status = delivery_status[-1]
        assert status["clients"] == 2 and status["received_requests"] >= 2
        assert status["source_triangles"] > 0 and status["context_triangles"] > 0
        assert status["sent_pages"] > 0 and status["waiting_for_subscribers"] == 0
        assert status["last_request_age_seconds"] < 2
    finally:
        executor.remove_node(server)
        executor.remove_node(probe)
        server.destroy_node()
        probe.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


@pytest.mark.skipif(os.environ.get("HORUS_ADAPTIVE_ROS_TEST") != "1", reason="isolated native ROS test")
def test_real_cli_sigint_saves_the_original_captured_pages(tmp_path):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String, UInt8MultiArray
    from horus.mapping.adaptive_mesh import ensure_native
    ensure_native()
    files = sorted(Path(os.environ["HORUS_ADAPTIVE_FIXTURE"]).glob("*.hmsh"))[:4]
    assert len(files) == 4
    payloads = [path.read_bytes() for path in files]
    headers = [decode_live_header(payload) for payload in payloads]
    destination = tmp_path / "original"
    environment = dict(os.environ, ROS_DOMAIN_ID="219", ROS_LOCALHOST_ONLY="1",
                       ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    with (tmp_path / "server.log").open("wb") as log:
        process = subprocess.Popen([sys.executable, "-m", "horus.mapping.adaptive_mesh_ros",
            "--input-topic", "/archive_wire/source", "--output-topic", "/archive_wire/view",
            "--frame-id", "map", "--save-source", str(destination), "--source-floor-z=-1.6"],
            stdout=log, stderr=subprocess.STDOUT, env=environment, start_new_session=True)
    rclpy.init(domain_id=219)
    probe = rclpy.create_node("adaptive_archive_probe")
    source = probe.create_publisher(UInt8MultiArray, "/archive_wire/source/indexed", 16)
    inventory = probe.create_publisher(String, "/archive_wire/source/indexed_manifest",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    requests = probe.create_publisher(String, "/archive_wire/view/adaptive_request", 8)
    manifests = []
    listener = probe.create_subscription(String, "/archive_wire/view/clients/" + "c" * 32 + "/indexed_manifest",
        lambda msg: manifests.append(json.loads(msg.data)), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        end = time.monotonic() + 20
        sequence = 0
        while time.monotonic() < end:
            assert process.poll() is None, (tmp_path / "server.log").read_text()
            sequence += 1
            inventory.publish(String(data=json.dumps({"version": 1, "epoch": str(headers[0].epoch), "frame_id": "map",
                "pages": [{"chunk_id": header.chunk_id, "revision": str(header.revision), "operation": 0} for header in headers]})))
            for payload in payloads:
                source.publish(UInt8MultiArray(data=array("B", payload)))
            requests.publish(String(data=json.dumps({"version": 1, "client_id": "c" * 32, "seq": sequence,
                "frame_id": "map", "triangle_budget": 2000000, "memory_budget": 192 << 20, "regions": []})))
            for _ in range(10):
                rclpy.spin_once(probe, timeout_sec=.02)
            if manifests and len(manifests[-1]["pages"]) == 4:
                break
        assert manifests and len(manifests[-1]["pages"]) == 4, (tmp_path / "server.log").read_text()
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=15) == 0, (tmp_path / "server.log").read_text()
        assert "publisher's context is invalid" not in (tmp_path / "server.log").read_text()
        metadata = json.loads((destination / "manifest.json").read_text())
        assert metadata["input_inventory_verified"] is True
        assert metadata["source_floor_z_m"] == -1.6
        assert sorted(path.read_bytes() for path in (destination / "pages").glob("*.hmsh")) == sorted(payloads)
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=10)
        probe.destroy_node()
        rclpy.shutdown()
