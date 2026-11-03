"""Protocol fixtures only: real TCP relay -> ROS -> actual live checker, no hardware/GPU."""
from contextlib import ExitStack
import json
import os
from pathlib import Path
import signal
import socketserver
import subprocess
import sys
import threading
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("HORUS_FIELD_ROS_TEST") != "1",
                               reason="Explicit isolated ROS integration run required")


def test_real_relay_converts_packets_and_preserves_head_position(tmp_path):
    assert os.readlink("/proc/self/ns/net") != os.environ["HORUS_TEST_HOST_NETNS"]
    import cv2
    import numpy as np
    pixels = np.full((12, 16, 3), (30, 80, 140), dtype=np.uint8)
    ok, encoded = cv2.imencode(".jpg", pixels)
    assert ok
    jpeg = encoded.tobytes()
    pose = {"type": "pose", "position": {"x": 1., "y": .4, "z": 2.},
            "forward": {"x": 0., "y": 0., "z": 1.}, "up": {"x": 0., "y": 1., "z": 0.}}
    stop = threading.Event()

    class Server(socketserver.ThreadingTCPServer):
        daemon_threads = True
        allow_reuse_address = False

    def handler(kind):
        class Handler(socketserver.BaseRequestHandler):
            def handle(self):
                self.request.settimeout(1)
                greeting = {"pose": "HORUS_POSE_STREAM_V1", "video": "HORUS_PV_STREAM_V1", "control": "HORUS_UMQ_STREAM_V1"}[kind]
                try:
                    self.request.sendall((greeting + "\n").encode())
                    while not stop.wait(.05):
                        if kind == "pose":
                            self.request.sendall((json.dumps(dict(pose, timestamp=time.monotonic())) + "\n").encode())
                        elif kind == "video":
                            header = {"type": "jpeg_frame", "content_length": len(jpeg), "width": 16, "height": 12,
                                "timestamp": time.monotonic(), "tracking_valid": True, "camera_pose": pose,
                                "camera_intrinsics": {"width": 16, "height": 12, "fx": 20., "fy": 20., "cx": 8., "cy": 6.}}
                            self.request.sendall((json.dumps(header) + "\n").encode() + jpeg)
                except OSError:
                    pass
        return Handler

    root = Path(__file__).resolve().parents[2]
    connector = Path.home() / "horus_connector/scripts/field_teammate_hololens_relay.py"
    environment = dict(os.environ, ROS_DOMAIN_ID="219", ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    children = []
    servers = []
    try:
        with ExitStack() as stack:
            for kind in ("pose", "video", "control"):
                server = Server(("127.0.0.1", 0), handler(kind))
                threading.Thread(target=server.serve_forever, daemon=True).start()
                servers.append(server)
            ports = [server.server_address[1] for server in servers]
            relay_log = stack.enter_context((tmp_path / "relay.log").open("wb"))
            relay = subprocess.Popen([sys.executable, str(connector), "--name", "reception_test",
                "--hololens-host", "127.0.0.1", "--spatial-input-port", str(ports[0]),
                "--pv-port", str(ports[1]), "--umq-port", str(ports[2]), "--disable-voice", "--video-profile", "app"],
                env=environment, stdout=relay_log, stderr=subprocess.STDOUT, start_new_session=True)
            children.append(relay)
            health_log = stack.enter_context((tmp_path / "health.log").open("wb"))
            receipt = tmp_path / "reception.ndjson"
            checker = subprocess.Popen([sys.executable, str(root / "examples/tools/field_teammate_health.py"),
                "--name", "reception_test", "--receive-only", "--output", str(receipt)],
                env=environment, stdout=health_log, stderr=subprocess.STDOUT, start_new_session=True)
            children.append(checker)
            deadline = time.monotonic() + 22
            accepted = None
            while time.monotonic() < deadline:
                assert all(process.poll() is None for process in children), (tmp_path / "health.log").read_text()
                if receipt.is_file():
                    rows = [json.loads(line) for line in receipt.read_text().splitlines()]
                    accepted = next((row for row in rows if row["all_required_data_present"]), None)
                    if accepted:
                        break
                time.sleep(.2)
            assert accepted, (tmp_path / "health.log").read_text()
            np.testing.assert_allclose(accepted["map_to_head_position_m"], [2., -1., .4], atol=1e-6)
            np.testing.assert_allclose(accepted["streams"]["camera pose"]["details"]["position_m"], [2., -1., .4], atol=1e-6)
            assert accepted["streams"]["camera info"]["details"]["model"] == "plumb_bob"
            assert accepted["streams"]["video"]["details"] == {"width": 16, "height": 12}
            assert accepted["streams"]["head TF"]["count"] > 3
    finally:
        for process in children:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in children:
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        stop.set()
        for server in servers:
            server.shutdown()
            server.server_close()
