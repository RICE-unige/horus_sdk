"""Opt-in real model / ROS CLI test; use a dedicated localhost ROS domain."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import pytest

pytest.importorskip("rclpy")
pytest.importorskip("torch")
pytest.importorskip("transformers")
import cv2
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage


def test_actual_converter_keeps_original_and_source_header(tmp_path):
    model = os.environ.get("HORUS_PORTAL_TEST_MODEL_CACHE")
    source_path = os.environ.get("HORUS_PORTAL_TEST_IMAGE")
    if not model or not source_path:
        pytest.skip("Set HORUS_PORTAL_TEST_MODEL_CACHE and HORUS_PORTAL_TEST_IMAGE for opt-in validation")
    rgb = cv2.cvtColor(cv2.imread(source_path), cv2.COLOR_BGR2RGB)
    source_bytes = rgb.tobytes()
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node("horus_portal_validation", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    source = node.create_publisher(Image, "/horus/portal_validation/source", qos_profile_sensor_data)
    received = []
    node.create_subscription(CompressedImage, "/horus/portal_validation/sbs", received.append, qos_profile_sensor_data)
    stamps = set()
    log_path = tmp_path / "converter.log"
    with log_path.open("w") as log:
        process = subprocess.Popen([sys.executable, "-m", "horus.teaching.camera_portal",
            "--input", "/horus/portal_validation/source", "--output", "/horus/portal_validation/sbs",
            "--model-cache", model, "--max-age", "1"], stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 30
            last = 0.
            while len(received) < 3 and time.monotonic() < deadline:
                assert process.poll() is None, log_path.read_text()
                if time.monotonic() - last >= .1:
                    image = Image(width=rgb.shape[1], height=rgb.shape[0], step=rgb.shape[1]*3,
                                  encoding="rgb8", data=source_bytes)
                    image.header.frame_id = "original_optical"
                    image.header.stamp = node.get_clock().now().to_msg()
                    stamps.add((image.header.stamp.sec, image.header.stamp.nanosec))
                    source.publish(image)
                    last = time.monotonic()
                executor.spin_once(timeout_sec=.02)
            assert len(received) >= 3, log_path.read_text()
            for output in received:
                assert output.header.frame_id == "original_optical"
                assert (output.header.stamp.sec, output.header.stamp.nanosec) in stamps
                decoded = cv2.imdecode(np.frombuffer(output.data, np.uint8), cv2.IMREAD_COLOR)
                assert decoded.shape == (rgb.shape[0], rgb.shape[1]*2, 3)
            assert rgb.tobytes() == source_bytes
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)
        assert process.returncode == 0, log_path.read_text()
