#!/usr/bin/env python3
"""Qualify renderer view/frame exchange on an isolated ROS domain, without a headset."""
import argparse
import copy
import io
import json
from pathlib import Path
import subprocess
import time

import numpy as np
from PIL import Image as PillowImage
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from horus.remote_rendering.v3_protocol import unpack_ros_diagnostic_frame
from horus.remote_rendering.frame_protocol import decode_frame_marker


def check(binary, mode, index):
    node = rclpy.create_node(f"native_remote_peer_{index}")
    frames, statuses = [], []
    sensor_qos = QoSProfile(depth=32, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Image if mode == "rgb" else CompressedImage,
                             "/native_remote/image", frames.append, sensor_qos)
    node.create_subscription(String, "/native_remote/status",
                             lambda m: statuses.append(json.loads(m.data)), 16)
    publisher = node.create_publisher(String, "/native_remote/view", sensor_qos)
    keyframe = node.create_publisher(String, "/native_remote/image/request_keyframe", 1)
    template = json.loads((Path(__file__).parents[2] / "contracts/fixtures/remote_protocol.json").read_text())["view"]
    for eye in ("left_eye", "right_eye"):
        template[eye]["position"] = {"x": -0.03 if eye == "left_eye" else 0.03, "y": 0, "z": 0}
        template[eye]["rotation"] = {"x": 0, "y": 0, "z": 0, "w": 1}
        template[eye]["fov"] = {"left": -1, "right": 1, "up": 1, "down": -1}
    process = subprocess.Popen([binary, mode], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        start, last_send, sequence = time.monotonic(), 0.0, 0
        while process.poll() is None and time.monotonic() - start < 14:
            elapsed = time.monotonic() - start
            if publisher.get_subscription_count() and elapsed - last_send > 0.05 and (elapsed < 3 or 4.5 < elapsed < 7):
                sequence += 1
                view = copy.deepcopy(template)
                view["view_sequence"] = sequence
                view["sample_time_ns"] = sequence * 1000000
                if elapsed > 4.5:
                    view["view_stream_id"] = "stream-b"
                    view["workspace"]["revision"] = 5
                publisher.publish(String(data=json.dumps(view)))
                if sequence % 5 == 0:
                    publisher.publish(String(data=json.dumps(view)))  # duplicated source pose
                if sequence % 7 == 0:
                    publisher.publish(String(data="{invalid"))
                if mode == "h264" and sequence % 10 == 0:
                    keyframe.publish(String(data="idr"))
                last_send = elapsed
            rclpy.spin_once(node, timeout_sec=0.02)
        stdout, stderr = process.communicate(timeout=3)
        assert process.returncode == 0, stdout + stderr
        final = json.loads(stdout.strip().splitlines()[-1])
        assert final["state"] == "stopped" and not final["running"]
        assert final["views_rejected"] > 0 and final["frames_discarded"] > 0, final
        assert final["frames_published"] >= 10, final
        assert not final["error"], final
        assert any(s["state"] == "stale_view" for s in statuses), statuses
        assert len(frames) >= 10
        revisions, frame_ids = set(), set()
        for frame in frames:
            if mode == "diagnostic":
                metadata, png = unpack_ros_diagnostic_frame(bytes(frame.data))
                metadata = metadata.to_dict()
                image = np.asarray(PillowImage.open(io.BytesIO(png)).convert("RGB"))
                assert image.shape == (48, 384, 3) and np.count_nonzero(image) >= 6
            else:
                assert frame.header.frame_id.startswith("horus.remote_render.v3:")
                metadata = json.loads(frame.header.frame_id.split(":", 1)[1])
                if mode == "rgb":
                    image = np.frombuffer(bytes(frame.data), np.uint8).reshape(64, 384, 3)
                    assert frame.encoding == "rgb8" and frame.step == 384 * 3
                    assert decode_frame_marker(image[-16:]) == metadata["frame_id"] % 65536
                else:
                    assert frame.format == "h264; stream-format=byte-stream; alignment=au"
                    assert bytes(frame.data).startswith((b"\0\0\0\1", b"\0\0\1"))
            assert metadata["frame_id"] not in frame_ids, "Duplicate output frame"
            frame_ids.add(metadata["frame_id"])
            revisions.add(metadata["workspace_revision"])
        assert revisions == {4, 5}, revisions
        print(json.dumps({"binary": binary, "mode": mode, "result": "passed",
                          "frames": len(frames), "rejected_views": final["views_rejected"],
                          "duplicate_frames_discarded": final["frames_discarded"],
                          "stale_detection": True, "workspace_restart": True}))
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--nvenc", action="store_true", help="Also use the actual NVIDIA encoder")
    parser.add_argument("binaries", nargs="+")
    args = parser.parse_args()
    rclpy.init()
    try:
        for index, binary in enumerate(args.binaries):
            for mode in ["diagnostic", "rgb"] + (["h264"] if args.nvenc else []):
                check(binary, mode, index)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
