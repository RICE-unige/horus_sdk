"""Opt-in ROS + CUDA + NVENC test, isolated from robot control and Quest."""

import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
import subprocess
import struct

import pytest


@pytest.mark.skipif(os.environ.get("HORUS_TEST_GPU_VIDEO") != "1",
                    reason="requires sourced ROS, CUDA, and local scene assets")
def test_ros_diagnostic_delivers_decodable_stereo_with_matching_pose(monkeypatch):
    import cv2
    import numpy as np
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    from horus.remote_rendering.v3_protocol import (
        EyeFov, EyeView, Quaternion, Vector3, ViewState, WorkspaceState,
        ROS_DIAGNOSTIC_FORMAT_VERSION, unpack_ros_diagnostic_frame,
    )

    path = Path(__file__).parents[1] / "examples/tools/remote_map_render_agent.py"
    spec = importlib.util.spec_from_file_location("diagnostic_agent_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.syspath_prepend(str(path.parent))
    monkeypatch.setattr(sys, "argv", [str(path), "--protocol", "v3", "--transport",
        "ros_compressed", "--scene", "sponza_mesh", "--width", "1152", "--height", "1152",
        "--fps", "15", "--render-update-fps", "15", "--session-id", "diagnostic-test"])
    rclpy.init()
    executor = SingleThreadedExecutor()
    agent = probe = None
    received, sent = [], set()
    try:
        agent = module.RemoteMapRenderSource(module.parse_args())
        probe = rclpy.create_node("diagnostic_stereo_probe")
        executor.add_node(agent)
        executor.add_node(probe)
        publisher = probe.create_publisher(String, agent.args.viewer_pose_topic, 4)
        probe.create_subscription(CompressedImage, agent.ros_debug_publisher.topic_name, received.append, 4)
        deadline, next_pose = time.monotonic() + 15, 0
        while time.monotonic() < deadline and len(received) < 3:
            if publisher.get_subscription_count() and time.monotonic() >= next_pose:
                sequence = len(sent) + 1
                stamp = time.monotonic_ns()
                state = ViewState(
                    "diagnostic-test", sequence, stamp, stamp + 13_888_889, 13_888_889,
                    "workspace-local", WorkspaceState(0, Vector3(0, 0, 0), Quaternion(0, 0, 0, 1), 0.1),
                    EyeView(Vector3(-0.032, 2, -4), Quaternion(0, 0, 0, 1), EyeFov(-1, 1, 1, -1)),
                    EyeView(Vector3(0.032, 2, -4), Quaternion(0, 0, 0, 1), EyeFov(-1, 1, 1, -1)),
                    1152, 1152, 72,
                )
                publisher.publish(String(data=state.to_json()))
                sent.add(sequence)
                next_pose = time.monotonic() + .2
            executor.spin_once(timeout_sec=.01)
        assert len(received) >= 3
        for frame in received:
            assert frame.format == ROS_DIAGNOSTIC_FORMAT_VERSION
            metadata, png = unpack_ros_diagnostic_frame(bytes(frame.data))
            assert metadata.session_id == "diagnostic-test"
            assert metadata.source_view_sequence in sent
            image = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
            assert image is not None
            assert image.shape[:2] == (metadata.video_height, metadata.width_per_eye * 2)
            assert image.max() > image.min()
    finally:
        if agent is not None:
            agent.close()
            executor.remove_node(agent)
            agent.destroy_node()
        if probe is not None:
            executor.remove_node(probe)
            probe.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


@pytest.mark.skipif(os.environ.get("HORUS_TEST_GPU_VIDEO") != "1",
                    reason="requires sourced ROS, NVENC, and local scene assets")
def test_encoder_drains_without_new_poses_and_preserves_all_access_units(monkeypatch, tmp_path):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    from horus.remote_rendering.v3_protocol import (
        EyeFov, EyeView, Quaternion, Vector3, ViewState, WorkspaceState,
    )

    path = Path(__file__).parents[1] / "examples/tools/remote_map_render_agent.py"
    spec = importlib.util.spec_from_file_location("pipeline_agent_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.syspath_prepend(str(path.parent))
    if os.environ.get("HORUS_TEST_POINT_HOST_READBACK") == "1":
        assert os.environ.get("HORUS_RENDER_SCENE") == "cow_lady_points"
        # Test-only reference for a same-source comparison with the old host path.
        monkeypatch.delattr(module.CudaPointRenderer, "render_video_views_cuda")
    monkeypatch.setattr(sys, "argv", [str(path), "--protocol", "v3", "--transport",
        "webrtc", "--scene", os.environ.get("HORUS_RENDER_SCENE", "sponza_mesh"),
        "--width", "1152", "--height", "1152", "--visibility-mask-height", "288",
        "--fps", "60", "--render-update-fps", "60", "--session-id", "pipeline-test",
        "--dataset-path", os.environ.get("HORUS_RENDER_DATASET", "")])
    rclpy.init()
    agent = None
    probe = None
    executor = SingleThreadedExecutor()
    received = []
    children = []
    capture_path = os.environ.get("HORUS_TEST_ENCODED_CAPTURE")
    capture = open(capture_path, "wb") if capture_path else None
    duration = float(os.environ.get("HORUS_RENDER_TEST_SECONDS", "12"))
    try:
        agent = module.RemoteMapRenderSource(module.parse_args())
        if agent.args.scene == "cow_lady_points":
            from horus.remote_rendering.real_scene import load_colored_vertex_ply, resolve_cow_lady_ply
            points, _ = load_colored_vertex_ply(resolve_cow_lady_ply())
            assert agent._renderer_name == "cuda_pose_adaptive_points"
            assert agent.source_point_count == len(points)
        probe = rclpy.create_node("remote_video_probe")
        executor.add_node(agent)
        executor.add_node(probe)
        qos = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE)
        publisher = probe.create_publisher(String, agent.args.viewer_pose_topic, qos)
        def on_frame(message):
            received.append((time.monotonic(), json.loads(message.header.frame_id.split(":", 1)[1])))
            if capture is not None:
                metadata = message.header.frame_id.encode("utf-8")
                encoded = bytes(message.data)
                capture.write(struct.pack("<II", len(metadata), len(encoded)))
                capture.write(metadata)
                capture.write(encoded)
        subscription = probe.create_subscription(
            CompressedImage, agent.image_publisher.topic_name, on_frame, qos)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and (
            publisher.get_subscription_count() < 1 or subscription.get_publisher_count() < 1
        ):
            executor.spin_once(timeout_sec=0.01)
        assert publisher.get_subscription_count() == 1
        assert subscription.get_publisher_count() == 1
        if os.environ.get("HORUS_TEST_WEBRTC_BRIDGE") == "1":
            bridge = Path.home() / "horus_ws/install/horus_unity_bridge/lib/horus_unity_bridge/horus_unity_bridge_node"
            config = Path.home() / "horus_ws/install/horus_unity_bridge/share/horus_unity_bridge/config/bridge_config.yaml"
            children.append(subprocess.Popen([str(bridge), "--ros-args", "--params-file", str(config),
                "-p", "tcp_port:=19040", "-p", "horuslink_bulk_port:=19041"],
                stdout=(tmp_path / "bridge.log").open("w"), stderr=subprocess.STDOUT))
            rtc_python = Path.home() / ".cache/horus/remote_render/rtc-probe-venv/bin/python"
            probe_script = Path(__file__).parent / "integration/probe_remote_webrtc_stream.py"
            children.append(subprocess.Popen([str(rtc_python), str(probe_script), "--seconds", str(duration - 10)],
                stdout=(tmp_path / "webrtc.log").open("w"), stderr=subprocess.STDOUT))
        start = time.monotonic()
        sequence = 0
        next_pose = start
        while time.monotonic() - start < duration:
            now = time.monotonic()
            if now >= next_pose:
                sequence += 1
                next_pose += 1 / 60
                stamp = time.monotonic_ns()
                x = math.sin(now - start) * 0.3
                state = ViewState(
                    "pipeline-test", sequence, stamp, stamp + 13_888_889, 13_888_889,
                    "workspace-local", WorkspaceState(0, Vector3(0, 0, 0), Quaternion(0, 0, 0, 1), 0.1),
                    EyeView(Vector3(x - 0.032, 2, -4), Quaternion(0, 0, 0, 1), EyeFov(-1, 1, 1, -1)),
                    EyeView(Vector3(x + 0.032, 2, -4), Quaternion(0, 0, 0, 1), EyeFov(-1, 1, 1, -1)),
                    1152, 1152, 72,
                )
                publisher.publish(String(data=state.to_json()))
            executor.spin_once(timeout_sec=0.001)
        # No new views: the final encoded frame must still reach the subscriber.
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert agent.webrtc_image_type == "h264"
        indices = [frame["encoder_frame_index"] for _, frame in received]
        assert len(indices) > 30
        assert indices == list(range(indices[0], indices[-1] + 1))
        steady = [(stamp, data) for stamp, data in received if start + 2 < stamp < start + duration]
        fps = (len(steady) - 1) / (steady[-1][0] - steady[0][0])
        print(f"PC_PIPELINE fps={fps:.2f} frames={len(received)} "
              f"gpu_ms={agent.last_cuda_ms:.2f} render_ms={agent.last_render_ms:.2f} "
              f"encoded_backlog={len(agent._h264_encoder._async_outputs)} "
              f"nvenc_pending={len(agent._h264_encoder._pending_metadata)} "
              f"final_pose={sequence} geometry={agent.source_point_count}")
        assert received[-1][1]["source_view_sequence"] == sequence
        assert not agent._h264_encoder._pending_metadata
        if children:
            assert children[1].wait(timeout=10) == 0, (tmp_path / "webrtc.log").read_text()
            result = json.loads((tmp_path / "webrtc.log").read_text().splitlines()[-1])
            print("WEBRTC_BRIDGE", result)
            assert result['decoded'] > 30
            assert result['unmatched'] == 0
    finally:
        if capture is not None:
            capture.close()
        for child in children:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=5)
        if agent is not None:
            agent.close()
            executor.remove_node(agent)
            agent.destroy_node()
        if probe is not None:
            executor.remove_node(probe)
            probe.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
