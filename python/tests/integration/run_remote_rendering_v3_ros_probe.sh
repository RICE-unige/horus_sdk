#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
set -u
unset CYCLONEDDS_URI || true

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$root"
export PYTHONPATH="$root/python${PYTHONPATH:+:$PYTHONPATH}"
export HORUS_V3_INTEGRATION_SCENE="${HORUS_V3_INTEGRATION_SCENE:-synthetic}"

python3 - <<'PY'
from __future__ import annotations

import importlib.util
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from horus.remote_rendering import (
    EyeFov,
    EyeView,
    V3Quaternion,
    V3Vector3,
    ViewState,
    WorkspaceState,
    unpack_v3_ros_diagnostic_frame,
)


def load_agent_module():
    path = Path("python/examples/tools/remote_map_render_agent.py").resolve()
    spec = importlib.util.spec_from_file_location("horus_v3_render_agent", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load render agent from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


agent_module = load_agent_module()
original_argv = sys.argv
sys.argv = [
    "remote_map_render_agent.py",
    "--protocol",
    "v3",
    "--transport",
    "ros_compressed",
    "--scene",
    os.environ["HORUS_V3_INTEGRATION_SCENE"],
    "--width",
    "384",
    "--height",
    "240",
    "--depth-width",
    "160",
    "--depth-height",
    "120",
    "--fps",
    "30",
    "--render-update-fps",
    "30",
    "--session-id",
    "integration-v3",
]
try:
    args = agent_module.parse_args()
finally:
    sys.argv = original_argv

rclpy.init()
agent = agent_module.RemoteMapRenderSource(args)
probe = rclpy.create_node("horus_v3_integration_probe")
executor = SingleThreadedExecutor()
executor.add_node(agent)
executor.add_node(probe)

pose_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
frame_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
publisher = probe.create_publisher(
    String,
    "/horus/remote_render/v3/view_state",
    pose_qos,
)
received: list[CompressedImage] = []
subscription = probe.create_subscription(
    CompressedImage,
    "/horus/remote_render/v3/stereo",
    received.append,
    frame_qos,
)

try:
    discovery_deadline = time.monotonic() + 5.0
    while time.monotonic() < discovery_deadline:
        executor.spin_once(timeout_sec=0.05)
        if (
            publisher.get_subscription_count() == 1
            and subscription.get_publisher_count() == 1
        ):
            break
    if publisher.get_subscription_count() != 1:
        raise SystemExit("v3 agent view-state subscription was not discovered")
    if subscription.get_publisher_count() != 1:
        raise SystemExit("v3 agent stereo publisher was not discovered")

    deadline = time.monotonic() + 30.0
    sequence = 1
    while time.monotonic() < deadline and not received:
        now = time.monotonic_ns()
        state = ViewState(
            session_id="integration-v3",
            view_sequence=sequence,
            sample_time_ns=now,
            predicted_display_time_ns=now + 13_888_889,
            predicted_display_period_ns=13_888_889,
            reference_space_id="workspace-local",
            workspace=WorkspaceState(
                3,
                V3Vector3(0, 0, 0),
                V3Quaternion(0, 0, 0, 1),
                0.1,
            ),
            left_eye=EyeView(
                V3Vector3(-0.032, 6.0, -12.0),
                V3Quaternion(0, 0, 0, 1),
                EyeFov(-0.86, 0.78, 0.82, -0.76),
            ),
            right_eye=EyeView(
                V3Vector3(0.032, 6.0, -12.0),
                V3Quaternion(0, 0, 0, 1),
                EyeFov(-0.78, 0.86, 0.82, -0.76),
            ),
            viewport_width=384,
            viewport_height=240,
            refresh_rate_hz=72.0,
        )
        message = String()
        message.data = state.to_json()
        publisher.publish(message)
        sequence += 1
        executor.spin_once(timeout_sec=0.1)

    if not received:
        raise SystemExit("no v3 stereo frame received")
    metadata, png = unpack_v3_ros_diagnostic_frame(bytes(received[-1].data))
    image = cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if metadata.session_id != "integration-v3":
        raise SystemExit(f"wrong session: {metadata.session_id}")
    if metadata.workspace_revision != 3:
        raise SystemExit(f"wrong workspace revision: {metadata.workspace_revision}")
    if metadata.source_view_sequence < 1:
        raise SystemExit(
            f"invalid source view sequence: {metadata.source_view_sequence}"
        )
    if image is None or image.shape != (266, 768, 3):
        raise SystemExit(
            f"wrong stereo image shape: {None if image is None else image.shape}"
        )
    left = image[:240, :384]
    right = image[:240, 384:]
    if np.array_equal(left, right):
        raise SystemExit("left and right eye renders are unexpectedly identical")
    if np.std(left.astype(np.float32)) < 1:
        raise SystemExit("rendered scene is blank")
    cv2.imwrite("/tmp/horus-v3-render-check.png", image[:240])
    print(
        "V3_ROS_INTEGRATION_OK",
        f"renderer={agent.renderer_name}",
        f"scene={args.scene}",
        f"frame={metadata.frame_id}",
        f"source_view={metadata.source_view_sequence}",
        f"shape={image.shape}",
        f"bytes={len(received[-1].data)}",
    )
finally:
    executor.remove_node(probe)
    executor.remove_node(agent)
    agent.close()
    probe.destroy_node()
    agent.destroy_node()
    executor.shutdown()
    if rclpy.ok():
        rclpy.shutdown()
PY
