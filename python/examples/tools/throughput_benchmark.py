#!/usr/bin/env python3
"""Cross-language serializer throughput benchmark (Python reference).

Measures how many registration payloads per second the Python SDK builds. This
is the single-core, GIL-bound baseline the native C++/Rust SDKs are compared
against (their benchmarks add a multi-threaded figure). Run with an optional
payload count:

    PYTHONPATH=python python3 python/examples/tools/throughput_benchmark.py 50000
"""

from __future__ import annotations

import os
import sys
import time

os.environ.setdefault("HORUS_SDK_NO_BANNER", "1")

from horus.bridge.robot_registry import RobotRegistryClient  # noqa: E402
from horus.robot import Robot, RobotType  # noqa: E402
from horus.sensors import Camera  # noqa: E402


def _build_client() -> RobotRegistryClient:
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    client._robot_description_resolver = None
    client._robot_description_by_robot = {}
    client._robot_description_by_id = {}
    return client


def main() -> int:
    total = int(sys.argv[1]) if len(sys.argv) > 1 else 50000

    client = _build_client()
    robot = Robot(name="robot_0", robot_type=RobotType.WHEELED)
    robot.add_sensor(
        Camera(
            name="camera_0",
            frame_id="camera_link",
            topic="/robot_0/camera/image_raw/compressed",
        )
    )
    dataviz = robot.create_dataviz()

    client._build_robot_config_dict(robot, dataviz)  # warmup

    sink = 0
    start = time.perf_counter()
    for _ in range(total):
        payload = client._build_robot_config_dict(robot, dataviz)
        sink += len(payload["sensors"])
    elapsed = time.perf_counter() - start
    single = total / elapsed if elapsed > 0 else 0.0

    print(
        f"python serializer throughput: single={single:.0f} payloads/s "
        f"(GIL-bound, single core) sink={sink}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
