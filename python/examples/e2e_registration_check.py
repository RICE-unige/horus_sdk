#!/usr/bin/env python3
"""
Minimal end-to-end registration check for the Unity client architecture.

Requires:
- ROS 2 running
- horus_unity_bridge running on port 10000
- HORUS MR app connected as a TCP client
"""

import os
import sys
import time

# Ensure we can import the horus package when running from repo root
script_dir = os.path.dirname(os.path.abspath(__file__))
package_root = os.path.join(script_dir, "..")
if package_root not in sys.path:
    sys.path.insert(0, package_root)

from horus.robot import Robot, RobotType
from horus.sensors import LaserScan
from horus.bridge.robot_registry import get_robot_registry_client


def main() -> int:
    robot = Robot(name="SdkBot_E2E", robot_type=RobotType.WHEELED)
    robot.add_sensor(
        LaserScan(
            name="Front Lidar",
            topic="/scan",
            frame_id="laser_frame",
            min_range=0.1,
            max_range=12.0,
            color="#00FFFF",
            point_size=0.1,
        )
    )

    dataviz = robot.create_dataviz()

    registry = get_robot_registry_client()
    ok, result = registry.register_robot(robot, dataviz, keep_alive=False)
    if not ok:
        print(f"Registration failed: {result}")
        return 1

    print(f"Registration OK: {result}")

    # Heartbeat check (optional but recommended)
    heartbeat_ok = False
    if registry.node is not None:
        deadline = time.time() + 5.0
        while time.time() < deadline:
            try:
                import rclpy

                rclpy.spin_once(registry.node, timeout_sec=0.1)
            except Exception:
                time.sleep(0.1)

            last = getattr(registry, "_last_heartbeat_time", 0)
            if last and (time.time() - last) < 2.5:
                heartbeat_ok = True
                break

    if heartbeat_ok:
        print("Heartbeat OK")
        return 0

    print("Heartbeat not detected (Unity may be disconnected)")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
