#!/usr/bin/env python3
"""Verify both TF demo binaries on an isolated ROS domain, not their console text."""
import argparse
import json
import math
import os
import subprocess
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("executables", nargs="+")
    args = parser.parse_args()
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not os.environ.get("ROS_DOMAIN_ID"):
        raise RuntimeError("Run on an explicit isolated ROS_DOMAIN_ID with ROS_LOCALHOST_ONLY=1")
    rclpy.init()
    node = rclpy.create_node("native_tf_qualification")
    try:
        for index, executable in enumerate(args.executables):
            name = f"native_tf_{index}"
            dynamic, fixed = [], []

            def receive(message):
                dynamic.extend(t for t in message.transforms if t.child_frame_id == f"{name}/base_link")

            sub = node.create_subscription(TFMessage, "/tf", receive, 100)
            process = subprocess.Popen([executable, "--robot-name", name, "--rate-hz", "20", "--cycles", "200"],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            late = None
            try:
                start = time.monotonic()
                while len(dynamic) < 40 and time.monotonic() - start < 8:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    assert process.poll() is None, process.communicate()
                assert len(dynamic) >= 40, f"Only {len(dynamic)} moving frames received"
                late = node.create_subscription(TFMessage, "/tf_static", fixed.append, QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                end = time.monotonic() + 3
                while not fixed and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
                actual = {t.child_frame_id: t for m in fixed for t in m.transforms}
                for frame, xyz in (("camera_link", (0.3, 0., 0.5)), ("laser_link", (0., 0., 0.4))):
                    t = actual[f"{name}/{frame}"]
                    assert t.header.frame_id == f"{name}/base_link"
                    assert (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z) == xyz
                stamps = []
                for t in dynamic:
                    p, q = t.transform.translation, t.transform.rotation
                    assert t.header.frame_id == "map" and math.isclose(p.z, 0.2)
                    assert math.isclose(p.x * p.x + p.y * p.y, 1., abs_tol=1e-9)
                    assert math.isclose(q.z * q.z + q.w * q.w, 1., abs_tol=1e-9)
                    stamps.append(t.header.stamp.sec + t.header.stamp.nanosec / 1e9)
                assert all(b > a for a, b in zip(stamps, stamps[1:])), "TF timestamp did not advance"
                assert abs(time.time() - stamps[-1]) < 5, "TF stamp uses uptime instead of ROS/wall time"
                assert abs(dynamic[-1].transform.translation.y - dynamic[0].transform.translation.y) > 0.3
                rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
                assert 15 < rate < 25, rate
                process.terminate()
                out, err = process.communicate(timeout=3)
                assert process.returncode == 0, out + err
                print(json.dumps(dict(executable=executable, result="passed", received=len(dynamic),
                                      measured_hz=round(rate, 2), retained_fixed_frames=2,
                                      ros_timestamps=True, graceful_shutdown=True)), flush=True)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate(timeout=3)
                node.destroy_subscription(sub)
                if late is not None:
                    node.destroy_subscription(late)
            for option in ("nan", "0", "-1", "241"):
                bad = subprocess.run([executable, "--rate-hz", option], capture_output=True, timeout=3)
                assert bad.returncode != 0, f"Invalid rate {option} accepted"
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
