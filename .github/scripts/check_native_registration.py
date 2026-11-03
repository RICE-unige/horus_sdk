#!/usr/bin/env python3
"""Test native executables against a ROS-only app ACK peer, not a real Quest.

Run in an isolated localhost domain. This harness publishes no robot commands.
The clients themselves link the C++ middleware runtime and never execute Python.
"""
import argparse
import json
import subprocess
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("executables", nargs="+")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("horus_native_contract_peer")
    acknowledgements = node.create_publisher(String, "/horus/registration_ack", 256)
    qos = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    requests = []
    lock = threading.Lock()

    def receive(msg):
        payload = json.loads(msg.data)
        name = payload["robot_name"]
        if not name.startswith("native_contract_"):
            return
        with lock:
            requests.append(payload)
        acknowledgements.publish(String(data=json.dumps({"success": True, "robot_name": name,
            "robot_id": "app-id-" + name, "assigned_color": "#12AB34"})))

    subscription = node.create_subscription(String, "/horus/registration", receive, qos)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        for index, executable in enumerate(args.executables):
            name = f"native_contract_{index}"
            start = time.monotonic()
            result = subprocess.run([executable, "--robot-name", name, "--timeout", "5", "--unregister"],
                                    text=True, capture_output=True, timeout=15)
            if result.returncode:
                raise AssertionError(result.stdout + result.stderr)
            with lock:
                sent = [p for p in requests if p["robot_name"] == name]
            assert any(p["action"] == "register" and p["sensors"][0]["type"] == "laser_scan" for p in sent), sent
            assert any(p["action"] == "unregister" for p in sent), sent
            print(json.dumps({"executable": executable, "result": "passed", "elapsed_seconds": time.monotonic()-start}))
    finally:
        executor.shutdown()
        thread.join()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
