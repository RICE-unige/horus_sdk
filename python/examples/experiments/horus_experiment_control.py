#!/usr/bin/env python3
"""Send start/stop/status commands to the HORUS MR experiment recorder."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["start", "stop", "mark", "status"])
    parser.add_argument("--topic", default="/horus/experiments/control")
    parser.add_argument("--run-id", default="")
    parser.add_argument("--experiment", default="manual")
    parser.add_argument("--condition", default="default")
    parser.add_argument("--name", default="")
    parser.add_argument("--notes", default="")
    parser.add_argument("--repeat", type=int, default=3, help="Publish repeats for best-effort delivery.")
    parser.add_argument("--period", type=float, default=0.2)
    return parser.parse_args()


class ExperimentControlNode(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("horus_experiment_control")
        self.publisher = self.create_publisher(String, topic, 10)


def main() -> int:
    args = parse_args()
    payload = {
        "action": args.action,
        "run_id": args.run_id,
        "experiment": args.experiment,
        "condition": args.condition,
        "name": args.name,
        "notes": args.notes,
    }

    rclpy.init()
    node = ExperimentControlNode(args.topic)
    try:
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        for _ in range(max(1, args.repeat)):
            node.publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(max(0.0, args.period))
        print(msg.data, flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
