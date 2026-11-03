#!/usr/bin/env python3
"""Actual DDS costmap snapshots, concurrent patches and recovery in a test domain."""
import copy
import json
import os
import subprocess
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav2_msgs.msg import Costmap, CostmapUpdate
from nav2_msgs.srv import GetCostmap


def run(binary, raw):
    process = subprocess.Popen([binary, "raw" if raw else "occupancy"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    node = rclpy.create_node("costmap_contract_peer")
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    maps = []
    node.create_subscription(OccupancyGrid, "/horus_native_costmap/output", lambda msg: maps.append(msg), retained)
    full = node.create_publisher(Costmap if raw else OccupancyGrid, "/horus_native_costmap/full", retained)
    patches = node.create_publisher(CostmapUpdate if raw else OccupancyGridUpdate, "/horus_native_costmap/updates", 64)
    snapshot = Costmap()
    snapshot.header.frame_id = "map"
    snapshot.header.stamp.sec = 20
    snapshot.metadata.size_x = snapshot.metadata.size_y = 2
    snapshot.metadata.resolution = 0.05
    snapshot.metadata.origin.orientation.w = 1.0
    snapshot.data = [0, 253, 254, 255]
    calls = 0

    def patch(value, x=0, stamp=21):
        msg = CostmapUpdate() if raw else OccupancyGridUpdate()
        msg.header.frame_id = "map"
        msg.header.stamp.sec = stamp
        msg.x, msg.y = x, 0
        if raw:
            msg.size_x = msg.size_y = 1
        else:
            msg.width = msg.height = 1
        msg.data = [value]
        return msg

    def serve(request, response):
        nonlocal calls
        calls += 1
        response.map = copy.deepcopy(snapshot)
        if calls == 1:
            # Arrives while the native snapshot request is pending. It must be
            # journaled and replayed on the returned, older complete snapshot.
            patches.publish(patch(254 if raw else 100))
            snapshot.data[0] = 254
            snapshot.header.stamp.sec = 21
            time.sleep(0.1)
        return response

    def spin_until(predicate, timeout=5):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if process.poll() is not None:
                raise AssertionError(process.communicate())
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError(f"Costmap timeout: calls={calls}, maps={[list(m.data) for m in maps[-4:]]}")

    try:
        if raw:
            stale = copy.deepcopy(snapshot)
            stale.header.stamp.sec = 1
            stale.data = [0] * 4
        else:
            stale = OccupancyGrid()
            stale.header.frame_id = "map"
            stale.header.stamp.sec = 1
            stale.info.width = stale.info.height = 2
            stale.info.resolution = 0.05
            stale.info.origin.orientation.w = 1.0
            stale.data = [0] * 4
        full.publish(stale)
        end = time.monotonic() + 0.6
        spin_until(lambda: time.monotonic() >= end)
        assert not maps, "Cached full map is not a valid late-join snapshot with incremental input"
        node.create_service(GetCostmap, "/horus_native_costmap/snapshot", serve)
        spin_until(lambda: maps and list(maps[-1].data) == [100, 99, 100, -1])
        assert maps[-1].header.stamp.sec == 21
        assert maps[-1].info.width == 2 and maps[-1].info.origin.orientation.w == 1
        before = calls
        patches.publish(patch(0, x=10, stamp=22))
        spin_until(lambda: calls > before and maps and list(maps[-1].data) == [100, 99, 100, -1])
        deadline = time.monotonic() + 18
        while process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        stdout, stderr = process.communicate(timeout=1)
        assert process.returncode == 0, (stdout, stderr)
        status = json.loads(stdout.strip().splitlines()[-1])
        assert status["invalidations"] >= 1 and status["recoveries"] >= 2 and status["journal_bytes"] <= 8 * 1024 * 1024, status
        assert status["ready"] and not status["error"], status
        print(f"PASS {binary}: raw={raw}, late-join snapshot, concurrent patch replay, invalid-patch recovery; {status}", flush=True)
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.communicate(timeout=4)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
        node.destroy_node()


if __name__ == "__main__":
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not os.environ.get("ROS_DOMAIN_ID"):
        raise SystemExit("Set a dedicated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1")
    rclpy.init()
    try:
        for binary in sys.argv[1:]:
            for raw in (False, True):
                run(binary, raw)
    finally:
        rclpy.shutdown()
