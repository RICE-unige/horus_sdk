"""Capture actual ROS receipts and sensor metadata, not just topic advertisements."""

import argparse
import json
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan, CompressedImage, Imu, PointCloud2
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, UInt8MultiArray
from rosgraph_msgs.msg import Clock

INDEXED_MESH_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("hospital_stream_check")
    counts, last, arrivals = defaultdict(int), {}, defaultdict(list)
    clocks = []
    sensor_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
    latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = []

    def receive(topic, message):
        counts[topic] += 1
        last[topic] = message
        arrivals[topic].append(time.monotonic())
        if isinstance(message, Clock):
            clocks.append(message.clock.sec + message.clock.nanosec*1e-9)

    topics = [("/clock", Clock, sensor_qos), ("/tf", TFMessage, sensor_qos),
              ("/shared_map", OccupancyGrid, latch), ("/hospital/simulation/status", String, sensor_qos),
              ("/horus/hospital/mesh/indexed", UInt8MultiArray, INDEXED_MESH_QOS),
              ("/horus/hospital/mesh/indexed_manifest", String, sensor_qos)]
    for name in ("carter1", "carter2", "carter3"):
        topics += [(f"/{name}/scan", LaserScan, sensor_qos), (f"/{name}/odom", Odometry, sensor_qos),
                   (f"/{name}/imu", Imu, sensor_qos),
                   (f"/{name}/camera/obstacles", PointCloud2, sensor_qos),
                   (f"/{name}/ground_truth/odom", Odometry, sensor_qos),
                   (f"/{name}/camera/camera_info", CameraInfo, sensor_qos),
                   (f"/{name}/camera/color/image_raw", Image, sensor_qos),
                   (f"/{name}/camera/depth/image_raw", Image, sensor_qos),
                   (f"/{name}/camera/color/image_raw/compressed", CompressedImage, sensor_qos),
                   (f"/{name}/map", OccupancyGrid, latch)]
    for topic, kind, qos in topics:
        subscriptions.append(node.create_subscription(kind, topic, lambda message, t=topic: receive(t, message), qos))
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
    report = {"duration_wall_s": args.duration, "topics": {}}
    for topic, kind, qos in topics:
        row = {"received": counts[topic]}
        times = arrivals[topic]
        if len(times) > 1:
            row["receipt_hz"] = (len(times) - 1) / (times[-1] - times[0])
            row["largest_gap_s"] = max(np.diff(times)).item()
        message = last.get(topic)
        if hasattr(message, "header"):
            row["frame"] = message.header.frame_id
            row["stamp"] = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if isinstance(message, LaserScan):
            finite = np.asarray(message.ranges)[np.isfinite(message.ranges)]
            row.update(beams=len(message.ranges), finite_hits=int(np.isfinite(message.ranges).sum()),
                       minimum=float(finite.min()) if finite.size else None,
                       maximum=float(finite.max()) if finite.size else None)
        elif isinstance(message, CameraInfo):
            row.update(width=message.width, height=message.height, k=list(message.k))
        elif isinstance(message, Image):
            row.update(width=message.width, height=message.height, encoding=message.encoding, step=message.step, bytes=len(message.data))
            if message.encoding in ("rgb8", "rgba8", "bgr8", "bgra8"):
                channels = 4 if "a8" in message.encoding else 3
                image = np.ndarray((message.height, message.width, channels), np.uint8, buffer=message.data,
                                   strides=(message.step, channels, 1))
                if message.encoding.startswith("rgb"):
                    image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR if channels == 4 else cv2.COLOR_RGB2BGR)
                cv2.imwrite(str(args.output.parent / (topic.split("/")[1] + "-camera.png")), image)
                row.update(mean=float(image.mean()), std=float(image.std()))
            elif message.encoding == "32FC1":
                values = np.ndarray((message.height, message.width), np.dtype(">f4" if message.is_bigendian else "<f4"),
                                    buffer=message.data, strides=(message.step, 4))
                good = np.isfinite(values) & (values > 0)
                row.update(valid_depth_fraction=float(good.mean()), depth_min=float(values[good].min()) if good.any() else None,
                           depth_max=float(values[good].max()) if good.any() else None)
        elif isinstance(message, OccupancyGrid):
            values = np.asarray(message.data)
            row.update(width=message.info.width, height=message.info.height, resolution=message.info.resolution,
                       known_cells=int((values >= 0).sum()), occupied_cells=int((values > 50).sum()))
        elif isinstance(message, Odometry):
            p = message.pose.pose.position
            row.update(position=[p.x, p.y, p.z], linear_x=message.twist.twist.linear.x)
        elif isinstance(message, UInt8MultiArray):
            row.update(bytes=len(message.data), indexed_header_valid=bytes(message.data[:4]) == b"HSMH")
        elif isinstance(message, PointCloud2):
            row.update(points=message.width*message.height,point_step=message.point_step,bytes=len(message.data))
        elif isinstance(message, String):
            row["status"] = json.loads(message.data)
        report["topics"][topic] = row
    if len(clocks) > 1:
        report["simulation_to_wall_ratio"] = (clocks[-1]-clocks[0]) / (arrivals["/clock"][-1]-arrivals["/clock"][0])
    report["missing_topics"] = [topic for topic, row in report["topics"].items() if not row["received"]]
    report["invalid_topics"] = [topic for topic, row in report["topics"].items()
                                if row.get("indexed_header_valid") is False]
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps({topic: row["received"] for topic, row in report["topics"].items()}, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
