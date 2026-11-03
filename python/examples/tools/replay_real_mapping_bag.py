#!/usr/bin/env python3
"""Replay only measured map messages, never recorded robot command topics."""

import argparse
import json
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter

from real_mapping_data import record


def replay(mode, dataset, output_topic, ready_file=None):
    from pathlib import Path
    from nvblox_msgs.msg import Mesh
    from sensor_msgs.msg import PointCloud2
    entry = record(dataset, "bag")
    input_topic = entry["mesh_topic" if mode == "nvblox" else "points_topic"]
    message_type = Mesh if mode == "nvblox" else PointCloud2
    reader = SequentialReader()
    reader.open(StorageOptions(uri=entry["path"], storage_id="mcap"), ConverterOptions("", ""))
    reader.set_filter(StorageFilter(topics=[input_topic]))
    rclpy.init()
    node = rclpy.create_node("horus_real_mapping_replay")
    publisher = node.create_publisher(message_type, output_topic, QoSProfile(
        depth=8 if mode == "nvblox" else 1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    blocks = {}
    last_message = None
    summary = dict(provenance=entry["provenance"], bag=entry["path"], input_topic=input_topic,
                   output_topic=output_topic, timing="recorded relative timestamps, 1x target; hold completed map")
    print(json.dumps(summary), flush=True)
    if ready_file:
        Path(ready_file).write_text(json.dumps(summary) + "\n")
    try:
        while rclpy.ok() and publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        first_stamp = None
        start = time.monotonic()
        count = 0
        while rclpy.ok() and reader.has_next():
            _, serialized, stamp = reader.read_next()
            message = deserialize_message(serialized, message_type)
            if message.header.frame_id != entry["frame"]:
                raise ValueError(f"Unexpected source frame {message.header.frame_id}; no guessed TF")
            if first_stamp is None:
                first_stamp = stamp
            due = start + (stamp - first_stamp) / 1e9
            while rclpy.ok() and time.monotonic() < due:
                rclpy.spin_once(node, timeout_sec=max(0.0, min(0.05, due - time.monotonic())))
            # Coordinates already share the recorded map frame; only its label and
            # replay wall-clock stamp change for this isolated workspace.
            message.header.frame_id = "map"
            message.header.stamp = node.get_clock().now().to_msg()
            if mode == "nvblox":
                if len(message.block_indices) != len(message.blocks):
                    raise ValueError("Incomplete nvblox block update")
                if message.clear:
                    blocks.clear()
                if count == 0:
                    message.clear = True
                for index, block in zip(message.block_indices, message.blocks):
                    key = (index.x, index.y, index.z)
                    if not block.vertices:
                        blocks.pop(key, None)
                    else:
                        blocks[key] = (index, block)
            publisher.publish(message)
            last_message = message
            count += 1
        if not count:
            raise ValueError(f"No recorded messages on {input_topic}")
        print(f"Replay complete: {count} messages; holding final data, no automatic map reset", flush=True)
        # A final full inventory permits adapter reconnection after the recording.
        while rclpy.ok():
            if mode == "nvblox":
                items = list(blocks.values())
                for offset in range(0, len(items), 32):
                    message = Mesh(block_size_m=last_message.block_size_m, clear=False)
                    message.header.frame_id = "map"
                    message.header.stamp = node.get_clock().now().to_msg()
                    for index, block in items[offset:offset + 32]:
                        message.block_indices.append(index)
                        message.blocks.append(block)
                    publisher.publish(message)
                    # spin_once may return immediately on a graph event. Enforce
                    # wall-clock pacing so a repair inventory cannot overflow
                    # the adapter's bounded DDS history while it merges pages.
                    next_part = time.monotonic() + 0.2
                    while rclpy.ok() and time.monotonic() < next_part:
                        rclpy.spin_once(node, timeout_sec=min(0.02, max(0.0, next_part - time.monotonic())))
            else:
                last_message.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(last_message)
            end = time.monotonic() + 10
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("nvblox", "points-refresh"))
    parser.add_argument("--dataset", default="g1")
    parser.add_argument("--topic", required=True)
    parser.add_argument("--ready-file")
    args = parser.parse_args()
    replay(args.mode, args.dataset, args.topic, args.ready_file)
