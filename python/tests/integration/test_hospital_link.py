"""Opt-in real hospital streams through the running dual-lane bridge.

This is a transport peer, not a headset; it does not fabricate registration ACKs.
"""

import os
import struct
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("HORUS_HOSPITAL_LINK_TEST") != "1",
                              reason="Requires the owned local hospital demo")


def test_all_robot_and_map_streams_cross_both_bridge_lanes():
    from test_teach_demo_link import Peer, tlv
    peer = Peer(10000)
    topics = [("/tf", "tf2_msgs/msg/TFMessage", 1),
              ("/tf_static", "tf2_msgs/msg/TFMessage", 1),
              ("/shared_map", "nav_msgs/msg/OccupancyGrid", 2),
              ("/horus/hospital/mesh/indexed", "std_msgs/msg/UInt8MultiArray", 2)]
    for name in ("carter1", "carter2", "carter3"):
        topics += [(f"/{name}/odom", "nav_msgs/msg/Odometry", 1),
                   (f"/{name}/scan", "sensor_msgs/msg/LaserScan", 1),
                   (f"/{name}/camera/color/image_raw/compressed", "sensor_msgs/msg/CompressedImage", 2)]
    counts, sizes, samples = {}, {}, {}
    try:
        for channel, (topic, kind, lane) in enumerate(topics, 1):
            if channel == 4:
                # Match the Quest's explicit live-map subscription policy.
                peer.send(0, 2, tlv(1,b"\x02") + tlv(2,struct.pack("<H",1)) +
                    tlv(4,struct.pack("<H",channel)) + tlv(5,topic.encode()) + tlv(6,kind.encode()) +
                    tlv(7,bytes([lane])) + tlv(8,b"\x01") + tlv(21,b"\x01"))
                peer.wait(lambda: channel in peer.acks)
            else:
                peer.channel(channel, topic, message=kind, lane=lane)
        deadline = time.monotonic()+20
        while time.monotonic() < deadline:
            peer.poll(.02)
            for channel, messages in peer.messages.items():
                if messages:
                    counts[channel] = counts.get(channel, 0)+len(messages)
                    sizes[channel] = sizes.get(channel, 0)+sum(map(len, messages))
                    samples[channel] = messages[-1]
                    messages.clear()
        missing = [topic for channel,(topic,_,_) in enumerate(topics,1) if not counts.get(channel)]
        assert not missing, f"Missing actual bridge deliveries: {missing}"
        assert counts[1] > 100, counts
        assert b"HSMH" in samples[4], "Indexed geometry wire header missing"
        for channel,(topic,kind,_) in enumerate(topics,1):
            if kind.endswith("CompressedImage"):
                assert b"\xff\xd8" in samples[channel], f"No actual JPEG: {topic}"
            print(topic, counts[channel], sizes[channel])
    finally:
        peer.close()
