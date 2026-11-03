"""Opt-in real DDS test for generic mesh delivery; no robot/Quest traffic."""

import json
import os
import time

import pytest

if os.environ.get("HORUS_MAPPING_ROS_TESTS") != "1":
    pytest.skip("explicit isolated ROS integration opt-in required", allow_module_level=True)
if os.environ.get("ROS_DOMAIN_ID") != "231" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise RuntimeError("live mesh integration requires local-only domain 231")

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from std_msgs.msg import String, UInt8MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from horus.mapping.ros_mesh import arguments, create_node
from horus.mapping import decode_live_header


@pytest.mark.parametrize("source_type", ["Marker", "MarkerArray"])
def test_duplicate_change_delete_and_late_join_repair(source_type):
    args, _ = arguments(["--input-topic", "/independent/site", "--input-type", source_type,
                         "--output-topic", "/vendor/custom_mesh", "--triangles-per-page", "1"])
    rclpy.init()
    adapter = create_node(args)
    source = rclpy.create_node("independent_map_producer")
    receiver = rclpy.create_node("headset_emulation")
    executor = SingleThreadedExecutor()
    for node in (adapter, source, receiver): executor.add_node(node)
    pub = source.create_publisher(Marker if source_type == "Marker" else MarkerArray, args.input_topic, 64)
    received, late = [], []
    sub = receiver.create_subscription(UInt8MultiArray, args.output_topic + "/indexed",
                                       lambda m: received.append(bytes(m.data)), 64)
    request = receiver.create_publisher(String, args.output_topic + "/indexed_request", 16)

    def until(predicate, seconds=5):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            if predicate(): return
        assert predicate(), "DDS delivery did not complete"

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline: executor.spin_once(timeout_sec=0.01)

    def publish(m):
        pub.publish(m if source_type == "Marker" else MarkerArray(markers=[m]))

    try:
        until(lambda: pub.get_subscription_count() == 1 and request.get_subscription_count() == 1)
        m = Marker()
        m.header.frame_id = "map"
        m.ns, m.id, m.type = "surface", 7, Marker.TRIANGLE_LIST
        m.pose.orientation.w = 1.
        m.scale.x = m.scale.y = m.scale.z = 1.
        m.color.r = m.color.a = 1.
        m.points = [Point(x=0., y=0., z=0.), Point(x=1., y=0., z=0.), Point(x=0., y=1., z=0.)]
        publish(m)
        until(lambda: len(received) == 1)
        first = received[0]
        for value in range(20):
            m.header.stamp.sec = value
            publish(m)
            spin_for(0.02)
        spin_for(0.2)
        assert len(received) == 1
        assert adapter.map.state.unchanged_pages == 20
        m.color.g = 1.
        publish(m)
        until(lambda: len(received) == 2)
        assert decode_live_header(received[-1]).revision == 2

        # No source republishes: new observers explicitly recover current cache state.
        later = receiver.create_subscription(UInt8MultiArray, args.output_topic + "/indexed",
                                             lambda msg: late.append(bytes(msg.data)), 64)
        until(lambda: adapter.publisher.get_subscription_count() == 2)
        spin_for(0.2)
        assert not late
        request.publish(String(data=json.dumps({"request": "snapshot"})))
        retry = receiver.create_timer(2, lambda: request.publish(String(data=json.dumps({"request": "snapshot"}))))
        until(lambda: len(late) >= 1)
        receiver.destroy_timer(retry)
        assert late[0] == received[1] and late[0] != first
        assert adapter.map.state.page_count == 1
        m.action = Marker.DELETE
        publish(m)
        until(lambda: decode_live_header(received[-1]).operation == 1)
        assert adapter.map.state.page_count == 0
        spin_for(2.1)
        request.publish(String(data=json.dumps({"request": "pages",
            "epoch": str(adapter.map.state.epoch), "ids": [1]})))
        before = len(late)
        until(lambda: len(late) > before)
        assert decode_live_header(late[-1]).operation == 1
        assert adapter.rejected == 0

        # First geometry may arrive before its TF. It must recover without a
        # second geometry publication and must not reappear after a later delete.
        m.action = Marker.ADD
        m.header.frame_id = "source_map"
        m.header.stamp.sec = 0
        publish(m)
        until(lambda: len(adapter.deferred) == 1)
        broadcaster = StaticTransformBroadcaster(source)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "map", "source_map"
        transform.transform.rotation.w = 1.
        transform.transform.translation.x = 5.
        broadcaster.sendTransform(transform)
        until(lambda: not adapter.deferred and adapter.map.state.page_count == 1)
        assert adapter.rejected == 0
        m.action = Marker.DELETE
        publish(m)
        until(lambda: adapter.map.state.page_count == 0)
    finally:
        executor.shutdown()
        for node in (adapter, source, receiver): node.destroy_node()
        rclpy.shutdown()
