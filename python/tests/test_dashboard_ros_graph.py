"""Opt-in real DDS graph checks. Run in an isolated localhost-only ROS domain."""

import os
import time
import uuid

import pytest

from horus.utils.dashboard_status import topic_graph


@pytest.mark.skipif(os.getenv("HORUS_TEST_ROS_GRAPH") != "1", reason="requires isolated ROS test domain")
def test_real_dds_endpoints_do_not_imply_receipt():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    context = Context()
    context.init(domain_id=int(os.environ["ROS_DOMAIN_ID"]))
    observer = rclpy.create_node("horus_dashboard_test_observer", context=context)
    bridge = rclpy.create_node("horus_unity_bridge", context=context)
    source = rclpy.create_node("horus_dashboard_test_source", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(bridge)
    topic = "/horus_dashboard_test_" + uuid.uuid4().hex
    received = []
    publisher = source.create_publisher(String, topic, 1)

    def wait_for(predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            executor.spin_once(timeout_sec=0.05)
        assert predicate()

    try:
        wait_for(lambda: topic_graph(observer, topic)["link"] == "NO_BRIDGE")
        subscription = bridge.create_subscription(String, topic, lambda msg: received.append(msg.data), 1)
        wait_for(lambda: topic_graph(observer, topic)["link"] == "OK")
        assert not received
        publisher.publish(String(data="dashboard probe"))
        wait_for(lambda: received == ["dashboard probe"])
        bridge.destroy_subscription(subscription)
        wait_for(lambda: topic_graph(observer, topic)["link"] == "NO_BRIDGE")
    finally:
        executor.shutdown()
        for node in (observer, bridge, source):
            node.destroy_node()
        context.shutdown()
