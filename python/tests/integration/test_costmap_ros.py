"""Real local ROS costmap recovery and patch transport, without robot commands."""
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs")
from nav2_msgs.msg import Costmap, CostmapUpdate
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from rclpy.qos import QoSProfile, DurabilityPolicy

from horus.mapping.ros_costmap import arguments, create_node


def full(value, stamp):
    msg = Costmap()
    msg.header.frame_id = "map"
    msg.header.stamp.sec = stamp
    msg.metadata.size_x, msg.metadata.size_y = 4, 3
    msg.metadata.resolution = .05
    msg.metadata.origin.orientation.w = 1.0
    msg.data = [value] * 12
    return msg


@pytest.mark.parametrize("input_format", ["raw", "occupancy"])
def test_late_join_uses_authoritative_snapshot_then_applies_patches_and_recovers(input_format):
    context = rclpy.Context()
    rclpy.init(context=context)
    source = rclpy.create_node("costmap_test_source", context=context)
    raw = input_format == "raw"
    args, _ = arguments(["--input-topic", "/test_costmap/raw", "--input-format", input_format,
        "--updates-topic", "/test_costmap/updates", "--snapshot-service", "/test_costmap/get",
        "--output-topic", "/test_costmap/normalized", "--resync-seconds", "0.2", "--publish-hz", "30"])
    snapshot = full(254, 10)
    requests = []
    def respond(request, response):
        requests.append(request)
        response.map = snapshot
        return response
    source.create_service(GetCostmap, "/test_costmap/get", respond)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    source_map = source.create_publisher(Costmap if raw else OccupancyGrid, "/test_costmap/raw", latched)
    cached = full(0, 1)
    if not raw:
        cached = OccupancyGrid()
        cached.header.frame_id, cached.header.stamp.sec = "map", 1
        cached.info.width, cached.info.height = 4, 3
        cached.info.resolution, cached.info.origin.orientation.w = .05, 1.0
        cached.data = [0] * 12
    source_map.publish(cached)  # Old cached full map, not a complete current state.
    patches = source.create_publisher(CostmapUpdate if raw else OccupancyGridUpdate, "/test_costmap/updates", 64)
    received = []
    source.create_subscription(OccupancyGrid, "/test_costmap/normalized", received.append, 10)
    adapter = create_node(args, context=context)
    executor = rclpy.executors.SingleThreadedExecutor(context=context)
    executor.add_node(source)
    executor.add_node(adapter)
    def wait(predicate):
        deadline = time.monotonic() + 5
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert predicate()
    try:
        wait(lambda: len(received) > 0)
        assert requests
        assert all(list(msg.data) == [100] * 12 for msg in received)
        wait(lambda: patches.get_subscription_count() == 1)
        update = CostmapUpdate() if raw else OccupancyGridUpdate()
        update.header.frame_id = "map"
        update.header.stamp.sec = 11
        update.x, update.y = 1, 1
        if raw:
            update.size_x, update.size_y, update.data = 2, 1, [253, 255]
        else:
            update.width, update.height, update.data = 2, 1, [99, -1]
        snapshot = full(254, 11)
        snapshot.data[5:7] = type(snapshot.data)("B", [253, 255])
        patches.publish(update)
        wait(lambda: list(received[-1].data)[5:7] == [99, -1])
        assert list(received[-1].data) == [100] * 5 + [99, -1] + [100] * 5
        adapter.invalidate("simulated lost patch")
        snapshot = full(0, 12)
        wait(lambda: received[-1].header.stamp.sec == 12)
        assert list(received[-1].data) == [0] * 12

        # An overrun during recovery must not publish a partial reconstruction.
        from rclpy.task import Future
        from unittest.mock import Mock
        adapter.timer.cancel()
        adapter.future = Future()
        adapter.requested_at = time.monotonic()
        adapter.publisher = Mock()
        for _ in range(65):
            adapter.receive(update, False)
        assert adapter.journal_lost and not adapter.ready
        assert len(adapter.journal) <= 64
        response = GetCostmap.Response()
        response.map = snapshot
        adapter.future.set_result(response)
        adapter.tick()
        adapter.publisher.publish.assert_not_called()
        assert adapter.future is None and not adapter.ready

        # A stalled service request is removed rather than accumulating futures.
        client = Mock()
        client.service_is_ready.return_value = False
        adapter.client = client
        pending = adapter.future = Future()
        adapter.requested_at = time.monotonic() - 3
        adapter.tick()
        client.remove_pending_request.assert_called_once_with(pending)
        assert pending.cancelled() and adapter.future is None
        adapter.publisher.publish.assert_not_called()
    finally:
        executor.shutdown()
        adapter.destroy_node()
        source.destroy_node()
        context.shutdown()
