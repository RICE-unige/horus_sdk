import importlib
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

EXAMPLES = Path(__file__).parents[1] / "examples"
sys.path.insert(0, str(EXAMPLES))
path_module = importlib.import_module("tools.field_teammate_path")


def pose(x=0.0, sec=1, frame="map"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame, stamp=SimpleNamespace(sec=sec, nanosec=0)),
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=0.0, z=1.6),
                             orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))


def test_trail_keeps_real_height_pose_orientation_and_source_timestamp():
    trail = path_module.PoseTrail()
    source = pose(x=0.3, sec=30)
    assert trail.append(source)
    assert trail.poses[0] is source
    assert trail.poses[0].pose.position.z == 1.6
    assert trail.poses[0].header.stamp.sec == 30


def test_stationary_jitter_does_not_grow_history():
    trail = path_module.PoseTrail()
    assert trail.append(pose())
    for sec in range(2, 100):
        assert not trail.append(pose(x=0.005, sec=sec))
    assert len(trail.poses) == 1
    assert trail.append(pose(x=0.04, sec=100))


def test_trail_is_bounded_and_does_not_decimate_recent_positions():
    trail = path_module.PoseTrail(max_points=3)
    for index in range(8):
        assert trail.append(pose(x=index * 0.03, sec=index + 1))
    assert len(trail.poses) == 3
    assert [message.pose.position.x for message in trail.poses] == pytest.approx([0.15, 0.18, 0.21])


@pytest.mark.parametrize("mutate", [
    lambda p: setattr(p.header, "frame_id", "unrelated_origin"),
    lambda p: setattr(p.pose.position, "x", math.nan),
    lambda p: setattr(p.pose.orientation, "w", 0.0),
    lambda p: setattr(p.pose.orientation, "x", math.inf),
    lambda p: setattr(p.header.stamp, "nanosec", 1_000_000_000),
    lambda p: setattr(p.header.stamp, "sec", -1),
])
def test_invalid_input_does_not_poison_history_or_block_valid_followup(mutate):
    trail = path_module.PoseTrail()
    invalid = pose(sec=100)
    mutate(invalid)
    assert not trail.append(invalid)
    assert not trail.poses
    assert trail.append(pose(sec=1))


def test_duplicate_and_out_of_order_poses_are_not_appended():
    trail = path_module.PoseTrail()
    assert trail.append(pose(sec=5))
    assert not trail.append(pose(x=1, sec=5))
    assert not trail.append(pose(x=2, sec=4))
    assert len(trail.poses) == 1
    assert trail.append(pose(x=0.05, sec=6))


@pytest.mark.parametrize("kwargs", [{"max_points": 1}, {"max_points": 20001},
    {"frame": ""}, {"min_distance": math.nan}, {"min_distance": 0.0}])
def test_invalid_history_settings_are_rejected(kwargs):
    with pytest.raises(ValueError):
        path_module.PoseTrail(**kwargs)


@pytest.mark.skipif(os.environ.get("HORUS_FIELD_ROS_TEST") != "1", reason="Requires an isolated ROS test run")
def test_actual_ros_path_and_late_joiner_delivery():
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as RosPath
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    topic = f"/horus_field_path_test_{os.getpid()}"
    environment = dict(os.environ, ROS_DOMAIN_ID="218", ROS_LOCALHOST_ONLY="1",
                       ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    process = subprocess.Popen([sys.executable, str(EXAMPLES / "tools/field_teammate_path.py"),
        "--input-topic", topic + "/pose", "--output-topic", topic + "/path"], env=environment,
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    context = Context()
    rclpy.init(context=context, domain_id=218)
    node = rclpy.create_node("field_path_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        publisher = node.create_publisher(PoseStamped, topic + "/pose",
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        received, late = [], []
        node.create_subscription(RosPath, topic + "/path", received.append,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        def wait_for(condition, timeout=12, tick=None):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                assert process.poll() is None, process.stderr.read()
                if tick is not None:
                    tick()
                executor.spin_once(timeout_sec=0.05)
                if condition():
                    return
            raise AssertionError(f"ROS trajectory delivery timed out; observed path sizes: {[len(p.poses) for p in received]}")

        wait_for(lambda: publisher.get_subscription_count() > 0)
        source = PoseStamped()
        source.header.frame_id = "map"
        source.pose.orientation.w = 1.0
        source.pose.position.z = 1.6
        for index in range(5):
            source.header.stamp.sec = index + 1
            source.pose.position.x = index * 0.04
            publisher.publish(source)
            # Best-effort discovery does not guarantee that the first sample
            # arrives. Repeating the same stamp must not duplicate the trail.
            wait_for(lambda: received and received[-1].header.stamp.sec == index + 1,
                     tick=lambda: publisher.publish(source))
        wait_for(lambda: received and len(received[-1].poses) == 5)
        path = received[-1]
        assert path.header.frame_id == "map" and path.header.stamp.sec == 5
        assert [p.pose.position.x for p in path.poses] == pytest.approx([0, .04, .08, .12, .16])
        assert all(p.pose.position.z == 1.6 for p in path.poses)
        node.create_subscription(RosPath, topic + "/path", late.append,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        wait_for(lambda: bool(late))
        assert len(late[-1].poses) == 5
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        process.stderr.close()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
