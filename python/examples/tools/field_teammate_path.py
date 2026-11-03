#!/usr/bin/env python3
"""Publish a bounded, measured HoloLens trajectory independently of Rerun."""

from collections import deque
from copy import deepcopy
import argparse
import math
import time


MAX_PATH_POINTS = 20_000
MIN_DISTANCE_M = 0.02


class PoseTrail:
    def __init__(self, frame="map", max_points=MAX_PATH_POINTS, min_distance=MIN_DISTANCE_M):
        if not frame or not 2 <= max_points <= MAX_PATH_POINTS:
            raise ValueError("Use a nonempty frame and 2..20000 path points")
        if not math.isfinite(min_distance) or min_distance <= 0:
            raise ValueError("Path spacing must be finite and positive")
        self.frame = frame
        self.poses = deque(maxlen=max_points)
        self.min_distance = min_distance
        self.last_stamp = -1

    def append(self, message):
        if message.header.frame_id != self.frame:
            return False
        position, rotation = message.pose.position, message.pose.orientation
        coordinates = (position.x, position.y, position.z)
        quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
        if not all(math.isfinite(value) for value in (*coordinates, *quaternion)):
            return False
        if abs(sum(value * value for value in quaternion) - 1.0) >= 0.02:
            return False
        stamp = message.header.stamp
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            return False
        nanoseconds = stamp.sec * 1_000_000_000 + stamp.nanosec
        if nanoseconds <= self.last_stamp:
            return False
        self.last_stamp = nanoseconds
        if self.poses:
            previous = self.poses[-1].pose.position
            if math.dist(coordinates, (previous.x, previous.y, previous.z)) < self.min_distance:
                return False
        self.poses.append(message)
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default="/field_teammate_1/research/long_throw/pose")
    parser.add_argument("--output-topic", default="/field_teammate_1/trajectory")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--output-frame", default="", help="Coordinate-identical presentation frame alias; positions are not changed")
    args = parser.parse_args()
    if not args.input_topic.startswith("/") or not args.output_topic.startswith("/"):
        parser.error("Input and output topics must be absolute")

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    class TrajectoryPublisher(Node):
        def __init__(self):
            super().__init__("horus_field_trajectory")
            self.trail = PoseTrail(frame=args.frame_id)
            self.dirty = False
            self.last_publish = -math.inf
            self.subscribers = 0
            output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.publisher = self.create_publisher(Path, args.output_topic, output_qos)
            self.subscription = self.create_subscription(
                PoseStamped, args.input_topic, self.on_pose,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.timer = self.create_timer(0.5, self.publish_path)
            self.get_logger().info(f"Measured trail: {args.input_topic} -> {args.output_topic}, frame={args.frame_id}")

        def on_pose(self, message):
            self.dirty = self.trail.append(message) or self.dirty

        def publish_path(self):
            now = time.monotonic()
            subscribers = self.publisher.get_subscription_count()
            # The retained sample serves late ROS subscribers. A slow refresh
            # also serves Quest reconnects through an already subscribed bridge.
            refresh = subscribers > self.subscribers or now - self.last_publish >= 5.0
            self.subscribers = subscribers
            if not self.trail.poses or not (self.dirty or refresh):
                return
            message = Path()
            message.header.frame_id = args.output_frame or self.trail.frame
            message.header.stamp = self.trail.poses[-1].header.stamp
            message.poses = list(self.trail.poses)
            if args.output_frame and args.output_frame != self.trail.frame:
                message.poses = [deepcopy(pose) for pose in self.trail.poses]
                for pose in message.poses:
                    pose.header.frame_id = args.output_frame
            self.publisher.publish(message)
            self.last_publish = now
            self.dirty = False

    rclpy.init()
    node = None
    try:
        node = TrajectoryPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
