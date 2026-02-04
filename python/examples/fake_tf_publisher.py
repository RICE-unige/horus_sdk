import argparse
import math
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    sys.exit(1)


def quaternion_from_yaw(yaw_rad):
    half = yaw_rad * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class FakeTFPublisher(Node):
    def __init__(self, robot_name, map_frame, base_frame, rate_hz, radius, height, omega, scale, publish_camera):
        super().__init__("horus_fake_tf_publisher")
        self.robot_name = robot_name
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.rate_hz = max(rate_hz, 0.1)
        self.radius = radius
        self.height = height
        self.omega = omega
        self.scale = scale
        self.publish_camera = publish_camera
        self.start_time = time.time()

        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", 1)

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._on_timer)

        if self.publish_camera:
            self._publish_static_camera()

        self.get_logger().info(
            f"Publishing fake TF for '{self.robot_name}' on /tf at {self.rate_hz} Hz"
        )

    def _publish_static_camera(self):
        parent = f"{self.robot_name}/{self.base_frame}"
        child = f"{self.robot_name}/camera_link"
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = 0.2 * self.scale
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = 0.1 * self.scale
        qx, qy, qz, qw = quaternion_from_yaw(0.0)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

        static_msg = TFMessage(transforms=[msg])
        self.tf_static_pub.publish(static_msg)
        self.get_logger().info("Published static camera_link TF")

    def _on_timer(self):
        now = self.get_clock().now().to_msg()
        elapsed = time.time() - self.start_time
        angle = elapsed * self.omega

        x = math.cos(angle) * self.radius * self.scale
        y = math.sin(angle) * self.radius * self.scale
        z = self.height * self.scale

        yaw = angle + math.pi * 0.5
        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        parent = self.map_frame
        child = f"{self.robot_name}/{self.base_frame}"

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_pub.publish(TFMessage(transforms=[tf]))


def build_parser():
    parser = argparse.ArgumentParser(
        description="Publish fake TF frames for Horus MR testing."
    )
    parser.add_argument("--robot-name", default="test_bot", help="Robot namespace prefix")
    parser.add_argument("--map-frame", default="map", help="Fixed map frame name")
    parser.add_argument("--base-frame", default="base_link", help="Base frame name (no prefix)")
    parser.add_argument("--rate", type=float, default=30.0, help="Publish rate (Hz)")
    parser.add_argument("--radius", type=float, default=1.0, help="Circle radius in meters")
    parser.add_argument("--height", type=float, default=0.0, help="Z height in meters")
    parser.add_argument("--omega", type=float, default=0.5, help="Angular speed (rad/s)")
    parser.add_argument("--scale", type=float, default=1.0, help="Position scale multiplier")
    parser.add_argument("--static-camera", action="store_true", help="Publish a static camera_link")
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    rclpy.init()
    node = FakeTFPublisher(
        robot_name=args.robot_name,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
        rate_hz=args.rate,
        radius=args.radius,
        height=args.height,
        omega=args.omega,
        scale=args.scale,
        publish_camera=args.static_camera,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
