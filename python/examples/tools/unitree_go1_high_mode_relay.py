#!/usr/bin/env python3
"""Support runtime for the live Unitree Go1 HORUS MR example.

HORUS MR publishes empty action messages for legged robots:
    /unitree_go1/stand_up
    /unitree_go1/sit_down

The Unitree ROS 2 SDK exposes the real command as:
    /unitree_go1/legged_sdk/set_high_mode

Mode mapping used here:
    10 -> stand
    20 -> sit

This node publishes the missing front optical camera static TF expected by the
front image header. It also converts the live LaserScan into the compact
collision-risk JSON stream consumed by the HORUS collision alert DataViz layer:
    /unitree_go1/scan -> /unitree_go1/collision_risk
"""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Empty
from tf2_ros import StaticTransformBroadcaster

from horus.dataviz import CollisionRiskAnalyzer

try:
    from unitree_ros2_interface.srv import SetHighMode
except ImportError as exc:  # pragma: no cover - depends on the user's ROS workspace.
    raise SystemExit(
        "Missing unitree_ros2_interface. Source the workspace that provides it, for example:\n"
        "  source ~/horus_ws/install/setup.bash"
    ) from exc

ROBOT_NAME = "unitree_go1"
STAND_TOPIC = f"/{ROBOT_NAME}/stand_up"
SIT_TOPIC = f"/{ROBOT_NAME}/sit_down"
HIGH_MODE_SERVICE = f"/{ROBOT_NAME}/legged_sdk/set_high_mode"
SCAN_TOPIC = f"/{ROBOT_NAME}/scan"
COLLISION_RISK_TOPIC = f"/{ROBOT_NAME}/collision_risk"
CAMERA_FACE_FRAME = f"{ROBOT_NAME}/camera_face"
CAMERA_LEFT_FACE_FRAME = f"{ROBOT_NAME}/camera_left_face"
CAMERA_OPTICAL_LEFT_FACE_FRAME = f"{ROBOT_NAME}/camera_optical_left_face"
STAND_MODE = 10
SIT_MODE = 20


class UnitreeGo1SupportRelay(Node):
    def __init__(self) -> None:
        super().__init__("unitree_go1_support_relay")
        self._client = self.create_client(SetHighMode, HIGH_MODE_SERVICE)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_front_camera_static_frames()
        self.create_subscription(Empty, STAND_TOPIC, self._on_stand_up, 10)
        self.create_subscription(Empty, SIT_TOPIC, self._on_sit_down, 10)
        self._collision_risk = CollisionRiskAnalyzer(
            self,
            robot_name=ROBOT_NAME,
            source_type=CollisionRiskAnalyzer.SOURCE_LASER_SCAN,
            source_topic=SCAN_TOPIC,
            output_topic=COLLISION_RISK_TOPIC,
            threshold_m=1.2,
            publish_hz=10.0,
        )
        self.get_logger().info(
            f"Relaying {STAND_TOPIC}->{HIGH_MODE_SERVICE} mode={STAND_MODE}, "
            f"{SIT_TOPIC}->{HIGH_MODE_SERVICE} mode={SIT_MODE}."
        )
        self.get_logger().info(
            f"Publishing collision risk from {SCAN_TOPIC} to {COLLISION_RISK_TOPIC}."
        )

    def _publish_front_camera_static_frames(self) -> None:
        stamp = self.get_clock().now().to_msg()
        transforms = [
            self._make_static_transform(
                stamp,
                CAMERA_FACE_FRAME,
                CAMERA_LEFT_FACE_FRAME,
                xyz=(0.0, 0.025, 0.0),
                rpy=(0.0, 0.0, 0.0),
            ),
            self._make_static_transform(
                stamp,
                CAMERA_LEFT_FACE_FRAME,
                CAMERA_OPTICAL_LEFT_FACE_FRAME,
                xyz=(0.0, 0.0, 0.0),
                rpy=(-math.pi / 2.0, 0.0, -math.pi / 2.0),
            ),
        ]
        self._static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info(
            f"Publishing front camera static TF: {CAMERA_FACE_FRAME} -> "
            f"{CAMERA_LEFT_FACE_FRAME} -> {CAMERA_OPTICAL_LEFT_FACE_FRAME}."
        )

    @staticmethod
    def _make_static_transform(stamp, parent: str, child: str, *, xyz, rpy) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(xyz[0])
        msg.transform.translation.y = float(xyz[1])
        msg.transform.translation.z = float(xyz[2])
        qx, qy, qz, qw = UnitreeGo1SupportRelay._quaternion_from_euler(*rpy)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg

    @staticmethod
    def _quaternion_from_euler(roll: float, pitch: float, yaw: float):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _on_stand_up(self, _msg: Empty) -> None:
        self._send_mode(STAND_MODE, "Stand Up")

    def _on_sit_down(self, _msg: Empty) -> None:
        self._send_mode(SIT_MODE, "Sit Down")

    def _send_mode(self, mode: int, label: str) -> None:
        if not self._client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning(f"{label} ignored: service unavailable: {HIGH_MODE_SERVICE}")
            return

        request = SetHighMode.Request()
        request.mode = int(mode)
        future = self._client.call_async(request)
        future.add_done_callback(lambda done: self._log_result(done, label, mode))

    def _log_result(self, future, label: str, mode: int) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - runtime transport failure.
            self.get_logger().error(f"{label} mode={mode} failed: {exc}")
            return

        if bool(getattr(response, "res", False)):
            self.get_logger().info(f"{label} mode={mode} accepted by Unitree SDK.")
        else:
            self.get_logger().warning(f"{label} mode={mode} returned res=false from Unitree SDK.")


def main() -> None:
    rclpy.init()
    node = UnitreeGo1SupportRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
