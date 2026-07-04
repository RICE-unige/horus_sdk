#!/usr/bin/env python3
"""Offline stand-in for a HoloLens field-teammate companion app.

This node publishes everything a real HoloLens companion would publish for a
field teammate -- pose (TF), status, localization confidence, a first-person
video frame, and guidance state -- and answers operator guidance requests with
acknowledge / complete responses. It lets the full "On the Map / In the Team"
contract be exercised end to end without a headset.

The topic names are derived from the SDK's ``FieldTeammateConfig`` so this mock
can never drift from the registration contract.

Run (ROS 2 sourced, from the repo root):
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/tools/mock_field_teammate_node.py --name field_teammate_1
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import sys

# Keep the import quiet; this is a runtime tool, not an interactive session.
os.environ.setdefault("HORUS_SDK_NO_BANNER", "1")

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Float32, String
    from tf2_msgs.msg import TFMessage
    from tf2_ros import TransformBroadcaster
except Exception as exc:  # pragma: no cover - depends on a sourced ROS 2 env
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    raise SystemExit(1)

from horus.robot import FieldTeammateConfig

# A genuinely valid 1x1 JPEG so downstream decoders accept the FPV frame.
_PLACEHOLDER_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQEAYABgAAD/2wBDAP//////////////////////////////"
    "////////////////////////////////////////////////////2wBDAf//////"
    "////////////////////////////////////////////////////////////////"
    "////////////////wAARCAABAAEDASIAAhEBAxEB/8QAFAABAAAAAAAAAAAAAAAAAA"
    "AAAv/EABQQAQAAAAAAAAAAAAAAAAAAAAD/xAAUAQEAAAAAAAAAAAAAAAAAAAAA/8QA"
    "FBEBAAAAAAAAAAAAAAAAAAAAAP/aAAwDAQACEQMRAD8AvwA//9k="
)


def _yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class MockFieldTeammateNode(Node):
    def __init__(self, name: str, wearable: str, rate_hz: float) -> None:
        super().__init__("mock_field_teammate")
        self._name = name
        self._contract = FieldTeammateConfig.from_values(name=name, wearable_type=wearable)
        topics = self._contract.to_payload()["topics"]

        self._map_frame = "map"
        self._base_frame = self._contract.base_frame
        self._camera_frame = self._contract.camera_frame

        self._tf_broadcaster = TransformBroadcaster(self)
        self._status_pub = self.create_publisher(String, topics["status"], 10)
        self._confidence_pub = self.create_publisher(Float32, topics["localization_confidence"], 10)
        self._fpv_pub = self.create_publisher(CompressedImage, topics["first_person_video"], 10)
        self._guidance_state_pub = self.create_publisher(String, topics["guidance_state"], 10)
        self._guidance_response_pub = self.create_publisher(String, topics["guidance_response"], 10)

        self._guidance_request_sub = self.create_subscription(
            String, topics["guidance_request"], self._on_guidance_request, 10
        )
        for key in ("guidance_annotation", "guidance_route", "guidance_warning", "audio"):
            self.create_subscription(
                String, topics[key], self._make_logger(key), 10
            )

        self._phase = 0.0
        self._rate_hz = max(1.0, float(rate_hz))
        self._frame_period = 1.0 / self._rate_hz
        self._timer = self.create_timer(self._frame_period, self._tick)
        self._state = "idle"

        self.get_logger().info(
            f"Mock field teammate '{name}' ({wearable}) publishing on {self._base_frame}; "
            f"answering guidance on {topics['guidance_request']}."
        )

    def _make_logger(self, kind: str):
        def _handler(msg: String) -> None:
            self.get_logger().info(f"received {kind}: {msg.data[:160]}")

        return _handler

    def _on_guidance_request(self, msg: String) -> None:
        self.get_logger().info(f"guidance request: {msg.data[:160]}")
        self._state = "acting"
        self._publish_guidance_state()
        self._guidance_response_pub.publish(
            String(data=json.dumps({"teammate": self._name, "response": "acknowledge"}))
        )

    def _publish_guidance_state(self) -> None:
        self._guidance_state_pub.publish(
            String(data=json.dumps({"teammate": self._name, "state": self._state}))
        )

    def _tick(self) -> None:
        now = self.get_clock().now().to_msg()
        self._phase += self._frame_period * 0.4

        # Walk a slow 1 m circle so the teammate visibly moves on the map.
        x = math.cos(self._phase)
        y = math.sin(self._phase)
        yaw = self._phase + math.pi / 2.0
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)

        base_tf = TransformStamped()
        base_tf.header.stamp = now
        base_tf.header.frame_id = self._map_frame
        base_tf.child_frame_id = self._base_frame
        base_tf.transform.translation.x = x
        base_tf.transform.translation.y = y
        base_tf.transform.translation.z = 1.6  # head height
        base_tf.transform.rotation.x = qx
        base_tf.transform.rotation.y = qy
        base_tf.transform.rotation.z = qz
        base_tf.transform.rotation.w = qw

        camera_tf = TransformStamped()
        camera_tf.header.stamp = now
        camera_tf.header.frame_id = self._base_frame
        camera_tf.child_frame_id = self._camera_frame
        camera_tf.transform.translation.x = 0.10
        camera_tf.transform.translation.z = 0.05
        camera_tf.transform.rotation.w = 1.0

        self._tf_broadcaster.sendTransform([base_tf, camera_tf])

        # Localization confidence oscillates through the HIGH/MEDIUM/LOW bands so
        # the confidence-gated guidance path can be exercised.
        confidence = 0.75 + 0.25 * math.sin(self._phase * 0.5)
        self._confidence_pub.publish(Float32(data=float(confidence)))

        self._status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "teammate": self._name,
                        "state": self._state,
                        "battery": 0.9,
                        "localization_confidence": round(confidence, 3),
                    }
                )
            )
        )

        fpv = CompressedImage()
        fpv.header.stamp = now
        fpv.header.frame_id = self._camera_frame
        fpv.format = "jpeg"
        fpv.data = _PLACEHOLDER_JPEG
        self._fpv_pub.publish(fpv)

        self._publish_guidance_state()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--name", default="field_teammate_1", help="Teammate entity name/namespace.")
    parser.add_argument("--wearable", default="hololens2", help="Wearable device type.")
    parser.add_argument("--rate", type=float, default=20.0, help="Publish rate in Hz.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    node = MockFieldTeammateNode(args.name, args.wearable, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
