#!/usr/bin/env python3
"""Shared TF support node for the HORUS robot-description showroom.

The showroom still uses one robot_state_publisher per robot so each URDF is exercised
through the normal ROS path. Joint-state defaults and showroom floor anchors are shared
here to avoid launching two extra ROS participants per robot.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster

EXAMPLES_DIR = Path(__file__).resolve().parents[1]
if str(EXAMPLES_DIR) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_DIR))

from robot_description_showroom_specs import SHOWROOM_FLEET

ASSETS_DIR = EXAMPLES_DIR / ".local_assets" / "robot_descriptions"
JOINT_STATE_RATE_HZ = 20.0


def _movable_joint_names(urdf_path: Path) -> List[str]:
    root = ET.parse(urdf_path).getroot()
    names: List[str] = []
    for joint in root.findall("joint"):
        joint_type = str(joint.attrib.get("type", "")).strip().lower()
        if joint_type in {"", "fixed"}:
            continue
        name = str(joint.attrib.get("name", "")).strip()
        if name:
            names.append(name)
    return names


class ShowroomTfSupportNode(Node):
    def __init__(self) -> None:
        super().__init__("showroom_tf_support")
        self._joint_publishers: Dict[str, object] = {}
        self._joint_names_by_robot: Dict[str, List[str]] = {}

        for spec in SHOWROOM_FLEET:
            urdf_path = ASSETS_DIR / spec.urdf_name
            if not urdf_path.is_file():
                raise FileNotFoundError(f"Missing URDF for {spec.name}: {urdf_path}")

            joints = _movable_joint_names(urdf_path)
            if joints:
                self._joint_names_by_robot[spec.name] = joints
                self._joint_publishers[spec.name] = self.create_publisher(
                    JointState,
                    f"/{spec.name}/joint_states",
                    10,
                )

        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._tf_publisher = self.create_publisher(TFMessage, "/tf", 10)
        self._publish_static_anchors()
        self._static_timer = self.create_timer(2.0, self._publish_static_anchors)
        self._timer = self.create_timer(1.0 / JOINT_STATE_RATE_HZ, self._publish_joint_states)
        self.get_logger().info(
            f"Showroom TF support active for {len(SHOWROOM_FLEET)} robots; "
            f"publishing joint states for {len(self._joint_publishers)} robots."
        )

    def _publish_static_anchors(self) -> None:
        now = self.get_clock().now().to_msg()
        transforms: List[TransformStamped] = []
        for spec in SHOWROOM_FLEET:
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = "world"
            transform.child_frame_id = f"{spec.name}/{spec.urdf_root_frame}"
            transform.transform.translation.x = float(spec.x)
            transform.transform.translation.y = float(spec.y)
            transform.transform.translation.z = float(spec.z)
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            transforms.append(transform)

        self._static_broadcaster.sendTransform(transforms)
        self._tf_publisher.publish(TFMessage(transforms=transforms))

    def _publish_joint_states(self) -> None:
        now = self.get_clock().now().to_msg()
        for robot_name, joint_names in self._joint_names_by_robot.items():
            msg = JointState()
            msg.header.stamp = now
            msg.name = joint_names
            msg.position = [0.0] * len(joint_names)
            publisher = self._joint_publishers.get(robot_name)
            if publisher is not None:
                publisher.publish(msg)


def main() -> int:
    rclpy.init()
    node = ShowroomTfSupportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
