"""Check visual-link coverage, rigid camera mounts and odometry against physics."""

import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry


def heading(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def frames_ready(buffer, truth, links):
    return all(name in truth and buffer.can_transform("global_odom", name+"/base_link",
        rclpy.time.Time.from_msg(truth[name].header.stamp)) and all(buffer.can_transform(
        "global_odom", name+"/"+link, rclpy.time.Time()) for link in (*links, "camera_optical"))
        for name in ("carter1", "carter2", "carter3"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    links = [link.get("name") for link in ET.parse(args.urdf).getroot().findall("link")]
    rclpy.init()
    node = Node("hospital_frame_check")
    buffer, truth, parents, violations = Buffer(), {}, {}, []
    subscriptions = []

    def frames(message, static):
        for value in message.transforms:
            old = parents.get(value.child_frame_id)
            if old and old != value.header.frame_id:
                violations.append(f"{value.child_frame_id} has parents {old} and {value.header.frame_id}")
            parents[value.child_frame_id] = value.header.frame_id
            if static:
                buffer.set_transform_static(value, "hospital")
            else:
                buffer.set_transform(value, "hospital")

    subscriptions.append(node.create_subscription(TFMessage, "/tf", lambda m: frames(m, False), 100))
    subscriptions.append(node.create_subscription(TFMessage, "/tf_static", lambda m: frames(m, True),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    for name in ("carter1", "carter2", "carter3"):
        subscriptions.append(node.create_subscription(Odometry, f"/{name}/ground_truth/odom",
            lambda m,n=name: truth.update({n:m}), 3))
    deadline = time.monotonic()+30
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        if frames_ready(buffer, truth, links):
            break
    report = {"passed": False, "parents": parents, "violations": violations, "robots": []}
    try:
        assert not violations, violations
        assert frames_ready(buffer, truth, links), "Timed out waiting for the complete public TF tree and physical poses"
        for name in ("carter1", "carter2", "carter3"):
            for link in links:
                buffer.lookup_transform("global_odom", name+"/"+link, rclpy.time.Time())
            timestamp = rclpy.time.Time.from_msg(truth[name].header.stamp)
            base = buffer.lookup_transform("global_odom", name+"/base_link", timestamp).transform
            camera = buffer.lookup_transform(name+"/base_link", name+"/camera_optical", rclpy.time.Time()).transform
            position = [base.translation.x, base.translation.y, base.translation.z]
            actual = truth[name].pose.pose
            delta = math.dist(position, [actual.position.x,actual.position.y,actual.position.z])
            yaw_error = abs(math.atan2(math.sin(heading(base.rotation)-heading(actual.orientation)),
                                      math.cos(heading(base.rotation)-heading(actual.orientation))))
            offset = [camera.translation.x,camera.translation.y,camera.translation.z]
            assert math.dist(offset, [.35,0,.58]) < 1e-6, offset
            assert delta < .05, (name,"physical position disagreement",delta)
            assert yaw_error < .03, (name,"physical yaw disagreement",yaw_error)
            report["robots"].append(dict(name=name, visual_links=len(links),
                physical_position_error_m=delta, physical_yaw_error_rad=yaw_error,
                compared_stamp_ns=timestamp.nanoseconds, camera_offset=offset))
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2))
        print(json.dumps(report["robots"], indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
