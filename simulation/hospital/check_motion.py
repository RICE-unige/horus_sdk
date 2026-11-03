"""Exercise simulated teleop, independent motion and HORUS-to-Nav2 task topics.

This deliberately moves the three simulated robots. It never addresses hardware.
"""

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy

ROBOTS = ("carter1", "carter2", "carter3")


def xy(message):
    p = message.pose.pose.position
    return (p.x, p.y)


def yaw(message):
    q = message.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class MotionCheck(Node):
    def __init__(self):
        super().__init__("hospital_motion_check")
        self.truth, self.odom, self.origins, self.status = {}, {}, {}, {}
        self.subscriptions_owned = []
        self.teleop, self.goals, self.cancels, self.waypoints = {}, {}, {}, {}
        for name in ROBOTS:
            self.teleop[name] = self.create_publisher(Twist, f"/{name}/teleop_cmd_vel", 2)
            self.goals[name] = self.create_publisher(PoseStamped, f"/{name}/horus/goal_pose", 2)
            self.cancels[name] = self.create_publisher(String, f"/{name}/horus/goal_cancel", 2)
            self.waypoints[name] = self.create_publisher(NavPath, f"/{name}/horus/waypoint_path", 2)
            self.subscriptions_owned += [
                self.create_subscription(Odometry, f"/{name}/ground_truth/odom", lambda m,n=name: self.truth.update({n:m}), 3),
                self.create_subscription(Odometry, f"/{name}/odom", lambda m,n=name: self.odom.update({n:m}), 3),
                self.create_subscription(String, f"/{name}/horus/goal_status", lambda m,n=name: self.status.update({n:json.loads(m.data)}), 5),
                self.create_subscription(String, f"/{name}/horus/waypoint_status", lambda m,n=name: self.status.update({n+"_path":json.loads(m.data)}), 5),
            ]
        self.subscriptions_owned.append(self.create_subscription(TFMessage, "/tf_static", self.frames,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def frames(self, message):
        for value in message.transforms:
            if value.header.frame_id == "global_odom" and value.child_frame_id.endswith("/odom"):
                self.origins[value.child_frame_id.split("/")[0]] = value.transform

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if predicate():
                return True
        return False

    def global_pose(self, name):
        pose = deepcopy(self.odom[name].pose.pose)
        origin = self.origins[name]
        q = origin.rotation
        heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        x,y = pose.position.x, pose.position.y
        pose.position.x = origin.translation.x + math.cos(heading)*x - math.sin(heading)*y
        pose.position.y = origin.translation.y + math.sin(heading)*x + math.cos(heading)*y
        heading += yaw(self.odom[name])
        pose.orientation.z, pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        return pose, heading

    def run(self):
        if not self.spin_until(lambda: all(n in self.truth and n in self.odom and n in self.origins for n in ROBOTS), 30):
            raise RuntimeError(f"Missing simulator streams: truth={list(self.truth)}, odom={list(self.odom)}, origins={list(self.origins)}")
        results = []
        for name in ROBOTS:
            start_yaw = yaw(self.truth[name])
            for direction in (1,-1):
                turn = Twist()
                turn.angular.z = direction*.35
                deadline = time.monotonic()+2
                while time.monotonic() < deadline:
                    self.teleop[name].publish(turn)
                    rclpy.spin_once(self, timeout_sec=.04)
                self.teleop[name].publish(Twist())
                self.spin_until(lambda: False, .5)
                if direction == 1:
                    rotation = abs(math.atan2(math.sin(yaw(self.truth[name])-start_yaw),
                                              math.cos(yaw(self.truth[name])-start_yaw)))
            _, odom_yaw = self.global_pose(name)
            yaw_error = abs(math.atan2(math.sin(odom_yaw-yaw(self.truth[name])),
                                      math.cos(odom_yaw-yaw(self.truth[name]))))
            if rotation < .2 or yaw_error > .1:
                raise RuntimeError(f"{name}: physical rotation={rotation}, odometry yaw error={yaw_error}")
            before = {n:xy(self.truth[n]) for n in ROBOTS}
            command = Twist()
            command.linear.x = .18
            deadline = time.monotonic()+3
            while time.monotonic() < deadline:
                self.teleop[name].publish(command)
                rclpy.spin_once(self, timeout_sec=.04)
            for _ in range(5):
                self.teleop[name].publish(Twist())
                self.spin_until(lambda: False, .05)
            self.spin_until(lambda: False, 1)
            displacement = {n:math.dist(before[n], xy(self.truth[n])) for n in ROBOTS}
            row = {"robot":name, "teleop_displacement_m":displacement,
                   "physical_rotation_rad":rotation, "odometry_yaw_error_rad":yaw_error}
            results.append(row)
            if displacement[name] < .12 or any(displacement[n] > .06 for n in ROBOTS if n!=name):
                raise RuntimeError(f"Independent physical teleop failed: {row}")
            pose, heading = self.global_pose(name)
            goal = PoseStamped()
            goal.header.frame_id = "global_odom"
            goal.pose = pose
            goal.pose.position.x += .6*math.cos(heading)
            goal.pose.position.y += .6*math.sin(heading)
            goal.pose.position.z = 0.0
            self.status.pop(name, None)
            self.goals[name].publish(goal)
            complete = self.spin_until(lambda: self.status.get(name, {}).get("status") in (
                "goal_reached", "goal_failed", "goal_cancelled"), 120)
            row["goal_status"] = self.status.get(name)
            row["physical_position"] = xy(self.truth[name])
            row["goal_position"] = [goal.pose.position.x, goal.pose.position.y]
            row["physical_goal_error_m"] = math.dist(row["physical_position"], row["goal_position"])
            if not complete or row["goal_status"]["status"] != "goal_reached":
                raise RuntimeError(f"Navigation did not complete: {row}")
            if row["physical_goal_error_m"] > .3:
                raise RuntimeError(f"Navigation reported success away from physical goal: {row}")
            # Exercise cancellation through the same HORUS topic, not a direct Nav2 bypass.
            pose, heading = self.global_pose(name)
            goal.pose = pose
            goal.pose.position.x += 2*math.cos(heading)
            goal.pose.position.y += 2*math.sin(heading)
            goal.pose.position.z = 0.0
            self.status.pop(name, None)
            self.goals[name].publish(goal)
            if not self.spin_until(lambda: self.status.get(name,{}).get("status") == "goal_active", 15):
                raise RuntimeError(f"Cancellation test never became active: {self.status.get(name)}")
            self.cancels[name].publish(String(data="validation_cancel"))
            if not self.spin_until(lambda: self.status.get(name,{}).get("status") == "goal_cancelled", 15):
                raise RuntimeError(f"Cancel acknowledgement missing: {self.status.get(name)}")
            row["cancel_status"] = self.status[name]
            self.spin_until(lambda: False, .5)
            stopped = xy(self.truth[name])
            self.spin_until(lambda: False, 1)
            row["post_cancel_motion_m"] = math.dist(stopped, xy(self.truth[name]))
            if row["post_cancel_motion_m"] > .05:
                raise RuntimeError(f"Robot kept moving after cancellation: {row}")
            # Two short, physically reachable waypoints exercise the second action path.
            pose, heading = self.global_pose(name)
            path = NavPath()
            path.header.frame_id = "global_odom"
            for distance in (.3, .6):
                waypoint = PoseStamped()
                waypoint.header.frame_id = "global_odom"
                waypoint.pose = deepcopy(pose)
                waypoint.pose.position.x += distance*math.cos(heading)
                waypoint.pose.position.y += distance*math.sin(heading)
                waypoint.pose.position.z = 0.0
                path.poses.append(waypoint)
            self.status.pop(name+"_path", None)
            self.waypoints[name].publish(path)
            if not self.spin_until(lambda: self.status.get(name+"_path",{}).get("status") in ("path_completed", "path_failed", "path_cancelled"), 120):
                raise RuntimeError("Waypoint result timeout")
            row["waypoint_status"] = self.status[name+"_path"]
            if row["waypoint_status"]["status"] != "path_completed":
                raise RuntimeError(f"Waypoint action failed: {row}")
            print(json.dumps(row), flush=True)
        return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = MotionCheck()
    report = {"passed": False}
    try:
        report["robots"] = node.run()
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        for name in ROBOTS:
            node.cancels[name].publish(String(data="validation_finished"))
            node.teleop[name].publish(Twist())
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
