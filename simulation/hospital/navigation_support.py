"""Connect HORUS task topics to Nav2 actions using the simulator's time domain."""

from copy import deepcopy
import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from std_msgs.msg import String


class HospitalNavigation(Node):
    def __init__(self):
        super().__init__("hospital_navigation_tasks")
        self.states = {}
        self.pending = {}
        self.subscriptions_owned = []
        for name in ("carter1", "carter2", "carter3"):
            for kind, action, topic in (("goal", NavigateToPose, "navigate_to_pose"),
                                         ("path", NavigateThroughPoses, "navigate_through_poses")):
                key = (name, kind)
                self.states[key] = {"client": ActionClient(self, action, f"/{name}/{topic}"),
                    "action": action, "phase": "idle", "handle": None, "cancel": False, "generation": 0,
                    "total": 0, "last_feedback": 0,
                    "publisher": self.create_publisher(String, f"/{name}/horus/{'goal' if kind=='goal' else 'waypoint'}_status", 5)}
            self.subscriptions_owned += [
                self.create_subscription(PoseStamped, f"/{name}/horus/goal_pose", lambda m, n=name: self.request(n, "goal", m), 5),
                self.create_subscription(Path, f"/{name}/horus/waypoint_path", lambda m, n=name: self.request(n, "path", m), 5),
                self.create_subscription(String, f"/{name}/horus/goal_cancel", lambda m, n=name: self.cancel(n), 5),
            ]
        self.timer = self.create_timer(0.05, self.flush)

    def status(self, key, status, index=-1, error=""):
        state = self.states[key]
        state["publisher"].publish(String(data=json.dumps({"robot_name": key[0], "status": status,
            "state": status, "current_index": index, "total": state["total"],
            "frame_id": "global_odom", "ts_unix_ms": round(time.time()*1000), "error": error})))

    def request(self, name, kind, message):
        poses = [message] if kind == "goal" else message.poses
        if not poses or any(not all(math.isfinite(value) for value in
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
             pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)) for pose in poses):
            self.status((name, kind), f"{kind}_failed", error="Empty or non-finite task")
            return
        self.cancel(name, clear_pending=False)
        self.pending[name] = (kind, deepcopy(message), time.monotonic())
        self.status((name, kind), f"{kind}_pending")

    def cancel(self, name, clear_pending=True):
        if clear_pending:
            self.pending.pop(name, None)
        for kind in ("goal", "path"):
            state = self.states[(name, kind)]
            state["cancel"] = True
            if state["handle"] is not None and state["phase"] == "active":
                state["phase"] = "cancelling"
                state["handle"].cancel_goal_async()

    def flush(self):
        for name, (kind, message, requested) in list(self.pending.items()):
            if time.monotonic() - requested > 30:
                del self.pending[name]
                self.status((name, kind), f"{kind}_failed", error="Nav2 was unavailable or cancellation did not finish within 30s")
                continue
            if any(self.states[(name, candidate)]["phase"] != "idle" for candidate in ("goal", "path")):
                continue
            key = (name, kind)
            state = self.states[key]
            if not state["client"].server_is_ready():
                continue
            del self.pending[name]
            goal = state["action"].Goal()
            if kind == "goal":
                message.header.frame_id = message.header.frame_id or "global_odom"
                message.header.stamp = Time()
                goal.pose = message
                state["total"] = 1
            else:
                for pose in message.poses:
                    pose.header.frame_id = pose.header.frame_id or message.header.frame_id or "global_odom"
                    pose.header.stamp = Time()
                goal.poses = message.poses
                state["total"] = len(message.poses)
            state.update(phase="pending", handle=None, cancel=False, generation=state["generation"]+1)
            generation = state["generation"]
            future = state["client"].send_goal_async(goal,
                feedback_callback=lambda feedback, k=key, g=generation: self.feedback(k, g, feedback))
            future.add_done_callback(lambda result, k=key, g=generation: self.accepted(k, g, result))

    def accepted(self, key, generation, future):
        state = self.states[key]
        try:
            handle = future.result()
        except Exception as error:
            state["phase"] = "idle"
            self.status(key, f"{key[1]}_failed", error=str(error))
            return
        if not handle.accepted:
            state["phase"] = "idle"
            self.status(key, f"{key[1]}_failed", error="Nav2 rejected task")
            return
        state.update(handle=handle, phase="active")
        if state["cancel"] or generation != state["generation"]:
            state["phase"] = "cancelling"
            handle.cancel_goal_async()
        else:
            self.status(key, f"{key[1]}_active", 0)
        handle.get_result_async().add_done_callback(lambda result: self.finished(key, generation, result))

    def feedback(self, key, generation, message):
        state = self.states[key]
        if generation == state["generation"] and time.monotonic()-state["last_feedback"] > .5:
            remaining = getattr(message.feedback, "number_of_poses_remaining", state["total"])
            self.status(key, f"{key[1]}_active", state["total"]-remaining)
            state["last_feedback"] = time.monotonic()

    def finished(self, key, generation, future):
        state = self.states[key]
        if generation != state["generation"]:
            return
        try:
            result = future.result()
            status = ("goal_reached" if key[1] == "goal" else "path_completed") if result.status == 4 else f"{key[1]}_cancelled" if result.status == 5 else f"{key[1]}_failed"
            self.status(key, status, state["total"] if result.status == 4 else -1,
                        getattr(result.result, "error_msg", ""))
        except Exception as error:
            self.status(key, f"{key[1]}_failed", error=str(error))
        finally:
            state.update(phase="idle", handle=None)


def main():
    rclpy.init()
    node = HospitalNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for name in ("carter1", "carter2", "carter3"):
            node.cancel(name)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
