#!/usr/bin/env python3
"""Generate task/capability/workspace contracts using the public Python SDK."""
import json
from pathlib import Path

from horus.bridge.robot_registry import RobotRegistryClient
from horus.robot import Robot, RobotType


def build():
    robot = Robot("native_helpers", RobotType.DRONE)
    robot.add_metadata("task_config", {"custom": {"enabled": True}})
    robot.configure_waypoint_task(enabled=False, path_topic="/route", frame_id="map")
    robot.configure_go_to_point_task(goal_topic="/goal", frame_id="world",
                                     min_altitude_m=1, max_altitude_m=5,
                                     allow_unbounded_placement=True)
    robot.configure_capabilities(teleoperable=False, guidable=True)
    robot.configure_workspace_experiment(contract_version=" experiment.v1 ")
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized, client.node = False, None
    client._robot_description_resolver = None
    client._robot_description_by_robot = {}
    client._robot_description_by_id = {}
    wire = client._build_robot_config_dict(robot, robot.create_dataviz())
    return {"tasks": wire["control"]["tasks"], "workspace": wire["workspace_config"],
            "capabilities": robot.get_capabilities(), "metadata": robot.metadata["task_config"]}


if __name__ == "__main__":
    path = Path(__file__).resolve().parents[2] / "contracts/fixtures/robot_helpers.json"
    path.write_text(json.dumps(build(), indent=2) + "\n")
