import json
from pathlib import Path

from horus.bridge.robot_registry import RobotRegistryClient
from horus.dataviz import DataViz


def test_costmap_matches_shared_native_fixture():
    viz = DataViz("maps")
    viz.add_costmap(" /local_costmap/costmap ", " odom ")
    # Payload serialization is pure; no ROS context or bridge is needed.
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    actual = client._serialize_visualization_payload(viz.get_enabled_visualizations()[0], "global")
    path = Path(__file__).parents[2] / "contracts/fixtures/costmap.json"
    assert actual == json.loads(path.read_text())
