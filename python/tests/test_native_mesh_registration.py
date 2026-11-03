"""One wire fixture evaluated independently by all three SDK implementations."""
import json
from pathlib import Path

from horus.bridge.robot_registry import RobotRegistryClient
from horus.dataviz import DataViz


def test_live_mesh_registration_contract():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    path = Path(__file__).parents[2] / "contracts/fixtures/live_mesh_registration.json"
    for case in json.loads(path.read_text()):
        viz = DataViz("mesh")
        helper = {"live": viz.add_live_mesh, "nvblox": viz.add_nvblox_mesh, "mesh": viz.add_3d_mesh}[case["helper"]]
        helper("/horus/maps/live", "map", case["options"])
        actual = client._serialize_visualization_payload(viz.get_enabled_visualizations()[0], "global")
        assert actual == case["expected"], case["name"]


def test_native_map_rendering_options_contract():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    path = Path(__file__).parents[2] / "contracts/fixtures/map_render_options.json"
    for case in json.loads(path.read_text()):
        viz = DataViz("maps")
        helper = {"point_cloud": viz.add_3d_map, "gaussian_splat": viz.add_gaussian_splat_map, "octomap": viz.add_3d_octomap}[case["type"]]
        helper("/map", "odom", case["options"])
        actual = client._serialize_visualization_payload(viz.get_enabled_visualizations()[0], "global")[case["type"]]
        assert {key: actual[key] for key in case["fields"]} == case["fields"]
        assert all(key not in actual for key in case.get("absent", []))
