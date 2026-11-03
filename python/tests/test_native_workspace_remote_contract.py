"""Remote view and map-only registration fixtures shared with native SDKs."""
import json
from pathlib import Path

import pytest

from horus.bridge.robot_registry import RobotRegistryClient
from horus.dataviz import DataViz
from horus.robot import WorkspaceVisualization


def test_remote_render_native_contract():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    fixtures = json.loads((Path(__file__).parents[2] / "contracts/fixtures/remote_render.json").read_text())
    for case in fixtures["cases"]:
        viz = DataViz("remote")
        viz.add_remote_rendered_map("/remote/map", "world", case["options"])
        expected = dict(fixtures["defaults"], **case["expected"])
        for key in case.get("absent", []):
            expected.pop(key)
        payload = client._serialize_visualization_payload(viz.visualizations[0], "global")
        assert payload["remote_render"] == expected, case["name"]
        assert payload["type"] == "remote_render" and payload["frame"] == "world"
        viz.add_remote_rendered_map("/remote/map", "world")
        assert len(viz.visualizations) == 1
    for options in fixtures["invalid"]:
        viz = DataViz("invalid")
        with pytest.raises(ValueError):
            viz.add_remote_rendered_map("/remote/map", "world", options)
        assert not viz.visualizations


def test_workspace_envelope_has_no_robot_controls():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    entity = WorkspaceVisualization("map_view")
    viz = entity.create_dataviz()
    assert not viz.visualizations
    viz.add_remote_rendered_map("/remote/map", "world")
    payload = client._build_robot_config_dict(entity, viz)
    assert payload["entity_kind"] == "workspace_visualization"
    assert not any(payload["capabilities"].values())
    assert not payload["robot_manager_config"]["enabled"]
    assert not any(payload["robot_manager_config"]["sections"].values())
    assert payload["sensors"] == payload["visualizations"] == []
    assert payload["control"]["drive_topic"] == ""
    assert not payload["control"]["teleop"]["enabled"]
    assert all(not value["enabled"] for value in payload["control"]["tasks"].values())
    assert payload["global_visualizations"][0]["type"] == "remote_render"
