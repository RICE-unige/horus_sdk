import pytest

from horus.bridge.robot_registry import RobotRegistryClient
from horus.dataviz import DataViz, VisualizationType
from horus.robot import WorkspaceVisualization


def client():
    registry = RobotRegistryClient.__new__(RobotRegistryClient)
    registry.ros_initialized = False
    registry.node = None
    return registry


def test_costmaps_are_independent_and_do_not_replace_occupancy():
    viz = DataViz("navigation")
    viz.add_occupancy_grid("/map")
    viz.add_costmap("/global_costmap/costmap")
    viz.add_costmap("/local_costmap/costmap", alpha=0.8, height_offset=0.012)
    entries = client()._build_global_visualizations_payload([viz])
    assert len(entries) == 3
    costmaps = {entry["topic"]: entry for entry in entries if entry["type"] == "costmap"}
    assert len(costmaps) == 2
    assert costmaps["/local_costmap/costmap"]["costmap"]["alpha"] == 0.8
    for entry in costmaps.values():
        assert entry["scope"] == "global"
        assert entry["frame"] == "map"
        assert entry["enabled"] is True
        assert entry["transport_lane"] == "bulk_replaceable"
        assert entry["costmap"]["show_unknown_space"] is False
        assert entry["costmap"]["stale_timeout"] == 5.0


def test_costmap_reregistration_updates_same_topic_only():
    viz = DataViz("navigation")
    viz.add_costmap("/a")
    viz.add_costmap("/b")
    viz.add_costmap(" /a ", alpha=0.4, show_unknown_space=True, tf_topic="/robot/tf", stale_timeout=0)
    assert len(viz.visualizations) == 2
    entries = client()._build_global_visualizations_payload([viz, viz])
    assert len(entries) == 2
    entry = next(e for e in entries if e["topic"] == "/a")
    assert entry["costmap"] == {"alpha": 0.4, "show_unknown_space": True,
        "height_offset": 0.01, "stale_timeout": 0.0, "tf_topic": "/robot/tf"}


def test_costmaps_support_workspace_only_registration_and_disable():
    resource = WorkspaceVisualization("costmaps")
    viz = resource.create_dataviz()
    viz.add_costmap("/global_costmap/costmap")
    viz.visualizations[0].enabled = False
    config = client()._build_robot_config_dict(resource, viz)
    assert config["entity_kind"] == "workspace_visualization"
    assert config["global_visualizations"][0]["type"] == VisualizationType.COSTMAP.value
    assert config["global_visualizations"][0]["enabled"] is False


@pytest.mark.parametrize("options", [
    {"topic": " "}, {"frame_id": ""}, {"tf_topic": None},
    {"alpha": -0.1}, {"alpha": 1.1}, {"alpha": float("nan")},
    {"alpha": True}, {"alpha": "0.5"}, {"height_offset": float("inf")},
    {"height_offset": -0.1}, {"height_offset": 2.0}, {"stale_timeout": -1},
    {"stale_timeout": float("nan")}, {"show_unknown_space": "false"},
])
def test_costmap_rejects_invalid_settings(options):
    with pytest.raises(ValueError):
        DataViz("navigation").add_costmap(**{"topic": "/costmap", **options})
