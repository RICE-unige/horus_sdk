import pytest

from horus.dataviz.mesh_display import mesh_display_options


def test_mesh_display_defaults_are_opt_in_and_copy_input():
    original = {"lod_enabled": True}
    result = mesh_display_options(original)
    assert result["lod_enabled"] is True
    assert result["spacewarp_enabled"] is False
    assert original == {"lod_enabled": True}


@pytest.mark.parametrize("value", [None, [], {"lod_enabled": "false"}, {"lod_memory_mib": True},
    {"lod_memory_mib": 193}, {"lod_error_pixels": float("nan")}, {"lod_error_pixels": 0},
    {"spacewarp_enabled": 1}, {"unknown": True}])
def test_mesh_display_rejects_ambiguous_or_unbounded_profiles(value):
    with pytest.raises(ValueError):
        mesh_display_options(value)


def test_mesh_display_survives_registration_payload():
    from types import SimpleNamespace
    from horus.bridge.robot_registry import RobotRegistryClient
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized, client.node = False, None
    visualization = SimpleNamespace(data_source=SimpleNamespace(topic="/map_mesh", frame_id="map"),
        viz_type="mesh", render_options={"display": {"lod_enabled": True, "spacewarp_enabled": True}})
    payload = client._serialize_visualization_payload(visualization)
    assert payload["mesh"]["display"]["lod_enabled"] is True
    assert payload["mesh"]["display"]["spacewarp_enabled"] is True
    visualization.render_options = {}
    assert "display" not in client._serialize_visualization_payload(visualization)["mesh"]
