"""Tests for SDK 3D-map workflow helpers."""

from horus.utils.map_3d_workflow import (
    Map3DMode,
    MeshTransport,
    MeshUpdatePolicy,
    build_map_3d_process_specs,
    build_pointcloud_to_mesh_converter_command,
    coerce_mesh_transport_to_marker,
    resolve_converter_update_mode,
    resolve_map_3d_mode,
)


def test_resolve_map_3d_mode_defaults_to_off():
    mode, warnings = resolve_map_3d_mode(None, with_3d_map=False, with_3d_mesh=False)
    assert mode == Map3DMode.OFF
    assert warnings == []


def test_resolve_map_3d_mode_supports_legacy_pointcloud_alias():
    mode, warnings = resolve_map_3d_mode(None, with_3d_map=True, with_3d_mesh=False)
    assert mode == Map3DMode.POINTCLOUD
    assert any("Deprecated flags detected" in warning for warning in warnings)


def test_resolve_map_3d_mode_supports_legacy_mesh_alias():
    mode, warnings = resolve_map_3d_mode(None, with_3d_map=False, with_3d_mesh=True)
    assert mode == Map3DMode.MESH
    assert any("Deprecated flags detected" in warning for warning in warnings)


def test_resolve_map_3d_mode_prefers_mesh_when_both_legacy_aliases_are_set():
    mode, warnings = resolve_map_3d_mode(None, with_3d_map=True, with_3d_mesh=True)
    assert mode == Map3DMode.MESH
    assert any("selecting mesh mode by precedence" in warning for warning in warnings)


def test_resolve_map_3d_mode_explicit_flag_overrides_legacy_aliases():
    mode, warnings = resolve_map_3d_mode("pointcloud", with_3d_map=False, with_3d_mesh=True)
    assert mode == Map3DMode.POINTCLOUD
    assert any("Ignoring legacy mode alias" in warning for warning in warnings)


def test_resolve_converter_update_mode_snapshot_uses_requested_interval():
    update_mode, republish = resolve_converter_update_mode(
        MeshUpdatePolicy.SNAPSHOT,
        republish_interval=9.0,
    )
    assert update_mode == "once"
    assert republish == 9.0


def test_resolve_converter_update_mode_snapshot_without_keepalive():
    update_mode, republish = resolve_converter_update_mode(
        MeshUpdatePolicy.SNAPSHOT,
        republish_interval=0.0,
    )
    assert update_mode == "once"
    assert republish == 0.0


def test_resolve_converter_update_mode_periodic_uses_default_interval():
    update_mode, republish = resolve_converter_update_mode(
        MeshUpdatePolicy.PERIODIC,
        republish_interval=0.0,
    )
    assert update_mode == "on_change"
    assert republish == 2.0


def test_resolve_converter_update_mode_continuous_disables_republish():
    update_mode, republish = resolve_converter_update_mode(
        MeshUpdatePolicy.CONTINUOUS,
        republish_interval=7.0,
    )
    assert update_mode == "continuous"
    assert republish == 0.0


def test_build_map_3d_process_specs_for_mesh_includes_converter_defaults():
    specs = build_map_3d_process_specs(
        mode=Map3DMode.MESH,
        python_executable="python3",
        script_dir="/tmp/examples",
        map_3d_topic="/map_3d",
        map_3d_frame="map",
        map_3d_mesh_topic="/map_3d_mesh",
    )

    assert len(specs) == 2
    assert specs[0].name == "fake_3d_map_publisher"
    assert "fake_3d_map_publisher.py" in specs[0].command[1]

    converter = specs[1]
    assert converter.name == "pointcloud_to_voxel_mesh_marker"
    assert "pointcloud_to_voxel_mesh_marker.py" in converter.command[1]
    assert "--update-policy" in converter.command
    assert "snapshot" in converter.command
    assert "--update-mode" in converter.command
    assert "once" in converter.command
    assert "--mesh-transport" in converter.command
    assert MeshTransport.MARKER.value in converter.command


def test_build_map_3d_process_specs_for_mesh_defaults_disable_periodic_snapshot_keepalive():
    specs = build_map_3d_process_specs(
        mode=Map3DMode.MESH,
        python_executable="python3",
        script_dir="/tmp/examples",
        map_3d_topic="/map_3d",
        map_3d_frame="map",
        map_3d_mesh_topic="/map_3d_mesh",
    )

    converter = specs[1].command
    republish_index = converter.index("--on-change-republish-interval") + 1
    assert converter[republish_index] == "0.0"
    chunk_cap_index = converter.index("--chunk-max-triangles") + 1
    assert converter[chunk_cap_index] == "3000"


def test_build_map_3d_process_specs_detailed_mode_uses_realistic_publisher():
    specs = build_map_3d_process_specs(
        mode=Map3DMode.POINTCLOUD,
        python_executable="python3",
        script_dir="/tmp/examples",
        map_3d_topic="/map_3d",
        map_3d_frame="map",
        map_3d_mesh_topic="/map_3d_mesh",
        map_3d_detailed=True,
    )

    assert len(specs) == 1
    assert "fake_3d_map_publisher_realistic.py" in specs[0].command[1]


def test_build_map_3d_process_specs_for_mesh_periodic_maps_to_on_change():
    specs = build_map_3d_process_specs(
        mode=Map3DMode.MESH,
        python_executable="python3",
        script_dir="/tmp/examples",
        map_3d_topic="/map_3d",
        map_3d_frame="map",
        map_3d_mesh_topic="/map_3d_mesh",
        mesh_update_policy="periodic",
        mesh_republish_interval=0.0,
    )

    converter = specs[1].command
    update_policy_index = converter.index("--update-policy") + 1
    update_mode_index = converter.index("--update-mode") + 1
    republish_index = converter.index("--on-change-republish-interval") + 1

    assert converter[update_policy_index] == "periodic"
    assert converter[update_mode_index] == "on_change"
    assert converter[republish_index] == "2.0"


def test_build_map_3d_process_specs_for_off_returns_empty():
    specs = build_map_3d_process_specs(
        mode=Map3DMode.OFF,
        python_executable="python3",
        script_dir="/tmp/examples",
        map_3d_topic="/map_3d",
        map_3d_frame="map",
        map_3d_mesh_topic="/map_3d_mesh",
    )
    assert specs == []


def test_pointcloud_to_mesh_command_contains_expected_topics():
    command = build_pointcloud_to_mesh_converter_command(
        python_executable="python3",
        script_dir="/tmp/examples",
        cloud_topic="/source_cloud",
        mesh_topic="/mesh_out",
        update_policy="continuous",
    )
    assert "--cloud-topic" in command
    assert "/source_cloud" in command
    assert "--mesh-topic" in command
    assert "/mesh_out" in command
    assert "--update-policy" in command
    assert "continuous" in command


def test_pointcloud_to_mesh_command_supports_marker_array_transport_flags():
    command = build_pointcloud_to_mesh_converter_command(
        python_executable="python3",
        script_dir="/tmp/examples",
        cloud_topic="/source_cloud",
        mesh_topic="/mesh_out",
        mesh_array_topic="/mesh_array_out",
        mesh_transport=MeshTransport.MARKER_ARRAY.value,
        mesh_chunk_max_triangles=4096,
    )
    assert "--mesh-array-topic" in command
    assert "/mesh_array_out" in command
    assert "--mesh-transport" in command
    assert MeshTransport.MARKER.value in command
    assert MeshTransport.MARKER_ARRAY.value not in command
    chunk_cap_index = command.index("--chunk-max-triangles") + 1
    assert command[chunk_cap_index] == "4096"


def test_pointcloud_to_mesh_command_clamps_marker_array_chunk_size():
    command = build_pointcloud_to_mesh_converter_command(
        python_executable="python3",
        script_dir="/tmp/examples",
        cloud_topic="/source_cloud",
        mesh_topic="/mesh_out",
        mesh_transport="marker_array",
        mesh_chunk_max_triangles=32,
    )
    chunk_cap_index = command.index("--chunk-max-triangles") + 1
    assert command[chunk_cap_index] == "256"


def test_coerce_mesh_transport_to_marker_reports_marker_array_requests():
    transport, coerced = coerce_mesh_transport_to_marker(MeshTransport.MARKER_ARRAY.value)
    assert transport == MeshTransport.MARKER.value
    assert coerced is True
