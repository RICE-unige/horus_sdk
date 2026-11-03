import importlib
import json
from pathlib import Path
import socket
import sys
from types import SimpleNamespace

import numpy as np
import pytest

EXAMPLES = Path(__file__).parents[1] / "examples"
sys.path.insert(0, str(EXAMPLES))
demo = importlib.import_module("tools.field_teammate_demo")
health = importlib.import_module("tools.field_teammate_health")
registration = importlib.import_module("field_teammate_mapping_registration")


def test_plan_uses_real_connector_and_mapping_once_without_synthetic_sources(tmp_path):
    plan = dict(demo.command_plan("192.0.2.10", tmp_path))
    assert set(plan) == {"relay", "scene", "path", "mapping", "adapter", "mesh_health", "registration", "health"}
    assert "/field_teammate_1/research/long_throw/pose" in plan["path"]
    assert "/field_teammate_1/trajectory" in plan["path"]
    assert "field_teammate_1_scene/source" in plan["path"]
    assert "--blocks-per-axis" in plan["adapter"] and "8" in plan["adapter"]
    assert "--presentation-frame" in plan["adapter"]
    assert "--mesh-worker" in plan["mesh_health"]
    assert "--mesh-status-topic" in plan["health"]
    assert "--video-profile" in plan["relay"] and "mapping" in plan["relay"]
    assert "--disable-voice" in plan["relay"]
    assert "--head-only" in plan["relay"]
    assert "session_name:=scan" in plan["mapping"]
    assert f"output_root:={tmp_path}" in plan["mapping"]
    assert f"calibration_directory:={tmp_path / 'calibration'}" in plan["mapping"]
    assert not (tmp_path / "scan").exists()
    assert not any("mock" in arg or "synthetic" in arg for cmd in plan.values() for arg in cmd)
    assert "mapping_profile:=quality" in plan["mapping"]
    assert "voxel_size:=0.01" in plan["mapping"]


@pytest.mark.parametrize("estimated", [False, True])
@pytest.mark.parametrize("profile,voxel", [("balanced", .02), ("3cm", .03), ("performance", .05)])
def test_explicit_profile_reaches_the_actual_ros_launch_without_changing_sources(tmp_path, estimated, profile, voxel):
    plan = dict(demo.command_plan("192.0.2.10", tmp_path, mapping_profile=profile,
                                 estimated=estimated, depth_runtime=tmp_path / "depth"))
    assert f"mapping_profile:={profile}" in plan["mapping"]
    assert f"voxel_size:={voxel}" in plan["mapping"]
    assert "session_name:=scan" in plan["mapping"]
    assert "/horus_measured_mapper/mesh" in plan["adapter"]
    if estimated:
        assert "live_hybrid_mapping.launch.py" in plan["mapping"]
        assert f"learned_voxel_size:={voxel}" in plan["mapping"]
        assert "/horus_hybrid_mapper/mesh" in plan["estimated_adapter"]
    else:
        assert "live_dense_mapping.launch.py" in plan["mapping"]
        assert "estimated_adapter" not in plan


def test_mapping_profile_defaults_stay_compatible_and_invalid_profiles_are_rejected(tmp_path):
    assert demo.build_parser().parse_args([]).mapping_profile == "quality"
    assert demo.build_parser().parse_args(["--mapping-profile", "performance"]).mapping_profile == "performance"
    assert demo.build_parser().parse_args(["--mapping-profile", "balanced"]).mapping_profile == "balanced"
    assert demo.build_parser().parse_args(["--mapping-profile", "3cm"]).mapping_profile == "3cm"
    with pytest.raises(SystemExit):
        demo.build_parser().parse_args(["--mapping-profile", "unknown"])
    with pytest.raises(ValueError, match="Mapping profile"):
        demo.command_plan("192.0.2.10", tmp_path, mapping_profile="unknown")


@pytest.mark.parametrize("profile,lod,warp", [
    ("off", False, False), ("lod", True, False),
    ("spacewarp", False, True), ("combined", True, True),
])
@pytest.mark.parametrize("adaptive", [False, True])
def test_live_mesh_display_profile_reaches_only_the_map(tmp_path, profile, lod, warp, adaptive):
    plan = dict(demo.command_plan("192.0.2.10", tmp_path, adaptive=adaptive,
                                 mesh_render_profile=profile))
    command = plan["registration"]
    assert command[command.index("--mesh-render-profile") + 1] == profile
    assert "voxel_size:=0.01" in plan["mapping"]
    entities, views = registration.build_entities(adaptive=adaptive, mesh_render_profile=profile)
    mesh = views[1].visualizations[0]
    assert mesh.render_options["display"]["lod_enabled"] is lod
    assert mesh.render_options["display"]["spacewarp_enabled"] is warp
    assert mesh.render_options["display"]["lod_error_pixels"] == 1.0
    assert not any("display" in view.render_options for view in views[0].visualizations)
    assert not entities[0].get_capabilities()["teleoperable"]


def test_live_launcher_uses_combined_but_standalone_registration_stays_opt_in(tmp_path):
    assert demo.build_parser().parse_args([]).mesh_render_profile == "combined"
    assert "combined" in dict(demo.command_plan("192.0.2.10", tmp_path))["registration"]
    _, views = registration.build_entities()
    assert not views[1].visualizations[0].render_options["display"]["spacewarp_enabled"]
    with pytest.raises(ValueError, match="mesh render profile"):
        registration.build_entities(mesh_render_profile="unknown")
    with pytest.raises(ValueError, match="mesh render profile"):
        demo.command_plan("192.0.2.10", tmp_path, mesh_render_profile="unknown")


@pytest.mark.parametrize("memory,package,executable", [
    ("device", "nvblox_ros", "nvblox_node"),
    ("unified", "horus_nvblox_memory", "horus_nvblox_node"),
])
def test_explicit_mapping_memory_selects_real_program_without_changing_geometry(tmp_path, memory, package, executable):
    assert demo.mapper_program(memory) == (package, executable)
    assert demo.build_parser().parse_args(["--mapping-memory", memory]).mapping_memory == memory
    plan = dict(demo.command_plan("192.0.2.10", tmp_path, estimated=True, adaptive=True,
                                 depth_runtime=tmp_path / "depth", mapping_memory=memory))
    assert f"mapping_memory_type:={memory}" in plan["mapping"]
    assert "voxel_size:=0.01" in plan["mapping"]
    assert "learned_voxel_size:=0.01" in plan["mapping"]
    assert "learned_depth_max_range_m:=10.0" in plan["mapping"]


def test_mapping_memory_remains_device_by_default_and_invalid_values_fail():
    assert demo.build_parser().parse_args([]).mapping_memory == "device"
    with pytest.raises(ValueError, match="Mapping memory"):
        demo.mapper_program("automatic")
    with pytest.raises(SystemExit):
        demo.build_parser().parse_args(["--mapping-memory", "automatic"])


def test_nested_mapper_exit_receipt_is_not_confused_with_a_retained_mesh(tmp_path):
    assert demo.exited_mappers(tmp_path) == {}
    directory = tmp_path / "scan"
    directory.mkdir()
    path = directory / "mapper_exit_horus_hybrid_mapper.json"
    record = {"schema": "horus.mapper_exit.v1", "session_directory": str(directory),
              "mapper_node": "/horus_hybrid_mapper", "pid": 1234, "returncode": 99}
    path.write_text(json.dumps(record))
    assert demo.exited_mappers(tmp_path) == {"/horus_hybrid_mapper": 99}
    path.write_text(json.dumps({**record, "session_directory": str(tmp_path / "other")}))
    with pytest.raises(RuntimeError, match="Invalid mapper exit"):
        demo.exited_mappers(tmp_path)


def test_save_keeps_surviving_mapper_without_waiting_for_known_dead_service(tmp_path, monkeypatch):
    scan = tmp_path / "scan"
    scan.mkdir()
    metadata = tmp_path / "owned-session.json"
    metadata.write_text(json.dumps({"session_directory": str(scan)}))
    (scan / "mapper_exit_horus_hybrid_mapper.json").write_text(json.dumps({
        "schema": "horus.mapper_exit.v1", "session_directory": str(scan),
        "mapper_node": "/horus_hybrid_mapper", "pid": 1234, "returncode": 99,
    }))
    def local_path(value):
        return metadata if str(value).startswith("/tmp/horus_dense_mapping_domain_") else Path(value)
    monkeypatch.setattr(demo, "Path", local_path)
    commands = []
    def run(command, **_kwargs):
        commands.append(command)
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(demo.subprocess, "run", run)
    assert demo.save_map(tmp_path, estimated=True) is False
    assert len(commands) == 1
    assert commands[0][commands[0].index("--mapper-node") + 1] == "/horus_measured_mapper"


@pytest.mark.parametrize("profile,voxel", [("balanced", .02), ("3cm", .03), ("performance", .05)])
def test_profile_preflight_reports_effective_resolution_without_starting_a_session(monkeypatch, capsys, profile, voxel):
    monkeypatch.setattr(demo, "preflight", lambda *_args: {})
    monkeypatch.setattr(demo, "reject_existing_session", lambda *_args, **_kwargs: pytest.fail("check started session"))
    monkeypatch.setattr(sys, "argv", ["field-demo", "--check", "--mapping-profile", profile, "--estimated-depth"])
    assert demo.main() == 0
    result = json.loads(capsys.readouterr().out)
    assert result["mapping_profile"] == profile
    assert result["measured_voxel_size_m"] == result["estimated_voxel_size_m"] == voxel
    assert "no height/radius exclusions" in result["map_coverage"]


def test_receive_only_does_not_launch_mapper_adapter_or_viewer(tmp_path):
    plan = dict(demo.command_plan("192.0.2.10", tmp_path, mapping=False))
    assert "--head-only" not in plan["relay"]
    assert set(plan) == {"relay", "registration", "health"}
    assert "--receive-only" in plan["health"] and "--receive-only" in plan["registration"]


def test_rerun_visualizer_keeps_runtime_path_as_one_launch_argument(tmp_path):
    runtime = tmp_path / "runtime with spaces"
    plan = dict(demo.command_plan("192.0.2.10", tmp_path, visualizer="rerun", rerun_runtime=runtime))
    assert "viz:=rerun" in plan["mapping"]
    assert f"rerun_runtime:={runtime}" in plan["mapping"]
    assert "viz:=rviz" not in plan["mapping"]


def test_no_view_overrides_selected_visualizer(tmp_path):
    plan = dict(demo.command_plan("192.0.2.10", tmp_path, view=False, visualizer="rerun"))
    assert "viz:=none" in plan["mapping"]
    assert not any(arg.startswith("rerun_runtime:=") for arg in plan["mapping"])


def test_hybrid_path_keeps_two_sources_but_registers_the_same_single_map_as_rerun(tmp_path):
    plan = dict(demo.command_plan("192.0.2.10", tmp_path, estimated=True,
        depth_runtime=tmp_path / "depth runtime", backend="lingbot", estimated_range=10))
    assert "live_hybrid_mapping.launch.py" in plan["mapping"]
    assert f"learned_depth_runtime:={tmp_path / 'depth runtime'}" in plan["mapping"]
    assert "learned_depth_max_range_m:=10" in plan["mapping"]
    assert "voxel_size:=0.01" in plan["mapping"]
    assert "learned_voxel_size:=0.02" in plan["mapping"]
    assert "--estimated-depth" in plan["registration"] and "--estimated-depth" in plan["health"]
    assert "/horus_measured_mapper/mesh" in plan["adapter"]
    assert "/horus_hybrid_mapper/mesh" in plan["estimated_adapter"]
    assert "/horus/field/estimated_mesh" in plan["estimated_adapter"]
    assert "initial_map_source:=estimated" in plan["mapping"]
    assert plan["registration"][plan["registration"].index("--map-source") + 1] == "estimated"
    entities, visualizations = registration.build_entities(estimated=True)
    assert len(entities) == len(visualizations) == 2
    assert visualizations[1].visualizations[0].data_source.topic == "/horus/field/estimated_mesh"


@pytest.mark.parametrize("adaptive", [False, True])
@pytest.mark.parametrize("estimated,requested,source", [
    (False, None, "measured"), (False, "measured", "measured"),
    (True, None, "estimated"), (True, "measured", "measured"),
    (True, "estimated", "estimated"),
])
def test_selected_map_is_unambiguous_in_actual_global_payload(tmp_path, adaptive, estimated, requested, source):
    from horus.bridge.robot_registry import RobotRegistryClient
    plan = dict(demo.command_plan("192.0.2.10", tmp_path, estimated=estimated,
        depth_runtime=tmp_path, adaptive=adaptive, map_source=requested))
    assert plan["registration"][plan["registration"].index("--map-source") + 1] == source
    topic = "/horus/field/estimated_mesh" if source == "estimated" else "/horus/field/mesh"
    if estimated:
        assert f"initial_map_source:={source}" in plan["mapping"]
        assert "estimated_adapter" in plan and "adapter" in plan
    if adaptive:
        assert plan["adaptive"][plan["adaptive"].index("--input-topic") + 1] == topic
        topic = "/horus/field/adaptive_mesh"
    entities, visualizations = registration.build_entities(estimated=estimated, adaptive=adaptive, map_source=requested)
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized, client.node = False, None
    maps = [v for v in client._build_global_visualizations_payload(visualizations) if v["type"] == "mesh"]
    assert len(maps) == 1, "MR's first-match selection must never hide the requested source"
    assert maps[0]["topic"] == topic
    assert maps[0]["mesh"]["indexed_live"] is True
    assert maps[0]["mesh"]["use_vertex_colors"] is True
    if adaptive:
        assert maps[0]["mesh"]["adaptive"]["enabled"] is True
    else:
        assert "adaptive" not in maps[0]["mesh"]
    assert len(entities) == len(visualizations) == 2


@pytest.mark.parametrize("source", ["estimated", "unknown"])
def test_unavailable_or_invalid_source_is_not_silently_replaced(tmp_path, source):
    with pytest.raises(ValueError, match="source"):
        registration.build_entities(map_source=source)
    with pytest.raises(ValueError, match="source"):
        demo.command_plan("192.0.2.10", tmp_path, map_source=source)


def test_existing_relay_is_not_duplicated_or_killed(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "20")
    proc = tmp_path / "123"
    proc.mkdir()
    (proc / "cmdline").write_bytes(b"python3\0/scripts/field_teammate_hololens_relay.py\0")
    (proc / "environ").write_bytes(b"ROS_DOMAIN_ID=20\0UNRELATED_SECRET=do-not-log\0")
    with pytest.raises(RuntimeError, match="already runs") as error:
        demo.reject_existing_session(False, tmp_path)
    assert "do-not-log" not in str(error.value)
    assert proc.is_dir()


def test_status_is_waiting_invalid_live_then_stale_and_bounded():
    monitor = health.Reception()
    assert monitor.snapshot(["video"], now=10)["video"]["state"] == "waiting"
    monitor.observe("video", valid=False, now=10)
    assert monitor.snapshot(["video"], now=10)["video"]["state"] == "invalid"
    for index in range(150):
        monitor.observe("video", now=11 + index / 15)
    row = monitor.snapshot(["video"], now=21)["video"]
    assert row["state"] == "live" and row["rate_hz"] == pytest.approx(15)
    assert len(monitor.samples["video"]["arrivals"]) == 120
    assert monitor.snapshot(["video"], now=30)["video"]["state"] == "stale"


def test_static_mesh_is_retained_not_falsely_reported_as_high_rate():
    monitor = health.Reception()
    monitor.observe("MR mesh", now=1)
    row = monitor.snapshot(["MR mesh"], now=20)["MR mesh"]
    assert row["state"] == "retained" and row["rate_hz"] == 0


def test_mesh_worker_merges_actual_counters_without_resetting_age_or_inventing_frames():
    remote = {"time_monotonic": 10., "streams": {"mesh": {
        "state": "live", "count": 23, "rate_hz": 1., "arrival_age_sec": 2.,
        "stamp_age_sec": 3., "details": {"blocks_in_update": 100}}}}
    streams = {"depth": {"count": 50}, "mesh": {"state": "waiting"}}
    health.merge_mesh_snapshot(streams, remote, now=15.)
    assert streams["depth"]["count"] == 50
    assert streams["mesh"]["count"] == 23
    assert streams["mesh"]["arrival_age_sec"] == 7.
    assert streams["mesh"]["stamp_age_sec"] == 8.
    assert streams["mesh"]["state"] == "retained"
    health.merge_mesh_snapshot(streams, remote, now=21.)
    assert streams["mesh"]["state"] == "monitor_stale"
    assert remote["streams"]["mesh"]["state"] == "live"


def test_invalid_numeric_telemetry_cannot_crash_json_writer():
    value = health.safe_numbers({"fx": float("nan"), "values": [float("inf"), 1]})
    assert json.loads(json.dumps(value, allow_nan=False)) == {"fx": None, "values": [None, 1]}


def test_depth_check_respects_padding_and_sensor_endianness():
    raw = np.array([[1, 2, 99], [3, np.nan, 99]], dtype=">f4")
    msg = SimpleNamespace(encoding="32FC1", width=2, height=2, step=12, data=raw.tobytes(), is_bigendian=True)
    row = health.depth_details(msg)
    assert row["valid_pixels"] == 3 and row["median_metres"] == 2
    msg.data = msg.data[:-1]
    with pytest.raises(ValueError, match="complete"):
        health.depth_details(msg)


def test_combined_registration_is_non_controllable_and_uses_workspace_mesh():
    entities, visualizations = registration.build_entities()
    teammate, resource = entities
    assert not teammate.get_capabilities()["controllable"]
    assert not teammate.get_capabilities()["teleoperable"]
    assert resource.get_entity_kind() == "workspace_visualization"
    assert any(v.data_source.topic == "/horus/field/mesh" for v in visualizations[1].visualizations)
    mesh = next(v for v in visualizations[1].visualizations if v.data_source.topic == "/horus/field/mesh")
    assert mesh.render_options["use_indexed_binary"]
    assert health.MR_MESH_TOPIC == mesh.data_source.topic + "/indexed"


def test_measured_path_is_registered_in_map_and_matches_custom_name_publisher(tmp_path):
    entities, visualizations = registration.build_entities(name="field_partner")
    path = next(v for v in visualizations[0].visualizations if v.viz_type.value == "path")
    assert path.data_source.topic == "/field_partner/trajectory"
    assert path.data_source.frame_id == "field_partner_scene/floor"
    assert path.render_options["color"] == "#FFC247"
    tree = next(v for v in visualizations[0].visualizations if v.viz_type.value == "transform_tree")
    assert tree.data_source.frame_id == "field_partner_scene/floor"
    plan = dict(demo.command_plan("192.0.2.10", tmp_path, name="field_partner", view=False))
    assert path.data_source.topic in plan["path"]
    assert "/field_partner/research/long_throw/pose" in plan["path"]
    assert "viz:=none" in plan["mapping"]
    assert entities[0].get_field_teammate_config()["profile"]["body_model"] == "head_only"
    assert entities[0].resolve_tf_frame() == "field_partner_scene/camera"
    assert entities[0].dimensions.height == pytest.approx(.38)
    assert entities[0].get_sensor("fpv_camera").frame_id == "field_partner_scene/camera"
    assert entities[0].get_sensor("fpv_camera").topic == "/field_partner/fpv/image_raw/compressed"


def test_receive_only_registration_keeps_head_camera_without_unpublished_map_or_path():
    entities, visualizations = registration.build_entities(mapping=False)
    assert len(entities) == len(visualizations) == 1
    assert not any(v.viz_type.value in ("trajectory", "mesh") for v in visualizations[0].visualizations)


def test_serialized_map_path_and_head_camera_are_in_the_actual_registration_payload():
    from horus.bridge.robot_registry import RobotRegistryClient
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized, client.node = False, None
    client._robot_description_resolver = None
    client._robot_description_by_robot, client._robot_description_by_id = {}, {}
    entities, visualizations = registration.build_entities(estimated=True)
    payloads = [client._build_robot_config_dict(entity, viz)
                for entity, viz in zip(entities, visualizations)]
    assert payloads[0]["field_teammate_config"]["profile"]["body_model"] == "head_only"
    assert payloads[0]["control"]["teleop"]["enabled"] is False
    assert payloads[0]["robot_description_manifest"]["supports_visual_meshes"]
    globals_ = client._build_global_visualizations_payload(visualizations)
    tree = next(v for v in globals_ if v["type"] == "transform_tree")
    assert tree["frame"] == "field_teammate_1_scene/floor"
    path = next(v for v in payloads[0]["visualizations"] if v["type"] == "path")
    assert path["scope"] == "robot" and path["frame"] == "field_teammate_1_scene/floor"
    assert path["topic"] == "/field_teammate_1/trajectory"
    assert path["color"] == "#FFC247" and path["path"]["line_width"] == 2
    assert path["path"]["role"] == "local"
    assert all(v["frame"] == "field_teammate_1_scene/floor" for v in globals_ if v["type"] == "mesh")
    assert {v["topic"] for v in globals_ if v["type"] == "mesh"} == {"/horus/field/estimated_mesh"}


def test_stationary_travelled_path_is_retained_without_faking_new_pose_updates():
    monitor = health.Reception()
    monitor.observe("travelled path", now=1)
    row = monitor.snapshot(["travelled path"], now=20)["travelled path"]
    assert row["state"] == "retained" and row["rate_hz"] == 0
    assert row["count"] == 1


def test_shutdown_signals_only_owned_groups(tmp_path, monkeypatch):
    signals = []
    class Process:
        pid = 345
        def poll(self): return None
        def wait(self, timeout): return 0
    children = demo.Children(tmp_path)
    children.processes = [("owned", Process())]
    monkeypatch.setattr(demo.os, "killpg", lambda pid, sig: signals.append(pid))
    children.stop()
    assert signals[0] == 345 and set(signals) == {345}


def test_hybrid_shutdown_saves_both_maps_without_overwriting_the_measured_label(tmp_path, monkeypatch):
    metadata = tmp_path / "owner.json"
    metadata.write_text(json.dumps({"session_directory": str(tmp_path / "scan")}))
    original_path = demo.Path
    monkeypatch.setattr(demo, "Path", lambda value: metadata if str(value).startswith("/tmp/horus_dense_mapping_domain_") else original_path(value))
    calls = []
    def run(command, **kwargs):
        calls.append(command)
        assert kwargs["timeout"] >= 300
        assert int(command[command.index("--timeout") + 1]) >= 120
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(demo.subprocess, "run", run)
    assert demo.save_map(tmp_path, estimated=True)
    assert len(calls) == 2
    assert calls[0][calls[0].index("--label") + 1] == "final"
    assert calls[1][calls[1].index("--label") + 1] == "final_estimated"
    assert "/horus_measured_mapper" in calls[0] and "/horus_hybrid_mapper" in calls[1]


@pytest.mark.parametrize("valid,expected", [(False, 2), (True, 0)])
def test_process_startup_without_sensor_data_is_not_a_pass(tmp_path, monkeypatch, valid, expected):
    stopped = []
    class Children:
        def __init__(self, directory): self.directory, self.processes = directory, []
        def start(self, *_args, **_kwargs):
            (self.directory / "reception.ndjson").write_text(json.dumps({"all_required_data_present": valid}) + "\n")
        def stop(self): stopped.append(True)
    monkeypatch.setenv("HORUS_FIELD_OUTPUT_ROOT", str(tmp_path))
    monkeypatch.setattr(demo, "preflight", lambda *_args: {})
    monkeypatch.setattr(demo, "reject_existing_session", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(demo, "probe_headset", lambda *_args, **_kwargs: {})
    monkeypatch.setattr(demo, "Children", Children)
    monkeypatch.setattr(demo, "command_plan", lambda *_args: [("health", ["fixture"])])
    monkeypatch.setattr(sys, "argv", ["field-demo", "192.0.2.10", "--receive-only", "--duration", ".001"])
    assert demo.main() == expected
    saved = json.loads(next(tmp_path.glob("*/exit.json")).read_text())
    assert saved["data_validated_at_stop"] is valid and stopped == [True]
