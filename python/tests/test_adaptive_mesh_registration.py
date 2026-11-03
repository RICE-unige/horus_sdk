import importlib
import json
from pathlib import Path
import sys

import pytest

from horus.bridge.robot_registry import RobotRegistryClient
from horus.mapping.adaptive_mesh_ros import arguments, client_topic_token, validate_request
from horus.robot import Robot, RobotType


def request():
    return {"version": 1, "client_id": "a" * 32, "seq": 1, "frame_id": "source",
            "triangle_budget": 500000, "memory_budget": 192 << 20,
            "regions": [{"id": "manual", "shape": "box", "center": [1, 2, 3],
                         "size": [2, 2, 2], "rotation": [0, 0, 0, 1]}]}


def test_nested_policy_survives_real_registration_serializer():
    viz = Robot("robot", RobotType.WHEELED).create_dataviz()
    viz.add_adaptive_mesh("/mapping/adaptive", "source")
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    entry = client._build_global_visualizations_payload([viz])[0]
    assert entry["topic"] == "/mapping/adaptive"
    assert entry["mesh"]["adaptive"]["enabled"] is True
    assert entry["mesh"]["adaptive"]["regions"][0]["id"] == "inspection"
    assert entry["mesh"]["indexed_live"] is True
    assert entry["transport_lane"] == "bulk_strict"


@pytest.mark.parametrize("changes", [{"memory_budget_mib": 2}, {"min_triangle_budget": 600000},
                                    {"interests": [{"id": "x", "size": [1, 0, 1]}]},
                                    {"interests": [{"id": "x", "rotation": [0, 0, 0, 0]}]},
                                    {"interests": [{"id": "x", "priority": -1}]},
                                    {"interests": [{"id": "x", "shape": "sphere", "size": [1, 2, 3]}]},
                                    {"interests": [{"id": "x"}, {"id": "x"}]}])
def test_sdk_rejects_invalid_regions_and_resource_policies(changes):
    viz = Robot("robot", RobotType.WHEELED).create_dataviz()
    with pytest.raises(ValueError):
        viz.add_adaptive_mesh("/adaptive", **changes)


def test_adaptive_request_keeps_source_frame_and_metric_coordinates():
    value = request()
    assert validate_request(json.dumps(value)) == value


@pytest.mark.parametrize("first", "0123456789abcdef")
def test_every_guid_prefix_has_a_ros_legal_view_namespace_without_changing_identity(first):
    value = request()
    value["client_id"] = first + "a" * 31
    assert validate_request(json.dumps(value))["client_id"] == value["client_id"]
    token = client_topic_token(value["client_id"])
    assert token == ("c_" if first.isdigit() else "") + value["client_id"]
    assert token[0].isalpha()


@pytest.mark.parametrize("field,value", [("client_id", "../../other"), ("seq", -1), ("seq", True),
                                       ("triangle_budget", 0), ("memory_budget", 1), ("regions", [None])])
def test_malformed_requests_cannot_create_unbounded_or_cross_client_views(field, value):
    message = request()
    message[field] = value
    with pytest.raises((ValueError, TypeError)):
        validate_request(json.dumps(message))


@pytest.mark.parametrize("field,value", [("center", [float("nan"), 1, 1]), ("size", [-1, 1, 1]),
                                       ("rotation", [0, 0, 0, 0]), ("priority", float("inf"))])
def test_malformed_interest_is_rejected_before_map_work(field, value):
    message = request()
    message["regions"][0][field] = value
    with pytest.raises(ValueError):
        validate_request(json.dumps(message))


@pytest.mark.parametrize("estimated", [False, True])
def test_field_adaptive_launch_keeps_fine_source_and_starts_provider(tmp_path, estimated):
    sys.path.insert(0, str(Path(__file__).parents[1] / "examples"))
    demo = importlib.import_module("tools.field_teammate_demo")
    registration = importlib.import_module("field_teammate_mapping_registration")
    plan = dict(demo.command_plan("192.0.2.10", tmp_path, estimated=estimated,
        depth_runtime=tmp_path / "depth", adaptive=True))
    assert "voxel_size:=0.01" in plan["mapping"]
    assert "--adaptive" in plan["registration"]
    assert "horus.mapping.adaptive_mesh_ros" in plan["adaptive"]
    assert ("/horus/field/estimated_mesh" if estimated else "/horus/field/mesh") in plan["adaptive"]
    if estimated:
        assert "learned_voxel_size:=0.01" in plan["mapping"]
    entities, visualizations = registration.build_entities(estimated=estimated, adaptive=True)
    assert len(entities) == 2
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    entry = next(entry for entry in client._build_global_visualizations_payload(visualizations) if entry["type"] == "mesh")
    assert entry["mesh"]["adaptive"]["regions"][0]["frame"] == "field_teammate_1_scene/camera"
    assert entry["topic"] == "/horus/field/adaptive_mesh"


def test_provider_requires_distinct_topics_and_explicit_frame():
    args = arguments(["--input-topic", "/original", "--output-topic", "/adaptive", "--frame-id", "source"])
    assert args.frame_id == "source"
    with pytest.raises(SystemExit):
        arguments(["--input-topic", "/same", "--output-topic", "/same", "--frame-id", "source"])


def test_original_checkpoint_is_exact_replayable_and_never_overwrites(tmp_path):
    from horus.mapping.adaptive_mesh import AdaptiveMeshStore, ensure_native
    from horus.mapping.adaptive_mesh_ros import save_original_pages
    from horus.mapping.mesh_protocol import BlockGeometry, encode_live_payload
    sys.path.insert(0, str(Path(__file__).parents[1] / "examples"))
    from saved_map_registration import load_saved_map
    ensure_native()
    geometry = BlockGeometry(((0., 0., 0.), (1., 0., 0.), (0., 1., 0.)),
                             ((255, 80, 20, 255),) * 3, (0, 1, 2))
    payload = encode_live_payload(chunk_id=42, page_key=(0, 0, 0), epoch=7, revision=2,
        operation=0, geometry=geometry, max_payload_bytes=1024)
    store = AdaptiveMeshStore()
    assert store.ingest(payload)
    expected = {"epoch": "7", "pages": [{"chunk_id": 42, "revision": "2", "operation": 0}]}
    destination = tmp_path / "original"
    record = save_original_pages(store, destination, "source", expected, -1.6)
    assert record["input_inventory_verified"] is True
    assert (destination / "pages/00000042.hmsh").read_bytes() == payload
    saved = load_saved_map(destination)
    assert saved.pages[0].payload == payload
    assert saved.metadata["source_floor_z_m"] == -1.6
    with pytest.raises(FileExistsError):
        save_original_pages(store, destination, "source", expected, -1.6)
    assert load_saved_map(destination).pages[0].payload == payload


def test_incomplete_source_checkpoint_is_not_mislabeled_complete(tmp_path):
    from horus.mapping.adaptive_mesh import AdaptiveMeshStore, ensure_native
    from horus.mapping.adaptive_mesh_ros import save_original_pages
    from horus.mapping.mesh_protocol import BlockGeometry, encode_live_payload
    ensure_native()
    store = AdaptiveMeshStore()
    assert store.ingest(encode_live_payload(chunk_id=1, page_key=(0, 0, 0), epoch=7, revision=2,
        operation=0, geometry=BlockGeometry(((0., 0., 0.), (1., 0., 0.), (0., 1., 0.)), (), (0, 1, 2)), max_payload_bytes=1024))
    record = save_original_pages(store, tmp_path / "partial", "source", {"epoch": "7", "pages": [
        {"chunk_id": 1, "revision": "3", "operation": 0}]})
    assert record["input_inventory_verified"] is False
    assert record["replay_requires_datum"] is True
    sys.path.insert(0, str(Path(__file__).parents[1] / "examples"))
    from saved_map_registration import load_saved_map
    with pytest.raises(ValueError, match="incomplete or unverified"):
        load_saved_map(tmp_path / "partial")
