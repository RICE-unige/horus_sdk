"""Field-teammate registration contract: capability default-deny + fail-closed.

A field teammate is a human represented in HORUS through the robot registration
pipeline, but it must never be controllable. These tests assert the *serialized*
payload (so a wrapper or direct metadata mutation cannot evade the contract) and
that the serializer fails closed when a robot-control affordance would leak.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from horus.bridge.registration_payload import FieldTeammateSafetyError
from horus.bridge.robot_registry import RobotRegistryClient
from horus.description.robot_description_resolver import RobotDescriptionResolver
from horus.robot import EntityCapabilities, FieldTeammate, Robot, RobotDimensions, RobotType

from examples.tools.field_teammate_articulated_assets import (
    FieldTeammateProfile,
    ensure_field_teammate_articulated_assets,
)


_FIXTURE = (
    Path(__file__).resolve().parents[2]
    / "contracts"
    / "fixtures"
    / "field_teammate_hololens.json"
)
_HUMAN_URDF = (
    Path(__file__).resolve().parents[1]
    / "examples"
    / ".local_assets"
    / "field_teammate_description"
    / "urdf"
    / "field_teammate_human.urdf"
)


def _build_client() -> RobotRegistryClient:
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    client._robot_description_resolver = None
    client._robot_description_by_robot = {}
    client._robot_description_by_id = {}
    return client


def _payload(robot) -> dict:
    dataviz = robot.create_dataviz()
    return _build_client()._build_robot_config_dict(robot, dataviz)


def test_field_teammate_payload_matches_contract_fixture():
    expected = json.loads(_FIXTURE.read_text(encoding="utf-8"))
    config = _payload(FieldTeammate("field_teammate_1"))

    assert config["robot_name"] == expected["robot_name"]
    assert config["robot_type"] == "human"
    assert config["entity_kind"] == "field_teammate"
    assert config["capabilities"] == expected["capabilities"]
    assert config["control"]["teleop"]["enabled"] is False
    assert config["control"]["tasks"]["go_to_point"]["enabled"] is False
    assert config["control"]["tasks"]["waypoint"]["enabled"] is False
    assert config["robot_manager_config"]["sections"] == expected["robot_manager_config"]["sections"]
    assert config["field_teammate_config"] == expected["field_teammate_config"]


def test_field_teammate_capabilities_are_default_deny():
    config = _payload(FieldTeammate("ft"))
    caps = config["capabilities"]
    assert caps["controllable"] is False
    assert caps["teleoperable"] is False
    assert caps["taskable"] is False
    assert caps["guidable"] is True
    assert caps["observable"] is True
    assert caps["communicative"] is True


def test_field_teammate_has_no_dimensions_by_default():
    config = _payload(FieldTeammate("ft"))
    assert "dimensions" not in config


def test_field_teammate_human_model_is_baked(tmp_path):
    profile = FieldTeammateProfile(height_m=1.78, sex="female")
    bundle = ensure_field_teammate_articulated_assets(profile, cache_root=tmp_path)
    teammate = FieldTeammate(
        "field_teammate_1",
        dimensions=RobotDimensions(length=0.34, width=0.38, height=profile.normalized_height_m),
        profile_height_m=profile.normalized_height_m,
        profile_sex=profile.normalized_sex,
        body_model="skinned_profile_v1",
    )
    teammate.configure_robot_description(
        source="local",
        urdf_path=str(bundle.urdf_path),
        base_frame="base",
        mesh_root=str(bundle.mesh_root),
        include_visual_meshes=True,
        visual_mesh_triangle_budget=80000,
        body_mesh_mode="runtime_high_mesh",
        visual_link_pose_source="static",
    )

    config = _payload(teammate)
    assert config["field_teammate_config"]["profile"] == {
        "human_height_m": profile.normalized_height_m,
        "sex": "female",
        "body_model": "skinned_profile_v1",
    }
    manifest = config["robot_description_manifest"]
    assert manifest["base_frame"] == "base"
    assert manifest["supports_visual_meshes"] is True
    assert manifest["mesh_asset_count"] == 1
    assert config["capabilities"]["controllable"] is False

    artifact = RobotDescriptionResolver().resolve_for_robot(teammate)
    assert artifact is not None
    assert artifact.payload_dict["visual_link_pose_source"] == "static"
    assets = artifact.payload_dict["mesh_assets"]
    total_triangles = sum(int(asset["triangle_count"]) for asset in assets)
    assert 12000 < total_triangles < 80000
    assert len(artifact.payload_dict["visual_links"]) == 1
    assert assets[0]["colors_b64"], "skinned human mesh must preserve baked vertex colors"

    # Orientation/placement contract: the body stands upright with feet at z=0,
    # profile height tall, and is thinner front-to-back (X) than side-to-side (Y) so it
    # faces +X, the camera/FPV forward axis. If this regresses (e.g. the URDF
    # rotation is reverted), the projected FPV view lands beside the face.
    visuals_by_mesh = {visual["mesh_id"]: visual for visual in artifact.payload_dict["visual_links"]}
    world_mins = []
    world_maxs = []
    for asset in assets:
        visual = visuals_by_mesh[asset["mesh_id"]]
        origin = visual["origin_xyz"]
        world_mins.append([origin[i] + asset["bounds_min"][i] for i in range(3)])
        world_maxs.append([origin[i] + asset["bounds_max"][i] for i in range(3)])
    bmin = [min(bounds[i] for bounds in world_mins) for i in range(3)]
    bmax = [max(bounds[i] for bounds in world_maxs) for i in range(3)]
    assert abs(bmin[2]) < 0.05, f"feet not at z=0: {bmin[2]}"
    assert 1.6 < bmax[2] < 2.0, f"unexpected body height: {bmax[2]}"
    assert (bmax[0] - bmin[0]) < (bmax[1] - bmin[1]), "body does not face +X"


def test_configure_field_teammate_on_plain_robot():
    robot = Robot(name="person", robot_type=RobotType.HUMAN)
    robot.configure_field_teammate(wearable_type="aria")
    config = _payload(robot)
    assert config["entity_kind"] == "field_teammate"
    assert config["field_teammate_config"]["wearable_type"] == "aria"
    assert config["capabilities"]["controllable"] is False


def test_custom_guidance_topics_are_honored():
    config = _payload(
        FieldTeammate(
            "scout",
            first_person_video_topic="/scout/head_cam/compressed",
            guidance_request_topic="/scout/orders",
        )
    )
    topics = config["field_teammate_config"]["topics"]
    assert topics["first_person_video"] == "/scout/head_cam/compressed"
    assert topics["guidance_request"] == "/scout/orders"
    # Unset topics still fall back to per-entity defaults.
    assert topics["guidance_response"] == "/scout/guidance/response"


def test_fail_closed_when_teleop_re_enabled_via_metadata():
    teammate = FieldTeammate("ft")
    # Simulate a wrapper or caller mutating metadata to re-enable teleop.
    teammate.configure_teleop(enabled=True)
    with pytest.raises(FieldTeammateSafetyError):
        _payload(teammate)


def test_fail_closed_when_a_task_re_enabled_via_metadata():
    teammate = FieldTeammate("ft")
    teammate.configure_go_to_point_task(enabled=True)
    with pytest.raises(FieldTeammateSafetyError):
        _payload(teammate)


def test_fail_closed_when_controllable_capability_forced():
    teammate = FieldTeammate("ft")
    # Directly poke metadata to grant a denied capability.
    caps = dict(teammate.get_metadata("entity_capabilities"))
    caps["controllable"] = True
    teammate.add_metadata("entity_capabilities", caps)
    with pytest.raises(FieldTeammateSafetyError):
        _payload(teammate)


def test_regular_robot_is_controllable_and_marked_robot():
    config = _payload(Robot(name="rover", robot_type=RobotType.WHEELED))
    assert config["entity_kind"] == "robot"
    caps = config["capabilities"]
    assert caps["controllable"] is True
    assert caps["teleoperable"] is True
    assert caps["taskable"] is True
    assert caps["guidable"] is False


def test_entity_capabilities_value_object_merge():
    base = EntityCapabilities.for_field_teammate()
    merged = EntityCapabilities.from_values(base=base, communicative=False)
    assert merged.controllable is False
    assert merged.guidable is True
    assert merged.communicative is False


def test_field_teammate_topic_contract_frozen():
    """Freeze the field_teammate.v1 topic shapes.

    The live relay is owned by horus_connector
    (scripts/field_teammate_hololens_relay.py, the "teammate" role) and
    hardcodes these topic shapes WITHOUT importing the SDK. The connector has
    its own dry-run contract test and Zenoh transport-scope test. If this test
    ever needs updating, update BOTH connector files in lockstep and bump the
    contract version.
    """
    teammate = FieldTeammate("field_teammate_1")
    config = teammate.get_field_teammate_config() or {}
    assert config.get("contract_version") == "field_teammate.v1"
    assert (config.get("topics") or {}) == {
        "first_person_video": "/field_teammate_1/fpv/image_raw/compressed",
        "localization_confidence": "/field_teammate_1/localization_confidence",
        "guidance_request": "/field_teammate_1/guidance/request",
        "guidance_response": "/field_teammate_1/guidance/response",
        "guidance_state": "/field_teammate_1/guidance/state",
        "guidance_annotation": "/field_teammate_1/guidance/annotation",
        "guidance_route": "/field_teammate_1/guidance/route",
        "guidance_warning": "/field_teammate_1/guidance/warning",
        "status": "/field_teammate_1/status",
        "audio": "/field_teammate_1/audio/message",
    }


def test_field_teammate_name_is_sanitized_for_ros_topic_contract():
    """Keep SDK topic derivation aligned with the connector relay.

    The live connector relay cannot publish ROS topics with spaces, hyphens, or
    other punctuation in the teammate namespace. The SDK registration payload
    must sanitize names the same way, otherwise HORUS can register one namespace
    while the relay publishes another.
    """
    teammate = FieldTeammate("field-teammate 1/dev")
    config = teammate.get_field_teammate_config() or {}

    assert config["base_frame"] == "field_teammate_1_dev/base"
    assert config["camera_frame"] == "field_teammate_1_dev/camera"
    assert (config.get("topics") or {}) == {
        "first_person_video": "/field_teammate_1_dev/fpv/image_raw/compressed",
        "localization_confidence": "/field_teammate_1_dev/localization_confidence",
        "guidance_request": "/field_teammate_1_dev/guidance/request",
        "guidance_response": "/field_teammate_1_dev/guidance/response",
        "guidance_state": "/field_teammate_1_dev/guidance/state",
        "guidance_annotation": "/field_teammate_1_dev/guidance/annotation",
        "guidance_route": "/field_teammate_1_dev/guidance/route",
        "guidance_warning": "/field_teammate_1_dev/guidance/warning",
        "status": "/field_teammate_1_dev/status",
        "audio": "/field_teammate_1_dev/audio/message",
    }
