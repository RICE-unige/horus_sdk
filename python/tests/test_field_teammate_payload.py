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
from horus.robot import EntityCapabilities, FieldTeammate, Robot, RobotType


_FIXTURE = (
    Path(__file__).resolve().parents[2]
    / "contracts"
    / "fixtures"
    / "field_teammate_hololens.json"
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
