import dataclasses
import json
import uuid
from pathlib import Path

import pytest

from horus import ControllerControl as C, ControllerHint, ControllerHintContext, ControllerHintProfile, ControllerHints, TeleopHintMode as M
from horus.robot import Robot, RobotType
from test_semantic_config_helpers import _build_config


def configuration():
    return ControllerHints((ControllerHintProfile(M.MINIMAP, (
        ControllerHint(C.LEFT_STICK_Y, "move_forward", "Forward/back"),
        ControllerHint(C.RIGHT_SECONDARY, "light", "Inspection light"),
    ), "inspection"), ControllerHintProfile(M.TEACH, (
        ControllerHint(C.LEFT_PRIMARY, "posture_style", "Style: crouch"),
    ), "collection")), state_topic="/inspection/controller_hints")


def test_registration_is_display_only_optional_and_roundtrips():
    robot = Robot("hint_demo", RobotType.WHEELED)
    before = _build_config(robot)
    assert "controller_hints" not in before
    hints = configuration()
    robot.configure_controller_hints(hints)
    after = _build_config(robot)
    assert after["control"] == before["control"]
    assert after["controller_hints"] == hints.to_dict()
    assert ControllerHints.from_dict(json.loads(json.dumps(hints.to_dict()))) == hints
    robot.configure_controller_hints(None)
    assert "controller_hints" not in _build_config(robot)


def test_synthetic_shared_registration_fixture_matches_actual_sdk_builder():
    import importlib.util
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location("hints_fixture", root / ".github/scripts/generate_controller_hints_contract.py")
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    assert module.fixture() == json.loads((root / "contracts/fixtures/controller_hints_registration.json").read_text())
    unicode = json.loads((root / "contracts/fixtures/controller_hints_unicode.json").read_text())
    assert ControllerHints.from_dict(unicode).to_dict() == unicode


@pytest.mark.parametrize("control", list(C))
def test_all_physical_control_identifiers_are_typed(control):
    assert ControllerHint(control.value, "action", "Label").control == control


@pytest.mark.parametrize("changes", [dict(control="menu"), dict(action_id="bad id"), dict(label="x"*25),
    dict(label="line\nbreak"), dict(label="<"*25), dict(label="\u2028"), dict(label="\ud800")])
def test_invalid_or_unreadable_binding_is_rejected(changes):
    with pytest.raises((ValueError, TypeError)):
        ControllerHint(**dict(dict(control=C.LEFT_PRIMARY, action_id="action", label="Label"), **changes))


def test_profiles_are_bounded_mode_scoped_and_reject_duplicates():
    binding = ControllerHint(C.LEFT_TRIGGER, "grasp", "Pinch")
    with pytest.raises(ValueError): ControllerHintProfile("workspace", (binding,))
    with pytest.raises(ValueError): ControllerHintProfile(M.TEACH, (binding, binding))
    with pytest.raises(ValueError): ControllerHints((ControllerHintProfile(M.TEACH, (binding,)),)*2)
    with pytest.raises(ValueError): ControllerHints(state_topic="relative")
    with pytest.raises(ValueError): ControllerHints(state_timeout_seconds=True)
    value = configuration().to_dict(); value["config_hash"] = "0"*64
    with pytest.raises(ValueError): ControllerHints.from_dict(value)
    assert ControllerHint(C.RIGHT_STICK_CLICK, "unused", "  ").label == ""


def test_dynamic_reply_is_bound_to_robot_profile_config_and_fresh_context():
    hints = configuration()
    value = dict(schema_version=1, robot_name="hint_demo", config_hash=hints.digest,
        profile_id="collection", mode="teach", context_id=str(uuid.uuid4()), context_sequence=4)
    context = ControllerHintContext.from_json(json.dumps(value))
    update = hints.reply(context, robot_name="hint_demo", labels={"posture_style":"Style: kneel"}, revision=2)
    wire = json.loads(update.to_json())
    assert wire["context_id"] == context.context_id and wire["context_sequence"] == 4
    assert wire["labels"] == [dict(action_id="posture_style",label="Style: kneel")]
    assert "control" not in wire and "authority" not in wire
    for c in (dataclasses.replace(context, robot_name="other"), dataclasses.replace(context, config_hash="0"*64),
              dataclasses.replace(context, profile_id="other"), dataclasses.replace(context, mode=M.IMMERSIVE)):
        with pytest.raises(ValueError): hints.reply(c, robot_name="hint_demo", labels={}, revision=0)
    with pytest.raises(ValueError): hints.reply(context, robot_name="hint_demo", labels={"undeclared":"Do thing"}, revision=1)
    with pytest.raises(ValueError): hints.reply(context, robot_name="hint_demo", labels={}, revision=-1)
    with pytest.raises(ValueError): hints.reply(context, robot_name="hint_demo", labels={}, revision=True)


def test_teach_grasp_descriptions_derive_from_existing_tools_without_changing_hash(tmp_path):
    from horus.teaching import TeachConfig
    config = TeachConfig(str(tmp_path), grasp_bindings={"left_index_pinch":"left_trigger"})
    before = config.digest
    assert config.registration("humanoid")["grasp_hints"] == [dict(control="left_trigger", action_id="left_index_pinch", label="Index pinch")]
    assert before == config.digest
    assert "grasp_hints" not in config.to_dict()
