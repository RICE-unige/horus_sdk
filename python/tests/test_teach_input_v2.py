import copy
import dataclasses
import json
import uuid
from pathlib import Path

import pytest

from horus.teaching import TeachConfig
from horus.teaching.config import INPUT_FEATURES
from horus.teaching.dataset import Episode
from horus.teaching.input import negotiate_input
from horus.teaching.session import TeachSession
from test_teaching import MockAdapter, config, input_sample, poll, request, arm, align


FIXTURE = json.loads((Path(__file__).resolve().parents[2] / "contracts/fixtures/teaching.json").read_text())


@pytest.fixture
def session_v2(tmp_path):
    cfg = config(tmp_path, adapter_id="mock", provenance="fixture", workflow="practice",
        joint_names=("joint",), joint_groups={"arm": ("joint",)}, command_frame="base", calibration_id="cal1",
        input_schema_version=2, required_input_features=INPUT_FEATURES, clutch_binding="both_primary",
        grasp_bindings={"left_grasp": "left_grip", "right_grasp": "right_grip",
                        "left_pinch": "left_trigger", "right_pinch": "right_trigger"})
    s = TeachSession(cfg, "humanoid", MockAdapter())
    assert s.request(request(s, "open", input_schema_version=2, input_features=list(INPUT_FEATURES)))["accepted"]
    align(s)
    s.update_lease("operator", "authority1", 3)
    yield s
    s.close()


def sample_v2(s, sequence=1):
    values = copy.deepcopy(FIXTURE["input_v2"]["sample"])
    values["frame_id"] = "quest_tracking_flu"
    return input_sample(s, sequence, **values)


def test_golden_configuration_legacy_and_v2():
    for case in FIXTURE["cases"]:
        c = TeachConfig.from_dict(case["input"])
        assert c.to_dict() == case["normalized"]
        assert c.digest == case["digest"]
        assert json.loads(json.dumps(c.registration("teach_robot"))) == case["registration"]


def test_continuous_hash_matches_native_languages():
    c = TeachConfig("/data/teach", input_schema_version=2, engagement_mode="continuous")
    assert c.digest == "b4a500fb99e6ba3460b1212b74b2bb990be0e4371a3328f88bcf1b6d056690fe"


@pytest.mark.parametrize("values", [dict(input_schema_version=True), dict(input_schema_version=3),
    dict(input_schema_version=2., required_input_features=()), dict(required_input_features=("head_pose",)),
    dict(input_schema_version=2, required_input_features=("unknown",)),
    dict(input_schema_version=2, required_input_features=("head_pose", "head_pose"))])
def test_invalid_v2_config(tmp_path, values):
    with pytest.raises((ValueError, TypeError)):
        config(tmp_path, **values)


@pytest.mark.parametrize("open_fields", [{}, {"input_schema_version": 1}, {"input_schema_version": 2},
    {"input_schema_version": 2, "input_features": []},
    {"input_schema_version": 2, "input_features": [*INPUT_FEATURES, "unknown"]},
    {"input_schema_version": 2, "input_features": [*INPUT_FEATURES, "head_pose"]},
    {"input_schema_version": True, "input_features": list(INPUT_FEATURES)}])
def test_legacy_or_incomplete_open_cannot_claim_session(tmp_path, open_fields):
    s = TeachSession(config(tmp_path, input_schema_version=2, required_input_features=INPUT_FEATURES), "humanoid")
    try:
        assert not s.request(request(s, "open", **open_fields))["accepted"]
        assert not s.owner and not s.session_id and not s.input_features
    finally:
        s.close()


def test_negotiation_is_order_independent_and_cleared_on_close(session_v2):
    s = session_v2
    assert s.request(request(s, "open", input_schema_version=2, input_features=list(reversed(INPUT_FEATURES))))["accepted"]
    assert s.state()["input_features"] == sorted(INPUT_FEATURES)
    assert s.request(request(s, "close"))["accepted"]
    assert s.state()["input_features"] == []
    assert not s.request(request(s, "open"))["accepted"]


def test_all_measurements_reach_adapter_and_recording(session_v2, tmp_path):
    s = session_v2
    assert s.request(request(s, "record", episode_id=str(uuid.uuid4())))["accepted"]
    poll(s, "recording")
    assert arm(s)["accepted"]
    assert s.input(sample_v2(s))
    s.tick()
    assert not s.adapter.commands
    sample = sample_v2(s, 2)
    assert s.input(sample)
    s.tick()
    mapped = dict(sample, frame_id="teach_calibrated_floor_flu", source_frame_id="quest_tracking_flu")
    for key, value in mapped.items():
        assert s.adapter.commands[-1][key] == value
    assert s.adapter.commands[-1]["tools"] == dict(left_grasp=.6, right_grasp=.6, left_pinch=.4, right_pinch=.4)
    assert s.request(request(s, "finish", outcome="success"))["accepted"]
    poll(s, "saved")
    episode = Episode(tmp_path / s.episode_id)
    assert not episode.validate()
    intents = [e for e in episode.events() if e["kind"] == "operator_intent"]
    assert len(intents) == 2
    assert intents[-1]["data"] == mapped


def test_short_button_edges_survive_latest_pose_coalescing(session_v2):
    s = session_v2
    assert arm(s)["accepted"]
    first = sample_v2(s, 1)
    assert s.input(first)
    s.tick()
    pressed = sample_v2(s, 2)
    pressed["left_stick_click"] = not first["left_stick_click"]
    pressed["source_monotonic_seconds"] = 10.01
    assert s.input(pressed)
    released = sample_v2(s, 3)
    released["source_monotonic_seconds"] = 10.02
    released["left"]["position"][0] = .123
    released["input_edges"] = [{"button": "forged", "pressed": True}]
    assert s.input(released)
    s.tick()
    assert len(s.adapter.commands) == 1, "Edges must not replay older motion captures"
    latest = s.adapter.commands[0]
    assert latest["sequence"] == 3 and latest["source_monotonic_seconds"] == 10.02
    assert latest["left"]["position"][0] == pytest.approx(.123)
    assert latest["input_edges"] == [
        dict(button="left_stick_click", pressed=pressed["left_stick_click"], sequence=2, source_monotonic_seconds=10.01),
        dict(button="left_stick_click", pressed=first["left_stick_click"], sequence=3, source_monotonic_seconds=10.02),
    ]
    assert s.input(sample_v2(s, 4))
    s.tick()
    assert s.adapter.commands[-1]["input_edges"] == []


def test_pending_edges_are_bounded_and_do_not_cross_hold(session_v2):
    s = session_v2
    assert arm(s)["accepted"]
    assert s.input(sample_v2(s, 1))
    s.tick()
    for seq in range(2, s.MAX_PENDING_INPUT_EDGES + 3):
        sample = sample_v2(s, seq)
        sample["left_stick_click"] = seq % 2 == 0
        accepted = s.input(sample)
    assert not accepted
    assert s.control == "holding" and s.latest is None
    assert not s._input_edges and s._input_buttons is None
    assert s.hold_reason == "input edge queue overflow"


@pytest.mark.parametrize("fault", FIXTURE["input_v2"]["invalid"])
def test_invalid_measurement_cannot_reach_adapter(session_v2, fault):
    s = session_v2
    assert arm(s)["accepted"]
    assert s.input(sample_v2(s))
    s.tick()
    bad = sample_v2(s, 2)
    bad.update(fault.get("patch", {}))
    if "remove" in fault:
        bad.pop(fault["remove"])
    assert not s.input(bad)
    s.tick()
    assert s.control == "holding"
    assert not s.adapter.commands


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_input_holds(session_v2, bad):
    s = session_v2
    arm(s)
    sample = sample_v2(s)
    sample["head"]["position"][0] = bad
    assert not s.input(sample)
    assert s.control == "holding"


def test_optional_head_loss_is_explicit_and_allowed(session_v2):
    s = session_v2
    s.config = dataclasses.replace(s.config, required_input_features=())
    assert arm(s)["accepted"]
    sample = sample_v2(s)
    sample.update(head_tracking_valid=False, head=None)
    assert s.input(sample)
    s.tick()
    assert s.adapter.anchors[-1]["head"] is None


def test_v2_watchdog_and_old_epoch(session_v2):
    s = session_v2
    arm(s)
    old = sample_v2(s)
    assert s.input(old)
    s.tick()
    s.last_input -= 1
    s.tick()
    assert s.control == "holding"
    arm(s)
    old["sequence"] += 1
    assert not s.input(old)
    assert s.input(sample_v2(s))
    s.tick()
    assert len(s.adapter.anchors) == 2 and not s.adapter.commands
    s.owner_seen -= 6
    s.tick()
    assert not s.input_features and not s.owner


def test_legacy_default_not_silently_upgraded(tmp_path):
    c = config(tmp_path)
    assert "input_schema_version" not in c.to_dict()
    assert "required_input_features" not in c.registration("humanoid")
    assert negotiate_input(c, {}) == []
    with pytest.raises(ValueError):
        negotiate_input(c, dict(input_schema_version=2, input_features=list(INPUT_FEATURES)))


@pytest.fixture
def continuous_session(tmp_path):
    cfg = config(tmp_path, adapter_id="mock", provenance="fixture", workflow="practice",
        joint_names=("joint",), joint_groups={"arm": ("joint",)}, command_frame="base", calibration_id="cal1",
        input_schema_version=2, required_input_features=INPUT_FEATURES, engagement_mode="continuous")
    s = TeachSession(cfg, "humanoid", MockAdapter())
    try:
        assert not s.request(request(s, "open", input_schema_version=2, input_features=list(INPUT_FEATURES)))["accepted"]
        assert s.request(request(s, "open", input_schema_version=2, input_features=list(INPUT_FEATURES),
            engagement_mode="continuous"))["accepted"]
        align(s)
        s.update_lease("operator", "authority1", 3)
        yield s
    finally:
        s.close()


def continuous_sample(s, sequence=1):
    sample = sample_v2(s, sequence)
    sample.update(engagement_mode="continuous", left_primary=False, right_primary=False)
    return sample


@pytest.mark.parametrize("values", [dict(engagement_mode="unknown"),
    dict(engagement_mode="continuous"), dict(engage_countdown_seconds=True),
    dict(engage_countdown_seconds=2), dict(engage_countdown_seconds=11),
    dict(engage_countdown_seconds=float("nan")), dict(engage_countdown_seconds=float("inf")),
    dict(disengage_binding="both_grips")])
def test_invalid_continuous_config(tmp_path, values):
    with pytest.raises((ValueError, TypeError)):
        config(tmp_path, **values)


def test_continuous_accepts_released_buttons_until_explicit_hold(continuous_session):
    s = continuous_session
    assert s.state()["engagement_mode"] == "continuous"
    registration = s.config.registration("humanoid")
    assert registration["engage_countdown_seconds"] == 3.0
    assert registration["disengage_binding"] == "both_primary"
    assert not s.input(continuous_sample(s))  # An open session does not arm control.
    assert arm(s)["accepted"]
    assert s.input(continuous_sample(s))
    s.tick()
    assert len(s.adapter.anchors) == 1 and not s.adapter.commands
    for sequence in range(2, 6):
        assert s.input(continuous_sample(s, sequence))
        s.tick()
    assert len(s.adapter.commands) == 4
    assert s.adapter.commands[-1]["left_primary"] is False  # Never fake a held button.
    assert s.request(request(s, "hold"))["accepted"]
    assert not s.input(continuous_sample(s, 6))
    assert s.control == "holding"


@pytest.mark.parametrize("fault", [dict(left_primary=True, right_primary=True),
    dict(right_secondary=True), dict(left_buttons_valid=False), dict(right_buttons_valid=False),
    dict(tracking_valid=False), dict(left_tracking_valid=False), dict(head_tracking_valid=False),
    dict(ui_focus=True), dict(clutch=False), dict(engagement_mode="hold_to_run")])
def test_continuous_fault_holds_and_requires_new_engage(continuous_session, fault):
    s = continuous_session
    assert arm(s)["accepted"]
    value = continuous_sample(s)
    value.update(fault)
    assert not s.input(value)
    s.tick()
    assert s.control == "holding"
    assert not s.adapter.commands
    assert not s.input(continuous_sample(s, 2))
    assert arm(s)["accepted"]
    assert s.input(continuous_sample(s))
    s.tick()
    assert len(s.adapter.anchors) == 1 and not s.adapter.commands


def test_continuous_watchdog_and_authority_loss(continuous_session):
    s = continuous_session
    arm(s)
    assert s.input(continuous_sample(s))
    s.tick()
    s.last_input -= 1
    s.tick()
    assert s.control == "holding"
    assert s.state()["last_hold_reason"] == "input watchdog expired"
    stopped = s.state()["last_hold_input"]
    assert stopped["received_sequence"] == stopped["accepted_sequence"] == 1
    assert stopped["accepted_age_ms"] >= 1000
    assert stopped["callback_lock_wait_ms"] >= 0
    assert s.request(request(s, "heartbeat"))["last_hold_reason"] == "input watchdog expired"
    assert not s.input(continuous_sample(s, 2))
    assert s.state()["input_health"]["received_sequence"] == 2
    assert s.state()["input_health"]["accepted_sequence"] == 1
    assert s.state()["last_hold_input"] == stopped
    arm(s)
    assert s.input(continuous_sample(s))
    s.tick()
    s.update_lease("another_operator", "authority2", 3)
    s.tick()
    assert s.control == "holding"
    assert not s.adapter.commands


def test_continuous_has_no_fixed_duration_with_healthy_inputs(continuous_session, monkeypatch):
    s = continuous_session
    import time
    now = [time.monotonic()]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    arm(s)
    for sequence in range(1, 1801):
        now[0] += 1 / 30
        s.update_lease("operator", "authority1", 3)
        for stream in s.config.streams:
            s.note_stream(stream.stream_id)
        assert s.input(continuous_sample(s, sequence))
        s.tick()
        assert s.control == "engaged"
    assert not s.state()["last_hold_reason"]
    s.last_input -= 1
    s.tick()
    assert s.state()["last_hold_reason"] == "input watchdog expired"
    assert not s.input(continuous_sample(s, 1801))
