"""Software-only adapter doubles verify recovery never weakens stop semantics."""

from dataclasses import replace

import pytest

from horus.teaching import TeachConfig
from horus.teaching.config import INPUT_FEATURES
from horus.teaching.session import TeachSession
from test_teaching import MockAdapter, arm, config, request, poll, align
from test_teach_input_v2 import continuous_sample

make_request = request


class PauseAdapter(MockAdapter):
    capabilities = replace(MockAdapter.capabilities, supports_tracking_pause=True)

    def __init__(self):
        super().__init__()
        self.pauses = []
        self.confirm = True
        self.pause_fault = False
        self.healthy = True

    def ready(self):
        return self.healthy, "worker fault" if not self.healthy else ""

    def pause(self, context):
        self.pauses.append(context)
        if self.pause_fault:
            raise RuntimeError("pause failed")
        return self.confirm


@pytest.fixture(params=["fixture", "real"])
def session(tmp_path, request):
    cfg = config(tmp_path, adapter_id="mock", provenance=request.param, workflow="practice",
        joint_names=("joint",), joint_groups={"arm": ("joint",)}, command_frame="base", calibration_id="cal1",
        input_schema_version=2, required_input_features=INPUT_FEATURES, engagement_mode="continuous",
        tracking_recovery="auto_resume", grasp_bindings={"left_grasp": "left_grip"})
    adapter = PauseAdapter()
    if request.param == "real":
        adapter.capabilities = replace(adapter.capabilities, qualification="hardware",
            supports_physical_tracking_recovery=True)
    s = TeachSession(cfg, "humanoid", adapter)
    try:
        assert s.request(make_request(s, "open", input_schema_version=2, input_features=list(INPUT_FEATURES),
            engagement_mode="continuous", tracking_recovery="auto_resume"))["accepted"]
        s.update_lease("operator", "authority1", 3)
        align(s)
        assert arm(s)["accepted"]
        assert s.input(sample(s, 1))
        s.tick()
        yield s
    finally:
        s.close()


def sample(s, seq, **patch):
    value = continuous_sample(s, seq)
    value.update(tracking_recovery="auto_resume")
    value.update(patch)
    return value


@pytest.mark.parametrize("patch", [dict(provenance="unknown"), dict(input_schema_version=1),
    dict(engagement_mode="hold_to_run"), dict(tracking_recovery="unknown")])
def test_recovery_requires_valid_continuous_v2_profile(tmp_path, patch):
    values = dict(provenance="fixture", input_schema_version=2, engagement_mode="continuous",
                  tracking_recovery="auto_resume")
    values.update(patch)
    with pytest.raises(ValueError):
        config(tmp_path, **values)


def test_hash_and_registration_are_explicit():
    cfg = TeachConfig("/data/teach", provenance="fixture", input_schema_version=2,
                      engagement_mode="continuous", tracking_recovery="auto_resume")
    assert cfg.digest == "4073155edb3d2526d447ef4c370f81606329f467b8aa2171989db15bed9a6127"
    assert cfg.registration("robot")["tracking_recovery"] == "auto_resume"
    assert "tracking_recovery" not in replace(cfg, tracking_recovery="manual").to_dict()
    assert TeachConfig.from_dict(cfg.to_dict()).digest == cfg.digest


@pytest.mark.parametrize("patch", [dict(left_tracking_valid=False, left=None), dict(head_tracking_valid=False, head=None),
    dict(left_tracking_valid=False, left={}),
    dict(left_tracking_valid=False, left={"position": None, "rotation": None}),
    dict(head_tracking_valid=False, head={"position": [], "rotation": []}),
    dict(left_buttons_valid=False), dict(right_stick_valid=False), dict(left_grip_valid=False),
    dict(tracking_valid=False), dict(ui_focus=True), dict(challenge="expired")])
def test_pause_keeps_anchor_and_resumes_only_fresh_sample(session, patch):
    s = session
    epoch, authority = s.input_epoch, dict(s.motion_authority)
    queued = sample(s, 3)
    assert not s.input(sample(s, 2, **patch))
    assert s.control == "paused" and s.latest is None
    assert s.input_epoch == epoch and s.motion_authority == authority
    assert s.adapter.pauses[-1]["authority_generation"] == authority["authority_generation"]
    assert not s.input(queued)  # Old challenge cannot resume, even with a newer sequence.
    assert not s.input(sample(s, 3))  # Rejected sequences cannot be retried with a new challenge.
    assert s.input(sample(s, 4))
    s.tick()
    assert len(s.adapter.anchors) == 1 and len(s.adapter.commands) == 1
    assert s.adapter.holds == 0 and s.control == "engaged"


def test_async_pause_discards_targets_until_confirmed(session):
    s = session
    s.adapter.confirm = False
    assert s.input(sample(s, 2))  # Pending target must be removed before a tick can apply it.
    assert not s.input(sample(s, 3, tracking_valid=False))
    assert s.control == "pause_requested" and s.latest is None
    before_confirmation = sample(s, 5)
    assert not s.input(sample(s, 4))
    s.adapter.confirm = True
    s.next_pause_check = 0
    s.tick()
    assert s.control == "paused" and not s.adapter.commands
    assert not s.input(before_confirmation)
    assert s.input(sample(s, 6))
    s.tick()
    assert len(s.adapter.commands) == 1 and len(s.adapter.anchors) == 1


def test_watchdog_pause_and_long_gap_with_heartbeats(session):
    s = session
    s.last_input -= 1
    s.tick()
    assert s.control == "paused"
    for _ in range(20):
        s.last_input -= 1
        assert s.request(request(s, "heartbeat"))["accepted"]
        s.update_lease("operator", "authority1", 3)
        s.tick()
    assert s.control == "paused" and len(s.adapter.pauses) == 1
    assert s.input(sample(s, 2))
    s.tick()
    assert len(s.adapter.anchors) == 1


@pytest.mark.parametrize("patch", [dict(right_secondary=True), dict(left_primary=True, right_primary=True),
    dict(clutch=False), dict(tracking_recovery="manual"), dict(left_trigger=float("nan")),
    dict(left_buttons_valid="false"), dict(frame_id="wrong")])
def test_stop_or_malformed_input_cannot_be_softened_by_tracking_loss(session, patch):
    s = session
    assert not s.input(sample(s, 2, left_tracking_valid=False, **patch))
    assert s.control == "holding" and not s.input_epoch
    assert not s.input(sample(s, 3)) and not s.adapter.commands


def test_delayed_stop_is_not_a_recoverable_stale_sample(session):
    assert not session.input(sample(session, 2, challenge="old", right_secondary=True))
    assert session.control == "holding"


@pytest.mark.parametrize("fault", ["lease", "disconnect", "controller", "explicit_stop", "pause_fault"])
def test_pause_still_enforces_all_hard_stop_conditions(session, fault):
    s = session
    if fault == "pause_fault": s.adapter.pause_fault = True
    assert not s.input(sample(s, 2, tracking_valid=False))
    if fault == "lease": s.update_lease("other", "other", 3)
    elif fault == "disconnect": s.owner_seen -= 6
    elif fault == "controller": s.adapter.healthy = False
    elif fault == "explicit_stop": assert s.request(request(s, "hold"))["accepted"]
    s.tick()
    assert s.control in {"holding", "faulted"} and not s.input_epoch
    assert not s.input(sample(s, 3))


def test_unsupported_adapter_cannot_engage(session):
    s = session
    s.request(request(s, "hold"))
    s.adapter.capabilities = replace(s.adapter.capabilities, supports_tracking_pause=False)
    assert not arm(s)["accepted"]


@pytest.mark.parametrize("capability", [False, None, 1, "true"])
def test_physical_recovery_requires_explicit_adapter_opt_in(session, capability):
    s = session
    s.request(request(s, "hold"))
    s.config = replace(s.config, provenance="real")
    s.adapter.capabilities = replace(s.adapter.capabilities, qualification="hardware",
        supports_physical_tracking_recovery=capability)
    result = arm(s)
    assert not result["accepted"] and "physical tracking recovery" in result["reason"]
    assert s.control == "holding" and not s.input_epoch


def test_legacy_hardware_capabilities_fail_closed(session):
    from dataclasses import asdict
    from types import SimpleNamespace

    s = session
    s.request(request(s, "hold"))
    s.config = replace(s.config, provenance="real")
    capabilities = asdict(s.adapter.capabilities)
    capabilities.update(qualification="hardware")
    capabilities.pop("supports_physical_tracking_recovery")
    s.adapter.capabilities = SimpleNamespace(**capabilities)
    assert not arm(s)["accepted"]


@pytest.mark.parametrize("capabilities", [dict(qualification="simulation"),
    dict(controller_watchdog="unavailable"), dict(authority_contract="unavailable"),
    dict(supports_tracking_pause=False), dict(calibration_id="different"),
    dict(joint_names=("wrong_joint",)), dict(command_frame="wrong")])
def test_physical_opt_in_cannot_override_controller_requirements(session, capabilities):
    s = session
    s.request(request(s, "hold"))
    s.config = replace(s.config, provenance="real")
    fields = dict(qualification="hardware", supports_physical_tracking_recovery=True)
    fields.update(capabilities)
    s.adapter.capabilities = replace(s.adapter.capabilities, **fields)
    assert not arm(s)["accepted"]


def test_manual_profile_cannot_auto_resume_even_with_hardware_opt_in(session):
    s = session
    s.request(request(s, "hold"))
    s.config = replace(s.config, provenance="real", tracking_recovery="manual")
    s.adapter.capabilities = replace(s.adapter.capabilities, qualification="hardware",
        supports_physical_tracking_recovery=True)
    assert arm(s)["accepted"]
    assert s.input(sample(s, 1, tracking_recovery="manual"))
    s.tick()
    assert not s.input(sample(s, 2, tracking_recovery="manual", tracking_valid=False))
    assert s.control == "holding" and not s.input_epoch and not s.adapter.pauses
    assert not s.input(sample(s, 3, tracking_recovery="manual"))


def test_real_provenance_roundtrip_and_no_startup_actuation(tmp_path):
    cfg = config(tmp_path, adapter_id="mock", provenance="real", workflow="practice",
        joint_names=("joint",), joint_groups={"arm": ("joint",)}, command_frame="base", calibration_id="cal1",
        input_schema_version=2, engagement_mode="continuous", tracking_recovery="auto_resume")
    assert TeachConfig.from_dict(cfg.to_dict()).provenance == "real"
    assert cfg.to_dict()["provenance"] == "real"
    assert replace(cfg, tracking_recovery="manual").digest != cfg.digest
    adapter = PauseAdapter()
    adapter.capabilities = replace(adapter.capabilities, qualification="hardware",
        supports_physical_tracking_recovery=True)
    s = TeachSession(cfg, "humanoid", adapter)
    try:
        assert not s.state()["control_ready"]
        assert s.request(request(s, "open", input_schema_version=2, input_features=list(INPUT_FEATURES),
            engagement_mode="continuous", tracking_recovery="auto_resume"))["accepted"]
        align(s)
        assert s.state()["control_ready"]
        s.update_lease("operator", "authority1", 3)
        assert arm(s)["accepted"]
        s.tick()
        assert not adapter.commands and not adapter.anchors and not adapter.pauses and not adapter.holds
    finally:
        s.close()


@pytest.mark.parametrize("fault", ["lease_expiry", "new_generation", "required_camera", "collection_recorder"])
def test_recovery_cannot_bypass_revocation_or_required_data_failure(session, fault):
    from types import SimpleNamespace
    from horus.teaching import TeachCamera, TeachStream

    s = session
    assert not s.input(sample(s, 2, tracking_valid=False))
    if fault == "lease_expiry":
        s.lease_deadline = 0
    elif fault == "new_generation":
        s.update_lease("operator", "authority2", 3)
    elif fault == "required_camera":
        s.config = replace(s.config,
            streams=(TeachStream("front", "/front", "sensor_msgs/msg/Image", role="camera"),),
            cameras=(TeachCamera("front_camera", stream_id="front", required_for_control=True),))
    else:
        s.config = replace(s.config, workflow="collection")
        s.recorder = SimpleNamespace(error="disk fault", dropped=0)
    s.tick()
    assert s.control == "holding" and not s.input_epoch
    assert not s.input(sample(s, 3)) and not s.adapter.commands
    s.recorder = None


def test_recording_contains_pause_resume_without_new_anchor(session):
    import uuid
    from horus.teaching.dataset import Episode

    s = session
    assert s.request(request(s, "record", episode_id=str(uuid.uuid4())))["accepted"]
    poll(s, "recording")
    assert arm(s)["accepted"]
    assert s.input(sample(s, 1))
    s.tick()
    assert not s.input(sample(s, 2, ui_focus=True))
    assert s.capture == "recording"
    assert s.input(sample(s, 3))
    s.tick()
    assert s.request(request(s, "finish", outcome="success"))["accepted"]
    poll(s, "saved")
    from pathlib import Path
    episode = Episode(Path(s.config.recording_root) / s.episode_id)
    kinds = [event["kind"] for event in episode.events()]
    assert kinds.count("tracking_pause") == kinds.count("tracking_resumed") == kinds.count("anchored") == 1
    contract = next(event["data"] for event in episode.events() if event["kind"] == "adapter_contract")
    assert contract["supports_physical_tracking_recovery"] is (s.config.provenance == "real")
    assert not episode.validate()
