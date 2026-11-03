import dataclasses
import json
import time
import uuid

import pytest

from horus.robot import Robot, RobotType, FieldTeammate
from horus.teaching import TeachConfig, TeachStream, TeachCamera
from horus.teaching.adapter import AdapterCapabilities, TeachAdapter
from horus.teaching.dataset import Episode
from horus.teaching.recording import EpisodeRecorder, Sample
from horus.teaching.session import TeachSession


def config(tmp_path, **values):
    return TeachConfig(**dict(dict(recording_root=str(tmp_path), minimum_free_bytes=1024), **values))


def request(session, action, **values):
    body = dict(request_id=str(uuid.uuid4()), config_hash=session.config.digest,
                owner="operator", session_id=session.session_id, action=action,
                expected_revision=session.revision)
    body["alignment_id"] = session.alignment["alignment_id"] if session.alignment else ""
    body.update(values)
    return body


class MockAdapter(TeachAdapter):
    capabilities = AdapterCapabilities("mock", "simulation", ("joint",), "base", "cal1", "pose", "mock-watchdog",
                                       authority_contract="test-only scoped authority")

    def __init__(self):
        self.commands, self.anchors, self.holds = [], [], 0
        self.hold_contexts = []

    def ready(self): return True, ""
    def anchor(self, sample): self.anchors.append(sample)
    def apply(self, sample):
        self.commands.append(sample)
        return {"accepted": True, "execution": "unknown"}
    def hold(self, context):
        self.holds += 1
        self.hold_contexts.append(context)
        return True


@pytest.fixture
def motion_session(tmp_path):
    adapter = MockAdapter()
    session = TeachSession(config(tmp_path, adapter_id="mock", provenance="fixture", workflow="practice",
        joint_names=("joint",), joint_groups={"arm": ("joint",)}, command_frame="base", calibration_id="cal1"), "humanoid", adapter)
    assert session.request(request(session, "open"))["accepted"]
    align(session)
    session.update_lease("operator", "authority1", 3)
    yield session
    session.close()


def input_sample(session, sequence=1, **values):
    sample = dict(session_id=session.session_id, input_epoch=session.input_epoch, owner="operator",
        sequence=sequence, challenge=session.state()["challenge"], clutch=True, tracking_valid=True,
        ui_focus=False, left={"position": [0, 0, 0], "rotation": [0, 0, 0, 1]},
        right={"position": [0, 0, 0], "rotation": [0, 0, 0, 1]})
    sample.update(frame_id="quest_tracking_flu",
        alignment_id=session.alignment["alignment_id"] if session.alignment else "",
        tracking_origin_id=session.alignment["tracking_origin_id"] if session.alignment else "")
    sample.update(values)
    return sample


def align(session, **patch):
    value = dict(schema_version=1, alignment_id=str(uuid.uuid4()), tracking_origin_id=str(uuid.uuid4()),
                 frame_id="quest_tracking_flu", child_frame_id="teach_calibrated_floor_flu",
                 position=[0, 0, 0], rotation=[0, 0, 0, 1])
    value.update(patch)
    result = session.request(request(session, "align", alignment=value))
    assert result["accepted"], result
    return value


def arm(session):
    return session.request(request(session, "engage", authority_generation="authority1"))


def poll(session, target):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        session.tick()
        if session.capture == target: return
        time.sleep(0.01)
    pytest.fail(f"Expected {target}, got {session.capture}: {session.error}")


def test_opt_in_type_and_roundtrip(tmp_path):
    cfg = config(tmp_path)
    robot = Robot("humanoid", RobotType.HUMANOID)
    robot.configure_teaching(cfg)
    assert TeachConfig.from_dict(robot.get_metadata("teaching_config")).digest == cfg.digest
    with pytest.raises(ValueError): Robot("rover", RobotType.WHEELED).configure_teaching(cfg)
    with pytest.raises(ValueError): FieldTeammate("person").configure_teaching(cfg)


@pytest.mark.parametrize("values", [dict(schema_version=2), dict(publish_rate_hz=float("nan")),
    dict(joint_names=("a",), joint_groups={"one": ("a",), "two": ("a",)}),
    dict(adapter_id="motor"), dict(cameras=(TeachCamera("front", stream_id="missing"),))])
def test_invalid_configuration(tmp_path, values):
    with pytest.raises(ValueError): config(tmp_path, **values)


def test_no_commands_on_open_or_arm_and_first_sample_anchors(motion_session):
    s = motion_session
    assert not s.adapter.commands
    assert arm(s)["accepted"]
    assert s.control == "armed"
    assert s.input(input_sample(s))
    s.tick()
    assert len(s.adapter.anchors) == 1 and not s.adapter.commands
    assert s.input(input_sample(s, 2))
    s.tick()
    assert len(s.adapter.commands) == 1


def test_lease_required_and_generation_change_holds(motion_session):
    s = motion_session
    s.update_lease("", "", 0)
    assert not arm(s)["accepted"]
    s.update_lease("operator", "authority1", 3)
    assert arm(s)["accepted"]
    s.update_lease("operator", "authority2", 3)
    assert s.control == "holding"
    assert not s.input(input_sample(s))


@pytest.mark.parametrize("fault", [dict(tracking_valid=False), dict(ui_focus=True),
    dict(challenge="expired"), dict(left={"position": [float("nan"), 0, 0], "rotation": [0, 0, 0, 1]})])
def test_bad_input_holds(motion_session, fault):
    s = motion_session
    arm(s)
    assert not s.input(input_sample(s, **fault))
    assert s.control == "holding"
    assert not s.adapter.commands


def test_latest_only_duplicate_and_reengagement(motion_session):
    s = motion_session
    arm(s)
    first = input_sample(s)
    assert s.input(first)
    assert not s.input(first)
    assert s.input(input_sample(s, 2))
    s.tick()
    assert s.adapter.anchors[0]["sequence"] == 2
    old_epoch = s.input_epoch
    s.request(request(s, "hold"))
    arm(s)
    assert s.input_epoch != old_epoch
    assert not s.input(dict(first, sequence=10))


def test_explicit_hold_clears_only_a_confirmed_fault(motion_session):
    s = motion_session
    arm(s)
    s._hold("adapter fault")
    s.control, s.error = "faulted", "reach exceeded"
    assert s.motion_authority is None
    assert s.request(request(s, "hold"))["accepted"]
    assert s.control == "holding" and not s.error
    assert arm(s)["accepted"]
    s.adapter.hold = lambda context: False
    s.control = "faulted"
    s.request(request(s, "hold"))
    assert s.motion_authority is not None
    assert s.control == "hold_requested"
    assert not arm(s)["accepted"]


def test_watchdog_holds(motion_session):
    s = motion_session
    arm(s)
    s.input(input_sample(s))
    s.last_input -= 1
    s.tick()
    assert s.control == "holding"


def test_idempotence_and_revision(motion_session):
    s = motion_session
    req = request(s, "engage", authority_generation="authority1")
    assert s.request(req)["accepted"]
    revision = s.revision
    assert s.request(req)["accepted"]
    assert s.revision == revision
    assert not s.request(dict(req, action="close"))["accepted"]
    assert not s.request(request(s, "close", expected_revision=-1))["accepted"]
    assert s.request(request(s, "hold", expected_revision=-1))["accepted"]


def test_rejected_request_retry_preserves_original_reason(motion_session):
    s = motion_session
    req = request(s, "engage", authority_generation="wrong")
    first = s.request(req)
    second = s.request(req)
    assert not first["accepted"] and not second["accepted"]
    assert first["reason"] == second["reason"] == "A current positive robot lease is required"


def test_observation_record_save_is_separate_from_control(tmp_path):
    s = TeachSession(config(tmp_path), "humanoid")
    try:
        s.request(request(s, "open"))
        assert not arm(s)["accepted"]
        req = request(s, "record", episode_id=str(uuid.uuid4()))
        assert s.request(req)["accepted"]
        assert s.request(req)["accepted"]
        poll(s, "recording")
        s.recorder.event("test", {"measured": 1})
        assert s.request(request(s, "finish", outcome="failure"))["accepted"]
        poll(s, "saved")
        episode = Episode(tmp_path / s.episode_id)
        assert episode.manifest["outcome"] == "failure"
        assert episode.validate() == []
        assert any(e["kind"] == "test" for e in episode.events())
    finally: s.close()


def test_missing_streams_and_overflow_not_acceptable(tmp_path):
    cfg = config(tmp_path, streams=(TeachStream("joints", "/joints", "sensor_msgs/msg/JointState"),), queue_budget_bytes=1024)
    r = EpisodeRecorder(cfg, "robot", str(uuid.uuid4()), "task")
    assert not r.append(Sample("joints", b"x" * 2048, time.time_ns(), time.monotonic_ns()))
    result = r.finish()
    assert result["capture"] == "saved" and result["quality"] == "unacceptable"
    assert "missing:joints" in result["quality_issues"]
    assert result["recorder_drops"] == 1


def test_disk_headroom_and_no_overwrite(tmp_path):
    with pytest.raises(OSError):
        EpisodeRecorder(config(tmp_path, minimum_free_bytes=10**20), "robot", str(uuid.uuid4()), "task")
    cfg = config(tmp_path)
    eid = str(uuid.uuid4())
    r = EpisodeRecorder(cfg, "robot", eid, "task")
    r.event("first", {})
    r.finish()
    with pytest.raises(FileExistsError): EpisodeRecorder(cfg, "robot", eid, "task")


def test_artifact_tampering_is_detected(tmp_path):
    r = EpisodeRecorder(config(tmp_path), "robot", str(uuid.uuid4()), "task")
    r.event("first", {})
    r.finish()
    with (r.path / "data.mcap").open("ab") as f: f.write(b"tampered")
    assert Episode(r.path).validate()


def test_idle_armed_session_does_not_expire_while_operator_is_present(motion_session):
    s = motion_session
    arm(s)
    s.last_input -= 30
    s.tick()
    assert s.control == "armed"


def test_configurable_single_hand_and_grasp(tmp_path):
    cfg = config(tmp_path, adapter_id="mock", provenance="fixture", workflow="practice",
        controlled_hands=("left",), clutch_binding="right_primary", grasp_bindings={"gripper": "left_trigger"},
        joint_names=("joint",), joint_groups={"arm": ("joint",)}, command_frame="base", calibration_id="cal1")
    s = TeachSession(cfg, "humanoid", MockAdapter())
    try:
        s.request(request(s, "open"))
        align(s)
        s.update_lease("operator", "authority1", 3)
        arm(s)
        assert s.input(input_sample(s, right=None, left_trigger=0.7))
        s.tick()
        assert s.adapter.anchors[-1]["tools"] == {"gripper": 0.7}
        assert not s.input(input_sample(s, 2, right=None, left_trigger=float("inf")))
        assert s.control == "holding"
    finally: s.close()


def test_conflicting_bindings_are_rejected(tmp_path):
    with pytest.raises(ValueError, match="cannot share"):
        config(tmp_path, grasp_bindings={"gripper": "left_grip"})


def test_finish_is_idempotent_and_does_not_count_finalization_as_a_gap(tmp_path):
    cfg = config(tmp_path)
    r = EpisodeRecorder(cfg, "robot", str(uuid.uuid4()), "task")
    r.event("test", {})
    a = r.finish("failure")
    assert r.finish("success") == a
    assert a["outcome"] == "failure"


def test_recovery_refuses_live_capture_and_preserves_original(tmp_path):
    import shutil
    from horus.teaching.recovery import recover
    from horus.teaching.recording import file_digest
    r = EpisodeRecorder(config(tmp_path), "robot", str(uuid.uuid4()), "task")
    try:
        with pytest.raises(RuntimeError, match="still active"): recover(r.path)
        for i in range(100): r.event("sample", {"i": i})
        r.finish()
    finally:
        if r.thread.is_alive(): r.finish(interrupted=True)
    interrupted = tmp_path / (str(uuid.uuid4()) + ".inprogress")
    shutil.copytree(r.path, interrupted)
    with (interrupted / "data.mcap").open("r+b") as f:
        f.truncate((interrupted / "data.mcap").stat().st_size - 200)
    original_hash = file_digest(interrupted / "data.mcap")
    restored = Episode(recover(interrupted))
    assert restored.manifest["capture"] == "interrupted"
    assert restored.manifest["quality"] == "unacceptable"
    assert restored.manifest["stream_counts"]["__events"] == 100
    assert not restored.validate()
    assert file_digest(interrupted / "data.mcap") == original_hash


def test_native_definition_configuration_tampering(tmp_path):
    r = EpisodeRecorder(config(tmp_path), "robot", str(uuid.uuid4()), "task")
    r.event("test", {})
    r.finish()
    episode = Episode(r.path)
    episode.manifest["config"]["task"] = "tampered"
    assert "Recording configuration hash mismatch" in episode.validate()


def test_cartesian_anchor_rotation_limits_and_measured_pose():
    pytest.importorskip("scipy")
    from horus.teaching.cartesian import CartesianAdapter, CartesianLimits
    from scipy.spatial.transform import Rotation
    pose = {"position": [0.4, -0.2, 0.8], "rotation": [0, 0, 0, 1]}
    cap = AdapterCapabilities("pose", "simulation", ("joint",), "base", "cal", "cartesian_pose_absolute",
                              "fixture", command_boundary="accepted", authority_contract="test-only scoped authority")
    sent = []
    def publish(bundle):
        sent.append(bundle)
        return {"ok": True, "boundary": "accepted", "execution": "unknown"}
    adapter = CartesianAdapter(cap, CartesianLimits((-2, -2, -2), (2, 2, 2), 5, 10,
        floor_to_command_rotation=tuple(Rotation.from_euler("z", 90, degrees=True).as_quat())),
        feedback=lambda: {"received_monotonic": time.monotonic(), "ready": True, "poses": {"left": pose}},
        publish=publish, hold=lambda context: True, hands=("left",))
    authority = dict(robot_name="robot", owner="operator", session_id="session", authority_generation="lease")
    sample = {"sequence": 1, "authority": authority, "left": {"position": [0, 0, 0], "rotation": [0, 0, 0, 1]}}
    adapter.anchor(sample)
    assert not sent
    adapter.previous_time -= 0.1
    sample["left"]["position"][0] = 0.1
    result = adapter.apply(sample)
    assert result["targets"]["left"]["position"] == pytest.approx([0.4, -0.1, 0.8])
    assert result["boundary"] == "accepted"
    assert result["authority"] == authority
    sample["left"]["position"][0] = 100
    with pytest.raises(RuntimeError, match="workspace limit"): adapter.apply(sample)
    assert len(sent) == 1
    assert adapter.hold(dict(authority, reason="operator"))


def test_recording_and_shutdown_never_send_unsolicited_hold(motion_session):
    s = motion_session
    assert s.request(request(s, "record", episode_id=str(uuid.uuid4())))["accepted"]
    poll(s, "recording")
    assert s.request(request(s, "finish"))["accepted"]
    poll(s, "saved")
    s.close()
    assert not s.adapter.commands and not s.adapter.holds


def test_hold_uses_original_authority_and_is_not_repeated_after_confirmation(motion_session):
    s = motion_session
    assert arm(s)["accepted"]
    assert s.input(input_sample(s, authority={"owner": "spoof"}))
    s.tick()
    assert s.adapter.anchors[0]["authority"]["owner"] == "operator"
    s.update_lease("different-operator", "authority2", 3)
    assert s.adapter.hold_contexts[-1]["authority_generation"] == "authority1"
    assert s.adapter.hold_contexts[-1]["owner"] == "operator"
    s.request(request(s, "hold"))
    s.close()
    assert s.adapter.holds == 1


def test_pending_hold_blocks_reengagement_and_confirms_later(motion_session):
    s = motion_session
    assert arm(s)["accepted"]
    s.adapter.hold = lambda context: False
    s.request(request(s, "hold"))
    assert s.control == "hold_requested"
    assert not arm(s)["accepted"]
    s.adapter.hold = lambda context: True
    s.next_hold_check = 0
    s.tick()
    assert s.control == "holding"
    assert arm(s)["accepted"]


def test_collection_queue_drop_updates_quality_and_holds(tmp_path):
    cfg = config(tmp_path, adapter_id="mock", provenance="fixture", workflow="collection",
        joint_names=("joint",), joint_groups={"arm": ("joint",)}, command_frame="base",
        calibration_id="cal1", queue_budget_bytes=1024)
    s = TeachSession(cfg, "robot", MockAdapter())
    try:
        s.request(request(s, "open"))
        align(s)
        s.update_lease("operator", "authority1", 3)
        s.request(request(s, "record", episode_id=str(uuid.uuid4())))
        poll(s, "recording")
        assert arm(s)["accepted"]
        assert not s.recorder.append(Sample("large", b"x" * 2048, time.time_ns(), time.monotonic_ns()))
        s.tick()
        assert s.quality == "unacceptable" and s.control == "holding"
    finally:
        s.close()
