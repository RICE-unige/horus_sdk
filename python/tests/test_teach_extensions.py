import dataclasses
import json
import threading
import time
import uuid

import pytest

from horus.teaching import EpisodeCatalog, TeachAction, TeachConfig
from horus.teaching.extensions import task_identity
from horus.teaching.session import TeachSession
from test_teaching import MockAdapter, config, request, poll, align, arm, input_sample


def settle(session):
    deadline = time.monotonic() + 5
    while session.extensions.busy and time.monotonic() < deadline:
        session.tick()
        time.sleep(.005)
    assert not session.extensions.busy


def make(tmp_path, **options):
    cfg = config(tmp_path, task="Put the cube in the box", task_management=True, episode_counts=True,
                 actions=(TeachAction("reset", "Reset scene"),))
    session = TeachSession(cfg, "humanoid", **options)
    settle(session)
    assert session.request(request(session, "open"))["accepted"]
    return session


def test_old_profiles_remain_byte_compatible(tmp_path):
    cfg = config(tmp_path)
    value = cfg.to_dict()
    assert not any(k in value for k in ("actions", "task_management", "episode_counts"))
    assert "extensions_version" not in cfg.registration("robot")
    assert TeachConfig.from_dict(value).digest == cfg.digest


def test_extension_roundtrip_and_validation(tmp_path):
    cfg = config(tmp_path, actions=(TeachAction("reset", "Reset"),), task_management=True)
    assert TeachConfig.from_dict(cfg.to_dict()).digest == cfg.digest
    assert cfg.registration("robot")["actions"][0]["action_id"] == "reset"
    with pytest.raises(ValueError): dataclasses.replace(cfg, actions=(cfg.actions[0], cfg.actions[0]))
    with pytest.raises(ValueError): dataclasses.replace(cfg, task_management=1)


def test_task_change_frozen_in_saved_manifest(tmp_path):
    s = make(tmp_path)
    try:
        digest = s.config.digest
        assert s.request(request(s, "select_task", task="  Open the drawer  "))["accepted"]
        settle(s)
        assert s.current_task == "Open the drawer" and s.config.digest == digest
        assert not s.request(request(s, "record", episode_id=str(uuid.uuid4()), task="old task"))["accepted"]
        episode = str(uuid.uuid4())
        assert s.request(request(s, "record", episode_id=episode, task=s.current_task))["accepted"]
        poll(s, "recording")
        assert not s.request(request(s, "select_task", task="another"))["accepted"]
        assert not s.request(request(s, "custom_action", action_id="reset"))["accepted"]
        assert s.request(request(s, "finish", outcome="success"))["accepted"]
        poll(s, "saved"); settle(s)
        manifest = json.loads((tmp_path / episode / "manifest.json").read_text())
        assert manifest["task"] == "Open the drawer"
        entry = next(t for t in s.state()["tasks"] if t["task_id"] == task_identity(s.current_task))
        assert entry["saved"] == entry["successful"] == 1
    finally: s.close()


def test_actions_are_idempotent_gated_and_outside_lock(tmp_path):
    calls, held = [], []
    def reset(context):
        calls.append(context); held.append(s.lock._is_owned())
        return "ok"
    s = make(tmp_path, action_handlers={"reset": reset})
    try:
        assert not s.request(request(s, "custom_action", action_id="reset", owner="other"))["accepted"]
        assert not s.request(request(s, "custom_action", action_id="reset", expected_revision=-1))["accepted"]
        req = request(s, "custom_action", action_id="reset")
        assert s.request(req)["accepted"]
        assert s.request(req)["accepted"]
        settle(s)
        assert len(calls) == 1 and held == [False]
        assert calls[0].idempotency_key == "action:" + req["request_id"]
        assert s.state()["last_action"]["status"] == "completed"
        s.motion_authority = {"active": True}
        assert not s.request(request(s, "custom_action", action_id="reset"))["accepted"]
        s.motion_authority = None
    finally: s.close()


def test_durable_save_hook_blocks_new_episode_and_failure_preserves_data(tmp_path):
    entered, release = threading.Event(), threading.Event()
    contexts = []
    def saved(context):
        assert (context.path / "manifest.json").is_file()
        assert not context.path.name.endswith(".inprogress")
        assert not s.lock._is_owned()
        contexts.append(context)
        entered.set(); assert release.wait(3)
        raise RuntimeError("reset failed")
    s = make(tmp_path, after_save=saved)
    try:
        episode = str(uuid.uuid4())
        assert s.request(request(s, "record", episode_id=episode, task=s.current_task))["accepted"]
        poll(s, "recording")
        assert s.request(request(s, "finish"))["accepted"]
        poll(s, "saved")
        assert entered.wait(2)
        assert s.state()["extension_busy"]
        assert not s.request(request(s, "record", episode_id=str(uuid.uuid4()), task=s.current_task))["accepted"]
        assert not s.request(request(s, "engage"))["accepted"]
        release.set(); settle(s)
        assert s.capture == "saved" and s.state()["save_hook"]["status"] == "failed"
        manifest = json.loads((tmp_path / episode / "manifest.json").read_text())
        assert manifest["capture"] == "saved" and not s.error
        assert contexts[0].idempotency_key == "save:" + episode
    finally: release.set(); s.close()
    replayed = []
    restarted = make(tmp_path, after_save=replayed.append)
    try:
        assert not replayed
        assert next(t for t in restarted.state()["tasks"] if t["task_id"] == task_identity(s.current_task))["saved"] == 1
    finally: restarted.close()


def test_catalog_counts_quality_outcome_and_interruption_separately(tmp_path):
    catalog = EpisodeCatalog(tmp_path)
    for capture, quality, outcome in (("saved", "acceptable", "success"), ("saved", "unacceptable", "unknown"), ("interrupted", "unacceptable", "abort")):
        identity = str(uuid.uuid4()); folder = tmp_path / identity; folder.mkdir()
        (folder / "manifest.json").write_text(json.dumps(dict(episode_id=identity, robot_name="r", task="task", capture=capture, quality=quality, outcome=outcome)))
    catalog.refresh(); catalog.refresh()
    assert catalog.snapshot("r") == [dict(task_id=task_identity("task"), prompt="task", saved=2, acceptable=1, successful=1, interrupted=1)]
    assert catalog.snapshot("other")[0]["saved"] == 0


def test_callback_ledger_never_replays_unacknowledged_work(tmp_path):
    catalog = EpisodeCatalog(tmp_path)
    assert catalog.begin_callback("save:one")
    catalog.refresh()
    assert not catalog.begin_callback("save:one")
    with catalog._connect() as db:
        assert db.execute("SELECT status FROM callbacks WHERE id='save:one'").fetchone()[0] == "running"


def test_catalog_never_counts_unpublished_saved_manifest(tmp_path):
    identity = str(uuid.uuid4()); folder = tmp_path / (identity + ".inprogress"); folder.mkdir()
    (folder / "manifest.json").write_text(json.dumps(dict(episode_id=identity, robot_name="r", task="task", capture="saved", quality="acceptable", outcome="success")))
    catalog = EpisodeCatalog(tmp_path); catalog.refresh()
    assert catalog.counts("r", "task")["saved"] == 0
    assert catalog.issues == [folder.name]


def test_count_initialization_is_unknown_until_catalogue_is_loaded(tmp_path):
    entered, release = threading.Event(), threading.Event()
    class DelayedCatalog(EpisodeCatalog):
        def refresh(self):
            entered.set()
            assert release.wait(3)
            super().refresh()
    cfg = config(tmp_path, task="task", episode_counts=True)
    session = TeachSession(cfg, "r", catalog=DelayedCatalog(tmp_path))
    try:
        assert entered.wait(2)
        assert session.state()["current_task_counts"] is None
        release.set(); settle(session)
        assert session.state()["current_task_counts"]["saved"] == 0
    finally:
        release.set(); session.close()


def test_read_only_counter_refresh_does_not_block_alignment_or_lease_gated_control(tmp_path):
    entered, release = threading.Event(), threading.Event()
    class DelayedCatalog(EpisodeCatalog):
        def refresh(self):
            entered.set()
            assert release.wait(3)
            super().refresh()
    cfg = config(tmp_path, task="task", episode_counts=True, adapter_id="mock", provenance="fixture",
        workflow="practice", joint_names=("joint",), joint_groups={"arm": ("joint",)},
        command_frame="base", calibration_id="cal1")
    session = TeachSession(cfg, "r", MockAdapter(), catalog=DelayedCatalog(tmp_path))
    try:
        assert entered.wait(2)
        assert session.request(request(session, "open"))["accepted"]
        assert session.state()["extension_busy"] and session.state()["extension_operation"] == "catalog"
        assert not arm(session)["accepted"]
        align(session)
        session.update_lease("operator", "authority1", 3)
        assert session.state()["control_ready"]
        assert not session.request(request(session, "align", owner="another operator", alignment=session.alignment))["accepted"]
        assert arm(session)["accepted"]
        assert session.input(input_sample(session)); session.tick()
        alignment = session.alignment
        assert not session.request(request(session, "record", episode_id=str(uuid.uuid4())))["accepted"]
        release.set(); settle(session)
        assert session.alignment == alignment and session.control == "engaged"
        assert session.state()["current_task_counts"]["saved"] == 0
        assert session.state()["extension_operation"] == ""
    finally:
        release.set(); session.close()


def test_preset_recording_does_not_expose_previous_task_count_while_loading(tmp_path):
    entered, release = threading.Event(), threading.Event()
    release.set()
    class DelayedCatalog(EpisodeCatalog):
        def counts(self, robot, prompt):
            if prompt == "New task":
                entered.set()
                assert release.wait(3)
            return super().counts(robot, prompt)
    catalog = DelayedCatalog(tmp_path)
    old = str(uuid.uuid4())
    folder = tmp_path / old; folder.mkdir()
    manifest = dict(episode_id=old, robot_name="r", task="Old task", capture="saved", quality="acceptable", outcome="unknown")
    (folder / "manifest.json").write_text(json.dumps(manifest))
    session = TeachSession(config(tmp_path, task="Old task", episode_counts=True), "r", catalog=catalog)
    try:
        settle(session)
        assert session.state()["current_task_counts"]["saved"] == 1
        assert session.request(request(session, "open"))["accepted"]
        release.clear()
        assert session.request(request(session, "record", task="New task", episode_id=str(uuid.uuid4())))["accepted"]
        assert entered.wait(2)
        snapshot = session.state()
        assert snapshot["current_task"] == "New task"
        assert snapshot["current_task_counts"] is None
        release.set(); settle(session)
        assert session.state()["current_task_counts"]["saved"] == 0
    finally:
        release.set(); session.close()


@pytest.mark.parametrize("outcome,successful", [("unknown", 0), ("failure", 0), ("success", 1)])
def test_counter_advances_once_only_after_finalize_without_promoting_outcome(tmp_path, outcome, successful):
    session = make(tmp_path)
    try:
        assert session.state()["current_task_counts"]["saved"] == 0
        episode = str(uuid.uuid4())
        assert session.request(request(session, "record", episode_id=episode, task=session.current_task))["accepted"]
        poll(session, "recording")
        assert session.state()["current_task_counts"]["saved"] == 0
        finish = request(session, "finish", outcome=outcome)
        assert session.request(finish)["accepted"]
        assert session.request(finish)["accepted"]
        poll(session, "saved"); settle(session)
        counts = session.state()["current_task_counts"]
        assert counts["saved"] == 1 and counts["successful"] == successful
        assert session.state()["outcome"] == outcome
        assert (tmp_path / episode / "manifest.json").is_file()
        assert session.request(request(session, "select_task", task="Another task"))["accepted"]
        settle(session)
        assert session.state()["current_task_counts"]["saved"] == 0
    finally:
        session.close()


def test_missing_action_handler_is_visible(tmp_path):
    s = make(tmp_path)
    try:
        assert s.state()["actions"][0] == dict(action_id="reset", enabled=False, reason="No handler installed")
        assert not s.request(request(s, "custom_action", action_id="reset"))["accepted"]
    finally: s.close()


def test_paged_tasks_keep_full_prompt_and_current_counts(tmp_path):
    catalog = EpisodeCatalog(tmp_path)
    for i in range(40): catalog.ensure_task(f"Task {i:02d}: " + "x" * 300)
    s = make(tmp_path, catalog=catalog)
    try:
        assert s.state()["task_total"] == 41 and len(s.state()["tasks"]) == 32
        assert s.request(request(s, "list_tasks", task_page=1))["accepted"]
        settle(s)
        state = s.state()
        assert state["task_page"] == 1 and len(state["tasks"]) == 9
        selected = state["tasks"][-1]
        assert len(selected["prompt"]) == 256
        assert s.request(request(s, "select_task", task_id=selected["task_id"]))["accepted"]
        settle(s)
        assert len(s.current_task) > 300
        assert s.state()["current_task_id"] == selected["task_id"]
        assert len(json.dumps(s.state()).encode()) < 65536
    finally: s.close()


def test_action_failure_allows_manual_retry_with_new_identity(tmp_path):
    calls = []
    def fail(context):
        calls.append(context.idempotency_key)
        return {"accepted": False, "reason": "Fixture unavailable"}
    s = make(tmp_path, action_handlers={"reset": fail})
    try:
        for _ in range(2):
            assert s.request(request(s, "custom_action", action_id="reset"))["accepted"]
            settle(s)
            assert s.state()["last_action"]["status"] == "failed"
        assert len(set(calls)) == 2 and not s.error and s.capture == "idle"
    finally: s.close()


def test_save_hook_waits_for_async_hold_confirmation(tmp_path):
    adapter = MockAdapter()
    allow_hold = False
    adapter.hold = lambda context: allow_hold
    calls = []
    cfg = config(tmp_path, task="task", task_management=True, episode_counts=True,
        workflow="practice", provenance="fixture", adapter_id="mock", joint_names=("joint",),
        joint_groups={"arm": ("joint",)}, command_frame="base", calibration_id="cal1")
    s = TeachSession(cfg, "humanoid", adapter, after_save=calls.append)
    try:
        settle(s); assert s.request(request(s, "open"))["accepted"]
        assert s.request(request(s, "record", episode_id=str(uuid.uuid4()), task="task"))["accepted"]
        poll(s, "recording")
        s.motion_authority = {"robot_name": "humanoid"}; s.control = "engaged"
        assert s.request(request(s, "finish"))["accepted"]
        poll(s, "saved")
        assert not calls and s.state()["extension_busy"]
        assert s.state()["save_hook"]["status"] == "waiting_for_hold"
        assert not s.request(request(s, "record", episode_id=str(uuid.uuid4()), task="task"))["accepted"]
        allow_hold = True; s.next_hold_check = 0
        s.tick(); settle(s)
        assert len(calls) == 1 and s.control == "holding"
    finally: allow_hold = True; s.close()


def test_save_hook_scheduling_failure_never_changes_native_result(tmp_path):
    s = make(tmp_path)
    try:
        def fail(*args): raise RuntimeError("worker unavailable")
        s.extensions.saved = fail
        episode = str(uuid.uuid4())
        assert s.request(request(s, "record", episode_id=episode, task=s.current_task))["accepted"]
        poll(s, "recording")
        assert s.request(request(s, "finish"))["accepted"]
        poll(s, "saved")
        assert s.capture == "saved" and s.extensions.save_hook["status"] == "failed"
        assert not s.error
        assert json.loads((tmp_path / episode / "manifest.json").read_text())["capture"] == "saved"
    finally: s.close()
