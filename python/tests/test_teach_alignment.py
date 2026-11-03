import copy
import math
import uuid

import pytest

from horus.teaching.alignment import map_input, validate_alignment
from test_teaching import motion_session, request, arm, align, input_sample, poll


def test_session_rejects_engagement_without_alignment(motion_session):
    s = motion_session
    assert s.request(request(s, "invalidate_alignment", expected_revision=-100))["accepted"]
    assert not s.state()["control_ready"]
    assert not arm(s)["accepted"]
    assert not s.adapter.anchors and not s.adapter.commands and s.adapter.holds == 0
    align(s)
    assert arm(s)["accepted"]


def test_hyphenated_uppercase_alignment_ids_match_native_contract(motion_session):
    value = dict(motion_session.alignment)
    value["alignment_id"] = value["alignment_id"].upper()
    value["tracking_origin_id"] = value["tracking_origin_id"].upper()
    assert validate_alignment(value) == value


@pytest.mark.parametrize("patch", [dict(position=[math.nan, 0, 0]), dict(position=[101, 0, 0]),
    dict(rotation=[0, 0, 0, 2]), dict(rotation=[.70710678, 0, 0, .70710678]),
    dict(rotation=[0, 0, math.inf, 1]), dict(schema_version=True), dict(alignment_id="bad"),
    dict(alignment_id="00112233445566778899aabbccddeeff"),
    dict(frame_id="map"), dict(child_frame_id="robot_base"), dict(position=[True, 0, 0])])
def test_invalid_alignment_is_rejected_atomically(motion_session, patch):
    s = motion_session
    before = copy.deepcopy(s.alignment)
    value = dict(before, **patch)
    assert not s.request(request(s, "align", alignment=value))["accepted"]
    assert s.alignment == before


def test_yaw_translation_applied_once_before_anchor_and_command(motion_session):
    s = motion_session
    align(s, position=[2, 3, 0], rotation=[0, 0, math.sqrt(.5), math.sqrt(.5)])
    assert arm(s)["accepted"]
    raw = input_sample(s, left=dict(position=[2, 4, 1], rotation=[0, 0, math.sqrt(.5), math.sqrt(.5)]))
    original = copy.deepcopy(raw)
    assert s.input(raw)
    s.tick()
    pose = s.adapter.anchors[-1]["left"]
    assert pose["position"] == pytest.approx([1, 0, 1])
    assert pose["rotation"] == pytest.approx([0, 0, 0, 1])
    assert raw == original
    assert s.adapter.anchors[-1]["frame_id"] == "teach_calibrated_floor_flu"
    assert s.input(dict(raw, sequence=2))
    s.tick()
    assert s.adapter.commands[-1]["left"] == pose


@pytest.mark.parametrize("key", ["alignment_id", "tracking_origin_id", "frame_id"])
def test_stale_alignment_input_stops_and_cannot_resume(motion_session, key):
    s = motion_session
    assert arm(s)["accepted"]
    assert s.input(input_sample(s))
    s.tick()
    assert not s.input(input_sample(s, 2, **{key: str(uuid.uuid4())}))
    assert s.control == "holding" and s.alignment is None
    assert not arm(s)["accepted"] and not s.adapter.commands


def test_alignment_cannot_change_during_control_and_invalidation_discards_pending(motion_session):
    s = motion_session
    assert arm(s)["accepted"]
    assert s.input(input_sample(s))
    assert not s.request(request(s, "align", alignment=s.alignment))["accepted"]
    assert s.request(request(s, "invalidate_alignment", expected_revision=-1))["accepted"]
    s.tick()
    assert not s.adapter.anchors and not s.adapter.commands and s.alignment is None


def test_alignment_is_recorded_when_capture_starts_after_alignment(motion_session):
    from horus.teaching.dataset import Episode
    s = motion_session
    assert s.request(request(s, "record", episode_id=str(uuid.uuid4())))["accepted"]
    poll(s, "recording")
    assert s.request(request(s, "finish"))["accepted"]
    poll(s, "saved")
    from pathlib import Path
    events = list(Episode(Path(s.config.recording_root) / s.episode_id).events())
    saved = [e["data"] for e in events if e["kind"] == "alignment"]
    assert len(saved) == 1 and saved[0]["alignment_id"] == s.alignment["alignment_id"]


def test_close_reopen_requires_new_alignment(motion_session):
    s = motion_session
    previous = s.alignment["alignment_id"]
    assert s.request(request(s, "close"))["accepted"]
    assert s.request(request(s, "open"))["accepted"]
    assert not s.request(request(s, "engage", authority_generation="authority1", alignment_id=previous))["accepted"]


def test_client_build_and_exact_invalidation_reason_survive_record_start(motion_session):
    from pathlib import Path
    from horus.teaching.dataset import Episode
    s = motion_session
    diagnostic = dict(build_guid="tested-build", app_version="0.2.0-rc.1", tracking_origin="Stage",
                      tracking_origin_id=s.alignment["tracking_origin_id"])
    assert s.request(request(s, "align", alignment=s.alignment, client_diagnostics=diagnostic))["accepted"]
    assert s.request(request(s, "record", episode_id=str(uuid.uuid4())))["accepted"]
    poll(s, "recording")
    stopped = dict(diagnostic, invalidation_reason="Headset recentered; align again")
    assert s.request(request(s, "invalidate_alignment", expected_revision=-10, client_diagnostics=stopped))["accepted"]
    assert s.alignment is None
    assert s.request(request(s, "finish"))["accepted"]
    poll(s, "saved")
    events = list(Episode(Path(s.config.recording_root) / s.episode_id).events())
    for kind in ("recording_started", "alignment"):
        event = next(e["data"] for e in events if e["kind"] == kind)
        assert event["client_diagnostics"] == diagnostic
    event = next(e["data"] for e in events if e["kind"] == "alignment_invalidated")
    assert event["reason"] == "tracking alignment invalidated"
    assert event["client_diagnostics"] == stopped


@pytest.mark.parametrize("diagnostic", [None, [], "not-an-object", {"build_guid": [], "extra": "ignore"},
    {"invalidation_reason": "long-reason-" * 100, "build_guid": "\u00e9" * 200}])
def test_optional_diagnostics_cannot_prevent_a_stop(motion_session, diagnostic):
    s = motion_session
    assert arm(s)["accepted"]
    assert s.input(input_sample(s))
    s.tick()
    assert s.request(request(s, "invalidate_alignment", expected_revision=-1,
                             client_diagnostics=diagnostic))["accepted"]
    assert s.alignment is None and s.control == "holding" and s.adapter.holds == 1


def test_diagnostic_context_is_bounded_and_session_scoped(motion_session):
    s = motion_session
    value = dict(build_guid="\u00e9" * 200, app_version=False, tracking_origin="Stage", unknown="ignored")
    assert s.request(request(s, "align", alignment=s.alignment, client_diagnostics=value))["accepted"]
    assert len(s.client_diagnostics["build_guid"].encode("utf-8")) <= 256
    assert set(s.client_diagnostics) == {"build_guid", "tracking_origin"}
    before = dict(s.client_diagnostics)
    assert not s.request(request(s, "invalidate_alignment", owner="another", client_diagnostics={"build_guid":"spoofed"}))["accepted"]
    assert s.client_diagnostics == before and s.alignment is not None
    assert s.request(request(s, "close"))["accepted"]
    assert s.request(request(s, "open"))["accepted"]
    assert not s.client_diagnostics
