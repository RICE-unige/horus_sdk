"""Tests for the field-teammate HRI study analysis scaffolding."""

from __future__ import annotations

from pathlib import Path

from horus.experiments import (
    FieldTeammateStudyRecorder,
    GuidanceOutcome,
    StudyCondition,
    StudyEventType,
    aggregate_sessions,
    build_study_report,
    load_sessions,
    session_metrics,
    summarize_values,
)
from horus.experiments.field_teammate_study import SessionRecord


def _write_session(
    path: Path,
    *,
    dyad_id: str,
    condition: StudyCondition,
    base_ns: int,
    mission_s: float,
    clarifications: int,
    nav_errors: int,
    guidance_latency_s: float,
) -> None:
    with FieldTeammateStudyRecorder(
        path, dyad_id=dyad_id, condition=condition, scenario="alpha"
    ) as rec:
        rec.session_start(timestamp_ns=base_ns)
        for i in range(clarifications):
            rid = f"{dyad_id}-clar-{i}"
            rec.guidance_request(rid, timestamp_ns=base_ns + 1_000_000)
            rec.guidance_response(
                rid,
                GuidanceOutcome.CLARIFY,
                timestamp_ns=base_ns + 1_000_000 + int(guidance_latency_s * 1e9),
            )
        # One acknowledged request to exercise latency pairing distinctly.
        rec.guidance_request("ack-1", timestamp_ns=base_ns + 2_000_000)
        rec.guidance_response(
            "ack-1",
            GuidanceOutcome.ACKNOWLEDGE,
            timestamp_ns=base_ns + 2_000_000 + int(guidance_latency_s * 1e9),
        )
        for _ in range(nav_errors):
            rec.navigation_error(timestamp_ns=base_ns + 3_000_000)
        rec.communication(duration_s=2.0, timestamp_ns=base_ns + 4_000_000)
        rec.localization_spotcheck(error_m=0.12, heading_error_deg=3.0, timestamp_ns=base_ns + 5_000_000)
        rec.record(StudyEventType.TASK_SUCCESS, timestamp_ns=base_ns + 6_000_000)
        rec.session_end(timestamp_ns=base_ns + int(mission_s * 1e9))


def test_session_metrics_from_recorded_log(tmp_path: Path):
    path = tmp_path / "dyad1.ndjson"
    _write_session(
        path,
        dyad_id="dyad1",
        condition=StudyCondition.IN_THE_TEAM,
        base_ns=1_000_000_000,
        mission_s=120.0,
        clarifications=2,
        nav_errors=1,
        guidance_latency_s=0.5,
    )
    session = SessionRecord.from_ndjson(path)
    metrics = session_metrics(session)

    assert metrics["mission_completion_time_s"] == 120.0
    assert metrics["clarification_requests"] == 2.0
    assert metrics["navigation_errors"] == 1.0
    assert metrics["guidance_requests"] == 3.0
    assert metrics["communication_count"] == 1.0
    assert metrics["task_success_rate"] == 1.0
    assert abs(metrics["mean_guidance_latency_s"] - 0.5) < 1e-6
    assert abs(metrics["mean_localization_error_m"] - 0.12) < 1e-9


def test_aggregate_uses_dyad_as_unit(tmp_path: Path):
    sessions = []
    for i, mission in enumerate((100.0, 110.0, 120.0)):
        p = tmp_path / f"map_{i}.ndjson"
        _write_session(
            p,
            dyad_id=f"dyad{i}",
            condition=StudyCondition.ON_THE_MAP,
            base_ns=1_000_000_000,
            mission_s=mission,
            clarifications=3,
            nav_errors=2,
            guidance_latency_s=0.8,
        )
        sessions.append(p)
    for i, mission in enumerate((70.0, 80.0)):
        p = tmp_path / f"team_{i}.ndjson"
        _write_session(
            p,
            dyad_id=f"team{i}",
            condition=StudyCondition.IN_THE_TEAM,
            base_ns=1_000_000_000,
            mission_s=mission,
            clarifications=1,
            nav_errors=0,
            guidance_latency_s=0.3,
        )
        sessions.append(p)

    aggregates = aggregate_sessions(load_sessions(sessions))

    on_map = aggregates[StudyCondition.ON_THE_MAP.value]["mission_completion_time_s"]
    assert on_map.n == 3
    assert abs(on_map.mean - 110.0) < 1e-6
    assert on_map.ci95_low < on_map.mean < on_map.ci95_high

    in_team = aggregates[StudyCondition.IN_THE_TEAM.value]["mission_completion_time_s"]
    assert in_team.n == 2
    assert abs(in_team.mean - 75.0) < 1e-6
    # Fewer clarifications in the richer condition, as recorded.
    assert aggregates[StudyCondition.IN_THE_TEAM.value]["clarification_requests"].mean == 1.0
    assert aggregates[StudyCondition.ON_THE_MAP.value]["clarification_requests"].mean == 3.0


def test_build_report_orders_conditions_additively(tmp_path: Path):
    for i, cond in enumerate(
        (StudyCondition.IN_THE_TEAM, StudyCondition.VOICE_VIDEO, StudyCondition.ON_THE_MAP)
    ):
        p = tmp_path / f"s{i}.ndjson"
        _write_session(
            p,
            dyad_id=f"d{i}",
            condition=cond,
            base_ns=1_000_000_000,
            mission_s=90.0,
            clarifications=1,
            nav_errors=0,
            guidance_latency_s=0.4,
        )
    report = build_study_report(load_sessions(sorted(tmp_path.glob("*.ndjson"))))
    assert list(report["conditions"].keys()) == [
        StudyCondition.VOICE_VIDEO.value,
        StudyCondition.ON_THE_MAP.value,
        StudyCondition.IN_THE_TEAM.value,
    ]
    assert report["unit_of_analysis"] == "dyad"


def test_summarize_values_confidence_interval():
    summary = summarize_values("x", "c", [10.0, 12.0, 14.0, 16.0, 18.0])
    assert summary.n == 5
    assert abs(summary.mean - 14.0) < 1e-9
    assert summary.ci95_low < 14.0 < summary.ci95_high
    # Single value collapses to a degenerate interval.
    single = summarize_values("x", "c", [5.0])
    assert single.ci95_low == single.ci95_high == 5.0
