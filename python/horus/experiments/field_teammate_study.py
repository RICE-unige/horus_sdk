"""Analysis scaffolding for the field-teammate HRI study.

"On the Map, In the Team" compares three additive conditions — voice + video
(C1), spatial representation (C2), and bidirectional MR guidance (C3) — and asks
whether putting a field teammate *on the map* and *in the team* improves shared
situation awareness, communication efficiency, workload, safety, and mission
performance.

This module is SDK-side and hardware-independent. It defines the study event
schema, a recorder built on the experiment NDJSON writer, and an analyzer that
reduces per-dyad session logs into per-condition summaries. The **dyad is the
unit of analysis**, and aggregates carry t-based 95% confidence intervals so a
small study reports honestly.

Nothing here depends on a HoloLens, the bridge, or the MR app: sessions are
plain newline-delimited JSON, so they can be produced by a live run, the mock
teammate node, or a replay.
"""

from __future__ import annotations

import json
import math
import statistics
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from .metrics import NdjsonEventWriter, now_ns


class StudyCondition(str, Enum):
    """The three additive study conditions."""

    VOICE_VIDEO = "c1_voice_video"
    ON_THE_MAP = "c2_on_the_map"
    IN_THE_TEAM = "c3_in_the_team"

    @classmethod
    def coerce(cls, value: Any) -> "StudyCondition":
        if isinstance(value, cls):
            return value
        text = str(value or "").strip().lower()
        for member in cls:
            if text in (member.value, member.name.lower()):
                return member
        raise ValueError(f"unknown study condition: {value!r}")


class StudyEventType(str, Enum):
    """Event kinds recorded during a session."""

    SESSION_START = "session_start"
    SESSION_END = "session_end"
    GUIDANCE_REQUEST = "guidance_request"
    GUIDANCE_RESPONSE = "guidance_response"
    CLARIFICATION = "clarification"
    COMMUNICATION = "communication"
    UNCERTAINTY_RAISED = "uncertainty_raised"
    UNCERTAINTY_RESOLVED = "uncertainty_resolved"
    NAVIGATION_ERROR = "navigation_error"
    WAYPOINT_REACHED = "waypoint_reached"
    TASK_SUCCESS = "task_success"
    TASK_FAILURE = "task_failure"
    SAFETY_EVENT = "safety_event"
    LOCALIZATION_SPOTCHECK = "localization_spotcheck"


class GuidanceOutcome(str, Enum):
    """How a teammate responded to a guidance request."""

    ACKNOWLEDGE = "acknowledge"
    CLARIFY = "clarify"
    REJECT = "reject"
    COMPLETE = "complete"


# ---------------------------------------------------------------------------
# Recording
# ---------------------------------------------------------------------------


class FieldTeammateStudyRecorder:
    """Append study events for one dyad/session to a newline-delimited JSON log.

    Built on the shared :class:`NdjsonEventWriter` so study logs sit alongside
    the rest of the HORUS experiment tooling. Timestamps default to the shared
    monotonic-anchored clock.
    """

    def __init__(
        self,
        path: Path,
        *,
        dyad_id: str,
        condition: Any,
        scenario: str,
        run_id: str = "",
    ) -> None:
        self.dyad_id = str(dyad_id)
        self.condition = StudyCondition.coerce(condition)
        self.scenario = str(scenario)
        self._writer = NdjsonEventWriter(
            Path(path),
            run_id=run_id or self.dyad_id,
            experiment="field_teammate_study",
            condition=self.condition.value,
            source="study",
        )

    def record(
        self,
        event_type: Any,
        *,
        timestamp_ns: Optional[int] = None,
        **payload: Any,
    ) -> None:
        kind = event_type.value if isinstance(event_type, StudyEventType) else str(event_type)
        event: Dict[str, Any] = {
            "event_type": kind,
            "dyad_id": self.dyad_id,
            "scenario": self.scenario,
        }
        event.update(payload)
        self._writer.write(event, timestamp_ns=timestamp_ns)

    # Convenience helpers for the common events. Each is a thin wrapper so a
    # live operator console, the mock node, or a replay can all emit the same
    # schema.
    def session_start(self, **payload: Any) -> None:
        self.record(StudyEventType.SESSION_START, **payload)

    def session_end(self, **payload: Any) -> None:
        self.record(StudyEventType.SESSION_END, **payload)

    def guidance_request(self, request_id: str, **payload: Any) -> None:
        self.record(StudyEventType.GUIDANCE_REQUEST, request_id=request_id, **payload)

    def guidance_response(self, request_id: str, outcome: Any, **payload: Any) -> None:
        resolved = outcome.value if isinstance(outcome, GuidanceOutcome) else str(outcome)
        self.record(
            StudyEventType.GUIDANCE_RESPONSE,
            request_id=request_id,
            outcome=resolved,
            **payload,
        )

    def communication(self, duration_s: float, **payload: Any) -> None:
        self.record(StudyEventType.COMMUNICATION, duration_s=float(duration_s), **payload)

    def navigation_error(self, **payload: Any) -> None:
        self.record(StudyEventType.NAVIGATION_ERROR, **payload)

    def localization_spotcheck(self, error_m: float, heading_error_deg: float = 0.0, **payload: Any) -> None:
        self.record(
            StudyEventType.LOCALIZATION_SPOTCHECK,
            error_m=float(error_m),
            heading_error_deg=float(heading_error_deg),
            **payload,
        )

    def close(self) -> None:
        self._writer.close()

    def __enter__(self) -> "FieldTeammateStudyRecorder":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


# ---------------------------------------------------------------------------
# Session model + per-session metrics
# ---------------------------------------------------------------------------


@dataclass
class SessionRecord:
    dyad_id: str
    condition: StudyCondition
    scenario: str
    events: List[Dict[str, Any]] = field(default_factory=list)

    @classmethod
    def from_ndjson(cls, path: Path) -> "SessionRecord":
        events: List[Dict[str, Any]] = []
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line:
                events.append(json.loads(line))
        events.sort(key=lambda e: int(e.get("timestamp_ns", 0)))
        if not events:
            raise ValueError(f"no events in session log: {path}")
        first = events[0]
        return cls(
            dyad_id=str(first.get("dyad_id", "")),
            condition=StudyCondition.coerce(first.get("condition")),
            scenario=str(first.get("scenario", "")),
            events=events,
        )

    def of_type(self, event_type: StudyEventType) -> List[Dict[str, Any]]:
        return [e for e in self.events if e.get("event_type") == event_type.value]


def _ns_to_s(value: float) -> float:
    return float(value) / 1e9


def session_metrics(session: SessionRecord) -> Dict[str, float]:
    """Reduce a single dyad session to its dependent variables."""
    events = session.events
    starts = session.of_type(StudyEventType.SESSION_START)
    ends = session.of_type(StudyEventType.SESSION_END)

    metrics: Dict[str, float] = {}

    if starts and ends:
        metrics["mission_completion_time_s"] = _ns_to_s(
            int(ends[-1]["timestamp_ns"]) - int(starts[0]["timestamp_ns"])
        )

    successes = len(session.of_type(StudyEventType.TASK_SUCCESS))
    failures = len(session.of_type(StudyEventType.TASK_FAILURE))
    total_tasks = successes + failures
    if total_tasks:
        metrics["task_success_rate"] = successes / total_tasks
    metrics["navigation_errors"] = float(len(session.of_type(StudyEventType.NAVIGATION_ERROR)))
    metrics["safety_events"] = float(len(session.of_type(StudyEventType.SAFETY_EVENT)))

    responses = session.of_type(StudyEventType.GUIDANCE_RESPONSE)
    clarifications = len(session.of_type(StudyEventType.CLARIFICATION)) + sum(
        1 for r in responses if r.get("outcome") == GuidanceOutcome.CLARIFY.value
    )
    metrics["clarification_requests"] = float(clarifications)
    metrics["guidance_requests"] = float(len(session.of_type(StudyEventType.GUIDANCE_REQUEST)))
    metrics["guidance_rejections"] = float(
        sum(1 for r in responses if r.get("outcome") == GuidanceOutcome.REJECT.value)
    )

    comms = session.of_type(StudyEventType.COMMUNICATION)
    metrics["communication_count"] = float(len(comms))
    metrics["communication_total_duration_s"] = float(
        sum(float(c.get("duration_s", 0.0)) for c in comms)
    )

    # Guidance delivery latency: response time minus matching request time,
    # paired on request_id.
    request_times: Dict[str, int] = {}
    for req in session.of_type(StudyEventType.GUIDANCE_REQUEST):
        rid = str(req.get("request_id", ""))
        if rid:
            request_times[rid] = int(req["timestamp_ns"])
    latencies: List[float] = []
    for resp in responses:
        rid = str(resp.get("request_id", ""))
        if rid in request_times:
            latencies.append(_ns_to_s(int(resp["timestamp_ns"]) - request_times[rid]))
    if latencies:
        metrics["mean_guidance_latency_s"] = statistics.fmean(latencies)

    # Time to resolve uncertainty: paired raised -> resolved on uncertainty_id.
    raised: Dict[str, int] = {}
    for ev in session.of_type(StudyEventType.UNCERTAINTY_RAISED):
        uid = str(ev.get("uncertainty_id", ""))
        if uid:
            raised[uid] = int(ev["timestamp_ns"])
    resolution_times: List[float] = []
    for ev in session.of_type(StudyEventType.UNCERTAINTY_RESOLVED):
        uid = str(ev.get("uncertainty_id", ""))
        if uid in raised:
            resolution_times.append(_ns_to_s(int(ev["timestamp_ns"]) - raised[uid]))
    if resolution_times:
        metrics["mean_uncertainty_resolution_s"] = statistics.fmean(resolution_times)

    spotchecks = session.of_type(StudyEventType.LOCALIZATION_SPOTCHECK)
    if spotchecks:
        metrics["mean_localization_error_m"] = statistics.fmean(
            float(s.get("error_m", 0.0)) for s in spotchecks
        )
        metrics["mean_heading_error_deg"] = statistics.fmean(
            float(s.get("heading_error_deg", 0.0)) for s in spotchecks
        )

    return metrics


# ---------------------------------------------------------------------------
# Aggregation (dyad as the unit) + statistics
# ---------------------------------------------------------------------------

# Two-sided 95% t critical values by degrees of freedom; falls back to the
# normal approximation for large samples.
_T95 = {
    1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365,
    8: 2.306, 9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145,
    15: 2.131, 16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093, 20: 2.086,
    21: 2.080, 22: 2.074, 23: 2.069, 24: 2.064, 25: 2.060, 26: 2.056,
    27: 2.052, 28: 2.048, 29: 2.045, 30: 2.042,
}


def _t_critical_95(df: int) -> float:
    if df <= 0:
        return float("nan")
    if df in _T95:
        return _T95[df]
    return 1.96


@dataclass
class MetricSummary:
    metric: str
    condition: str
    n: int
    mean: float
    sd: float
    sem: float
    ci95_low: float
    ci95_high: float

    def to_payload(self) -> Dict[str, Any]:
        return {
            "metric": self.metric,
            "condition": self.condition,
            "n": self.n,
            "mean": self.mean,
            "sd": self.sd,
            "sem": self.sem,
            "ci95_low": self.ci95_low,
            "ci95_high": self.ci95_high,
        }


def summarize_values(metric: str, condition: str, values: Sequence[float]) -> MetricSummary:
    clean = [float(v) for v in values if v is not None and not math.isnan(float(v))]
    n = len(clean)
    if n == 0:
        nan = float("nan")
        return MetricSummary(metric, condition, 0, nan, nan, nan, nan, nan)
    mean = statistics.fmean(clean)
    if n == 1:
        return MetricSummary(metric, condition, 1, mean, 0.0, 0.0, mean, mean)
    sd = statistics.stdev(clean)
    sem = sd / math.sqrt(n)
    margin = _t_critical_95(n - 1) * sem
    return MetricSummary(metric, condition, n, mean, sd, sem, mean - margin, mean + margin)


def aggregate_sessions(
    sessions: Iterable[SessionRecord],
) -> Dict[str, Dict[str, MetricSummary]]:
    """Aggregate per-dyad session metrics into per-condition summaries.

    Returns ``{condition_value: {metric_name: MetricSummary}}``. Each dyad
    contributes one value per metric, so the summary's ``n`` is the number of
    dyads, matching the dyad-as-unit analysis plan.
    """
    by_condition: Dict[str, List[Dict[str, float]]] = {}
    for session in sessions:
        by_condition.setdefault(session.condition.value, []).append(session_metrics(session))

    result: Dict[str, Dict[str, MetricSummary]] = {}
    for condition, dyad_metrics in by_condition.items():
        metric_names: List[str] = []
        for metrics in dyad_metrics:
            for name in metrics:
                if name not in metric_names:
                    metric_names.append(name)
        summaries: Dict[str, MetricSummary] = {}
        for name in metric_names:
            values = [m[name] for m in dyad_metrics if name in m]
            summaries[name] = summarize_values(name, condition, values)
        result[condition] = summaries
    return result


def load_sessions(paths: Iterable[Path]) -> List[SessionRecord]:
    return [SessionRecord.from_ndjson(Path(p)) for p in paths]


def build_study_report(sessions: Iterable[SessionRecord]) -> Dict[str, Any]:
    """Build a serializable per-condition report (ordered by the additive ladder)."""
    aggregates = aggregate_sessions(sessions)
    ordered = [c.value for c in StudyCondition if c.value in aggregates]
    return {
        "experiment": "field_teammate_study",
        "unit_of_analysis": "dyad",
        "conditions": {
            condition: {
                name: summary.to_payload()
                for name, summary in sorted(aggregates[condition].items())
            }
            for condition in ordered
        },
    }
