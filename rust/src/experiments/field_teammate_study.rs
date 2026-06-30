//! Analysis scaffolding for the field-teammate HRI study ("On the Map, In the
//! Team"). Port of `horus.experiments.field_teammate_study`.
//!
//! Hardware-independent: sessions are newline-delimited JSON, the dyad is the
//! unit of analysis, and aggregates carry t-based 95% confidence intervals.

use super::metrics::{now_ns, NdjsonEventWriter};
use serde_json::{json, Map, Value};
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StudyCondition {
    VoiceVideo,
    OnTheMap,
    InTheTeam,
}

impl StudyCondition {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::VoiceVideo => "c1_voice_video",
            Self::OnTheMap => "c2_on_the_map",
            Self::InTheTeam => "c3_in_the_team",
        }
    }

    pub fn coerce(value: &str) -> Option<Self> {
        match value.trim().to_ascii_lowercase().as_str() {
            "c1_voice_video" | "voicevideo" | "voice_video" => Some(Self::VoiceVideo),
            "c2_on_the_map" | "onthemap" | "on_the_map" => Some(Self::OnTheMap),
            "c3_in_the_team" | "intheteam" | "in_the_team" => Some(Self::InTheTeam),
            _ => None,
        }
    }

    /// The additive ladder order used when emitting reports.
    pub fn ladder() -> [StudyCondition; 3] {
        [Self::VoiceVideo, Self::OnTheMap, Self::InTheTeam]
    }
}

pub mod event_type {
    pub const SESSION_START: &str = "session_start";
    pub const SESSION_END: &str = "session_end";
    pub const GUIDANCE_REQUEST: &str = "guidance_request";
    pub const GUIDANCE_RESPONSE: &str = "guidance_response";
    pub const CLARIFICATION: &str = "clarification";
    pub const COMMUNICATION: &str = "communication";
    pub const UNCERTAINTY_RAISED: &str = "uncertainty_raised";
    pub const UNCERTAINTY_RESOLVED: &str = "uncertainty_resolved";
    pub const NAVIGATION_ERROR: &str = "navigation_error";
    pub const WAYPOINT_REACHED: &str = "waypoint_reached";
    pub const TASK_SUCCESS: &str = "task_success";
    pub const TASK_FAILURE: &str = "task_failure";
    pub const SAFETY_EVENT: &str = "safety_event";
    pub const LOCALIZATION_SPOTCHECK: &str = "localization_spotcheck";
}

pub mod guidance_outcome {
    pub const ACKNOWLEDGE: &str = "acknowledge";
    pub const CLARIFY: &str = "clarify";
    pub const REJECT: &str = "reject";
    pub const COMPLETE: &str = "complete";
}

/// Append study events for one dyad/session to a newline-delimited JSON log.
pub struct FieldTeammateStudyRecorder {
    writer: NdjsonEventWriter,
    dyad_id: String,
    scenario: String,
}

impl FieldTeammateStudyRecorder {
    pub fn create(
        path: impl AsRef<Path>,
        dyad_id: impl Into<String>,
        condition: StudyCondition,
        scenario: impl Into<String>,
    ) -> std::io::Result<Self> {
        let dyad_id = dyad_id.into();
        let writer = NdjsonEventWriter::create(
            path,
            dyad_id.clone(),
            "field_teammate_study",
            condition.as_str(),
            "study",
        )?;
        Ok(Self {
            writer,
            dyad_id,
            scenario: scenario.into(),
        })
    }

    pub fn record(
        &mut self,
        event_type: &str,
        mut payload: Map<String, Value>,
        timestamp_ns: Option<u128>,
    ) -> std::io::Result<()> {
        payload.insert("event_type".into(), Value::from(event_type));
        payload.insert("dyad_id".into(), Value::from(self.dyad_id.clone()));
        payload.insert("scenario".into(), Value::from(self.scenario.clone()));
        self.writer.write(payload, Some(timestamp_ns.unwrap_or_else(now_ns)))
    }

    pub fn flush(&mut self) -> std::io::Result<()> {
        self.writer.flush()
    }
}

/// One dyad session loaded from an NDJSON log.
pub struct SessionRecord {
    pub dyad_id: String,
    pub condition: StudyCondition,
    pub scenario: String,
    pub events: Vec<Value>,
}

impl SessionRecord {
    pub fn from_ndjson(path: impl AsRef<Path>) -> std::io::Result<Self> {
        let raw = std::fs::read_to_string(path)?;
        let mut events: Vec<Value> = raw
            .lines()
            .filter(|line| !line.trim().is_empty())
            .map(|line| serde_json::from_str(line))
            .collect::<Result<_, _>>()
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        events.sort_by_key(event_ts);
        let first = events.first().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::InvalidData, "no events in session log")
        })?;
        let condition = first
            .get("condition")
            .and_then(Value::as_str)
            .and_then(StudyCondition::coerce)
            .ok_or_else(|| {
                std::io::Error::new(std::io::ErrorKind::InvalidData, "unknown study condition")
            })?;
        Ok(Self {
            dyad_id: str_field(first, "dyad_id"),
            condition,
            scenario: str_field(first, "scenario"),
            events,
        })
    }

    fn of_type<'a>(&'a self, kind: &'a str) -> impl Iterator<Item = &'a Value> + 'a {
        self.events
            .iter()
            .filter(move |e| e.get("event_type").and_then(Value::as_str) == Some(kind))
    }
}

fn event_ts(event: &Value) -> i128 {
    event
        .get("timestamp_ns")
        .map(|v| {
            v.as_i64()
                .map(|n| n as i128)
                .or_else(|| v.as_u64().map(|n| n as i128))
                .or_else(|| v.as_f64().map(|n| n as i128))
                .unwrap_or(0)
        })
        .unwrap_or(0)
}

fn str_field(event: &Value, key: &str) -> String {
    event.get(key).and_then(Value::as_str).unwrap_or("").to_string()
}

fn f64_field(event: &Value, key: &str) -> f64 {
    event.get(key).and_then(Value::as_f64).unwrap_or(0.0)
}

const NS_PER_S: f64 = 1e9;

/// Reduce a single dyad session to its dependent variables.
pub fn session_metrics(session: &SessionRecord) -> BTreeMap<String, f64> {
    let mut metrics: BTreeMap<String, f64> = BTreeMap::new();

    let starts: Vec<&Value> = session.of_type(event_type::SESSION_START).collect();
    let ends: Vec<&Value> = session.of_type(event_type::SESSION_END).collect();
    if let (Some(start), Some(end)) = (starts.first(), ends.last()) {
        metrics.insert(
            "mission_completion_time_s".into(),
            (event_ts(end) - event_ts(start)) as f64 / NS_PER_S,
        );
    }

    let successes = session.of_type(event_type::TASK_SUCCESS).count();
    let failures = session.of_type(event_type::TASK_FAILURE).count();
    if successes + failures > 0 {
        metrics.insert(
            "task_success_rate".into(),
            successes as f64 / (successes + failures) as f64,
        );
    }
    metrics.insert(
        "navigation_errors".into(),
        session.of_type(event_type::NAVIGATION_ERROR).count() as f64,
    );
    metrics.insert(
        "safety_events".into(),
        session.of_type(event_type::SAFETY_EVENT).count() as f64,
    );

    let responses: Vec<&Value> = session.of_type(event_type::GUIDANCE_RESPONSE).collect();
    let clarify_responses = responses
        .iter()
        .filter(|r| r.get("outcome").and_then(Value::as_str) == Some(guidance_outcome::CLARIFY))
        .count();
    metrics.insert(
        "clarification_requests".into(),
        (session.of_type(event_type::CLARIFICATION).count() + clarify_responses) as f64,
    );
    metrics.insert(
        "guidance_requests".into(),
        session.of_type(event_type::GUIDANCE_REQUEST).count() as f64,
    );
    metrics.insert(
        "guidance_rejections".into(),
        responses
            .iter()
            .filter(|r| r.get("outcome").and_then(Value::as_str) == Some(guidance_outcome::REJECT))
            .count() as f64,
    );

    let comms: Vec<&Value> = session.of_type(event_type::COMMUNICATION).collect();
    metrics.insert("communication_count".into(), comms.len() as f64);
    metrics.insert(
        "communication_total_duration_s".into(),
        comms.iter().map(|c| f64_field(c, "duration_s")).sum(),
    );

    // Guidance delivery latency: response minus matching request, paired on id.
    let mut request_times: BTreeMap<String, i128> = BTreeMap::new();
    for req in session.of_type(event_type::GUIDANCE_REQUEST) {
        let rid = str_field(req, "request_id");
        if !rid.is_empty() {
            request_times.insert(rid, event_ts(req));
        }
    }
    let latencies: Vec<f64> = responses
        .iter()
        .filter_map(|resp| {
            let rid = str_field(resp, "request_id");
            request_times
                .get(&rid)
                .map(|&t| (event_ts(resp) - t) as f64 / NS_PER_S)
        })
        .collect();
    if !latencies.is_empty() {
        metrics.insert("mean_guidance_latency_s".into(), mean(&latencies));
    }

    let mut raised: BTreeMap<String, i128> = BTreeMap::new();
    for ev in session.of_type(event_type::UNCERTAINTY_RAISED) {
        let uid = str_field(ev, "uncertainty_id");
        if !uid.is_empty() {
            raised.insert(uid, event_ts(ev));
        }
    }
    let resolutions: Vec<f64> = session
        .of_type(event_type::UNCERTAINTY_RESOLVED)
        .filter_map(|ev| {
            let uid = str_field(ev, "uncertainty_id");
            raised.get(&uid).map(|&t| (event_ts(ev) - t) as f64 / NS_PER_S)
        })
        .collect();
    if !resolutions.is_empty() {
        metrics.insert("mean_uncertainty_resolution_s".into(), mean(&resolutions));
    }

    let spotchecks: Vec<&Value> = session.of_type(event_type::LOCALIZATION_SPOTCHECK).collect();
    if !spotchecks.is_empty() {
        metrics.insert(
            "mean_localization_error_m".into(),
            mean(&spotchecks.iter().map(|s| f64_field(s, "error_m")).collect::<Vec<_>>()),
        );
        metrics.insert(
            "mean_heading_error_deg".into(),
            mean(&spotchecks
                .iter()
                .map(|s| f64_field(s, "heading_error_deg"))
                .collect::<Vec<_>>()),
        );
    }

    metrics
}

#[derive(Debug, Clone)]
pub struct MetricSummary {
    pub metric: String,
    pub condition: String,
    pub n: usize,
    pub mean: f64,
    pub sd: f64,
    pub sem: f64,
    pub ci95_low: f64,
    pub ci95_high: f64,
}

impl MetricSummary {
    pub fn to_value(&self) -> Value {
        json!({
            "metric": self.metric,
            "condition": self.condition,
            "n": self.n,
            "mean": self.mean,
            "sd": self.sd,
            "sem": self.sem,
            "ci95_low": self.ci95_low,
            "ci95_high": self.ci95_high,
        })
    }
}

fn mean(values: &[f64]) -> f64 {
    if values.is_empty() {
        return f64::NAN;
    }
    values.iter().sum::<f64>() / values.len() as f64
}

fn sample_stdev(values: &[f64], mean: f64) -> f64 {
    let n = values.len();
    if n < 2 {
        return 0.0;
    }
    let var = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (n - 1) as f64;
    var.sqrt()
}

fn t_critical_95(df: usize) -> f64 {
    const TABLE: [f64; 30] = [
        12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262, 2.228, 2.201, 2.179,
        2.160, 2.145, 2.131, 2.120, 2.110, 2.101, 2.093, 2.086, 2.080, 2.074, 2.069, 2.064,
        2.060, 2.056, 2.052, 2.048, 2.045, 2.042,
    ];
    if df == 0 {
        f64::NAN
    } else if df <= 30 {
        TABLE[df - 1]
    } else {
        1.96
    }
}

pub fn summarize_values(metric: &str, condition: &str, values: &[f64]) -> MetricSummary {
    let clean: Vec<f64> = values.iter().copied().filter(|v| !v.is_nan()).collect();
    let n = clean.len();
    if n == 0 {
        return MetricSummary {
            metric: metric.into(),
            condition: condition.into(),
            n: 0,
            mean: f64::NAN,
            sd: f64::NAN,
            sem: f64::NAN,
            ci95_low: f64::NAN,
            ci95_high: f64::NAN,
        };
    }
    let m = mean(&clean);
    if n == 1 {
        return MetricSummary {
            metric: metric.into(),
            condition: condition.into(),
            n: 1,
            mean: m,
            sd: 0.0,
            sem: 0.0,
            ci95_low: m,
            ci95_high: m,
        };
    }
    let sd = sample_stdev(&clean, m);
    let sem = sd / (n as f64).sqrt();
    let margin = t_critical_95(n - 1) * sem;
    MetricSummary {
        metric: metric.into(),
        condition: condition.into(),
        n,
        mean: m,
        sd,
        sem,
        ci95_low: m - margin,
        ci95_high: m + margin,
    }
}

/// Aggregate per-dyad session metrics into per-condition summaries (dyad unit).
pub fn aggregate_sessions(
    sessions: &[SessionRecord],
) -> BTreeMap<String, BTreeMap<String, MetricSummary>> {
    let mut by_condition: BTreeMap<String, Vec<BTreeMap<String, f64>>> = BTreeMap::new();
    for session in sessions {
        by_condition
            .entry(session.condition.as_str().to_string())
            .or_default()
            .push(session_metrics(session));
    }

    let mut result: BTreeMap<String, BTreeMap<String, MetricSummary>> = BTreeMap::new();
    for (condition, dyad_metrics) in &by_condition {
        let mut metric_names: BTreeSet<String> = BTreeSet::new();
        for metrics in dyad_metrics {
            metric_names.extend(metrics.keys().cloned());
        }
        let mut summaries: BTreeMap<String, MetricSummary> = BTreeMap::new();
        for name in &metric_names {
            let values: Vec<f64> = dyad_metrics.iter().filter_map(|m| m.get(name).copied()).collect();
            summaries.insert(name.clone(), summarize_values(name, condition, &values));
        }
        result.insert(condition.clone(), summaries);
    }
    result
}

pub fn load_sessions(paths: &[impl AsRef<Path>]) -> std::io::Result<Vec<SessionRecord>> {
    paths.iter().map(SessionRecord::from_ndjson).collect()
}

/// Build a serializable per-condition report ordered by the additive ladder.
pub fn build_study_report(sessions: &[SessionRecord]) -> Value {
    let aggregates = aggregate_sessions(sessions);
    let mut conditions = Map::new();
    for condition in StudyCondition::ladder() {
        if let Some(summaries) = aggregates.get(condition.as_str()) {
            let mut metric_map = Map::new();
            for (name, summary) in summaries {
                metric_map.insert(name.clone(), summary.to_value());
            }
            conditions.insert(condition.as_str().to_string(), Value::Object(metric_map));
        }
    }
    json!({
        "experiment": "field_teammate_study",
        "unit_of_analysis": "dyad",
        "conditions": Value::Object(conditions),
    })
}
