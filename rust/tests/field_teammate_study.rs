use horus::experiments::field_teammate_study::{
    aggregate_sessions, build_study_report, event_type, guidance_outcome, summarize_values,
    FieldTeammateStudyRecorder, SessionRecord, StudyCondition,
};
use serde_json::{Map, Value};
use std::path::PathBuf;

fn obj(pairs: &[(&str, Value)]) -> Map<String, Value> {
    pairs.iter().cloned().map(|(k, v)| (k.to_string(), v)).collect()
}

fn write_session(
    path: &PathBuf,
    dyad_id: &str,
    condition: StudyCondition,
    base_ns: u128,
    mission_s: f64,
    clarifications: usize,
    nav_errors: usize,
    guidance_latency_s: f64,
) {
    let latency_ns = (guidance_latency_s * 1e9) as u128;
    let mut rec = FieldTeammateStudyRecorder::create(path, dyad_id, condition, "alpha").unwrap();
    rec.record(event_type::SESSION_START, Map::new(), Some(base_ns)).unwrap();
    for i in 0..clarifications {
        let rid = format!("{dyad_id}-clar-{i}");
        rec.record(
            event_type::GUIDANCE_REQUEST,
            obj(&[("request_id", Value::from(rid.clone()))]),
            Some(base_ns + 1_000_000),
        )
        .unwrap();
        rec.record(
            event_type::GUIDANCE_RESPONSE,
            obj(&[
                ("request_id", Value::from(rid)),
                ("outcome", Value::from(guidance_outcome::CLARIFY)),
            ]),
            Some(base_ns + 1_000_000 + latency_ns),
        )
        .unwrap();
    }
    rec.record(
        event_type::GUIDANCE_REQUEST,
        obj(&[("request_id", Value::from("ack-1"))]),
        Some(base_ns + 2_000_000),
    )
    .unwrap();
    rec.record(
        event_type::GUIDANCE_RESPONSE,
        obj(&[
            ("request_id", Value::from("ack-1")),
            ("outcome", Value::from(guidance_outcome::ACKNOWLEDGE)),
        ]),
        Some(base_ns + 2_000_000 + latency_ns),
    )
    .unwrap();
    for _ in 0..nav_errors {
        rec.record(event_type::NAVIGATION_ERROR, Map::new(), Some(base_ns + 3_000_000)).unwrap();
    }
    rec.record(
        event_type::COMMUNICATION,
        obj(&[("duration_s", Value::from(2.0))]),
        Some(base_ns + 4_000_000),
    )
    .unwrap();
    rec.record(
        event_type::LOCALIZATION_SPOTCHECK,
        obj(&[
            ("error_m", Value::from(0.12)),
            ("heading_error_deg", Value::from(3.0)),
        ]),
        Some(base_ns + 5_000_000),
    )
    .unwrap();
    rec.record(event_type::TASK_SUCCESS, Map::new(), Some(base_ns + 6_000_000)).unwrap();
    rec.record(event_type::SESSION_END, Map::new(), Some(base_ns + (mission_s * 1e9) as u128))
        .unwrap();
    rec.flush().unwrap();
}

fn approx(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

#[test]
fn session_metrics_match_python_reference() {
    let path = std::env::temp_dir().join(format!("horus_ft_study_{}.ndjson", std::process::id()));
    write_session(&path, "dyad1", StudyCondition::InTheTeam, 1_000_000_000, 120.0, 2, 1, 0.5);
    let session = SessionRecord::from_ndjson(&path).unwrap();
    let metrics = horus::experiments::session_metrics(&session);

    assert!(approx(metrics["mission_completion_time_s"], 120.0));
    assert!(approx(metrics["clarification_requests"], 2.0));
    assert!(approx(metrics["navigation_errors"], 1.0));
    assert!(approx(metrics["guidance_requests"], 3.0));
    assert!(approx(metrics["communication_count"], 1.0));
    assert!(approx(metrics["task_success_rate"], 1.0));
    assert!(approx(metrics["mean_guidance_latency_s"], 0.5));
    assert!(approx(metrics["mean_localization_error_m"], 0.12));
    let _ = std::fs::remove_file(&path);
}

#[test]
fn aggregate_uses_dyad_as_unit() {
    let dir = std::env::temp_dir().join(format!("horus_ft_agg_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let mut sessions = Vec::new();
    for (i, mission) in [100.0, 110.0, 120.0].iter().enumerate() {
        let p = dir.join(format!("map_{i}.ndjson"));
        write_session(&p, &format!("dyad{i}"), StudyCondition::OnTheMap, 1_000_000_000, *mission, 3, 2, 0.8);
        sessions.push(SessionRecord::from_ndjson(&p).unwrap());
    }
    for (i, mission) in [70.0, 80.0].iter().enumerate() {
        let p = dir.join(format!("team_{i}.ndjson"));
        write_session(&p, &format!("team{i}"), StudyCondition::InTheTeam, 1_000_000_000, *mission, 1, 0, 0.3);
        sessions.push(SessionRecord::from_ndjson(&p).unwrap());
    }

    let agg = aggregate_sessions(&sessions);
    let on_map = &agg["c2_on_the_map"]["mission_completion_time_s"];
    assert_eq!(on_map.n, 3);
    assert!(approx(on_map.mean, 110.0));
    assert!(on_map.ci95_low < on_map.mean && on_map.mean < on_map.ci95_high);

    let in_team = &agg["c3_in_the_team"]["mission_completion_time_s"];
    assert_eq!(in_team.n, 2);
    assert!(approx(in_team.mean, 75.0));
    assert!(approx(agg["c3_in_the_team"]["clarification_requests"].mean, 1.0));
    assert!(approx(agg["c2_on_the_map"]["clarification_requests"].mean, 3.0));

    let report = build_study_report(&sessions);
    let keys: Vec<&str> = report["conditions"].as_object().unwrap().keys().map(String::as_str).collect();
    assert_eq!(keys, vec!["c2_on_the_map", "c3_in_the_team"]);
    assert_eq!(report["unit_of_analysis"], "dyad");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn summarize_values_confidence_interval() {
    let s = summarize_values("x", "c", &[10.0, 12.0, 14.0, 16.0, 18.0]);
    assert_eq!(s.n, 5);
    assert!(approx(s.mean, 14.0));
    assert!(s.ci95_low < 14.0 && 14.0 < s.ci95_high);
    let single = summarize_values("x", "c", &[5.0]);
    assert!(approx(single.ci95_low, 5.0) && approx(single.ci95_high, 5.0));
}
