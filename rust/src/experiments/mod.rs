//! Experiment and benchmark helpers (port of `horus.experiments`).

pub mod field_teammate_study;
pub mod metrics;

pub use field_teammate_study::{
    aggregate_sessions, build_study_report, load_sessions, session_metrics, summarize_values,
    FieldTeammateStudyRecorder, MetricSummary, SessionRecord, StudyCondition,
};
pub use metrics::{now_ns, CsvMetricWriter, NdjsonEventWriter};
