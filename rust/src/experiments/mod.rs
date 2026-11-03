//! Experiment and benchmark helpers (port of `horus.experiments`).

pub mod field_teammate_study;
pub mod manifest;
pub mod metrics;
pub mod workloads;
pub use manifest::{default_run_id, ExperimentManifest, RunIdentity};
pub use workloads::{load_workload_config, workload_from_dict, WorkloadConfig};

pub use field_teammate_study::{
    aggregate_sessions, build_study_report, load_sessions, session_metrics, summarize_values,
    FieldTeammateStudyRecorder, MetricSummary, SessionRecord, StudyCondition,
};
pub use metrics::{now_ns, CsvMetricWriter, NdjsonEventWriter};
