//! Metric writers and a monotonic-anchored clock for HORUS benchmark runs.
//!
//! Mirrors `horus.experiments.metrics`: a wall-anchored monotonic clock (so
//! timestamps share an epoch with Quest/Unity wall time but never step
//! backwards on a VM), a newline-delimited JSON event writer, and a CSV metric
//! writer with stable common fields.

use once_cell::sync::Lazy;
use serde_json::{Map, Value};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

pub const COMMON_FIELDS: [&str; 4] = ["timestamp_ns", "run_id", "experiment", "condition"];

struct ClockAnchor {
    wall_ns: u128,
    mono: Instant,
}

static CLOCK: Lazy<ClockAnchor> = Lazy::new(|| ClockAnchor {
    wall_ns: SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos(),
    mono: Instant::now(),
});

/// Unix-epoch-like timestamp derived from a monotonic clock, anchored once to
/// wall time. Stays monotonic within the process even if wall time steps.
pub fn now_ns() -> u128 {
    CLOCK.wall_ns + CLOCK.mono.elapsed().as_nanos()
}

/// Write event records as newline-delimited JSON.
pub struct NdjsonEventWriter {
    writer: BufWriter<File>,
    run_id: String,
    experiment: String,
    condition: String,
    source: String,
}

impl NdjsonEventWriter {
    pub fn create(
        path: impl AsRef<Path>,
        run_id: impl Into<String>,
        experiment: impl Into<String>,
        condition: impl Into<String>,
        source: impl Into<String>,
    ) -> std::io::Result<Self> {
        let path = path.as_ref();
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent)?;
            }
        }
        Ok(Self {
            writer: BufWriter::new(File::create(path)?),
            run_id: run_id.into(),
            experiment: experiment.into(),
            condition: condition.into(),
            source: source.into(),
        })
    }

    pub fn write(&mut self, event: Map<String, Value>, timestamp_ns: Option<u128>) -> std::io::Result<()> {
        let mut payload = event;
        payload
            .entry("timestamp_ns")
            .or_insert_with(|| Value::from(timestamp_ns.unwrap_or_else(now_ns) as u64));
        payload
            .entry("run_id")
            .or_insert_with(|| Value::from(self.run_id.clone()));
        payload
            .entry("experiment")
            .or_insert_with(|| Value::from(self.experiment.clone()));
        payload
            .entry("condition")
            .or_insert_with(|| Value::from(self.condition.clone()));
        payload
            .entry("source")
            .or_insert_with(|| Value::from(self.source.clone()));
        // Stable key order keeps event logs diff-friendly across languages.
        let ordered: std::collections::BTreeMap<String, Value> = payload.into_iter().collect();
        let line = serde_json::to_string(&ordered)?;
        self.writer.write_all(line.as_bytes())?;
        self.writer.write_all(b"\n")
    }

    pub fn flush(&mut self) -> std::io::Result<()> {
        self.writer.flush()
    }
}

/// Append metric rows to a CSV with stable common fields prepended.
pub struct CsvMetricWriter {
    writer: BufWriter<File>,
    fields: Vec<String>,
    run_id: String,
    experiment: String,
    condition: String,
}

impl CsvMetricWriter {
    pub fn create(
        path: impl AsRef<Path>,
        run_id: impl Into<String>,
        experiment: impl Into<String>,
        condition: impl Into<String>,
        fieldnames: &[&str],
    ) -> std::io::Result<Self> {
        let path: PathBuf = path.as_ref().to_path_buf();
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent)?;
            }
        }
        let mut fields: Vec<String> = COMMON_FIELDS.iter().map(|s| s.to_string()).collect();
        for field in fieldnames {
            if !fields.iter().any(|f| f == field) {
                fields.push((*field).to_string());
            }
        }
        let mut writer = BufWriter::new(File::create(&path)?);
        writeln!(writer, "{}", fields.join(","))?;
        Ok(Self {
            writer,
            fields,
            run_id: run_id.into(),
            experiment: experiment.into(),
            condition: condition.into(),
        })
    }

    pub fn write(&mut self, row: &Map<String, Value>, timestamp_ns: Option<u128>) -> std::io::Result<()> {
        let cells: Vec<String> = self
            .fields
            .iter()
            .map(|field| match field.as_str() {
                "timestamp_ns" => row
                    .get("timestamp_ns")
                    .map(csv_cell)
                    .unwrap_or_else(|| (timestamp_ns.unwrap_or_else(now_ns) as u64).to_string()),
                "run_id" => row.get("run_id").map(csv_cell).unwrap_or_else(|| self.run_id.clone()),
                "experiment" => row
                    .get("experiment")
                    .map(csv_cell)
                    .unwrap_or_else(|| self.experiment.clone()),
                "condition" => row
                    .get("condition")
                    .map(csv_cell)
                    .unwrap_or_else(|| self.condition.clone()),
                other => row.get(other).map(csv_cell).unwrap_or_default(),
            })
            .collect();
        writeln!(self.writer, "{}", cells.join(","))
    }

    pub fn flush(&mut self) -> std::io::Result<()> {
        self.writer.flush()
    }
}

fn csv_cell(value: &Value) -> String {
    let raw = match value {
        Value::String(s) => s.clone(),
        other => other.to_string(),
    };
    if raw.contains(',') || raw.contains('"') || raw.contains('\n') {
        format!("\"{}\"", raw.replace('"', "\"\""))
    } else {
        raw
    }
}
