//! Run manifests compatible with the Python analysis tools. Commit identity is explicit.
use anyhow::{ensure, Result};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::path::{Path, PathBuf};

pub fn default_run_id(
    experiment: &str,
    condition: &str,
    repetition: u32,
    timestamp: Option<DateTime<Utc>>,
) -> String {
    fn slug(s: &str) -> String {
        let mut out = String::new();
        for c in s.to_lowercase().chars() {
            if c.is_alphanumeric() {
                out.push(c);
            } else if !out.is_empty() && !out.ends_with('_') {
                out.push('_');
            }
        }
        let out = out.trim_end_matches('_');
        if out.is_empty() {
            "run".into()
        } else {
            out.into()
        }
    }
    format!(
        "{}_{}_r{:02}_{}",
        slug(experiment),
        slug(condition),
        repetition,
        timestamp.unwrap_or_else(Utc::now).format("%Y%m%d_%H%M%S")
    )
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RunIdentity {
    pub run_id: String,
    pub experiment: String,
    pub condition: String,
    pub repetition: u32,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ExperimentManifest(pub Value);
impl ExperimentManifest {
    pub fn from_identity(id: &RunIdentity, options: Value) -> Result<Self> {
        let host = std::fs::read_to_string("/proc/sys/kernel/hostname")
            .unwrap_or_default()
            .trim()
            .to_owned();
        let mut fields = json!({"run_id":id.run_id,"experiment":id.experiment,"condition":id.condition,
            "repetition":id.repetition,"date":Utc::now().format("%Y-%m-%dT%H:%M:%S+00:00").to_string(),
            "duration_s":120.0,"warmup_s":10.0,"robot_count":1,"operator_count":1,"stream_count":0,
            "transport":"ros2","topology":"lan","resolution":"","target_fps":0.0,
            "robot_profile":"minimal","map_profile":"none","notes":"","source_git_commit":"",
            "host":host,"platform":format!("{}-{}",std::env::consts::OS,std::env::consts::ARCH),"extra":{}});
        let map = options
            .as_object()
            .ok_or_else(|| anyhow::anyhow!("Manifest options must be an object"))?;
        for (key, value) in map {
            ensure!(
                fields.get(key).is_some()
                    && !["run_id", "experiment", "condition", "repetition"].contains(&key.as_str()),
                "Unknown or identity manifest override: {key}"
            );
            fields[key] = value.clone();
        }
        Ok(Self(fields))
    }
    pub fn to_dict(&self) -> Value {
        self.0.clone()
    }
    pub fn save(&self, directory: impl AsRef<Path>) -> Result<PathBuf> {
        std::fs::create_dir_all(directory.as_ref())?;
        let path = directory.as_ref().join("run_manifest.json");
        std::fs::write(&path, serde_json::to_string_pretty(&self.0)? + "\n")?;
        Ok(path)
    }
}
