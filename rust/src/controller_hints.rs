//! Display descriptions only. No declaration binds or authorizes motor commands.
use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::{HashMap, HashSet};

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Eq, PartialEq, Hash)]
#[serde(rename_all = "snake_case")]
pub enum ControllerControl {
    LeftPrimary,
    LeftSecondary,
    LeftTrigger,
    LeftGrip,
    LeftStickX,
    LeftStickY,
    LeftStickClick,
    RightPrimary,
    RightSecondary,
    RightTrigger,
    RightGrip,
    RightStickX,
    RightStickY,
    RightStickClick,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, Eq, PartialEq, Hash)]
#[serde(rename_all = "snake_case")]
pub enum TeleopHintMode {
    Minimap,
    Immersive,
    FollowLeader,
    Teach,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ControllerHint {
    pub control: ControllerControl,
    pub action_id: String,
    pub label: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ControllerHintProfile {
    pub mode: TeleopHintMode,
    pub bindings: Vec<ControllerHint>,
    #[serde(default = "default_profile")]
    pub profile_id: String,
}
fn default_profile() -> String {
    "default".into()
}
#[derive(Clone, Debug)]
pub struct ControllerHints {
    pub profiles: Vec<ControllerHintProfile>,
    pub state_topic: String,
    pub state_timeout_seconds: f64,
}
impl Default for ControllerHints {
    fn default() -> Self {
        Self {
            profiles: vec![],
            state_topic: String::new(),
            state_timeout_seconds: 2.0,
        }
    }
}
fn identifier(value: &str) -> bool {
    let bytes = value.as_bytes();
    !bytes.is_empty()
        && bytes.len() <= 64
        && bytes[0].is_ascii_lowercase()
        && bytes
            .iter()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || *c == b'_')
}
fn label(value: &str) -> bool {
    value.encode_utf16().count() <= 24
        && !value
            .chars()
            .any(|c| c.is_control() || c == '\u{2028}' || c == '\u{2029}')
}
fn canonical(value: &Value) -> Result<String> {
    let source = serde_json::to_string(value)?;
    let mut result = String::new();
    // Match Python ensure_ascii and the shared native canonical serializer.
    for c in source.chars() {
        if c.is_ascii() {
            result.push(c);
        } else {
            for unit in c.encode_utf16(&mut [0u16; 2]) {
                result.push_str(&format!("\\u{unit:04x}"));
            }
        }
    }
    Ok(result)
}
impl ControllerHint {
    pub fn new(
        control: ControllerControl,
        action_id: impl Into<String>,
        text: impl Into<String>,
    ) -> Result<Self> {
        let action_id = action_id.into();
        let text = text.into();
        ensure!(
            identifier(&action_id) && label(&text),
            "Invalid controller hint action/label"
        );
        Ok(Self {
            control,
            action_id,
            label: text.trim().into(),
        })
    }
}
impl ControllerHintProfile {
    pub fn new(
        mode: TeleopHintMode,
        bindings: Vec<ControllerHint>,
        profile_id: impl Into<String>,
    ) -> Result<Self> {
        let result = Self {
            mode,
            bindings,
            profile_id: profile_id.into(),
        };
        result.validate()?;
        Ok(result)
    }
    fn validate(&self) -> Result<()> {
        ensure!(
            identifier(&self.profile_id) && self.bindings.len() <= 14,
            "Invalid hint profile"
        );
        let mut controls = HashSet::new();
        for b in &self.bindings {
            ensure!(
                controls.insert(b.control) && identifier(&b.action_id) && label(&b.label),
                "Invalid/duplicate hint binding"
            );
        }
        Ok(())
    }
}
impl ControllerHints {
    pub fn context_topic(&self) -> String {
        if self.state_topic.is_empty() {
            String::new()
        } else {
            format!("{}/context", self.state_topic)
        }
    }
    fn content(&self) -> Result<Value> {
        ensure!(
            self.profiles.len() <= 4
                && self.state_timeout_seconds.is_finite()
                && (0.5..=10.).contains(&self.state_timeout_seconds),
            "Invalid hint count/timeout"
        );
        ensure!(
            self.state_topic.len() <= 240
                && (self.state_topic.is_empty()
                    || regex::Regex::new(
                        r"^/(?:[a-zA-Z_][a-zA-Z0-9_]*)(?:/[a-zA-Z_][a-zA-Z0-9_]*)*$"
                    )?
                    .is_match(&self.state_topic)),
            "Invalid hint topic"
        );
        let mut modes = HashSet::new();
        for p in &self.profiles {
            p.validate()?;
            ensure!(modes.insert(p.mode), "Duplicate hint mode");
        }
        let profiles: Vec<Value> = self.profiles.iter().map(|p|json!({"mode":p.mode,"profile_id":p.profile_id,
            "bindings":p.bindings.iter().map(|b|json!({"control":b.control,"action_id":b.action_id,"label":b.label.trim()})).collect::<Vec<_>>()})).collect();
        let mut value = json!({"schema_version":1, "profiles":profiles, "state_timeout_seconds":self.state_timeout_seconds});
        if !self.state_topic.is_empty() {
            value["state_topic"] = json!(self.state_topic);
        }
        Ok(value)
    }
    pub fn digest(&self) -> Result<String> {
        Ok(format!(
            "{:x}",
            Sha256::digest(canonical(&self.content()?)?.as_bytes())
        ))
    }
    pub fn to_value(&self) -> Result<Value> {
        let mut value = self.content()?;
        value["config_hash"] = json!(self.digest()?);
        Ok(value)
    }
    pub fn from_value(value: Value) -> Result<Self> {
        let map = value
            .as_object()
            .ok_or_else(|| anyhow::anyhow!("Hint configuration must be an object"))?;
        ensure!(
            value.get("schema_version").and_then(Value::as_i64) == Some(1),
            "Invalid hint schema"
        );
        ensure!(
            map.keys().all(|k| [
                "schema_version",
                "profiles",
                "state_topic",
                "state_timeout_seconds",
                "config_hash"
            ]
            .contains(&k.as_str())),
            "Unknown hint field"
        );
        let result = Self {
            profiles: serde_json::from_value(
                value.get("profiles").cloned().unwrap_or(Value::Null),
            )?,
            state_topic: serde_json::from_value(
                value.get("state_topic").cloned().unwrap_or(json!("")),
            )?,
            state_timeout_seconds: serde_json::from_value(
                value
                    .get("state_timeout_seconds")
                    .cloned()
                    .unwrap_or(json!(2.0)),
            )?,
        };
        let hash = result.digest()?;
        ensure!(
            value.get("config_hash").is_none() || value["config_hash"] == json!(hash),
            "Hint configuration hash mismatch"
        );
        Ok(result)
    }
    pub fn reply(
        &self,
        context: &ControllerHintContext,
        robot: &str,
        labels: HashMap<String, String>,
        revision: i32,
    ) -> Result<ControllerHintUpdate> {
        context.validate()?;
        ensure!(
            context.robot_name == robot && context.config_hash == self.digest()?,
            "Hint context does not match robot/config"
        );
        let profile = self
            .profiles
            .iter()
            .find(|p| p.mode == context.mode && p.profile_id == context.profile_id)
            .ok_or_else(|| anyhow::anyhow!("Hint profile mismatch"))?;
        ensure!(
            labels
                .keys()
                .all(|k| profile.bindings.iter().any(|b| b.action_id == *k)),
            "Undeclared hint action"
        );
        ensure!(
            labels.values().all(|text| label(text)),
            "Invalid hint label"
        );
        let result = ControllerHintUpdate {
            context: context.clone(),
            revision,
            labels: labels
                .into_iter()
                .map(|(action_id, text)| ControllerHintLabel {
                    action_id,
                    label: text.trim().into(),
                })
                .collect(),
        };
        result.to_value()?;
        Ok(result)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerHintContext {
    pub schema_version: u32,
    pub robot_name: String,
    pub config_hash: String,
    pub profile_id: String,
    pub mode: TeleopHintMode,
    pub context_id: String,
    pub context_sequence: i32,
}
impl ControllerHintContext {
    pub fn from_json(payload: &str) -> Result<Self> {
        ensure!(payload.len() <= 16384, "Hint context too large");
        let value: Self = serde_json::from_str(payload)?;
        value.validate()?;
        Ok(value)
    }
    fn validate(&self) -> Result<()> {
        ensure!(
            self.schema_version == 1
                && self.context_sequence >= 0
                && !self.robot_name.is_empty()
                && self.robot_name.len() <= 128
                && !self.robot_name.chars().any(|c| c.is_control())
                && identifier(&self.profile_id),
            "Invalid hint context"
        );
        ensure!(
            self.config_hash.len() == 64
                && self
                    .config_hash
                    .bytes()
                    .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b)),
            "Invalid hint hash"
        );
        ensure!(
            uuid::Uuid::parse_str(&self.context_id)?.to_string()
                == self.context_id.to_ascii_lowercase(),
            "Invalid hint context UUID"
        );
        Ok(())
    }
}
#[derive(Clone, Debug, Serialize)]
pub struct ControllerHintLabel {
    pub action_id: String,
    pub label: String,
}
#[derive(Clone, Debug)]
pub struct ControllerHintUpdate {
    pub context: ControllerHintContext,
    pub revision: i32,
    pub labels: Vec<ControllerHintLabel>,
}
impl ControllerHintUpdate {
    pub fn to_value(&self) -> Result<Value> {
        self.context.validate()?;
        ensure!(
            self.revision >= 0 && self.labels.len() <= 14,
            "Invalid hint update"
        );
        let mut unique = HashSet::new();
        for l in &self.labels {
            ensure!(
                identifier(&l.action_id) && label(&l.label) && unique.insert(&l.action_id),
                "Invalid/duplicate hint label"
            );
        }
        let mut value = serde_json::to_value(&self.context)?;
        value["revision"] = json!(self.revision);
        value["labels"] = json!(self.labels);
        Ok(value)
    }
    pub fn to_json(&self) -> Result<String> {
        Ok(serde_json::to_string(&self.to_value()?)?)
    }
}
