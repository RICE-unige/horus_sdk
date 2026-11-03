use super::*;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TeachStream {
    pub stream_id: String,
    pub topic: String,
    pub message_type: String,
    pub role: String,
    pub required: bool,
    pub max_gap_seconds: f64,
    pub clock_id: String,
    pub frame_id: String,
    pub source_identity: String,
    pub stamp_semantics: String,
    pub calibration: Value,
    pub reliability: String,
    pub durability: String,
    pub queue_depth: u32,
}
impl TeachStream {
    pub fn new(
        id: impl Into<String>,
        topic: impl Into<String>,
        message_type: impl Into<String>,
    ) -> Self {
        Self {
            stream_id: id.into(),
            topic: topic.into(),
            message_type: message_type.into(),
            role: "observation".into(),
            required: true,
            max_gap_seconds: 1.0,
            clock_id: "ros".into(),
            frame_id: String::new(),
            source_identity: "unknown".into(),
            stamp_semantics: "unknown".into(),
            calibration: json!({}),
            reliability: "best_effort".into(),
            durability: "volatile".into(),
            queue_depth: 5,
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TeachPortal {
    pub mode: String,
    pub edge_fade: f64,
    pub corner_radius: f64,
    pub crop: f64,
    pub swap_eyes: bool,
    pub max_skew_ms: f64,
}
impl Default for TeachPortal {
    fn default() -> Self {
        Self { mode: "stereo".into(), edge_fade: 0.06, corner_radius: 0.08,
               crop: 0.02, swap_eyes: false, max_skew_ms: 50.0 }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TeachCamera {
    pub sensor_id: String,
    pub role: String,
    pub position: [f64; 3],
    pub width_m: f64,
    pub stream_id: String,
    pub required_for_control: bool,
    pub rotation_deg: [f64; 3],
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub portal: Option<TeachPortal>,
}
impl TeachCamera {
    pub fn new(sensor_id: impl Into<String>) -> Self {
        Self {
            sensor_id: sensor_id.into(),
            role: "front".into(),
            position: [0.0, 1.35, 1.2],
            width_m: 0.7,
            stream_id: String::new(),
            required_for_control: false,
            rotation_deg: [0.0, 0.0, 0.0],
            portal: None,
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TeachPointCloud {
    pub stream_id: String,
    pub point_size_m: f64,
    pub max_points: u32,
    pub max_age_seconds: f64,
}
impl TeachPointCloud {
    pub fn new(stream_id: impl Into<String>) -> Self {
        Self { stream_id: stream_id.into(), point_size_m: 0.004, max_points: 1_000_000,
               max_age_seconds: 1.0 }
    }
}
impl TeachConfig {
    /// Adds a stream and validates all cross-references before returning it.
    pub fn with_stream(&self, stream: TeachStream) -> Result<Self> {
        let mut config = self.to_value().clone();
        config["streams"]
            .as_array_mut()
            .unwrap()
            .push(serde_json::to_value(stream)?);
        Self::from_value(config)
    }
    pub fn with_camera(&self, camera: TeachCamera) -> Result<Self> {
        let mut config = self.to_value().clone();
        config["cameras"]
            .as_array_mut()
            .unwrap()
            .push(serde_json::to_value(camera)?);
        Self::from_value(config)
    }
    pub fn with_spatial_view(&self, view: TeachPointCloud) -> Result<Self> {
        let mut config = self.to_value().clone();
        if config.get("spatial_views").is_none() { config["spatial_views"] = json!([]); }
        config["spatial_views"].as_array_mut().unwrap().push(serde_json::to_value(view)?);
        Self::from_value(config)
    }
}
