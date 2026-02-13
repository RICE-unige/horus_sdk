use crate::core::types::{Metadata, SensorType};
use std::any::Any;
use std::fmt::Debug;
use std::sync::Arc;

pub type SensorRef = Arc<dyn Sensor>;

pub trait Sensor: Debug + Send + Sync {
    fn as_any(&self) -> &dyn Any;
    fn name(&self) -> &str;
    fn sensor_type(&self) -> SensorType;
    fn frame_id(&self) -> &str;
    fn topic(&self) -> &str;
    fn enabled(&self) -> bool;
    fn metadata(&self) -> &Metadata;
}

#[derive(Debug, Clone)]
pub struct Camera {
    pub name: String,
    pub frame_id: String,
    pub topic: String,
    pub enabled: bool,
    pub metadata: Metadata,
    pub is_stereo: bool,
    pub resolution: (u32, u32),
    pub fps: u32,
    pub fov: f32,
    pub encoding: String,
    pub streaming_type: String,
    pub minimap_streaming_type: String,
    pub teleop_streaming_type: String,
    pub startup_mode: String,
}

impl Camera {
    pub fn new(name: impl Into<String>, frame_id: impl Into<String>, topic: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            frame_id: frame_id.into(),
            topic: topic.into(),
            enabled: true,
            metadata: Metadata::new(),
            is_stereo: false,
            resolution: (640, 480),
            fps: 30,
            fov: 60.0,
            encoding: "bgr8".to_string(),
            streaming_type: "ros".to_string(),
            minimap_streaming_type: "ros".to_string(),
            teleop_streaming_type: "webrtc".to_string(),
            startup_mode: "minimap".to_string(),
        }
    }

    pub fn try_new_with_profiles(
        name: impl Into<String>,
        frame_id: impl Into<String>,
        topic: impl Into<String>,
        streaming_type: &str,
        minimap_streaming_type: &str,
        teleop_streaming_type: &str,
        startup_mode: &str,
    ) -> Result<Self, String> {
        let mut camera = Self::new(name, frame_id, topic);
        camera.streaming_type = normalize_transport(streaming_type, "streaming_type")?;
        camera.minimap_streaming_type =
            normalize_transport(minimap_streaming_type, "minimap_streaming_type")?;
        camera.teleop_streaming_type =
            normalize_transport(teleop_streaming_type, "teleop_streaming_type")?;

        let startup = startup_mode.trim().to_ascii_lowercase();
        if startup != "minimap" && startup != "teleop" {
            return Err("Camera startup_mode must be 'minimap' or 'teleop'".to_string());
        }
        camera.startup_mode = startup;
        Ok(camera)
    }

    pub fn get_camera_type(&self) -> &'static str {
        if self.is_stereo {
            "stereo"
        } else {
            "mono"
        }
    }

    pub fn get_resolution_str(&self) -> String {
        format!("{}x{}", self.resolution.0, self.resolution.1)
    }

    pub fn add_metadata(&mut self, key: impl Into<String>, value: serde_json::Value) {
        self.metadata.insert(key.into(), value);
    }

    pub fn get_metadata(&self, key: &str, default: serde_json::Value) -> serde_json::Value {
        self.metadata.get(key).cloned().unwrap_or(default)
    }
}

fn normalize_transport(value: &str, field_name: &str) -> Result<String, String> {
    let normalized = value.trim().to_ascii_lowercase();
    if normalized == "ros" || normalized == "webrtc" {
        Ok(normalized)
    } else {
        Err(format!("Camera {field_name} must be 'ros' or 'webrtc'"))
    }
}

impl Sensor for Camera {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::Camera
    }

    fn frame_id(&self) -> &str {
        &self.frame_id
    }

    fn topic(&self) -> &str {
        &self.topic
    }

    fn enabled(&self) -> bool {
        self.enabled
    }

    fn metadata(&self) -> &Metadata {
        &self.metadata
    }
}

#[derive(Debug, Clone)]
pub struct LaserScan {
    pub name: String,
    pub frame_id: String,
    pub topic: String,
    pub enabled: bool,
    pub metadata: Metadata,
    pub min_angle: f32,
    pub max_angle: f32,
    pub angle_increment: f32,
    pub min_range: f32,
    pub max_range: f32,
    pub range_resolution: f32,
    pub color: String,
    pub point_size: f32,
}

impl LaserScan {
    pub fn new(name: impl Into<String>, frame_id: impl Into<String>, topic: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            frame_id: frame_id.into(),
            topic: topic.into(),
            enabled: true,
            metadata: Metadata::new(),
            min_angle: -std::f32::consts::PI,
            max_angle: std::f32::consts::PI,
            angle_increment: 0.005,
            min_range: 0.1,
            max_range: 30.0,
            range_resolution: 0.01,
            color: "red".to_string(),
            point_size: 0.05,
        }
    }

    pub fn get_scan_range_degrees(&self) -> f32 {
        (self.max_angle - self.min_angle) * 180.0 / std::f32::consts::PI
    }

    pub fn get_num_points(&self) -> i32 {
        ((self.max_angle - self.min_angle) / self.angle_increment) as i32
    }

    pub fn add_metadata(&mut self, key: impl Into<String>, value: serde_json::Value) {
        self.metadata.insert(key.into(), value);
    }
}

impl Sensor for LaserScan {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::LaserScan
    }

    fn frame_id(&self) -> &str {
        &self.frame_id
    }

    fn topic(&self) -> &str {
        &self.topic
    }

    fn enabled(&self) -> bool {
        self.enabled
    }

    fn metadata(&self) -> &Metadata {
        &self.metadata
    }
}

#[derive(Debug, Clone)]
pub struct Lidar3D {
    pub name: String,
    pub frame_id: String,
    pub topic: String,
    pub enabled: bool,
    pub metadata: Metadata,
    pub vertical_fov: f32,
    pub horizontal_fov: f32,
    pub vertical_resolution: f32,
    pub horizontal_resolution: f32,
    pub min_range: f32,
    pub max_range: f32,
    pub points_per_second: i32,
    pub num_layers: i32,
}

impl Lidar3D {
    pub fn new(name: impl Into<String>, frame_id: impl Into<String>, topic: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            frame_id: frame_id.into(),
            topic: topic.into(),
            enabled: true,
            metadata: Metadata::new(),
            vertical_fov: 40.0,
            horizontal_fov: 360.0,
            vertical_resolution: 0.4,
            horizontal_resolution: 0.4,
            min_range: 0.5,
            max_range: 100.0,
            points_per_second: 1_000_000,
            num_layers: 64,
        }
    }

    pub fn get_point_cloud_size(&self) -> i32 {
        let h_points = (self.horizontal_fov / self.horizontal_resolution) as i32;
        let v_points = (self.vertical_fov / self.vertical_resolution) as i32;
        h_points * v_points
    }

    pub fn get_lidar_type(&self) -> String {
        format!("{}-layer 3D LiDAR", self.num_layers)
    }

    pub fn add_metadata(&mut self, key: impl Into<String>, value: serde_json::Value) {
        self.metadata.insert(key.into(), value);
    }
}

impl Sensor for Lidar3D {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::Lidar3D
    }

    fn frame_id(&self) -> &str {
        &self.frame_id
    }

    fn topic(&self) -> &str {
        &self.topic
    }

    fn enabled(&self) -> bool {
        self.enabled
    }

    fn metadata(&self) -> &Metadata {
        &self.metadata
    }
}
