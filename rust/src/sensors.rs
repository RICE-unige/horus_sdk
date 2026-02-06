use crate::core::types::{Metadata, SensorType};
use std::fmt::Debug;

/// Base sensor trait for all sensor types
pub trait Sensor: Debug + Send + Sync {
    fn get_name(&self) -> &str;
    fn get_sensor_type(&self) -> SensorType;
    fn get_frame_id(&self) -> &str;
    fn get_topic(&self) -> &str;
    fn is_enabled(&self) -> bool;
    fn enable(&mut self);
    fn disable(&mut self);
}

/// Camera sensor with vision-specific properties
#[derive(Debug, Clone)]
pub struct Camera {
    pub name: String,
    pub frame_id: String,
    pub topic: String,
    pub enabled: bool,
    pub is_stereo: bool,
    pub resolution: (u32, u32),
    pub fps: u32,
    pub fov: f32,
    pub encoding: String,
    pub metadata: Metadata,
}

impl Camera {
    pub fn new(
        name: impl Into<String>,
        frame_id: impl Into<String>,
        topic: impl Into<String>,
    ) -> Self {
        Self {
            name: name.into(),
            frame_id: frame_id.into(),
            topic: topic.into(),
            enabled: true,
            is_stereo: false,
            resolution: (640, 480),
            fps: 30,
            fov: 60.0,
            encoding: "bgr8".to_string(),
            metadata: Metadata::new(),
        }
    }
}

impl Sensor for Camera {
    fn get_name(&self) -> &str {
        &self.name
    }
    fn get_sensor_type(&self) -> SensorType {
        SensorType::Camera
    }
    fn get_frame_id(&self) -> &str {
        &self.frame_id
    }
    fn get_topic(&self) -> &str {
        &self.topic
    }
    fn is_enabled(&self) -> bool {
        self.enabled
    }
    fn enable(&mut self) {
        self.enabled = true;
    }
    fn disable(&mut self) {
        self.enabled = false;
    }
}

/// 2D Laser scanner sensor
#[derive(Debug, Clone)]
pub struct LaserScan {
    pub name: String,
    pub frame_id: String,
    pub topic: String,
    pub enabled: bool,
    pub min_angle: f32,
    pub max_angle: f32,
    pub angle_increment: f32,
    pub min_range: f32,
    pub max_range: f32,
    pub color: String,
    pub point_size: f32,
    pub metadata: Metadata,
}

impl LaserScan {
    pub fn new(
        name: impl Into<String>,
        frame_id: impl Into<String>,
        topic: impl Into<String>,
    ) -> Self {
        Self {
            name: name.into(),
            frame_id: frame_id.into(),
            topic: topic.into(),
            enabled: true,
            min_angle: -std::f32::consts::PI,
            max_angle: std::f32::consts::PI,
            angle_increment: 0.005,
            min_range: 0.1,
            max_range: 30.0,
            color: "red".to_string(),
            point_size: 0.05,
            metadata: Metadata::new(),
        }
    }
}

impl Sensor for LaserScan {
    fn get_name(&self) -> &str {
        &self.name
    }
    fn get_sensor_type(&self) -> SensorType {
        SensorType::LaserScan
    }
    fn get_frame_id(&self) -> &str {
        &self.frame_id
    }
    fn get_topic(&self) -> &str {
        &self.topic
    }
    fn is_enabled(&self) -> bool {
        self.enabled
    }
    fn enable(&mut self) {
        self.enabled = true;
    }
    fn disable(&mut self) {
        self.enabled = false;
    }
}

/// 3D LiDAR sensor
#[derive(Debug, Clone)]
pub struct Lidar3D {
    pub name: String,
    pub frame_id: String,
    pub topic: String,
    pub enabled: bool,
    pub vertical_fov: f32,
    pub horizontal_fov: f32,
    pub num_layers: u32,
    pub metadata: Metadata,
}

impl Lidar3D {
    pub fn new(
        name: impl Into<String>,
        frame_id: impl Into<String>,
        topic: impl Into<String>,
    ) -> Self {
        Self {
            name: name.into(),
            frame_id: frame_id.into(),
            topic: topic.into(),
            enabled: true,
            vertical_fov: 40.0,
            horizontal_fov: 360.0,
            num_layers: 64,
            metadata: Metadata::new(),
        }
    }
}

impl Sensor for Lidar3D {
    fn get_name(&self) -> &str {
        &self.name
    }
    fn get_sensor_type(&self) -> SensorType {
        SensorType::Lidar3D
    }
    fn get_frame_id(&self) -> &str {
        &self.frame_id
    }
    fn get_topic(&self) -> &str {
        &self.topic
    }
    fn is_enabled(&self) -> bool {
        self.enabled
    }
    fn enable(&mut self) {
        self.enabled = true;
    }
    fn disable(&mut self) {
        self.enabled = false;
    }
}
