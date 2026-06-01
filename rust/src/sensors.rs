use crate::core::types::{Metadata, SensorType};
use std::any::Any;
use std::fmt::Debug;
use std::sync::Arc;

pub type SensorRef = Arc<dyn Sensor>;

#[derive(Debug, Clone, Default)]
pub struct ProjectedViewConfig {
    pub position_offset: Option<(f32, f32, f32)>,
    pub rotation_offset: Option<(f32, f32, f32)>,
    pub scale_multiplier: Option<(f32, f32, f32)>,
    pub image_scale: Option<f32>,
    pub focal_length_scale: Option<f32>,
    pub projection_target_frame: Option<String>,
    pub show_frustum: Option<bool>,
    pub frustum_color: Option<String>,
}

#[derive(Debug, Clone, Default)]
pub struct MinimapViewConfig {
    pub size: Option<f32>,
    pub position_offset: Option<(f32, f32, f32)>,
    pub face_camera: Option<bool>,
    pub rotation_offset: Option<(f32, f32, f32)>,
}

#[derive(Debug, Clone, Default)]
pub struct ImmersiveViewConfig {
    pub ros_flip_x: Option<bool>,
    pub ros_flip_y: Option<bool>,
}

#[derive(Debug, Clone)]
pub struct WebRtcTransportConfig {
    pub client_signal_topic: String,
    pub server_signal_topic: String,
    pub bitrate_kbps: Option<i64>,
    pub framerate: Option<i64>,
    pub stun_server_url: Option<String>,
    pub turn_server_url: Option<String>,
    pub turn_username: Option<String>,
    pub turn_credential: Option<String>,
}

impl Default for WebRtcTransportConfig {
    fn default() -> Self {
        Self {
            client_signal_topic: "/horus/webrtc/client_signal".to_string(),
            server_signal_topic: "/horus/webrtc/server_signal".to_string(),
            bitrate_kbps: None,
            framerate: None,
            stun_server_url: None,
            turn_server_url: None,
            turn_username: None,
            turn_credential: None,
        }
    }
}

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
    pub stereo_layout: String,
    pub right_topic: String,
    pub minimap_topic: String,
    pub teleop_topic: String,
    pub minimap_image_type: String,
    pub teleop_image_type: String,
    pub minimap_max_fps: u32,
    pub teleop_stereo_layout: String,
    pub teleop_right_topic: String,
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
            stereo_layout: "mono".to_string(),
            right_topic: String::new(),
            minimap_topic: String::new(),
            teleop_topic: String::new(),
            minimap_image_type: String::new(),
            teleop_image_type: String::new(),
            minimap_max_fps: 30,
            teleop_stereo_layout: String::new(),
            teleop_right_topic: String::new(),
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

    pub fn configure_projected_view(&mut self, config: ProjectedViewConfig) {
        self.metadata.insert("display_mode".to_string(), serde_json::json!("projected"));
        put_vec3(&mut self.metadata, "projected_position_offset", config.position_offset);
        put_vec3(&mut self.metadata, "view_rotation_offset", config.rotation_offset);
        put_vec3(&mut self.metadata, "projected_scale_multiplier", config.scale_multiplier);
        put_if_some(&mut self.metadata, "image_scale", config.image_scale);
        put_if_some(&mut self.metadata, "focal_length_scale", config.focal_length_scale);
        put_if_some(&mut self.metadata, "projection_target_frame", config.projection_target_frame);
        put_if_some(&mut self.metadata, "show_frustum", config.show_frustum);
        put_if_some(&mut self.metadata, "frustum_color", config.frustum_color);
    }

    pub fn configure_minimap_view(&mut self, config: MinimapViewConfig) {
        put_if_some(&mut self.metadata, "overhead_size", config.size);
        put_vec3(&mut self.metadata, "overhead_position_offset", config.position_offset);
        put_if_some(&mut self.metadata, "overhead_face_camera", config.face_camera);
        put_vec3(&mut self.metadata, "overhead_rotation_offset", config.rotation_offset);
    }

    pub fn configure_immersive_view(&mut self, config: ImmersiveViewConfig) {
        put_if_some(&mut self.metadata, "immersive_ros_flip_x", config.ros_flip_x);
        put_if_some(&mut self.metadata, "immersive_ros_flip_y", config.ros_flip_y);
    }

    pub fn configure_webrtc_transport(&mut self, config: WebRtcTransportConfig) {
        self.metadata.insert(
            "webrtc_client_signal_topic".to_string(),
            serde_json::json!(config.client_signal_topic),
        );
        self.metadata.insert(
            "webrtc_server_signal_topic".to_string(),
            serde_json::json!(config.server_signal_topic),
        );
        put_if_some(&mut self.metadata, "webrtc_bitrate_kbps", config.bitrate_kbps);
        put_if_some(&mut self.metadata, "webrtc_framerate", config.framerate);
        put_if_some(&mut self.metadata, "webrtc_stun_server_url", config.stun_server_url);
        put_if_some(&mut self.metadata, "webrtc_turn_server_url", config.turn_server_url);
        put_if_some(&mut self.metadata, "webrtc_turn_username", config.turn_username);
        put_if_some(&mut self.metadata, "webrtc_turn_credential", config.turn_credential);
    }
}

fn put_if_some<T: serde::Serialize>(metadata: &mut Metadata, key: &str, value: Option<T>) {
    if let Some(value) = value {
        metadata.insert(key.to_string(), serde_json::json!(value));
    }
}

fn put_vec3(metadata: &mut Metadata, key: &str, value: Option<(f32, f32, f32)>) {
    if let Some((x, y, z)) = value {
        metadata.insert(key.to_string(), serde_json::json!({"x": x, "y": y, "z": z}));
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
