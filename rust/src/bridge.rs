use crate::dataviz::{DataSource, DataViz, VisualizationConfig};
use crate::robot::Robot;
use crate::sensors::{Camera, LaserScan};
use once_cell::sync::Lazy;
use regex::Regex;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::HashMap;
use std::fs;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Vector3Payload {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ResolutionPayload {
    pub width: u32,
    pub height: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraConfigPayload {
    pub streaming_type: String,
    pub minimap_streaming_type: String,
    pub teleop_streaming_type: String,
    pub minimap_topic: String,
    pub teleop_topic: String,
    pub minimap_image_type: String,
    pub teleop_image_type: String,
    pub minimap_max_fps: i32,
    pub teleop_stereo_layout: String,
    pub teleop_right_topic: String,
    pub startup_mode: String,
    pub is_stereo: bool,
    pub stereo_layout: String,
    pub right_topic: String,
    pub image_type: String,
    pub display_mode: String,
    pub use_tf: bool,
    pub projection_target_frame: String,
    pub webrtc_client_signal_topic: String,
    pub webrtc_server_signal_topic: String,
    pub webrtc_bitrate_kbps: i32,
    pub webrtc_framerate: i32,
    pub webrtc_stun_server_url: String,
    pub webrtc_turn_server_url: String,
    pub webrtc_turn_username: String,
    pub webrtc_turn_credential: String,
    pub image_scale: f32,
    pub focal_length_scale: f32,
    pub immersive_ros_flip_x: bool,
    pub immersive_ros_flip_y: bool,
    pub view_position_offset: Vector3Payload,
    pub view_rotation_offset: Vector3Payload,
    pub projected_position_offset: Vector3Payload,
    pub projected_scale_multiplier: Vector3Payload,
    pub show_frustum: bool,
    pub frustum_color: String,
    pub overhead_size: f32,
    pub overhead_position_offset: Vector3Payload,
    pub overhead_face_camera: bool,
    pub overhead_rotation_offset: Vector3Payload,
    pub resolution: ResolutionPayload,
    pub fps: i32,
    pub fov: f32,
    pub encoding: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VizConfigPayload {
    pub color: String,
    pub point_size: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorPayload {
    pub name: String,
    #[serde(rename = "type")]
    pub sensor_type: String,
    pub topic: String,
    pub frame: String,
    pub metadata: HashMap<String, Value>,
    pub viz_config: VizConfigPayload,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub camera_config: Option<CameraConfigPayload>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct OccupancyPayload {
    #[serde(default = "default_true")]
    pub show_unknown_space: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position_scale: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position_offset: Option<Vector3Payload>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rotation_offset_euler: Option<Vector3Payload>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisualizationPayload {
    #[serde(rename = "type")]
    pub viz_type: String,
    pub topic: String,
    pub scope: String,
    pub enabled: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frame: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub occupancy: Option<OccupancyPayload>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub velocity: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trail: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub collision: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub point_cloud: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gaussian_splat: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mesh: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub octomap: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub semantic_box: Option<Value>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotManagerSectionsPayload {
    pub status: bool,
    pub data_viz: bool,
    pub teleop: bool,
    pub tasks: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotManagerConfigPayload {
    pub enabled: bool,
    pub prefab_asset_path: String,
    pub prefab_resource_path: String,
    pub sections: RobotManagerSectionsPayload,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DimensionsPayload {
    pub length: f32,
    pub width: f32,
    pub height: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkspaceConfigPayload {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position_scale: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub compass: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tutorial: Option<Value>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlPayload {
    pub drive_topic: String,
    pub teleop: TeleopControlPayload,
    pub tasks: TaskControlPayload,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TeleopControlPayload {
    pub enabled: bool,
    pub command_topic: String,
    pub raw_input_topic: String,
    pub head_pose_topic: String,
    pub robot_profile: String,
    pub response_mode: String,
    pub publish_rate_hz: f64,
    pub custom_passthrough_only: bool,
    pub deadman: Value,
    pub axes: Value,
    pub discrete: Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TaskControlPayload {
    pub go_to_point: Value,
    pub waypoint: Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RosBindingPayload {
    pub logical_name: String,
    pub tf_mode: String,
    pub topic_mode: String,
    pub base_frame: String,
    pub tf_prefix: String,
    pub topic_prefix: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotRegistrationPayload {
    pub action: String,
    pub robot_name: String,
    pub robot_type: String,
    pub ros_binding: RosBindingPayload,
    pub sensors: Vec<SensorPayload>,
    pub visualizations: Vec<VisualizationPayload>,
    pub global_visualizations: Vec<VisualizationPayload>,
    pub control: ControlPayload,
    pub robot_manager_config: RobotManagerConfigPayload,
    pub timestamp: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dimensions: Option<DimensionsPayload>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub workspace_config: Option<WorkspaceConfigPayload>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robot_model_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub has_visual_mesh_model: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robot_description_manifest: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub robot_description_payload_json: Option<String>,
}

fn now_sec() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}

fn default_true() -> bool {
    true
}

fn normalize_transport(value: Option<&str>, fallback: &str) -> String {
    let normalized = value.unwrap_or("").trim().to_ascii_lowercase();
    if normalized == "ros" || normalized == "webrtc" {
        normalized
    } else {
        fallback.to_string()
    }
}

fn normalize_image_type(value: Option<&str>, fallback: &str) -> String {
    let normalized = value.unwrap_or("").trim().to_ascii_lowercase();
    if normalized == "raw" || normalized == "compressed" {
        normalized
    } else {
        fallback.to_string()
    }
}

fn normalize_layout(value: Option<&str>, fallback: &str) -> String {
    match value.unwrap_or("").trim().to_ascii_lowercase().as_str() {
        "sbs" | "side_by_side" | "side-by-side" => "side_by_side".to_string(),
        "dual_topic" | "dual" | "two_topics" | "two_topic" => "dual_topic".to_string(),
        "mono" => "mono".to_string(),
        "" => fallback.to_string(),
        _ => fallback.to_string(),
    }
}

fn coerce_text(value: Option<&Value>, default: &str) -> String {
    value
        .and_then(Value::as_str)
        .map(str::trim)
        .filter(|text| !text.is_empty())
        .unwrap_or(default)
        .to_string()
}

fn clamp_f32(value: f32, min: f32, max: f32) -> f32 {
    value.max(min).min(max)
}

fn clamp_i32(value: i32, min: i32, max: i32) -> i32 {
    value.max(min).min(max)
}

fn coerce_bool(value: Option<&Value>, default: bool) -> bool {
    match value {
        Some(Value::Bool(b)) => *b,
        Some(Value::String(s)) => match s.trim().to_ascii_lowercase().as_str() {
            "1" | "true" | "yes" | "on" => true,
            "0" | "false" | "no" | "off" => false,
            _ => default,
        },
        Some(Value::Number(n)) => n.as_f64().map(|v| v != 0.0).unwrap_or(default),
        _ => default,
    }
}

fn coerce_float(value: Option<&Value>, default: f32) -> f32 {
    match value {
        Some(Value::Number(n)) => n.as_f64().map(|v| v as f32).unwrap_or(default),
        Some(Value::String(s)) => s.trim().parse::<f32>().unwrap_or(default),
        Some(Value::Bool(v)) => {
            if *v {
                1.0
            } else {
                0.0
            }
        }
        _ => default,
    }
}

fn coerce_f64(value: Option<&Value>, default: f64) -> f64 {
    match value {
        Some(Value::Number(n)) => n.as_f64().unwrap_or(default),
        Some(Value::String(s)) => s.trim().parse::<f64>().unwrap_or(default),
        Some(Value::Bool(v)) => {
            if *v {
                1.0
            } else {
                0.0
            }
        }
        _ => default,
    }
}

fn clamp_f64(value: f64, min: f64, max: f64) -> f64 {
    value.max(min).min(max)
}

fn coerce_int(value: Option<&Value>, default: i32) -> i32 {
    match value {
        Some(Value::Number(n)) => n.as_i64().map(|v| v as i32).unwrap_or(default),
        Some(Value::String(s)) => s.trim().parse::<i32>().unwrap_or(default),
        Some(Value::Bool(v)) => {
            if *v {
                1
            } else {
                0
            }
        }
        _ => default,
    }
}

fn coerce_vec3(value: Option<&Value>, default_xyz: (f32, f32, f32)) -> Vector3Payload {
    let (dx, dy, dz) = default_xyz;
    let Some(value) = value else {
        return Vector3Payload {
            x: dx,
            y: dy,
            z: dz,
        };
    };
    if let Some(obj) = value.as_object() {
        return Vector3Payload {
            x: coerce_float(obj.get("x"), dx),
            y: coerce_float(obj.get("y"), dy),
            z: coerce_float(obj.get("z"), dz),
        };
    }
    if let Some(arr) = value.as_array() {
        if arr.len() == 3 {
            return Vector3Payload {
                x: coerce_float(arr.first(), dx),
                y: coerce_float(arr.get(1), dy),
                z: coerce_float(arr.get(2), dz),
            };
        }
    }
    Vector3Payload {
        x: dx,
        y: dy,
        z: dz,
    }
}

fn build_camera_config(sensor: &Camera) -> CameraConfigPayload {
    let metadata = &sensor.metadata;
    let legacy_streaming_type = normalize_transport(
        metadata
            .get("streaming_type")
            .and_then(Value::as_str)
            .or(Some(sensor.streaming_type.as_str())),
        "ros",
    );
    let minimap_streaming_type = normalize_transport(
        metadata
            .get("minimap_streaming_type")
            .and_then(Value::as_str)
            .or(Some(sensor.minimap_streaming_type.as_str())),
        &legacy_streaming_type,
    );
    let teleop_streaming_type = normalize_transport(
        metadata
            .get("teleop_streaming_type")
            .and_then(Value::as_str)
            .or(Some(sensor.teleop_streaming_type.as_str())),
        &legacy_streaming_type,
    );
    let startup_mode = metadata
        .get("startup_mode")
        .and_then(Value::as_str)
        .unwrap_or(sensor.startup_mode.as_str())
        .trim()
        .to_ascii_lowercase();
    let startup_mode = if startup_mode == "teleop" {
        startup_mode
    } else {
        "minimap".to_string()
    };

    let is_stereo = sensor.is_stereo;
    let raw_stereo_layout = metadata
        .get("stereo_layout")
        .and_then(Value::as_str)
        .unwrap_or(sensor.stereo_layout.as_str());
    let stereo_layout = if is_stereo {
        let normalized = normalize_layout(Some(raw_stereo_layout), "side_by_side");
        if normalized == "dual_topic" {
            "dual_topic".to_string()
        } else {
            "side_by_side".to_string()
        }
    } else {
        "mono".to_string()
    };
    let right_topic = coerce_text(metadata.get("right_topic"), &sensor.right_topic);
    let minimap_topic = coerce_text(metadata.get("minimap_topic"), &sensor.minimap_topic);
    let minimap_topic = if minimap_topic.trim().is_empty() {
        sensor.topic.clone()
    } else {
        minimap_topic
    };
    let teleop_topic = coerce_text(metadata.get("teleop_topic"), &sensor.teleop_topic);
    let teleop_topic = if teleop_topic.trim().is_empty() {
        minimap_topic.clone()
    } else {
        teleop_topic
    };
    let image_type =
        normalize_image_type(metadata.get("image_type").and_then(Value::as_str), "raw");
    let minimap_image_type = normalize_image_type(
        metadata
            .get("minimap_image_type")
            .and_then(Value::as_str)
            .or_else(|| Some(sensor.minimap_image_type.as_str())),
        &image_type,
    );
    let teleop_image_type = normalize_image_type(
        metadata
            .get("teleop_image_type")
            .and_then(Value::as_str)
            .or_else(|| Some(sensor.teleop_image_type.as_str())),
        &image_type,
    );
    let minimap_max_fps = clamp_i32(
        coerce_int(
            metadata.get("minimap_max_fps"),
            sensor.minimap_max_fps as i32,
        ),
        1,
        30,
    );
    let mut teleop_stereo_layout = normalize_layout(
        metadata
            .get("teleop_stereo_layout")
            .and_then(Value::as_str)
            .or_else(|| Some(sensor.teleop_stereo_layout.as_str())),
        &stereo_layout,
    );
    let mut teleop_right_topic = coerce_text(
        metadata.get("teleop_right_topic"),
        &sensor.teleop_right_topic,
    );
    if teleop_right_topic.trim().is_empty() {
        teleop_right_topic = right_topic.clone();
    }
    if !is_stereo {
        teleop_stereo_layout = "mono".to_string();
        teleop_right_topic.clear();
    }

    CameraConfigPayload {
        streaming_type: legacy_streaming_type,
        minimap_streaming_type,
        teleop_streaming_type,
        minimap_topic,
        teleop_topic,
        minimap_image_type,
        teleop_image_type,
        minimap_max_fps,
        teleop_stereo_layout,
        teleop_right_topic,
        startup_mode,
        is_stereo,
        stereo_layout,
        right_topic,
        image_type,
        display_mode: metadata
            .get("display_mode")
            .and_then(Value::as_str)
            .unwrap_or("projected")
            .to_ascii_lowercase(),
        use_tf: coerce_bool(metadata.get("use_tf"), true),
        projection_target_frame: metadata
            .get("projection_target_frame")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_string(),
        webrtc_client_signal_topic: metadata
            .get("webrtc_client_signal_topic")
            .and_then(Value::as_str)
            .unwrap_or("/horus/webrtc/client_signal")
            .to_string(),
        webrtc_server_signal_topic: metadata
            .get("webrtc_server_signal_topic")
            .and_then(Value::as_str)
            .unwrap_or("/horus/webrtc/server_signal")
            .to_string(),
        webrtc_bitrate_kbps: coerce_int(metadata.get("webrtc_bitrate_kbps"), 2000),
        webrtc_framerate: coerce_int(metadata.get("webrtc_framerate"), sensor.fps as i32),
        webrtc_stun_server_url: metadata
            .get("webrtc_stun_server_url")
            .and_then(Value::as_str)
            .unwrap_or("stun:stun.l.google.com:19302")
            .to_string(),
        webrtc_turn_server_url: metadata
            .get("webrtc_turn_server_url")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_string(),
        webrtc_turn_username: metadata
            .get("webrtc_turn_username")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_string(),
        webrtc_turn_credential: metadata
            .get("webrtc_turn_credential")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_string(),
        image_scale: coerce_float(metadata.get("image_scale"), 1.0),
        focal_length_scale: coerce_float(metadata.get("focal_length_scale"), 0.5),
        immersive_ros_flip_x: coerce_bool(metadata.get("immersive_ros_flip_x"), false),
        immersive_ros_flip_y: coerce_bool(metadata.get("immersive_ros_flip_y"), false),
        view_position_offset: coerce_vec3(metadata.get("view_position_offset"), (0.0, 0.0, 0.0)),
        view_rotation_offset: coerce_vec3(metadata.get("view_rotation_offset"), (0.0, 0.0, 0.0)),
        projected_position_offset: coerce_vec3(
            metadata.get("projected_position_offset"),
            (0.0, 0.0, 0.0),
        ),
        projected_scale_multiplier: coerce_vec3(
            metadata.get("projected_scale_multiplier"),
            (1.0, 1.0, 1.0),
        ),
        show_frustum: coerce_bool(metadata.get("show_frustum"), true),
        frustum_color: metadata
            .get("frustum_color")
            .and_then(Value::as_str)
            .unwrap_or("#FFFF00")
            .to_string(),
        overhead_size: coerce_float(metadata.get("overhead_size"), 1.0),
        overhead_position_offset: coerce_vec3(
            metadata.get("overhead_position_offset"),
            (0.0, 2.0, 0.0),
        ),
        overhead_face_camera: coerce_bool(metadata.get("overhead_face_camera"), true),
        overhead_rotation_offset: coerce_vec3(
            metadata.get("overhead_rotation_offset"),
            (90.0, 0.0, 0.0),
        ),
        resolution: ResolutionPayload {
            width: sensor.resolution.0,
            height: sensor.resolution.1,
        },
        fps: sensor.fps as i32,
        fov: sensor.fov,
        encoding: sensor.encoding.clone(),
    }
}

fn serialize_visualization_payload(
    visualization: &VisualizationConfig,
    scope_override: Option<&str>,
) -> Option<VisualizationPayload> {
    let data_source: &DataSource = &visualization.data_source;
    if data_source.topic.trim().is_empty() {
        return None;
    }

    let scope = scope_override.map(str::to_string).unwrap_or_else(|| {
        if visualization.is_robot_specific() {
            "robot"
        } else {
            "global"
        }
        .to_string()
    });

    let frame = if data_source.frame_id.trim().is_empty() {
        None
    } else {
        Some(data_source.frame_id.clone())
    };

    let mut occupancy = None;
    let mut path = None;
    let mut velocity = None;
    let mut trail = None;
    let mut collision = None;
    let mut point_cloud = None;
    let mut gaussian_splat = None;
    let mut mesh = None;
    let mut octomap = None;
    let mut semantic_box = None;
    if visualization.viz_type.as_str() == "path" {
        let mut payload = serde_json::Map::new();
        match data_source.source_type.as_str() {
            "robot_global_path" => {
                payload.insert("role".to_string(), json!("global"));
            }
            "robot_local_path" => {
                payload.insert("role".to_string(), json!("local"));
            }
            "robot_trajectory" => {
                payload.insert("role".to_string(), json!("trajectory"));
            }
            _ => {}
        }
        if let Some(value) = visualization.render_options.get("line_width") {
            payload.insert(
                "line_width".to_string(),
                json!(coerce_float(Some(value), 1.0)),
            );
        }
        if let Some(value) = visualization
            .render_options
            .get("line_style")
            .and_then(Value::as_str)
        {
            if !value.trim().is_empty() {
                payload.insert("line_style".to_string(), json!(value));
            }
        }
        if !payload.is_empty() {
            path = Some(Value::Object(payload));
        }
    }
    if visualization.viz_type.as_str() == "velocity_data" {
        velocity = Some(json!({
            "units": coerce_text(visualization.render_options.get("units"), "m/s"),
            "text_back_offset_m": coerce_float(visualization.render_options.get("text_back_offset_m"), 0.36),
            "floor_offset_m": coerce_float(visualization.render_options.get("floor_offset_m"), 0.01),
            "update_hz": coerce_float(visualization.render_options.get("update_hz"), 10.0),
        }));
    }
    if visualization.viz_type.as_str() == "odometry_trail" {
        trail = Some(json!({
            "max_points": coerce_float(visualization.render_options.get("max_points"), 48.0).round() as i32,
            "history_seconds": coerce_float(visualization.render_options.get("history_seconds"), 3.2),
            "min_spacing_m": coerce_float(visualization.render_options.get("min_spacing_m"), 0.07),
            "line_width_m": coerce_float(visualization.render_options.get("line_width_m"), 0.0048),
            "trail_back_offset_m": coerce_float(visualization.render_options.get("trail_back_offset_m"), 0.44),
        }));
    }
    if visualization.viz_type.as_str() == "collision_risk" {
        collision = Some(json!({
            "threshold_m": coerce_float(visualization.render_options.get("threshold_m"), 1.2),
            "radius_m": coerce_float(visualization.render_options.get("radius_m"), 1.2),
            "source": coerce_text(visualization.render_options.get("source"), "laser_scan"),
            "alpha_min": coerce_float(visualization.render_options.get("alpha_min"), 0.0),
            "alpha_max": coerce_float(visualization.render_options.get("alpha_max"), 0.55),
        }));
    }
    if visualization.viz_type.as_str() == "occupancy_grid" {
        let options = &visualization.render_options;
        let mut occ = OccupancyPayload::default();
        let mut any = false;
        if options.contains_key("show_unknown_space") {
            occ.show_unknown_space = coerce_bool(options.get("show_unknown_space"), true);
            any = true;
        }
        if options.contains_key("position_scale") {
            occ.position_scale = Some(coerce_float(options.get("position_scale"), 1.0));
            any = true;
        }
        if options.contains_key("position_offset") {
            occ.position_offset =
                Some(coerce_vec3(options.get("position_offset"), (0.0, 0.0, 0.0)));
            any = true;
        }
        if options.contains_key("rotation_offset_euler") {
            occ.rotation_offset_euler = Some(coerce_vec3(
                options.get("rotation_offset_euler"),
                (0.0, 0.0, 0.0),
            ));
            any = true;
        }
        if any {
            occupancy = Some(occ);
        }
    }
    if visualization.viz_type.as_str() == "point_cloud" {
        let options = &visualization.render_options;
        let min_point_size = coerce_float(options.get("min_point_size"), 0.002).max(0.0001);
        let max_point_size = coerce_float(options.get("max_point_size"), 0.04).max(min_point_size);
        let max_visible = coerce_int(options.get("max_visible_points_budget"), 200_000).max(1_000);
        let visible = clamp_i32(
            coerce_int(options.get("visible_points_budget"), 120_000),
            1_000,
            max_visible,
        );
        let render_mode = match coerce_text(options.get("render_mode"), "opaque_fast")
            .to_ascii_lowercase()
            .as_str()
        {
            "transparent_hq" => "transparent_hq",
            _ => "opaque_fast",
        };
        let point_shape = match coerce_text(options.get("point_shape"), "square")
            .to_ascii_lowercase()
            .as_str()
        {
            "disc" | "circle" => "circle",
            _ => "square",
        };
        let max_points_raw = coerce_int(options.get("max_points_per_frame"), 0);
        point_cloud = Some(json!({
            "point_size": coerce_float(options.get("point_size"), 0.05).max(0.001),
            "max_points_per_frame": if max_points_raw <= 0 { 0 } else { max_points_raw.max(1) },
            "base_sample_stride": coerce_int(options.get("base_sample_stride"), 1).max(1),
            "min_update_interval": coerce_float(options.get("min_update_interval"), 0.0).max(0.0),
            "enable_adaptive_quality": coerce_bool(options.get("enable_adaptive_quality"), false),
            "target_framerate": coerce_float(options.get("target_framerate"), 72.0).max(30.0),
            "min_quality_multiplier": clamp_f32(coerce_float(options.get("min_quality_multiplier"), 0.6), 0.25, 1.0),
            "min_distance": coerce_float(options.get("min_distance"), 0.0).max(0.0),
            "max_distance": coerce_float(options.get("max_distance"), 0.0).max(0.0),
            "replace_latest": coerce_bool(options.get("replace_latest"), true),
            "render_all_points": coerce_bool(options.get("render_all_points"), true),
            "auto_point_size_by_workspace_scale": coerce_bool(options.get("auto_point_size_by_workspace_scale"), true),
            "min_point_size": min_point_size,
            "max_point_size": max_point_size,
            "render_mode": render_mode,
            "point_shape": point_shape,
            "enable_view_frustum_culling": coerce_bool(options.get("enable_view_frustum_culling"), true),
            "frustum_padding": clamp_f32(coerce_float(options.get("frustum_padding"), 0.03), 0.0, 0.5),
            "enable_subpixel_culling": coerce_bool(options.get("enable_subpixel_culling"), true),
            "min_screen_radius_px": coerce_float(options.get("min_screen_radius_px"), 0.8).max(0.0),
            "visible_points_budget": visible,
            "max_visible_points_budget": max_visible,
            "map_static_mode": coerce_bool(options.get("map_static_mode"), true),
        }));
    }
    if visualization.viz_type.as_str() == "gaussian_splat" {
        let options = &visualization.render_options;
        let asset_format = match coerce_text(options.get("asset_format"), "3dgs_ply")
            .to_ascii_lowercase()
            .as_str()
        {
            "3dgs_ply" => "3dgs_ply",
            _ => "3dgs_ply",
        };
        let coordinate_space = match coerce_text(options.get("source_coordinate_space"), "colmap")
            .to_ascii_lowercase()
            .as_str()
        {
            "3dgs" => "3dgs",
            "opencv" => "opencv",
            "unity" => "unity",
            _ => "colmap",
        };
        let render_mode = match coerce_text(options.get("render_mode"), "splats")
            .to_ascii_lowercase()
            .as_str()
        {
            "debug_points" => "debug_points",
            "mono_center_eye" => "mono_center_eye",
            "no_covariance" => "no_covariance",
            _ => "splats",
        };
        let mut payload = json!({
            "manifest_topic": coerce_text(options.get("manifest_topic"), &data_source.topic),
            "chunk_begin_topic": coerce_text(options.get("chunk_begin_topic"), "/horus/gaussian_splat/chunk_begin"),
            "chunk_item_topic": coerce_text(options.get("chunk_item_topic"), "/horus/gaussian_splat/chunk_item"),
            "chunk_end_topic": coerce_text(options.get("chunk_end_topic"), "/horus/gaussian_splat/chunk_end"),
            "asset_format": asset_format,
            "source_coordinate_space": coordinate_space,
            "render_mode": render_mode,
            "max_splats": coerce_int(options.get("max_splats"), 350_000).max(1_000),
            "render_scale": clamp_f32(coerce_float(options.get("render_scale"), 0.5), 0.25, 1.0),
            "sh_order": clamp_i32(coerce_int(options.get("sh_order"), 2), 0, 3),
            "half_precision_sh": coerce_bool(options.get("half_precision_sh"), true),
            "adaptive_sort": coerce_bool(options.get("adaptive_sort"), true),
            "sort_passes": clamp_i32(coerce_int(options.get("sort_passes"), 2), 2, 4),
            "opacity_scale": clamp_f32(coerce_float(options.get("opacity_scale"), 1.0), 0.05, 20.0),
            "splat_scale": clamp_f32(coerce_float(options.get("splat_scale"), 1.0), 0.05, 4.0),
            "contribution_cull_threshold": clamp_f32(coerce_float(options.get("contribution_cull_threshold"), 0.1), 0.0, 1.0),
            "high_precision_rt": coerce_bool(options.get("high_precision_rt"), false),
            "pointcloud_fallback": coerce_bool(options.get("pointcloud_fallback"), true),
            "fallback_topic": coerce_text(options.get("fallback_topic"), "/map_gaussian_splat_preview"),
            "fallback_frame": coerce_text(options.get("fallback_frame"), frame.as_deref().unwrap_or("map")),
        });
        if let Some(fallback) = options.get("fallback_point_cloud") {
            payload["fallback_point_cloud"] = fallback.clone();
        }
        gaussian_splat = Some(payload);
    }
    if visualization.viz_type.as_str() == "mesh" {
        let options = &visualization.render_options;
        let coordinate_space = match coerce_text(options.get("source_coordinate_space"), "enu")
            .to_ascii_lowercase()
            .as_str()
        {
            "optical" => "optical",
            _ => "enu",
        };
        mesh = Some(json!({
            "use_vertex_colors": coerce_bool(options.get("use_vertex_colors"), true),
            "alpha": clamp_f32(coerce_float(options.get("alpha"), 1.0), 0.0, 1.0),
            "double_sided": coerce_bool(options.get("double_sided"), true),
            "max_triangles": coerce_int(options.get("max_triangles"), 200_000).max(1_000),
            "source_coordinate_space": coordinate_space,
        }));
    }
    if visualization.viz_type.as_str() == "octomap" {
        let options = &visualization.render_options;
        let coordinate_space = match coerce_text(options.get("source_coordinate_space"), "enu")
            .to_ascii_lowercase()
            .as_str()
        {
            "optical" => "optical",
            _ => "enu",
        };
        let render_mode = match coerce_text(options.get("render_mode"), "surface_mesh")
            .to_ascii_lowercase()
            .as_str()
        {
            "voxel_cubes" => "voxel_cubes",
            _ => "surface_mesh",
        };
        octomap = Some(json!({
            "render_mode": render_mode,
            "use_vertex_colors": coerce_bool(options.get("use_vertex_colors"), true),
            "alpha": clamp_f32(coerce_float(options.get("alpha"), 1.0), 0.0, 1.0),
            "double_sided": coerce_bool(options.get("double_sided"), false),
            "max_triangles": coerce_int(options.get("max_triangles"), 60_000).max(1_000),
            "source_coordinate_space": coordinate_space,
            "native_topic": coerce_text(options.get("native_topic"), "/map_3d_octomap"),
            "native_frame": coerce_text(options.get("native_frame"), "map"),
            "native_binary_only": coerce_bool(options.get("native_binary_only"), true),
        }));
    }
    if visualization.viz_type.as_str() == "semantic_box" {
        let id = coerce_text(visualization.render_options.get("id"), "");
        let label = coerce_text(visualization.render_options.get("label"), "");
        if !id.is_empty() && !label.is_empty() {
            semantic_box = Some(json!({
                "id": id,
                "label": label,
                "center": coerce_vec3(visualization.render_options.get("center"), (0.0, 0.0, 0.0)),
                "size": coerce_vec3(visualization.render_options.get("size"), (1.0, 1.0, 1.0)),
                "rotation_offset_euler": coerce_vec3(
                    visualization.render_options.get("rotation_offset_euler"),
                    (0.0, 0.0, 0.0),
                ),
            }));
        }
    }

    Some(VisualizationPayload {
        viz_type: visualization.viz_type.as_str().to_string(),
        topic: data_source.topic.clone(),
        scope,
        enabled: visualization.enabled,
        frame,
        color: visualization
            .render_options
            .get("color")
            .and_then(Value::as_str)
            .map(ToString::to_string),
        occupancy,
        path,
        velocity,
        trail,
        collision,
        point_cloud,
        gaussian_splat,
        mesh,
        octomap,
        semantic_box,
    })
}

fn dedupe_key(payload: &VisualizationPayload) -> (String, String, String) {
    if payload.viz_type == "semantic_box" {
        let semantic_id = payload
            .semantic_box
            .as_ref()
            .and_then(|value| value.get("id"))
            .and_then(Value::as_str)
            .unwrap_or("");
        (
            payload.viz_type.clone(),
            semantic_id.to_string(),
            payload.frame.clone().unwrap_or_default(),
        )
    } else {
        (
            payload.viz_type.clone(),
            payload.topic.clone(),
            payload.frame.clone().unwrap_or_default(),
        )
    }
}

fn ros_binding_payload(robot: &Robot) -> RosBindingPayload {
    let binding = robot.get_ros_binding();
    RosBindingPayload {
        logical_name: coerce_text(binding.get("logical_name"), &robot.name),
        tf_mode: coerce_text(binding.get("tf_mode"), "prefixed"),
        topic_mode: coerce_text(binding.get("topic_mode"), "prefixed"),
        base_frame: coerce_text(binding.get("base_frame"), "base_link"),
        tf_prefix: coerce_text(binding.get("tf_prefix"), &robot.name),
        topic_prefix: coerce_text(binding.get("topic_prefix"), &format!("/{}", robot.name)),
    }
}

fn build_teleop_control(robot: &Robot) -> TeleopControlPayload {
    let metadata = robot
        .metadata
        .get("teleop_config")
        .and_then(Value::as_object);
    let get = |key: &str| metadata.and_then(|m| m.get(key));
    let default_profile = match robot.get_type_str() {
        "legged" => "legged",
        "aerial" => "aerial",
        "drone" => "drone",
        _ => "wheeled",
    };
    let mut robot_profile = coerce_text(get("robot_profile"), default_profile).to_ascii_lowercase();
    if !matches!(
        robot_profile.as_str(),
        "wheeled" | "legged" | "aerial" | "drone" | "custom"
    ) {
        robot_profile = default_profile.to_string();
    }
    let mut response_mode = coerce_text(get("response_mode"), "analog").to_ascii_lowercase();
    if response_mode != "analog" && response_mode != "discrete" {
        response_mode = "analog".to_string();
    }

    let deadman = get("deadman").and_then(Value::as_object);
    let axes = get("axes").and_then(Value::as_object);
    let discrete = get("discrete").and_then(Value::as_object);
    let mut policy = coerce_text(deadman.and_then(|m| m.get("policy")), "either_grip_trigger")
        .to_ascii_lowercase();
    if !matches!(
        policy.as_str(),
        "either_index_trigger"
            | "left_index_trigger"
            | "right_index_trigger"
            | "either_grip_trigger"
    ) {
        policy = "either_grip_trigger".to_string();
    }

    TeleopControlPayload {
        enabled: coerce_bool(get("enabled"), true),
        command_topic: coerce_text(get("command_topic"), &robot.resolve_topic("cmd_vel")),
        raw_input_topic: coerce_text(
            get("raw_input_topic"),
            &format!("/horus/teleop/{}/joy", robot.name),
        ),
        head_pose_topic: coerce_text(
            get("head_pose_topic"),
            &format!("/horus/teleop/{}/head_pose", robot.name),
        ),
        robot_profile,
        response_mode,
        publish_rate_hz: clamp_f64(coerce_f64(get("publish_rate_hz"), 30.0), 5.0, 120.0),
        custom_passthrough_only: coerce_bool(get("custom_passthrough_only"), false),
        deadman: json!({
            "policy": policy,
            "timeout_ms": clamp_i32(coerce_int(deadman.and_then(|m| m.get("timeout_ms")), 200), 50, 2000),
        }),
        axes: json!({
            "deadzone": clamp_f64(coerce_f64(axes.and_then(|m| m.get("deadzone")), 0.15), 0.0, 0.5),
            "expo": clamp_f64(coerce_f64(axes.and_then(|m| m.get("expo")), 1.7), 1.0, 3.0),
            "linear_xy_max_mps": clamp_f64(coerce_f64(axes.and_then(|m| m.get("linear_xy_max_mps")), 1.0), 0.0, 5.0),
            "linear_z_max_mps": clamp_f64(coerce_f64(axes.and_then(|m| m.get("linear_z_max_mps")), 0.8), 0.0, 5.0),
            "angular_z_max_rps": clamp_f64(coerce_f64(axes.and_then(|m| m.get("angular_z_max_rps")), 1.2), 0.0, 6.0),
            "invert_linear_x": coerce_bool(axes.and_then(|m| m.get("invert_linear_x")), false),
            "invert_linear_y": coerce_bool(axes.and_then(|m| m.get("invert_linear_y")), false),
            "invert_linear_z": coerce_bool(axes.and_then(|m| m.get("invert_linear_z")), false),
            "invert_angular_z": coerce_bool(axes.and_then(|m| m.get("invert_angular_z")), false),
        }),
        discrete: json!({
            "threshold": clamp_f64(coerce_f64(discrete.and_then(|m| m.get("threshold")), 0.6), 0.1, 1.0),
            "linear_xy_step_mps": clamp_f64(coerce_f64(discrete.and_then(|m| m.get("linear_xy_step_mps")), 0.6), 0.0, 5.0),
            "linear_z_step_mps": clamp_f64(coerce_f64(discrete.and_then(|m| m.get("linear_z_step_mps")), 0.4), 0.0, 5.0),
            "angular_z_step_rps": clamp_f64(coerce_f64(discrete.and_then(|m| m.get("angular_z_step_rps")), 0.9), 0.0, 6.0),
        }),
    }
}

fn build_task_control(robot: &Robot) -> TaskControlPayload {
    let metadata = robot.metadata.get("task_config").and_then(Value::as_object);
    let go = metadata
        .and_then(|m| m.get("go_to_point"))
        .and_then(Value::as_object);
    let waypoint = metadata
        .and_then(|m| m.get("waypoint"))
        .and_then(Value::as_object);

    let min_altitude = clamp_f64(
        coerce_f64(go.and_then(|m| m.get("min_altitude_m")), 0.0),
        0.0,
        100.0,
    );
    let max_altitude_raw = coerce_f64(go.and_then(|m| m.get("max_altitude_m")), 10.0).min(100.0);
    let max_altitude = max_altitude_raw.max(min_altitude + 0.1);

    TaskControlPayload {
        go_to_point: json!({
            "enabled": coerce_bool(go.and_then(|m| m.get("enabled")), true),
            "goal_topic": coerce_text(go.and_then(|m| m.get("goal_topic")), &robot.resolve_topic("goal_pose")),
            "cancel_topic": coerce_text(go.and_then(|m| m.get("cancel_topic")), &robot.resolve_topic("goal_cancel")),
            "status_topic": coerce_text(go.and_then(|m| m.get("status_topic")), &robot.resolve_topic("goal_status")),
            "frame_id": coerce_text(go.and_then(|m| m.get("frame_id")), "map"),
            "position_tolerance_m": clamp_f64(coerce_f64(go.and_then(|m| m.get("position_tolerance_m")), 0.20), 0.01, 10.0),
            "yaw_tolerance_deg": clamp_f64(coerce_f64(go.and_then(|m| m.get("yaw_tolerance_deg")), 12.0), 0.1, 180.0),
            "min_altitude_m": min_altitude,
            "max_altitude_m": max_altitude,
        }),
        waypoint: json!({
            "enabled": coerce_bool(waypoint.and_then(|m| m.get("enabled")), true),
            "path_topic": coerce_text(waypoint.and_then(|m| m.get("path_topic")), &robot.resolve_topic("waypoint_path")),
            "status_topic": coerce_text(waypoint.and_then(|m| m.get("status_topic")), &robot.resolve_topic("waypoint_status")),
            "frame_id": coerce_text(waypoint.and_then(|m| m.get("frame_id")), "map"),
            "position_tolerance_m": clamp_f64(coerce_f64(waypoint.and_then(|m| m.get("position_tolerance_m")), 0.20), 0.01, 10.0),
            "yaw_tolerance_deg": clamp_f64(coerce_f64(waypoint.and_then(|m| m.get("yaw_tolerance_deg")), 12.0), 0.1, 180.0),
        }),
    }
}

fn build_workspace_config(
    robot: &Robot,
    workspace_scale: Option<f64>,
    compass_enabled: Option<bool>,
) -> Option<WorkspaceConfigPayload> {
    let position_scale = workspace_scale.and_then(|v| {
        if v.is_finite() && v > 0.0 {
            Some(v as f32)
        } else {
            None
        }
    });

    let compass_metadata = robot
        .metadata
        .get("workspace_compass_config")
        .and_then(Value::as_object);
    let compass =
        if compass_enabled.is_some() || compass_metadata.and_then(|m| m.get("enabled")).is_some() {
            let mut voice_mode =
                coerce_text(compass_metadata.and_then(|m| m.get("voice_mode")), "auto")
                    .to_ascii_lowercase();
            if !matches!(voice_mode.as_str(), "auto" | "batch" | "realtime") {
                voice_mode = "auto".to_string();
            }
            let mut gateway_port =
                coerce_int(compass_metadata.and_then(|m| m.get("gateway_port")), 8088);
            if gateway_port <= 0 || gateway_port > 65535 {
                gateway_port = 8088;
            }
            Some(json!({
                "enabled": compass_enabled.unwrap_or_else(|| {
                    coerce_bool(compass_metadata.and_then(|m| m.get("enabled")), false)
                }),
                "gateway_port": gateway_port,
                "voice_mode": voice_mode,
                "autonomy": "approve_actions",
                "contract_version": coerce_text(
                    compass_metadata.and_then(|m| m.get("contract_version")),
                    "compass.v1",
                ),
            }))
        } else {
            None
        };

    let tutorial_metadata = robot
        .metadata
        .get("workspace_tutorial_config")
        .and_then(Value::as_object);
    let tutorial = tutorial_metadata.and_then(|metadata| {
        let preset_id = coerce_text(metadata.get("preset_id"), "");
        if preset_id.is_empty() {
            None
        } else {
            Some(json!({
                "enabled": coerce_bool(metadata.get("enabled"), true),
                "preset_id": preset_id,
            }))
        }
    });

    if position_scale.is_none() && compass.is_none() && tutorial.is_none() {
        None
    } else {
        Some(WorkspaceConfigPayload {
            position_scale,
            compass,
            tutorial,
        })
    }
}

#[derive(Debug, Clone)]
struct NativeJointDescription {
    name: String,
    joint_type: String,
    parent_link: String,
    child_link: String,
}

fn capture_xml_attribute(text: &str, attribute: &str) -> Option<String> {
    let pattern = Regex::new(&format!(
        r#"\b{}\s*=\s*["']([^"']+)["']"#,
        regex::escape(attribute)
    ))
    .ok()?;
    pattern
        .captures(text)
        .and_then(|captures| captures.get(1))
        .map(|match_| match_.as_str().to_string())
}

fn extract_urdf_link_names(urdf: &str) -> Vec<String> {
    let Ok(pattern) = Regex::new(r#"<link\b[^>]*>"#) else {
        return Vec::new();
    };
    pattern
        .find_iter(urdf)
        .filter_map(|match_| capture_xml_attribute(match_.as_str(), "name"))
        .collect()
}

fn extract_urdf_joints(urdf: &str) -> Vec<NativeJointDescription> {
    let Ok(joint_pattern) = Regex::new(r#"<joint\b[\s\S]*?</joint>"#) else {
        return Vec::new();
    };
    let Ok(parent_pattern) = Regex::new(r#"<parent\b[^>]*>"#) else {
        return Vec::new();
    };
    let Ok(child_pattern) = Regex::new(r#"<child\b[^>]*>"#) else {
        return Vec::new();
    };

    joint_pattern
        .find_iter(urdf)
        .filter_map(|match_| {
            let block = match_.as_str();
            let name = capture_xml_attribute(block, "name")?;
            let parent_link = parent_pattern
                .find(block)
                .and_then(|parent| capture_xml_attribute(parent.as_str(), "link"))
                .unwrap_or_default();
            let child_link = child_pattern
                .find(block)
                .and_then(|child| capture_xml_attribute(child.as_str(), "link"))
                .unwrap_or_default();
            Some(NativeJointDescription {
                name,
                joint_type: capture_xml_attribute(block, "type")
                    .unwrap_or_else(|| "fixed".to_string()),
                parent_link,
                child_link,
            })
        })
        .collect()
}

fn build_robot_description_artifact(robot: &Robot) -> Option<(Value, String)> {
    let config = robot
        .metadata
        .get("robot_description_config")
        .and_then(Value::as_object)?;
    if !coerce_bool(config.get("enabled"), true) {
        return None;
    }
    let urdf_path = coerce_text(config.get("urdf_path"), "");
    if urdf_path.trim().is_empty() {
        return None;
    }

    let urdf = fs::read_to_string(&urdf_path).ok()?;
    if urdf.trim().is_empty() {
        return None;
    }
    let links = extract_urdf_link_names(&urdf);
    let joints = extract_urdf_joints(&urdf);
    let base_frame = coerce_text(config.get("base_frame"), "base_link");
    let source = coerce_text(config.get("source"), "ros");
    let chunk_size = clamp_i32(
        coerce_int(config.get("chunk_size_bytes"), 12000),
        1024,
        64000,
    );
    let payload = json!({
        "base_frame": base_frame.clone(),
        "joints": joints.iter().map(|joint| json!({
            "axis_xyz": [0.0, 0.0, 1.0],
            "child_link": joint.child_link.clone(),
            "name": joint.name.clone(),
            "origin_rpy": [0.0, 0.0, 0.0],
            "origin_xyz": [0.0, 0.0, 0.0],
            "parent_link": joint.parent_link.clone(),
            "type": joint.joint_type.clone(),
        })).collect::<Vec<_>>(),
        "links": links.iter().map(|name| json!({
            "collisions": [],
            "name": name,
        })).collect::<Vec<_>>(),
        "robot_name": robot.name.clone(),
        "version": "v2",
    });
    let payload_json = serde_json::to_string(&payload).ok()?;
    let description_hash = Sha256::digest(payload_json.as_bytes());

    let manifest = json!({
        "version": "v2",
        "description_id": format!("sha256:{description_hash:x}"),
        "source": source,
        "base_frame": base_frame,
        "link_count": links.len() as i64,
        "joint_count": joints.len() as i64,
        "collision_count": 0,
        "supports_collision": false,
        "supports_joints": !joints.is_empty(),
        "supports_visual_meshes": false,
        "mesh_asset_count": 0,
        "mesh_asset_encoded_bytes": 0,
        "is_transparent": coerce_bool(config.get("is_transparent"), false),
        "encoding": "json+gzip+base64",
        "chunk_size_bytes": chunk_size,
    });
    Some((manifest, payload_json))
}

#[derive(Default)]
pub struct RobotRegistryClient {
    registration_lock: AtomicBool,
}

impl RobotRegistryClient {
    pub fn new() -> Self {
        Self {
            registration_lock: AtomicBool::new(false),
        }
    }

    pub fn register_robot(
        &self,
        robot: &mut Robot,
        dataviz: &DataViz,
        _timeout_sec: f64,
        _keep_alive: bool,
        _show_dashboard: bool,
        workspace_scale: Option<f64>,
    ) -> (bool, Value) {
        if self
            .registration_lock
            .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
            .is_err()
        {
            return (false, json!({"error": "Registration already in progress"}));
        }
        let global_payload =
            self.build_global_visualizations_payload(std::slice::from_ref(dataviz));
        let payload =
            self.build_robot_config_dict(robot, dataviz, Some(global_payload), workspace_scale);
        self.registration_lock.store(false, Ordering::SeqCst);

        let result = json!({
            "success": false,
            "error": "Native HORUS registration transport is not implemented yet; build_robot_config_dict provides payload parity only.",
            "unsupported_feature": "bridge_registration",
            "payload": payload,
            "timeout_sec": _timeout_sec,
            "keep_alive": _keep_alive,
            "show_dashboard": _show_dashboard,
        });
        (false, result)
    }

    pub fn register_robots(
        &self,
        robots: &mut [Robot],
        datavizs: Option<Vec<DataViz>>,
        timeout_sec: f64,
        keep_alive: bool,
        show_dashboard: bool,
        workspace_scale: Option<f64>,
    ) -> (bool, Value) {
        if self
            .registration_lock
            .compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst)
            .is_err()
        {
            return (false, json!({"error": "Registration already in progress"}));
        }

        let datavizs =
            datavizs.unwrap_or_else(|| robots.iter().map(|r| r.create_dataviz(None)).collect());
        if datavizs.len() != robots.len() {
            self.registration_lock.store(false, Ordering::SeqCst);
            return (false, json!({"error": "Robot/dataviz length mismatch"}));
        }
        let mut payloads = Vec::with_capacity(robots.len());
        let global_payload = self.build_global_visualizations_payload(&datavizs);
        for (robot, dataviz) in robots.iter_mut().zip(datavizs.iter()) {
            let payload = self.build_robot_config_dict(
                robot,
                dataviz,
                Some(global_payload.clone()),
                workspace_scale,
            );
            payloads.push(payload);
        }
        self.registration_lock.store(false, Ordering::SeqCst);
        (
            false,
            json!({
                "success": false,
                "error": "Native HORUS registration transport is not implemented yet; build_robot_config_dict provides payload parity only.",
                "unsupported_feature": "bridge_registration",
                "payloads": payloads,
                "timeout_sec": timeout_sec,
                "keep_alive": keep_alive,
                "show_dashboard": show_dashboard,
            }),
        )
    }

    pub fn unregister_robot(&self, robot_id: &str, timeout_sec: f64) -> (bool, Value) {
        (
            false,
            json!({
                "success": false,
                "error": "Native HORUS unregister transport is not implemented yet.",
                "unsupported_feature": "bridge_registration",
                "robot_id": robot_id,
                "timeout_sec": timeout_sec,
            }),
        )
    }

    pub fn check_backend_availability(&self) -> bool {
        false
    }

    pub fn build_global_visualizations_payload(
        &self,
        datavizs: &[DataViz],
    ) -> Vec<VisualizationPayload> {
        let mut deduped: HashMap<(String, String, String), VisualizationPayload> = HashMap::new();
        let mut ordered = Vec::new();
        for dataviz in datavizs {
            for visualization in dataviz.get_enabled_visualizations() {
                if visualization.is_robot_specific() {
                    continue;
                }
                let Some(payload) = serialize_visualization_payload(&visualization, Some("global"))
                else {
                    continue;
                };
                let key = dedupe_key(&payload);
                if deduped.contains_key(&key) {
                    continue;
                }
                ordered.push(key.clone());
                deduped.insert(key, payload);
            }
        }
        ordered
            .into_iter()
            .filter_map(|k| deduped.remove(&k))
            .collect()
    }

    pub fn build_robot_config_dict(
        &self,
        robot: &Robot,
        dataviz: &DataViz,
        global_visualizations: Option<Vec<VisualizationPayload>>,
        workspace_scale: Option<f64>,
    ) -> RobotRegistrationPayload {
        let dimensions = robot.dimensions.as_ref().map(|d| DimensionsPayload {
            length: d.length as f32,
            width: d.width as f32,
            height: d.height as f32,
        });

        let mut sensors = Vec::with_capacity(robot.sensors.len());
        for sensor in &robot.sensors {
            let (default_color, default_point_size) =
                if let Some(scan) = sensor.as_any().downcast_ref::<LaserScan>() {
                    (scan.color.clone(), scan.point_size)
                } else {
                    ("white".to_string(), 0.05)
                };
            let mut payload = SensorPayload {
                name: sensor.name().to_string(),
                sensor_type: sensor.sensor_type().as_str().to_string(),
                topic: sensor.topic().to_string(),
                frame: sensor.frame_id().to_string(),
                metadata: sensor.metadata().clone(),
                viz_config: VizConfigPayload {
                    color: sensor
                        .metadata()
                        .get("color")
                        .and_then(Value::as_str)
                        .unwrap_or(&default_color)
                        .to_string(),
                    point_size: sensor
                        .metadata()
                        .get("point_size")
                        .map(|v| coerce_float(Some(v), default_point_size))
                        .unwrap_or(default_point_size),
                },
                camera_config: None,
            };

            if sensor.sensor_type().as_str() == "camera" {
                if let Some(camera) = sensor.as_any().downcast_ref::<Camera>() {
                    payload.camera_config = Some(build_camera_config(camera));
                }
            }
            sensors.push(payload);
        }

        let mut robot_visualizations = Vec::new();
        let mut fallback_global = Vec::new();
        for visualization in &dataviz.visualizations {
            let Some(payload) = serialize_visualization_payload(visualization, None) else {
                continue;
            };
            if payload.scope == "global" {
                fallback_global.push(payload);
            } else {
                robot_visualizations.push(payload);
            }
        }

        let global_visualizations = global_visualizations.unwrap_or_else(|| {
            let mut deduped = HashMap::new();
            let mut ordered = Vec::new();
            for payload in fallback_global {
                let key = dedupe_key(&payload);
                if deduped.contains_key(&key) {
                    continue;
                }
                deduped.insert(key.clone(), payload);
                ordered.push(key);
            }
            ordered
                .into_iter()
                .filter_map(|k| deduped.remove(&k))
                .collect()
        });

        let workspace_config = build_workspace_config(robot, workspace_scale, None);

        let manager_cfg = robot
            .metadata
            .get("robot_manager_config")
            .and_then(Value::as_object)
            .cloned()
            .unwrap_or_default();
        let sections = manager_cfg
            .get("sections")
            .and_then(Value::as_object)
            .cloned()
            .unwrap_or_default();

        let teleop_control = build_teleop_control(robot);
        let drive_topic = teleop_control.command_topic.clone();
        let robot_description_artifact = build_robot_description_artifact(robot);
        let robot_description_manifest = robot_description_artifact
            .as_ref()
            .map(|(manifest, _)| manifest.clone());
        let robot_description_payload_json =
            robot_description_artifact
                .as_ref()
                .and_then(|(_, payload_json)| {
                    if payload_json.len() <= 250_000 {
                        Some(payload_json.clone())
                    } else {
                        None
                    }
                });
        let description_has_visual_mesh = robot_description_manifest
            .as_ref()
            .and_then(|manifest| manifest.get("supports_visual_meshes"))
            .and_then(Value::as_bool)
            .unwrap_or(false);
        let local_has_visual_mesh = robot
            .metadata
            .get("local_body_model_config")
            .and_then(Value::as_object)
            .map(|config| {
                let enabled = coerce_bool(config.get("enabled"), false);
                let id = coerce_text(config.get("robot_model_id"), "");
                enabled && !id.trim().is_empty()
            })
            .unwrap_or(false);

        RobotRegistrationPayload {
            action: "register".to_string(),
            robot_name: robot.name.clone(),
            robot_type: robot.get_type_str().to_string(),
            ros_binding: ros_binding_payload(robot),
            sensors,
            visualizations: robot_visualizations,
            global_visualizations,
            control: ControlPayload {
                drive_topic,
                teleop: teleop_control,
                tasks: build_task_control(robot),
            },
            robot_manager_config: RobotManagerConfigPayload {
                enabled: coerce_bool(manager_cfg.get("enabled"), true),
                prefab_asset_path: manager_cfg
                    .get("prefab_asset_path")
                    .and_then(Value::as_str)
                    .unwrap_or("Assets/Prefabs/UI/RobotManager.prefab")
                    .to_string(),
                prefab_resource_path: manager_cfg
                    .get("prefab_resource_path")
                    .and_then(Value::as_str)
                    .unwrap_or("")
                    .to_string(),
                sections: RobotManagerSectionsPayload {
                    status: coerce_bool(sections.get("status"), true),
                    data_viz: coerce_bool(sections.get("data_viz"), true),
                    teleop: coerce_bool(sections.get("teleop"), true),
                    tasks: coerce_bool(sections.get("tasks"), true),
                },
            },
            timestamp: now_sec(),
            dimensions,
            workspace_config,
            robot_model_id: robot
                .metadata
                .get("local_body_model_config")
                .and_then(Value::as_object)
                .and_then(|config| {
                    if !coerce_bool(config.get("enabled"), false) {
                        return None;
                    }
                    let id = coerce_text(config.get("robot_model_id"), "")
                        .trim()
                        .to_ascii_lowercase();
                    if id.is_empty() {
                        None
                    } else {
                        Some(id)
                    }
                }),
            has_visual_mesh_model: if local_has_visual_mesh || description_has_visual_mesh {
                Some(true)
            } else {
                None
            },
            robot_description_manifest,
            robot_description_payload_json,
        }
    }

    pub fn extract_camera_topic_profiles(
        &self,
        payload: &RobotRegistrationPayload,
    ) -> HashMap<String, HashMap<String, String>> {
        let mut profiles = HashMap::new();
        for sensor in &payload.sensors {
            if sensor.sensor_type != "camera" {
                continue;
            }
            let Some(cfg) = &sensor.camera_config else {
                continue;
            };
            let minimap_topic = if cfg.minimap_topic.trim().is_empty() {
                sensor.topic.clone()
            } else {
                cfg.minimap_topic.clone()
            };
            let teleop_topic = if cfg.teleop_topic.trim().is_empty() {
                minimap_topic.clone()
            } else {
                cfg.teleop_topic.clone()
            };
            for (topic, source_mode) in [(minimap_topic, "minimap"), (teleop_topic, "teleop")] {
                profiles
                    .entry(topic)
                    .and_modify(|profile: &mut HashMap<String, String>| {
                        if profile
                            .get("source_mode")
                            .is_some_and(|existing| existing != source_mode)
                        {
                            profile.insert("source_mode".to_string(), "both".to_string());
                        }
                    })
                    .or_insert_with(|| {
                        HashMap::from([
                            ("minimap".to_string(), cfg.minimap_streaming_type.clone()),
                            ("teleop".to_string(), cfg.teleop_streaming_type.clone()),
                            ("startup".to_string(), cfg.startup_mode.clone()),
                            ("source_mode".to_string(), source_mode.to_string()),
                        ])
                    });
            }
        }
        profiles
    }
}

static REGISTRY_CLIENT: Lazy<Mutex<RobotRegistryClient>> =
    Lazy::new(|| Mutex::new(RobotRegistryClient::new()));

pub fn get_robot_registry_client() -> std::sync::MutexGuard<'static, RobotRegistryClient> {
    REGISTRY_CLIENT.lock().unwrap_or_else(|e| e.into_inner())
}

pub fn build_global_visualizations_payload(datavizs: &[DataViz]) -> Vec<VisualizationPayload> {
    let client = get_robot_registry_client();
    client.build_global_visualizations_payload(datavizs)
}

pub fn build_robot_config_dict(
    robot: &Robot,
    dataviz: &DataViz,
    workspace_scale: Option<f64>,
) -> RobotRegistrationPayload {
    let client = get_robot_registry_client();
    let global_visualizations =
        client.build_global_visualizations_payload(std::slice::from_ref(dataviz));
    client.build_robot_config_dict(robot, dataviz, Some(global_visualizations), workspace_scale)
}

pub fn extract_camera_topic_profiles(
    payload: &RobotRegistrationPayload,
) -> HashMap<String, HashMap<String, String>> {
    let client = get_robot_registry_client();
    client.extract_camera_topic_profiles(payload)
}

pub fn queued_reason_from_ack(ack: &Value) -> Option<String> {
    let status = ack.get("robot_id")?.as_str()?.trim();
    if !status.starts_with("Queued") {
        return None;
    }

    let mut detail = status.trim_start_matches("Queued").trim().to_string();
    if detail.starts_with('(') && detail.ends_with(')') && detail.len() >= 2 {
        detail = detail[1..detail.len() - 1].trim().to_string();
    }
    if detail.starts_with(':') || detail.starts_with('-') {
        detail = detail[1..].trim().to_string();
    }
    if detail.is_empty() {
        Some("Waiting for Workspace".to_string())
    } else {
        Some(detail)
    }
}
