use crate::dataviz::{DataViz, DataSource, VisualizationConfig};
use crate::robot::Robot;
use crate::sensors::{Camera, LaserScan};
use crate::utils::{get_topic_monitor, get_topic_status_board};
use once_cell::sync::Lazy;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::HashMap;
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
    pub startup_mode: String,
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
    pub view_position_offset: Vector3Payload,
    pub view_rotation_offset: Vector3Payload,
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
    pub position_scale: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlPayload {
    pub drive_topic: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotRegistrationPayload {
    pub action: String,
    pub robot_name: String,
    pub robot_type: String,
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
        return Vector3Payload { x: dx, y: dy, z: dz };
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
    Vector3Payload { x: dx, y: dy, z: dz }
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

    CameraConfigPayload {
        streaming_type: legacy_streaming_type,
        minimap_streaming_type,
        teleop_streaming_type,
        startup_mode,
        image_type: metadata
            .get("image_type")
            .and_then(Value::as_str)
            .unwrap_or("raw")
            .to_ascii_lowercase(),
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
        view_position_offset: coerce_vec3(metadata.get("view_position_offset"), (0.0, 0.0, 0.0)),
        view_rotation_offset: coerce_vec3(metadata.get("view_rotation_offset"), (0.0, 0.0, 0.0)),
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

    let scope = scope_override
        .map(str::to_string)
        .unwrap_or_else(|| if visualization.is_robot_specific() { "robot" } else { "global" }.to_string());

    let frame = if data_source.frame_id.trim().is_empty() {
        None
    } else {
        Some(data_source.frame_id.clone())
    };

    let mut occupancy = None;
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
            occ.position_offset = Some(coerce_vec3(options.get("position_offset"), (0.0, 0.0, 0.0)));
            any = true;
        }
        if options.contains_key("rotation_offset_euler") {
            occ.rotation_offset_euler =
                Some(coerce_vec3(options.get("rotation_offset_euler"), (0.0, 0.0, 0.0)));
            any = true;
        }
        if any {
            occupancy = Some(occ);
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
    })
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
        let global_payload = self.build_global_visualizations_payload(std::slice::from_ref(dataviz));
        let payload =
            self.build_robot_config_dict(robot, dataviz, Some(global_payload), workspace_scale);
        self.registration_lock.store(false, Ordering::SeqCst);

        let ack = json!({
            "success": true,
            "robot_name": robot.name,
            "robot_id": robot.name,
            "assigned_color": "#00FF00",
            "payload": payload,
        });
        self.apply_registration_metadata(robot, dataviz, &ack);
        (true, ack)
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

        let datavizs = datavizs.unwrap_or_else(|| robots.iter().map(|r| r.create_dataviz(None)).collect());
        if datavizs.len() != robots.len() {
            self.registration_lock.store(false, Ordering::SeqCst);
            return (false, json!({"error": "Robot/dataviz length mismatch"}));
        }
        let mut results = Vec::with_capacity(robots.len());
        let global_payload = self.build_global_visualizations_payload(&datavizs);
        for (robot, dataviz) in robots.iter_mut().zip(datavizs.iter()) {
            let payload = self.build_robot_config_dict(
                robot,
                dataviz,
                Some(global_payload.clone()),
                workspace_scale,
            );
            let (ok, ack) = self.register_robot_payload(
                robot,
                dataviz,
                payload,
                timeout_sec,
                keep_alive,
                show_dashboard,
            );
            if !ok {
                self.registration_lock.store(false, Ordering::SeqCst);
                return (false, ack);
            }
            results.push(ack);
        }
        self.registration_lock.store(false, Ordering::SeqCst);
        (true, json!({"success": true, "results": results}))
    }

    pub fn unregister_robot(&self, _robot_id: &str, _timeout_sec: f64) -> (bool, Value) {
        (true, json!({"message": "Unregistered (Local cleanup only)"}))
    }

    pub fn check_backend_availability(&self) -> bool {
        cfg!(feature = "ros2")
    }

    pub fn build_global_visualizations_payload(&self, datavizs: &[DataViz]) -> Vec<VisualizationPayload> {
        let mut deduped: HashMap<(String, String, String), VisualizationPayload> = HashMap::new();
        let mut ordered = Vec::new();
        for dataviz in datavizs {
            for visualization in dataviz.get_enabled_visualizations() {
                if visualization.is_robot_specific() {
                    continue;
                }
                let Some(payload) = serialize_visualization_payload(&visualization, Some("global")) else {
                    continue;
                };
                let key = (
                    payload.viz_type.clone(),
                    payload.topic.clone(),
                    payload.frame.clone().unwrap_or_default(),
                );
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
                let key = (
                    payload.viz_type.clone(),
                    payload.topic.clone(),
                    payload.frame.clone().unwrap_or_default(),
                );
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

        let workspace_config = workspace_scale
            .and_then(|v| if v.is_finite() && v > 0.0 { Some(v as f32) } else { None })
            .map(|position_scale| WorkspaceConfigPayload { position_scale });

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

        RobotRegistrationPayload {
            action: "register".to_string(),
            robot_name: robot.name.clone(),
            robot_type: robot.get_type_str().to_string(),
            sensors,
            visualizations: robot_visualizations,
            global_visualizations,
            control: ControlPayload {
                drive_topic: format!("/{}/cmd_vel", robot.name),
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
            profiles.insert(
                sensor.topic.clone(),
                HashMap::from([
                    ("minimap".to_string(), cfg.minimap_streaming_type.clone()),
                    ("teleop".to_string(), cfg.teleop_streaming_type.clone()),
                    ("startup".to_string(), cfg.startup_mode.clone()),
                ]),
            );
        }
        profiles
    }

    fn register_robot_payload(
        &self,
        robot: &mut Robot,
        dataviz: &DataViz,
        payload: RobotRegistrationPayload,
        _timeout_sec: f64,
        _keep_alive: bool,
        _show_dashboard: bool,
    ) -> (bool, Value) {
        let ack = json!({
            "success": true,
            "robot_name": robot.name,
            "robot_id": robot.name,
            "assigned_color": "#00FF00",
            "payload": payload,
        });
        self.apply_registration_metadata(robot, dataviz, &ack);
        (true, ack)
    }

    fn apply_registration_metadata(&self, robot: &mut Robot, dataviz: &DataViz, ack: &Value) {
        let robot_id = ack
            .get("robot_id")
            .and_then(Value::as_str)
            .unwrap_or(&robot.name)
            .to_string();
        let color = ack
            .get("assigned_color")
            .or_else(|| ack.get("color"))
            .and_then(Value::as_str)
            .unwrap_or("#00FF00")
            .to_string();

        robot.add_metadata("horus_robot_id", json!(robot_id));
        robot.add_metadata("horus_color", json!(color));
        robot.add_metadata("horus_registered", json!(true));

        let topics = collect_topics(dataviz);
        robot.add_metadata(
            "horus_topics",
            Value::Array(topics.iter().map(|topic| json!(topic)).collect()),
        );

        if !topics.is_empty() {
            let monitor = get_topic_monitor();
            monitor.watch_topics(&topics, None);
            monitor.start();
            let board = get_topic_status_board();
            for topic in &topics {
                board.on_subscribe(topic);
            }
        }
    }
}

static REGISTRY_CLIENT: Lazy<Mutex<RobotRegistryClient>> =
    Lazy::new(|| Mutex::new(RobotRegistryClient::new()));

pub fn get_robot_registry_client() -> std::sync::MutexGuard<'static, RobotRegistryClient> {
    REGISTRY_CLIENT.lock().unwrap_or_else(|e| e.into_inner())
}

fn collect_topics(dataviz: &DataViz) -> Vec<String> {
    let mut topics = Vec::new();
    for viz in dataviz.get_enabled_visualizations() {
        let topic = viz.data_source.topic.trim();
        if topic.is_empty() {
            continue;
        }
        if !topics.iter().any(|existing| existing == topic) {
            topics.push(topic.to_string());
        }
    }
    topics
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
