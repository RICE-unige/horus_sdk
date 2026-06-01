use crate::bridge::get_robot_registry_client;
use crate::core::types::{Metadata, RobotType, SensorType};
use crate::dataviz::DataViz;
use crate::sensors::{Sensor, SensorRef};
use crate::utils::{get_topic_monitor, get_topic_status_board};
use serde_json::{json, Value};
use std::sync::Arc;

#[derive(Debug, Clone)]
pub struct RobotManagerConfig {
    pub enabled: bool,
    pub status: bool,
    pub data_viz: bool,
    pub teleop: bool,
    pub tasks: bool,
}

impl RobotManagerConfig {
    pub fn enabled() -> Self {
        Self::default()
    }
}

impl Default for RobotManagerConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            status: true,
            data_viz: true,
            teleop: true,
            tasks: true,
        }
    }
}

#[derive(Debug, Clone)]
pub struct TeleopConfig {
    pub enabled: bool,
    pub command_topic: Option<String>,
    pub raw_input_topic: Option<String>,
    pub head_pose_topic: Option<String>,
    pub robot_profile: Option<String>,
    pub response_mode: Option<String>,
    pub publish_rate_hz: Option<f64>,
    pub custom_passthrough_only: Option<bool>,
    pub deadman_policy: Option<String>,
    pub deadman_timeout_ms: Option<i64>,
    pub deadzone: Option<f64>,
    pub expo: Option<f64>,
    pub linear_xy_max_mps: Option<f64>,
    pub linear_z_max_mps: Option<f64>,
    pub angular_z_max_rps: Option<f64>,
    pub invert_linear_x: Option<bool>,
    pub invert_linear_y: Option<bool>,
    pub invert_linear_z: Option<bool>,
    pub invert_angular_z: Option<bool>,
    pub discrete_threshold: Option<f64>,
    pub linear_xy_step_mps: Option<f64>,
    pub linear_z_step_mps: Option<f64>,
    pub angular_z_step_rps: Option<f64>,
}

impl TeleopConfig {
    pub fn enabled() -> Self {
        Self::default()
    }
}

impl Default for TeleopConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            command_topic: None,
            raw_input_topic: None,
            head_pose_topic: None,
            robot_profile: None,
            response_mode: None,
            publish_rate_hz: None,
            custom_passthrough_only: None,
            deadman_policy: None,
            deadman_timeout_ms: None,
            deadzone: None,
            expo: None,
            linear_xy_max_mps: None,
            linear_z_max_mps: None,
            angular_z_max_rps: None,
            invert_linear_x: None,
            invert_linear_y: None,
            invert_linear_z: None,
            invert_angular_z: None,
            discrete_threshold: None,
            linear_xy_step_mps: None,
            linear_z_step_mps: None,
            angular_z_step_rps: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct NavigationTaskConfig {
    pub go_to_point_enabled: bool,
    pub waypoint_enabled: bool,
    pub goal_topic: Option<String>,
    pub cancel_topic: Option<String>,
    pub goal_status_topic: Option<String>,
    pub waypoint_path_topic: Option<String>,
    pub waypoint_status_topic: Option<String>,
    pub frame_id: String,
    pub position_tolerance_m: Option<f64>,
    pub yaw_tolerance_deg: Option<f64>,
    pub min_altitude_m: Option<f64>,
    pub max_altitude_m: Option<f64>,
}

impl Default for NavigationTaskConfig {
    fn default() -> Self {
        Self {
            go_to_point_enabled: true,
            waypoint_enabled: true,
            goal_topic: None,
            cancel_topic: None,
            goal_status_topic: None,
            waypoint_path_topic: None,
            waypoint_status_topic: None,
            frame_id: "map".to_string(),
            position_tolerance_m: None,
            yaw_tolerance_deg: None,
            min_altitude_m: None,
            max_altitude_m: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct RobotDescriptionConfig {
    pub urdf_path: String,
    pub base_frame: String,
    pub source: String,
    pub ros_param_node: String,
    pub ros_param_name: String,
    pub chunk_size_bytes: i64,
    pub is_transparent: bool,
    pub include_visual_meshes: bool,
    pub visual_mesh_triangle_budget: i64,
    pub body_mesh_mode: String,
    pub enabled: bool,
}

impl RobotDescriptionConfig {
    pub fn new(urdf_path: impl Into<String>, base_frame: impl Into<String>) -> Self {
        Self {
            urdf_path: urdf_path.into(),
            base_frame: base_frame.into(),
            source: "ros".to_string(),
            ros_param_node: String::new(),
            ros_param_name: "robot_description".to_string(),
            chunk_size_bytes: 12000,
            is_transparent: false,
            include_visual_meshes: true,
            visual_mesh_triangle_budget: 90000,
            body_mesh_mode: "preview_mesh".to_string(),
            enabled: true,
        }
    }
}

#[derive(Debug, Clone)]
pub struct RobotDimensions {
    pub length: f64,
    pub width: f64,
    pub height: f64,
}

impl RobotDimensions {
    pub fn new(length: f64, width: f64, height: f64) -> Self {
        assert!(length >= 0.0 && width >= 0.0 && height >= 0.0);
        Self {
            length,
            width,
            height,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Robot {
    pub name: String,
    pub robot_type: RobotType,
    pub metadata: Metadata,
    pub sensors: Vec<SensorRef>,
    pub dimensions: Option<RobotDimensions>,
}

impl Robot {
    pub fn new(name: impl Into<String>, robot_type: RobotType) -> Self {
        let name = name.into();
        assert!(!name.is_empty(), "Robot name cannot be empty");
        Self {
            name,
            robot_type,
            metadata: Metadata::new(),
            sensors: Vec::new(),
            dimensions: None,
        }
    }

    pub fn with_dimensions(
        name: impl Into<String>,
        robot_type: RobotType,
        dimensions: RobotDimensions,
    ) -> Self {
        let mut robot = Self::new(name, robot_type);
        robot.dimensions = Some(dimensions);
        robot
    }

    pub fn get_type_str(&self) -> &'static str {
        self.robot_type.as_str()
    }

    pub fn add_metadata(&mut self, key: impl Into<String>, value: Value) {
        self.metadata.insert(key.into(), value);
    }

    pub fn get_metadata(&self, key: &str, default: Option<Value>) -> Option<Value> {
        self.metadata.get(key).cloned().or(default)
    }

    pub fn set_metadata(&mut self, key: impl Into<String>, value: Value) {
        self.add_metadata(key, value);
    }

    pub fn configure_ros_binding(
        &mut self,
        tf_mode: &str,
        topic_mode: &str,
        base_frame: Option<&str>,
    ) {
        let tf_mode = normalize_binding_mode(tf_mode);
        let topic_mode = normalize_binding_mode(topic_mode);
        let base_frame = base_frame
            .map(str::trim)
            .filter(|value| !value.is_empty())
            .unwrap_or("base_link");
        let tf_prefix = if tf_mode == "flat" { "" } else { self.name.as_str() };
        let topic_prefix = if topic_mode == "flat" {
            String::new()
        } else {
            format!("/{}", self.name)
        };
        self.metadata.insert(
            "ros_binding".to_string(),
            json!({
                "logical_name": self.name,
                "tf_mode": tf_mode,
                "topic_mode": topic_mode,
                "base_frame": base_frame,
                "tf_prefix": tf_prefix,
                "topic_prefix": topic_prefix,
            }),
        );
    }

    pub fn get_ros_binding(&self) -> Value {
        self.metadata
            .get("ros_binding")
            .and_then(Value::as_object)
            .cloned()
            .map(Value::Object)
            .unwrap_or_else(|| {
                json!({
                    "logical_name": self.name,
                    "tf_mode": "prefixed",
                    "topic_mode": "prefixed",
                    "base_frame": "base_link",
                    "tf_prefix": self.name,
                    "topic_prefix": format!("/{}", self.name),
                })
            })
    }

    pub fn resolve_topic(&self, topic_name: &str) -> String {
        let topic = topic_name.trim();
        if topic.is_empty() {
            return String::new();
        }
        if topic.starts_with('/') {
            return topic.to_string();
        }
        let binding = self.get_ros_binding();
        let prefix = binding
            .get("topic_prefix")
            .and_then(Value::as_str)
            .unwrap_or("");
        if prefix.trim().is_empty() {
            format!("/{topic}")
        } else {
            format!("{}/{}", prefix.trim_end_matches('/'), topic)
        }
    }

    pub fn resolve_tf_frame(&self, frame_id: &str) -> String {
        let frame = frame_id.trim();
        if frame.is_empty() {
            return String::new();
        }
        if frame.starts_with('/') {
            return frame.trim_start_matches('/').to_string();
        }
        let binding = self.get_ros_binding();
        let prefix = binding
            .get("tf_prefix")
            .and_then(Value::as_str)
            .unwrap_or("");
        if prefix.trim().is_empty() || frame.starts_with(&format!("{}/", prefix)) {
            frame.to_string()
        } else {
            format!("{}/{}", prefix.trim_matches('/'), frame)
        }
    }

    pub fn configure_robot_manager(&mut self, status: bool, data_viz: bool, teleop: bool, tasks: bool) {
        self.configure_robot_manager_with(RobotManagerConfig {
            enabled: true,
            status,
            data_viz,
            teleop,
            tasks,
        });
    }

    pub fn configure_robot_manager_with(&mut self, config: RobotManagerConfig) {
        self.metadata.insert(
            "robot_manager_config".to_string(),
            json!({
                "enabled": config.enabled,
                "prefab_asset_path": "Assets/Prefabs/UI/RobotManager.prefab",
                "prefab_resource_path": "",
                "sections": {
                    "status": config.status,
                    "data_viz": config.data_viz,
                    "teleop": config.teleop,
                    "tasks": config.tasks,
                }
            }),
        );
    }

    pub fn configure_teleop(&mut self, config: TeleopConfig) {
        let mut payload = serde_json::Map::new();
        payload.insert("enabled".to_string(), json!(config.enabled));
        put_if_some(&mut payload, "command_topic", config.command_topic);
        put_if_some(&mut payload, "raw_input_topic", config.raw_input_topic);
        put_if_some(&mut payload, "head_pose_topic", config.head_pose_topic);
        put_if_some(&mut payload, "robot_profile", config.robot_profile);
        put_if_some(&mut payload, "response_mode", config.response_mode);
        put_if_some(&mut payload, "publish_rate_hz", config.publish_rate_hz);
        put_if_some(&mut payload, "custom_passthrough_only", config.custom_passthrough_only);

        let mut deadman = serde_json::Map::new();
        put_if_some(&mut deadman, "policy", config.deadman_policy);
        put_if_some(&mut deadman, "timeout_ms", config.deadman_timeout_ms);
        if !deadman.is_empty() {
            payload.insert("deadman".to_string(), Value::Object(deadman));
        }

        let mut axes = serde_json::Map::new();
        put_if_some(&mut axes, "deadzone", config.deadzone);
        put_if_some(&mut axes, "expo", config.expo);
        put_if_some(&mut axes, "linear_xy_max_mps", config.linear_xy_max_mps);
        put_if_some(&mut axes, "linear_z_max_mps", config.linear_z_max_mps);
        put_if_some(&mut axes, "angular_z_max_rps", config.angular_z_max_rps);
        put_if_some(&mut axes, "invert_linear_x", config.invert_linear_x);
        put_if_some(&mut axes, "invert_linear_y", config.invert_linear_y);
        put_if_some(&mut axes, "invert_linear_z", config.invert_linear_z);
        put_if_some(&mut axes, "invert_angular_z", config.invert_angular_z);
        if !axes.is_empty() {
            payload.insert("axes".to_string(), Value::Object(axes));
        }

        let mut discrete = serde_json::Map::new();
        put_if_some(&mut discrete, "threshold", config.discrete_threshold);
        put_if_some(&mut discrete, "linear_xy_step_mps", config.linear_xy_step_mps);
        put_if_some(&mut discrete, "linear_z_step_mps", config.linear_z_step_mps);
        put_if_some(&mut discrete, "angular_z_step_rps", config.angular_z_step_rps);
        if !discrete.is_empty() {
            payload.insert("discrete".to_string(), Value::Object(discrete));
        }

        self.metadata.insert("teleop_config".to_string(), Value::Object(payload));
    }

    pub fn configure_navigation_tasks(&mut self, config: NavigationTaskConfig) {
        self.metadata.insert(
            "task_config".to_string(),
            json!({
                "go_to_point": {
                    "enabled": config.go_to_point_enabled,
                    "goal_topic": config.goal_topic,
                    "cancel_topic": config.cancel_topic,
                    "status_topic": config.goal_status_topic,
                    "frame_id": config.frame_id,
                    "position_tolerance_m": config.position_tolerance_m,
                    "yaw_tolerance_deg": config.yaw_tolerance_deg,
                    "min_altitude_m": config.min_altitude_m,
                    "max_altitude_m": config.max_altitude_m,
                },
                "waypoint": {
                    "enabled": config.waypoint_enabled,
                    "path_topic": config.waypoint_path_topic,
                    "status_topic": config.waypoint_status_topic,
                    "frame_id": config.frame_id,
                    "position_tolerance_m": config.position_tolerance_m,
                    "yaw_tolerance_deg": config.yaw_tolerance_deg,
                }
            }),
        );
    }

    pub fn configure_robot_description(&mut self, config: RobotDescriptionConfig) {
        let mut body_mesh_mode = config.body_mesh_mode.trim().to_ascii_lowercase();
        if body_mesh_mode == "max_quality_mesh" {
            body_mesh_mode = "runtime_high_mesh".to_string();
        }
        if !["collision_only", "preview_mesh", "runtime_high_mesh"].contains(&body_mesh_mode.as_str()) {
            body_mesh_mode = "preview_mesh".to_string();
        }
        let include_visual_meshes = config.include_visual_meshes && body_mesh_mode != "collision_only";
        self.metadata.insert(
            "robot_description_config".to_string(),
            json!({
                "enabled": config.enabled,
                "source": if config.source.trim().is_empty() { "ros" } else { config.source.as_str() },
                "urdf_path": config.urdf_path,
                "base_frame": if config.base_frame.trim().is_empty() { "base_link" } else { config.base_frame.as_str() },
                "ros_param_node": config.ros_param_node,
                "ros_param_name": if config.ros_param_name.trim().is_empty() { "robot_description" } else { config.ros_param_name.as_str() },
                "chunk_size_bytes": config.chunk_size_bytes.clamp(1024, 64000),
                "is_transparent": config.is_transparent,
                "include_visual_meshes": include_visual_meshes,
                "visual_mesh_triangle_budget": config.visual_mesh_triangle_budget.clamp(2000, 500000),
                "body_mesh_mode": body_mesh_mode,
            }),
        );
    }

    pub fn configure_local_body_model(&mut self, robot_model_id: &str, enabled: bool) {
        self.metadata.insert(
            "local_body_model_config".to_string(),
            json!({
                "enabled": enabled,
                "robot_model_id": robot_model_id.trim().to_ascii_lowercase(),
            }),
        );
    }

    pub fn configure_workspace_compass(&mut self, enabled: bool, gateway_port: u16, voice_mode: &str) {
        self.metadata.insert(
            "workspace_compass_config".to_string(),
            json!({
                "enabled": enabled,
                "gateway_port": gateway_port,
                "voice_mode": voice_mode,
                "autonomy": "approve_actions",
                "contract_version": "compass.v1",
            }),
        );
    }

    pub fn configure_workspace_tutorial(&mut self, preset_id: &str) {
        self.metadata.insert(
            "workspace_tutorial_config".to_string(),
            json!({
                "enabled": true,
                "preset_id": preset_id,
            }),
        );
    }

    pub fn add_sensor(&mut self, sensor: Arc<dyn Sensor>) -> Result<(), String> {
        if self.sensors.iter().any(|s| s.name() == sensor.name()) {
            return Err(format!("Sensor with name '{}' already exists", sensor.name()));
        }
        self.sensors.push(sensor);
        Ok(())
    }

    pub fn remove_sensor(&mut self, sensor_name: &str) -> bool {
        let before = self.sensors.len();
        self.sensors.retain(|sensor| sensor.name() != sensor_name);
        before != self.sensors.len()
    }

    pub fn get_sensor(&self, sensor_name: &str) -> Option<SensorRef> {
        self.sensors
            .iter()
            .find(|sensor| sensor.name() == sensor_name)
            .cloned()
    }

    pub fn get_sensors_by_type(&self, sensor_type: SensorType) -> Vec<SensorRef> {
        self.sensors
            .iter()
            .filter(|sensor| sensor.sensor_type() == sensor_type)
            .cloned()
            .collect()
    }

    pub fn get_sensor_count(&self) -> usize {
        self.sensors.len()
    }

    pub fn has_sensors(&self) -> bool {
        !self.sensors.is_empty()
    }

    pub fn create_dataviz(&self, dataviz_name: Option<&str>) -> DataViz {
        let mut dataviz = DataViz::new(
            dataviz_name
                .map(ToString::to_string)
                .unwrap_or_else(|| format!("{}_viz", self.name)),
        );
        for sensor in &self.sensors {
            dataviz.add_sensor_visualization(sensor.as_ref(), &self.name, None);
        }
        dataviz.add_robot_transform(&self.name, "/tf", &self.resolve_tf_frame("base_link"), None);
        dataviz
    }

    pub fn add_path_planning_to_dataviz(
        &self,
        dataviz: &mut DataViz,
        global_path_topic: Option<&str>,
        local_path_topic: Option<&str>,
        trajectory_topic: Option<&str>,
    ) {
        if let Some(topic) = global_path_topic {
            dataviz.add_robot_global_path(&self.name, topic, "map", None);
        }
        if let Some(topic) = local_path_topic {
            dataviz.add_robot_local_path(&self.name, topic, "map", None);
        }
        if let Some(topic) = trajectory_topic {
            dataviz.add_robot_trajectory(&self.name, topic, "map", None);
        }
    }

    pub fn create_full_dataviz(
        &self,
        dataviz_name: Option<&str>,
        global_path_topic: Option<&str>,
        local_path_topic: Option<&str>,
        trajectory_topic: Option<&str>,
    ) -> DataViz {
        let mut dataviz = self.create_dataviz(dataviz_name);
        self.add_path_planning_to_dataviz(
            &mut dataviz,
            global_path_topic,
            local_path_topic,
            trajectory_topic,
        );
        dataviz
    }

    pub fn add_navigation_safety_to_dataviz(&self, dataviz: &mut DataViz) {
        let odom_topic = self.resolve_topic("odom");
        dataviz.add_robot_velocity_data(&self.name, &odom_topic, "map", None);
        dataviz.add_robot_odometry_trail(&self.name, &odom_topic, "map", None);
        dataviz.add_robot_collision_risk(
            &self.name,
            &self.resolve_topic("collision_risk"),
            &self.resolve_tf_frame("base_link"),
            None,
        );
    }

    pub fn register_with_horus(
        &mut self,
        dataviz: Option<DataViz>,
        keep_alive: bool,
        show_dashboard: bool,
        workspace_scale: Option<f64>,
    ) -> (bool, Value) {
        let dataviz = dataviz.unwrap_or_else(|| self.create_dataviz(None));
        let registry = get_robot_registry_client();
        let (success, result) = registry.register_robot(
            self,
            &dataviz,
            10.0,
            keep_alive,
            show_dashboard,
            workspace_scale,
        );
        if success {
            self.add_metadata(
                "horus_topics",
                Value::Array(
                    dataviz
                        .get_enabled_visualizations()
                        .iter()
                        .map(|viz| Value::String(viz.data_source.topic.clone()))
                        .collect(),
                ),
            );
        }
        (success, result)
    }

    pub fn unregister_from_horus(&mut self) -> (bool, Value) {
        let Some(robot_id) = self
            .metadata
            .get("horus_robot_id")
            .and_then(Value::as_str)
            .map(ToString::to_string)
        else {
            return (false, json!({"error": "Robot not registered with HORUS"}));
        };

        let topics: Vec<String> = self
            .metadata
            .get("horus_topics")
            .and_then(Value::as_array)
            .map(|items| {
                items
                    .iter()
                    .filter_map(Value::as_str)
                    .map(ToString::to_string)
                    .collect()
            })
            .unwrap_or_default();

        let registry = get_robot_registry_client();
        let (success, result) = registry.unregister_robot(&robot_id, 5.0);
        if success {
            if !topics.is_empty() {
                let board = get_topic_status_board();
                for topic in &topics {
                    board.on_unsubscribe(topic);
                }
                let monitor = get_topic_monitor();
                monitor.unwatch_topics(&topics, false);
            }
            self.metadata.remove("horus_robot_id");
            self.metadata.remove("horus_color");
            self.metadata.remove("horus_registered");
            self.metadata.remove("horus_topics");
        }
        (success, result)
    }

    pub fn is_registered_with_horus(&self) -> bool {
        self.metadata
            .get("horus_registered")
            .and_then(Value::as_bool)
            .unwrap_or(false)
    }

    pub fn get_horus_id(&self) -> Option<String> {
        self.metadata
            .get("horus_robot_id")
            .and_then(Value::as_str)
            .map(ToString::to_string)
    }

    pub fn get_horus_color(&self) -> Option<String> {
        self.metadata
            .get("horus_color")
            .and_then(Value::as_str)
            .map(ToString::to_string)
    }
}

fn normalize_binding_mode(value: &str) -> &'static str {
    match value.trim().to_ascii_lowercase().as_str() {
        "flat" => "flat",
        _ => "prefixed",
    }
}

fn put_if_some<T: serde::Serialize>(values: &mut serde_json::Map<String, Value>, key: &str, value: Option<T>) {
    if let Some(value) = value {
        values.insert(key.to_string(), json!(value));
    }
}

pub fn register_robots(
    robots: &mut [Robot],
    datavizs: Option<Vec<DataViz>>,
    keep_alive: bool,
    show_dashboard: bool,
    timeout_sec: f64,
    workspace_scale: Option<f64>,
) -> (bool, Value) {
    let registry = get_robot_registry_client();
    registry.register_robots(
        robots,
        datavizs,
        timeout_sec,
        keep_alive,
        show_dashboard,
        workspace_scale,
    )
}
