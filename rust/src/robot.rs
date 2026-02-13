use crate::bridge::get_robot_registry_client;
use crate::core::types::{Metadata, RobotType, SensorType};
use crate::dataviz::DataViz;
use crate::sensors::{Sensor, SensorRef};
use crate::utils::{get_topic_monitor, get_topic_status_board};
use serde_json::{json, Value};
use std::sync::Arc;

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
        dataviz.add_robot_transform(&self.name, "/tf", &format!("{}_base_link", self.name), None);
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
