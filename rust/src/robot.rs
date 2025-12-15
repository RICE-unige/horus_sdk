use crate::core::types::{Metadata, RobotType};
use crate::sensors::Sensor;
use std::sync::Arc;

/// Robot object system for HORUS SDK
#[derive(Debug, Clone)]
pub struct Robot {
    pub name: String,
    pub robot_type: RobotType,
    pub metadata: Metadata,
    pub sensors: Vec<Arc<dyn Sensor>>,
}

impl Robot {
    pub fn new(name: impl Into<String>, robot_type: RobotType) -> Self {
        Self {
            name: name.into(),
            robot_type,
            metadata: Metadata::new(),
            sensors: Vec::new(),
        }
    }

    pub fn get_type_str(&self) -> &'static str {
        self.robot_type.as_str()
    }

    pub fn add_metadata(&mut self, key: impl Into<String>, value: serde_json::Value) {
        self.metadata.insert(key.into(), value);
    }

    pub fn get_metadata(&self, key: &str) -> Option<&serde_json::Value> {
        self.metadata.get(key)
    }

    pub fn add_sensor(&mut self, sensor: Arc<dyn Sensor>) {
        self.sensors.push(sensor);
    }

    pub fn get_sensor_count(&self) -> usize {
        self.sensors.len()
    }

    pub fn has_sensors(&self) -> bool {
        !self.sensors.is_empty()
    }
}
