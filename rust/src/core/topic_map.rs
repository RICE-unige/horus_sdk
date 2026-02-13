use crate::core::event_bus::publish_now;
use crate::core::types::{EventPriority, TopicDirection, TopicType};
use once_cell::sync::Lazy;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopicInfo {
    pub name: String,
    pub topic_type: TopicType,
    pub direction: TopicDirection,
    pub robot_namespace: Option<String>,
    pub sensor_name: Option<String>,
    pub frame_id: Option<String>,
    pub qos_profile: Option<serde_json::Value>,
    pub last_seen: f64,
    pub active: bool,
    pub metadata: HashMap<String, serde_json::Value>,
}

impl TopicInfo {
    pub fn new(name: impl Into<String>, topic_type: TopicType, direction: TopicDirection) -> Self {
        let name = name.into();
        let robot_namespace = extract_robot_namespace(&name);
        let sensor_name = extract_sensor_name(&name);
        Self {
            name,
            topic_type,
            direction,
            robot_namespace,
            sensor_name,
            frame_id: None,
            qos_profile: None,
            last_seen: now_sec(),
            active: true,
            metadata: HashMap::new(),
        }
    }

    pub fn is_sensor_topic(&self) -> bool {
        matches!(
            self.topic_type,
            TopicType::Image
                | TopicType::PointCloud
                | TopicType::LaserScan
                | TopicType::Imu
                | TopicType::Gps
        )
    }

    pub fn is_robot_topic(&self) -> bool {
        self.robot_namespace.is_some()
    }
}

fn now_sec() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs_f64()
}

fn extract_robot_namespace(topic: &str) -> Option<String> {
    let first = topic.trim_start_matches('/').split('/').next()?;
    let lower = first.to_ascii_lowercase();
    let markers = ["robot", "carter", "tb3", "turtlebot", "husky", "jackal"];
    if markers.iter().any(|m| lower.contains(m)) {
        Some(first.to_string())
    } else {
        None
    }
}

fn extract_sensor_name(topic: &str) -> Option<String> {
    let markers = ["camera", "lidar", "laser", "scan", "imu", "gps", "depth"];
    topic
        .trim_start_matches('/')
        .split('/')
        .find(|part| markers.iter().any(|m| part.to_ascii_lowercase().contains(m)))
        .map(ToString::to_string)
}

pub struct TopicMap {
    topics: parking_lot::RwLock<HashMap<String, TopicInfo>>,
    robot_namespaces: parking_lot::RwLock<HashSet<String>>,
    type_mapping: HashMap<&'static str, TopicType>,
}

impl TopicMap {
    pub fn new() -> Self {
        Self {
            topics: parking_lot::RwLock::new(HashMap::new()),
            robot_namespaces: parking_lot::RwLock::new(HashSet::new()),
            type_mapping: HashMap::from([
                ("sensor_msgs/msg/Image", TopicType::Image),
                ("sensor_msgs/Image", TopicType::Image),
                ("sensor_msgs/msg/PointCloud2", TopicType::PointCloud),
                ("sensor_msgs/PointCloud2", TopicType::PointCloud),
                ("sensor_msgs/msg/LaserScan", TopicType::LaserScan),
                ("sensor_msgs/LaserScan", TopicType::LaserScan),
                ("sensor_msgs/msg/Imu", TopicType::Imu),
                ("sensor_msgs/Imu", TopicType::Imu),
                ("nav_msgs/msg/Odometry", TopicType::Odometry),
                ("nav_msgs/Odometry", TopicType::Odometry),
                ("nav_msgs/msg/Path", TopicType::Path),
                ("nav_msgs/Path", TopicType::Path),
                ("geometry_msgs/msg/Twist", TopicType::Twist),
                ("geometry_msgs/Twist", TopicType::Twist),
                ("tf2_msgs/msg/TFMessage", TopicType::Tf),
                ("tf2_msgs/TFMessage", TopicType::Tf),
            ]),
        }
    }

    pub fn add_topic(
        &self,
        name: impl Into<String>,
        topic_type: TopicType,
        direction: TopicDirection,
    ) -> TopicInfo {
        let info = TopicInfo::new(name, topic_type, direction);
        if let Some(ns) = info.robot_namespace.clone() {
            self.robot_namespaces.write().insert(ns);
        }
        self.topics.write().insert(info.name.clone(), info.clone());
        publish_now(
            "topicmap.topic.discovered",
            serde_json::json!({
                "topic_name": info.name,
                "topic_type": info.topic_type.as_str(),
                "robot_namespace": info.robot_namespace,
                "sensor_name": info.sensor_name,
            }),
            EventPriority::Normal,
            "TopicMap",
        );
        info
    }

    pub fn add_topic_from_type_str(
        &self,
        name: impl Into<String>,
        type_str: &str,
        direction: TopicDirection,
    ) -> TopicInfo {
        let topic_type = self
            .type_mapping
            .get(type_str)
            .copied()
            .unwrap_or(TopicType::Unknown);
        self.add_topic(name, topic_type, direction)
    }

    pub fn remove_topic(&self, name: &str) -> bool {
        let removed = self.topics.write().remove(name);
        if let Some(info) = removed {
            publish_now(
                "topicmap.topic.removed",
                serde_json::json!({
                    "topic_name": name,
                    "robot_namespace": info.robot_namespace,
                }),
                EventPriority::Normal,
                "TopicMap",
            );
            return true;
        }
        false
    }

    pub fn get_topic(&self, name: &str) -> Option<TopicInfo> {
        self.topics.read().get(name).cloned()
    }

    pub fn get_topics_by_robot(&self, robot_namespace: &str) -> Vec<TopicInfo> {
        self.topics
            .read()
            .values()
            .filter(|topic| topic.robot_namespace.as_deref() == Some(robot_namespace))
            .cloned()
            .collect()
    }

    pub fn get_topics_by_type(&self, topic_type: TopicType) -> Vec<TopicInfo> {
        self.topics
            .read()
            .values()
            .filter(|topic| topic.topic_type == topic_type)
            .cloned()
            .collect()
    }

    pub fn get_sensor_topics(&self, robot_namespace: Option<&str>) -> Vec<TopicInfo> {
        self.topics
            .read()
            .values()
            .filter(|topic| topic.is_sensor_topic())
            .filter(|topic| {
                robot_namespace
                    .map(|ns| topic.robot_namespace.as_deref() == Some(ns))
                    .unwrap_or(true)
            })
            .cloned()
            .collect()
    }

    pub fn get_robot_namespaces(&self) -> HashSet<String> {
        self.robot_namespaces.read().clone()
    }

    pub fn get_all_topics(&self) -> Vec<TopicInfo> {
        self.topics.read().values().cloned().collect()
    }

    pub fn update_topic_activity(&self, name: &str, active: bool) {
        if let Some(topic) = self.topics.write().get_mut(name) {
            topic.active = active;
            topic.last_seen = now_sec();
            publish_now(
                "topicmap.topic.activity",
                serde_json::json!({
                    "topic_name": name,
                    "active": active,
                }),
                EventPriority::Low,
                "TopicMap",
            );
        }
    }

    pub fn create_topic_mapping(&self, robot_name: &str) -> HashMap<String, String> {
        let mut mapping = HashMap::new();
        for topic in self.get_topics_by_robot(robot_name) {
            if topic.is_sensor_topic() {
                if let Some(name) = topic.sensor_name {
                    mapping.insert(name, topic.name);
                }
            }
        }
        mapping
    }

    pub fn get_stats(&self) -> serde_json::Value {
        let topics = self.topics.read();
        let total_topics = topics.len();
        let active_topics = topics.values().filter(|t| t.active).count();
        let robot_count = self.robot_namespaces.read().len();
        let sensor_topics = topics.values().filter(|t| t.is_sensor_topic()).count();

        serde_json::json!({
            "total_topics": total_topics,
            "active_topics": active_topics,
            "robot_count": robot_count,
            "sensor_topics": sensor_topics,
        })
    }
}

impl Default for TopicMap {
    fn default() -> Self {
        Self::new()
    }
}

static GLOBAL_TOPIC_MAP: Lazy<TopicMap> = Lazy::new(TopicMap::new);

pub fn get_topic_map() -> &'static TopicMap {
    &GLOBAL_TOPIC_MAP
}
