use crate::core::types::{TopicDirection, TopicType};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Information about a ROS topic
#[derive(Debug, Clone)]
pub struct TopicInfo {
    pub name: String,
    pub topic_type: TopicType,
    pub direction: TopicDirection,
    pub robot_namespace: Option<String>,
    pub sensor_name: Option<String>,
    pub frame_id: Option<String>,
    pub active: bool,
    pub metadata: HashMap<String, serde_json::Value>,
}

impl TopicInfo {
    pub fn new(name: String, topic_type: TopicType, direction: TopicDirection) -> Self {
        Self {
            name,
            topic_type,
            direction,
            robot_namespace: None,
            sensor_name: None,
            frame_id: None,
            active: true,
            metadata: HashMap::new(),
        }
    }

    pub fn is_sensor_topic(&self) -> bool {
        matches!(
            self.topic_type,
            TopicType::Image | TopicType::PointCloud | TopicType::LaserScan | TopicType::IMU | TopicType::GPS
        )
    }

    pub fn is_robot_topic(&self) -> bool {
        self.robot_namespace.is_some()
    }
}

/// ROS topic discovery and mapping system for HORUS SDK
pub struct TopicMap {
    topics: Arc<RwLock<HashMap<String, TopicInfo>>>,
}

impl TopicMap {
    pub fn new() -> Self {
        Self {
            topics: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    pub async fn add_topic(&self, topic_info: TopicInfo) {
        let mut topics = self.topics.write().await;
        topics.insert(topic_info.name.clone(), topic_info);
    }

    pub async fn get_topic(&self, name: &str) -> Option<TopicInfo> {
        let topics = self.topics.read().await;
        topics.get(name).cloned()
    }

    pub async fn get_all_topics(&self) -> Vec<TopicInfo> {
        let topics = self.topics.read().await;
        topics.values().cloned().collect()
    }
}

impl Default for TopicMap {
    fn default() -> Self {
        Self::new()
    }
}

static GLOBAL_TOPIC_MAP: once_cell::sync::Lazy<TopicMap> =
    once_cell::sync::Lazy::new(TopicMap::new);

pub fn get_topic_map() -> &'static TopicMap {
    &GLOBAL_TOPIC_MAP
}
