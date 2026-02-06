use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Robot type classifications
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum RobotType {
    #[serde(rename = "wheeled")]
    Wheeled,
    #[serde(rename = "legged")]
    Legged,
    #[serde(rename = "aerial")]
    Aerial,
}

impl RobotType {
    pub fn as_str(&self) -> &'static str {
        match self {
            RobotType::Wheeled => "wheeled",
            RobotType::Legged => "legged",
            RobotType::Aerial => "aerial",
        }
    }
}

/// Sensor type classifications
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SensorType {
    #[serde(rename = "camera")]
    Camera,
    #[serde(rename = "laser_scan")]
    LaserScan,
    #[serde(rename = "lidar_3d")]
    Lidar3D,
    #[serde(rename = "imu")]
    IMU,
    #[serde(rename = "gps")]
    GPS,
    #[serde(rename = "odometry")]
    Odometry,
}

impl SensorType {
    pub fn as_str(&self) -> &'static str {
        match self {
            SensorType::Camera => "camera",
            SensorType::LaserScan => "laser_scan",
            SensorType::Lidar3D => "lidar_3d",
            SensorType::IMU => "imu",
            SensorType::GPS => "gps",
            SensorType::Odometry => "odometry",
        }
    }
}

/// ROS topic message types relevant to HORUS
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TopicType {
    // Sensor data topics
    Image,
    PointCloud,
    LaserScan,
    IMU,
    GPS,
    
    // Navigation topics
    Odometry,
    Path,
    OccupancyGrid,
    Goal,
    
    // Robot control topics
    Twist,
    JointState,
    
    // Transform topics
    TF,
    TFStatic,
    
    // HORUS-specific topics
    RobotStatus,
    RobotCommand,
    RobotConfig,
    
    // Generic/Unknown
    Unknown,
}

impl TopicType {
    pub fn as_str(&self) -> &'static str {
        match self {
            TopicType::Image => "sensor_msgs/Image",
            TopicType::PointCloud => "sensor_msgs/PointCloud2",
            TopicType::LaserScan => "sensor_msgs/LaserScan",
            TopicType::IMU => "sensor_msgs/Imu",
            TopicType::GPS => "sensor_msgs/NavSatFix",
            TopicType::Odometry => "nav_msgs/Odometry",
            TopicType::Path => "nav_msgs/Path",
            TopicType::OccupancyGrid => "nav_msgs/OccupancyGrid",
            TopicType::Goal => "geometry_msgs/PoseStamped",
            TopicType::Twist => "geometry_msgs/Twist",
            TopicType::JointState => "sensor_msgs/JointState",
            TopicType::TF => "tf2_msgs/TFMessage",
            TopicType::TFStatic => "tf2_msgs/TFMessage",
            TopicType::RobotStatus => "horus_interfaces/RobotStatus",
            TopicType::RobotCommand => "horus_interfaces/RobotCommand",
            TopicType::RobotConfig => "horus_interfaces/RobotConfig",
            TopicType::Unknown => "unknown",
        }
    }
}

/// Topic data direction
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TopicDirection {
    Publish,
    Subscribe,
    Bidirectional,
}

impl TopicDirection {
    pub fn as_str(&self) -> &'static str {
        match self {
            TopicDirection::Publish => "publish",
            TopicDirection::Subscribe => "subscribe",
            TopicDirection::Bidirectional => "bidirectional",
        }
    }
}

/// Event priority levels for processing order
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum EventPriority {
    Low = 0,
    Normal = 1,
    High = 2,
    Critical = 3,
}

/// Types of data sources for visualization
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DataSourceType {
    // Sensor-based data
    Sensor,
    
    // Robot-specific data
    RobotState,
    RobotTransform,
    RobotGlobalPath,
    RobotLocalPath,
    RobotTrajectory,
    
    // Environmental/World data (robot-independent)
    OccupancyGrid,
    Costmap,
    Map3D,
    GlobalNavigationPath,
    
    // Shared/Global data
    TFTree,
    GlobalMarkers,
    CoordinateFrame,
}

/// Types of data visualization rendering
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum VisualizationType {
    CameraFeed,
    LaserScan,
    PointCloud,
    OccupancyGrid,
    Trajectory,
    Path,
    Markers,
    TransformTree,
    CoordinateAxes,
    Mesh,
    Heatmap,
}

/// Predefined color schemes for robot visualizations
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ColorScheme {
    Bright,
    Pastel,
    Dark,
    Rainbow,
    Neon,
}

impl ColorScheme {
    pub fn as_str(&self) -> &'static str {
        match self {
            ColorScheme::Bright => "bright",
            ColorScheme::Pastel => "pastel",
            ColorScheme::Dark => "dark",
            ColorScheme::Rainbow => "rainbow",
            ColorScheme::Neon => "neon",
        }
    }
}

/// Type alias for metadata
pub type Metadata = HashMap<String, serde_json::Value>;
