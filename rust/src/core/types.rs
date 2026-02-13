use serde::{Deserialize, Serialize};
use std::collections::HashMap;

pub type Metadata = HashMap<String, serde_json::Value>;

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
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Wheeled => "wheeled",
            Self::Legged => "legged",
            Self::Aerial => "aerial",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SensorType {
    #[serde(rename = "camera")]
    Camera,
    #[serde(rename = "laser_scan")]
    LaserScan,
    #[serde(rename = "lidar_3d")]
    Lidar3D,
    #[serde(rename = "imu")]
    Imu,
    #[serde(rename = "gps")]
    Gps,
    #[serde(rename = "odometry")]
    Odometry,
}

impl SensorType {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Camera => "camera",
            Self::LaserScan => "laser_scan",
            Self::Lidar3D => "lidar_3d",
            Self::Imu => "imu",
            Self::Gps => "gps",
            Self::Odometry => "odometry",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TopicType {
    Image,
    PointCloud,
    LaserScan,
    Imu,
    Gps,
    Odometry,
    Path,
    OccupancyGrid,
    Goal,
    Twist,
    JointState,
    Tf,
    TfStatic,
    RobotStatus,
    RobotCommand,
    RobotConfig,
    Unknown,
}

impl TopicType {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Image => "sensor_msgs/Image",
            Self::PointCloud => "sensor_msgs/PointCloud2",
            Self::LaserScan => "sensor_msgs/LaserScan",
            Self::Imu => "sensor_msgs/Imu",
            Self::Gps => "sensor_msgs/NavSatFix",
            Self::Odometry => "nav_msgs/Odometry",
            Self::Path => "nav_msgs/Path",
            Self::OccupancyGrid => "nav_msgs/OccupancyGrid",
            Self::Goal => "geometry_msgs/PoseStamped",
            Self::Twist => "geometry_msgs/Twist",
            Self::JointState => "sensor_msgs/JointState",
            Self::Tf => "tf2_msgs/TFMessage",
            Self::TfStatic => "tf2_msgs/TFMessage",
            Self::RobotStatus => "horus_interfaces/RobotStatus",
            Self::RobotCommand => "horus_interfaces/RobotCommand",
            Self::RobotConfig => "horus_interfaces/RobotConfig",
            Self::Unknown => "unknown",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TopicDirection {
    Publish,
    Subscribe,
    Bidirectional,
}

impl TopicDirection {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Publish => "publish",
            Self::Subscribe => "subscribe",
            Self::Bidirectional => "bidirectional",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum EventPriority {
    Low = 0,
    Normal = 1,
    High = 2,
    Critical = 3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DataSourceType {
    Sensor,
    RobotState,
    RobotTransform,
    RobotGlobalPath,
    RobotLocalPath,
    RobotTrajectory,
    OccupancyGrid,
    Costmap,
    Map3D,
    GlobalNavigationPath,
    TfTree,
    GlobalMarkers,
    CoordinateFrame,
}

impl DataSourceType {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Sensor => "sensor",
            Self::RobotState => "robot_state",
            Self::RobotTransform => "robot_transform",
            Self::RobotGlobalPath => "robot_global_path",
            Self::RobotLocalPath => "robot_local_path",
            Self::RobotTrajectory => "robot_trajectory",
            Self::OccupancyGrid => "occupancy_grid",
            Self::Costmap => "costmap",
            Self::Map3D => "map_3d",
            Self::GlobalNavigationPath => "global_navigation_path",
            Self::TfTree => "tf_tree",
            Self::GlobalMarkers => "global_markers",
            Self::CoordinateFrame => "coordinate_frame",
        }
    }
}

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

impl VisualizationType {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::CameraFeed => "camera_feed",
            Self::LaserScan => "laser_scan",
            Self::PointCloud => "point_cloud",
            Self::OccupancyGrid => "occupancy_grid",
            Self::Trajectory => "trajectory",
            Self::Path => "path",
            Self::Markers => "markers",
            Self::TransformTree => "transform_tree",
            Self::CoordinateAxes => "coordinate_axes",
            Self::Mesh => "mesh",
            Self::Heatmap => "heatmap",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ColorScheme {
    Bright,
    Pastel,
    Dark,
    Rainbow,
    Neon,
}

impl ColorScheme {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Bright => "bright",
            Self::Pastel => "pastel",
            Self::Dark => "dark",
            Self::Rainbow => "rainbow",
            Self::Neon => "neon",
        }
    }
}
