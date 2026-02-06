#include "horus/core/types.hpp"

namespace horus {
namespace core {

std::string robot_type_to_string(RobotType type) {
    switch (type) {
        case RobotType::WHEELED: return "wheeled";
        case RobotType::LEGGED: return "legged";
        case RobotType::AERIAL: return "aerial";
        default: return "unknown";
    }
}

std::string sensor_type_to_string(SensorType type) {
    switch (type) {
        case SensorType::CAMERA: return "camera";
        case SensorType::LASER_SCAN: return "laser_scan";
        case SensorType::LIDAR_3D: return "lidar_3d";
        case SensorType::IMU: return "imu";
        case SensorType::GPS: return "gps";
        case SensorType::ODOMETRY: return "odometry";
        default: return "unknown";
    }
}

std::string topic_type_to_string(TopicType type) {
    switch (type) {
        case TopicType::IMAGE: return "sensor_msgs/Image";
        case TopicType::POINT_CLOUD: return "sensor_msgs/PointCloud2";
        case TopicType::LASER_SCAN: return "sensor_msgs/LaserScan";
        case TopicType::IMU: return "sensor_msgs/Imu";
        case TopicType::GPS: return "sensor_msgs/NavSatFix";
        case TopicType::ODOMETRY: return "nav_msgs/Odometry";
        case TopicType::PATH: return "nav_msgs/Path";
        case TopicType::OCCUPANCY_GRID: return "nav_msgs/OccupancyGrid";
        case TopicType::GOAL: return "geometry_msgs/PoseStamped";
        case TopicType::TWIST: return "geometry_msgs/Twist";
        case TopicType::JOINT_STATE: return "sensor_msgs/JointState";
        case TopicType::TF: return "tf2_msgs/TFMessage";
        case TopicType::TF_STATIC: return "tf2_msgs/TFMessage";
        case TopicType::ROBOT_STATUS: return "horus_interfaces/RobotStatus";
        case TopicType::ROBOT_COMMAND: return "horus_interfaces/RobotCommand";
        case TopicType::ROBOT_CONFIG: return "horus_interfaces/RobotConfig";
        case TopicType::UNKNOWN:
        default: return "unknown";
    }
}

std::string topic_direction_to_string(TopicDirection direction) {
    switch (direction) {
        case TopicDirection::PUBLISH: return "publish";
        case TopicDirection::SUBSCRIBE: return "subscribe";
        case TopicDirection::BIDIRECTIONAL: return "bidirectional";
        default: return "unknown";
    }
}

std::string data_source_type_to_string(DataSourceType type) {
    switch (type) {
        case DataSourceType::SENSOR: return "sensor";
        case DataSourceType::ROBOT_STATE: return "robot_state";
        case DataSourceType::ROBOT_TRANSFORM: return "robot_transform";
        case DataSourceType::ROBOT_GLOBAL_PATH: return "robot_global_path";
        case DataSourceType::ROBOT_LOCAL_PATH: return "robot_local_path";
        case DataSourceType::ROBOT_TRAJECTORY: return "robot_trajectory";
        case DataSourceType::OCCUPANCY_GRID: return "occupancy_grid";
        case DataSourceType::COSTMAP: return "costmap";
        case DataSourceType::MAP_3D: return "map_3d";
        case DataSourceType::GLOBAL_NAVIGATION_PATH: return "global_navigation_path";
        case DataSourceType::TF_TREE: return "tf_tree";
        case DataSourceType::GLOBAL_MARKERS: return "global_markers";
        case DataSourceType::COORDINATE_FRAME: return "coordinate_frame";
        default: return "unknown";
    }
}

std::string visualization_type_to_string(VisualizationType type) {
    switch (type) {
        case VisualizationType::CAMERA_FEED: return "camera_feed";
        case VisualizationType::LASER_SCAN: return "laser_scan";
        case VisualizationType::POINT_CLOUD: return "point_cloud";
        case VisualizationType::OCCUPANCY_GRID: return "occupancy_grid";
        case VisualizationType::TRAJECTORY: return "trajectory";
        case VisualizationType::PATH: return "path";
        case VisualizationType::MARKERS: return "markers";
        case VisualizationType::TRANSFORM_TREE: return "transform_tree";
        case VisualizationType::COORDINATE_AXES: return "coordinate_axes";
        case VisualizationType::MESH: return "mesh";
        case VisualizationType::HEATMAP: return "heatmap";
        default: return "unknown";
    }
}

std::string color_scheme_to_string(ColorScheme scheme) {
    switch (scheme) {
        case ColorScheme::BRIGHT: return "bright";
        case ColorScheme::PASTEL: return "pastel";
        case ColorScheme::DARK: return "dark";
        case ColorScheme::RAINBOW: return "rainbow";
        case ColorScheme::NEON: return "neon";
        default: return "unknown";
    }
}

} // namespace core
} // namespace horus
