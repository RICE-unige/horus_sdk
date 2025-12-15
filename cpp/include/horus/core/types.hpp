#ifndef HORUS_CORE_TYPES_HPP
#define HORUS_CORE_TYPES_HPP

#include <string>
#include <map>
#include <any>
#include <variant>

namespace horus {
namespace core {

/**
 * @brief Robot type classifications
 */
enum class RobotType {
    WHEELED,
    LEGGED,
    AERIAL
};

/**
 * @brief Sensor type classifications
 */
enum class SensorType {
    CAMERA,
    LASER_SCAN,
    LIDAR_3D,
    IMU,
    GPS,
    ODOMETRY
};

/**
 * @brief ROS topic message types relevant to HORUS
 */
enum class TopicType {
    // Sensor data topics
    IMAGE,
    POINT_CLOUD,
    LASER_SCAN,
    IMU,
    GPS,
    
    // Navigation topics
    ODOMETRY,
    PATH,
    OCCUPANCY_GRID,
    GOAL,
    
    // Robot control topics
    TWIST,
    JOINT_STATE,
    
    // Transform topics
    TF,
    TF_STATIC,
    
    // HORUS-specific topics
    ROBOT_STATUS,
    ROBOT_COMMAND,
    ROBOT_CONFIG,
    
    // Generic/Unknown
    UNKNOWN
};

/**
 * @brief Topic data direction
 */
enum class TopicDirection {
    PUBLISH,
    SUBSCRIBE,
    BIDIRECTIONAL
};

/**
 * @brief Event priority levels for processing order
 */
enum class EventPriority {
    LOW = 0,
    NORMAL = 1,
    HIGH = 2,
    CRITICAL = 3
};

/**
 * @brief Types of data sources for visualization
 */
enum class DataSourceType {
    // Sensor-based data
    SENSOR,
    
    // Robot-specific data
    ROBOT_STATE,
    ROBOT_TRANSFORM,
    ROBOT_GLOBAL_PATH,
    ROBOT_LOCAL_PATH,
    ROBOT_TRAJECTORY,
    
    // Environmental/World data (robot-independent)
    OCCUPANCY_GRID,
    COSTMAP,
    MAP_3D,
    GLOBAL_NAVIGATION_PATH,
    
    // Shared/Global data
    TF_TREE,
    GLOBAL_MARKERS,
    COORDINATE_FRAME
};

/**
 * @brief Types of data visualization rendering
 */
enum class VisualizationType {
    CAMERA_FEED,
    LASER_SCAN,
    POINT_CLOUD,
    OCCUPANCY_GRID,
    TRAJECTORY,
    PATH,
    MARKERS,
    TRANSFORM_TREE,
    COORDINATE_AXES,
    MESH,
    HEATMAP
};

/**
 * @brief Predefined color schemes for robot visualizations
 */
enum class ColorScheme {
    BRIGHT,
    PASTEL,
    DARK,
    RAINBOW,
    NEON
};

// Type aliases for convenience
using Metadata = std::map<std::string, std::any>;

// Conversion functions
std::string robot_type_to_string(RobotType type);
std::string sensor_type_to_string(SensorType type);
std::string topic_type_to_string(TopicType type);
std::string topic_direction_to_string(TopicDirection direction);
std::string data_source_type_to_string(DataSourceType type);
std::string visualization_type_to_string(VisualizationType type);
std::string color_scheme_to_string(ColorScheme scheme);

} // namespace core
} // namespace horus

#endif // HORUS_CORE_TYPES_HPP
