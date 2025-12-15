#ifndef HORUS_ROBOT_SENSORS_HPP
#define HORUS_ROBOT_SENSORS_HPP

#include "horus/core/types.hpp"
#include <string>
#include <memory>
#include <optional>

namespace horus {
namespace robot {

/**
 * @brief Base sensor class for all sensor types
 */
class Sensor {
public:
    Sensor(const std::string& name, core::SensorType sensor_type,
           const std::string& frame_id, const std::string& topic);
    virtual ~Sensor() = default;
    
    // Getters
    std::string get_name() const { return name_; }
    core::SensorType get_sensor_type() const { return sensor_type_; }
    std::string get_frame_id() const { return frame_id_; }
    std::string get_topic() const { return topic_; }
    bool is_enabled() const { return enabled_; }
    
    // Control
    void enable() { enabled_ = true; }
    void disable() { enabled_ = false; }
    
    // Metadata
    void add_metadata(const std::string& key, const std::any& value);
    std::optional<std::any> get_metadata(const std::string& key) const;
    
protected:
    std::string name_;
    core::SensorType sensor_type_;
    std::string frame_id_;
    std::string topic_;
    bool enabled_;
    core::Metadata metadata_;
};

/**
 * @brief Camera sensor with vision-specific properties
 */
class Camera : public Sensor {
public:
    Camera(const std::string& name, const std::string& frame_id, 
           const std::string& topic,
           bool is_stereo = false,
           std::pair<int, int> resolution = {640, 480},
           int fps = 30,
           float fov = 60.0f,
           const std::string& encoding = "bgr8");
    
    bool is_stereo() const { return is_stereo_; }
    std::pair<int, int> get_resolution() const { return resolution_; }
    int get_fps() const { return fps_; }
    float get_fov() const { return fov_; }
    std::string get_encoding() const { return encoding_; }
    std::string get_camera_type() const;
    std::string get_resolution_str() const;
    
private:
    bool is_stereo_;
    std::pair<int, int> resolution_;
    int fps_;
    float fov_;
    std::string encoding_;
};

/**
 * @brief 2D Laser scanner sensor
 */
class LaserScan : public Sensor {
public:
    LaserScan(const std::string& name, const std::string& frame_id,
              const std::string& topic,
              float min_angle = -3.14159f,
              float max_angle = 3.14159f,
              float angle_increment = 0.005f,
              float min_range = 0.1f,
              float max_range = 30.0f,
              float range_resolution = 0.01f,
              const std::string& color = "red",
              float point_size = 0.05f);
    
    float get_min_angle() const { return min_angle_; }
    float get_max_angle() const { return max_angle_; }
    float get_angle_increment() const { return angle_increment_; }
    float get_min_range() const { return min_range_; }
    float get_max_range() const { return max_range_; }
    std::string get_color() const { return color_; }
    float get_scan_range_degrees() const;
    int get_num_points() const;
    
private:
    float min_angle_;
    float max_angle_;
    float angle_increment_;
    float min_range_;
    float max_range_;
    float range_resolution_;
    std::string color_;
    float point_size_;
};

/**
 * @brief 3D LiDAR sensor
 */
class Lidar3D : public Sensor {
public:
    Lidar3D(const std::string& name, const std::string& frame_id,
            const std::string& topic,
            float vertical_fov = 40.0f,
            float horizontal_fov = 360.0f,
            float vertical_resolution = 0.4f,
            float horizontal_resolution = 0.4f,
            float min_range = 0.5f,
            float max_range = 100.0f,
            int points_per_second = 1000000,
            int num_layers = 64);
    
    float get_vertical_fov() const { return vertical_fov_; }
    float get_horizontal_fov() const { return horizontal_fov_; }
    int get_num_layers() const { return num_layers_; }
    int get_point_cloud_size() const;
    std::string get_lidar_type() const;
    
private:
    float vertical_fov_;
    float horizontal_fov_;
    float vertical_resolution_;
    float horizontal_resolution_;
    float min_range_;
    float max_range_;
    int points_per_second_;
    int num_layers_;
};

} // namespace robot
} // namespace horus

#endif // HORUS_ROBOT_SENSORS_HPP
