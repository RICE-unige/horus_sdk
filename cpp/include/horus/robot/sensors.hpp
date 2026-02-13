#ifndef HORUS_ROBOT_SENSORS_HPP
#define HORUS_ROBOT_SENSORS_HPP

#include "horus/core/types.hpp"
#include <any>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace horus {
namespace robot {

class Sensor {
public:
    Sensor(std::string name, core::SensorType sensor_type, std::string frame_id, std::string topic);
    virtual ~Sensor() = default;

    const std::string& get_name() const { return name_; }
    core::SensorType get_sensor_type() const { return sensor_type_; }
    const std::string& get_frame_id() const { return frame_id_; }
    const std::string& get_topic() const { return topic_; }
    bool is_enabled() const { return enabled_; }

    void enable() { enabled_ = true; }
    void disable() { enabled_ = false; }

    void add_metadata(const std::string& key, const std::any& value);
    std::optional<std::any> get_metadata(const std::string& key) const;
    const core::Metadata& get_all_metadata() const { return metadata_; }

private:
    std::string name_;
    core::SensorType sensor_type_;
    std::string frame_id_;
    std::string topic_;
    bool enabled_{true};

protected:
    core::Metadata metadata_;
};

class Camera : public Sensor {
public:
    Camera(
        std::string name,
        std::string frame_id,
        std::string topic,
        bool is_stereo = false,
        std::pair<int, int> resolution = {640, 480},
        int fps = 30,
        float fov = 60.0f,
        std::string encoding = "bgr8",
        std::string streaming_type = "ros",
        std::string minimap_streaming_type = "ros",
        std::string teleop_streaming_type = "webrtc",
        std::string startup_mode = "minimap");

    static bool is_valid_transport(const std::string& value);
    static bool is_valid_startup_mode(const std::string& value);

    bool is_stereo() const { return is_stereo_; }
    std::pair<int, int> get_resolution() const { return resolution_; }
    int get_fps() const { return fps_; }
    float get_fov() const { return fov_; }
    const std::string& get_encoding() const { return encoding_; }
    const std::string& get_streaming_type() const { return streaming_type_; }
    const std::string& get_minimap_streaming_type() const { return minimap_streaming_type_; }
    const std::string& get_teleop_streaming_type() const { return teleop_streaming_type_; }
    const std::string& get_startup_mode() const { return startup_mode_; }

    void set_streaming_type(const std::string& value);
    void set_minimap_streaming_type(const std::string& value);
    void set_teleop_streaming_type(const std::string& value);
    void set_startup_mode(const std::string& value);
    void set_resolution(std::pair<int, int> resolution) { resolution_ = resolution; }
    void set_fps(int fps) { fps_ = fps; }
    void set_fov(float fov) { fov_ = fov; }
    void set_encoding(std::string encoding) { encoding_ = std::move(encoding); }

    std::string get_camera_type() const;
    std::string get_resolution_str() const;

private:
    bool is_stereo_{false};
    std::pair<int, int> resolution_{640, 480};
    int fps_{30};
    float fov_{60.0f};
    std::string encoding_{"bgr8"};
    std::string streaming_type_{"ros"};
    std::string minimap_streaming_type_{"ros"};
    std::string teleop_streaming_type_{"webrtc"};
    std::string startup_mode_{"minimap"};
};

class LaserScan : public Sensor {
public:
    LaserScan(
        std::string name,
        std::string frame_id,
        std::string topic,
        float min_angle = -3.14159f,
        float max_angle = 3.14159f,
        float angle_increment = 0.005f,
        float min_range = 0.1f,
        float max_range = 30.0f,
        float range_resolution = 0.01f,
        std::string color = "red",
        float point_size = 0.05f);

    float get_min_angle() const { return min_angle_; }
    float get_max_angle() const { return max_angle_; }
    float get_angle_increment() const { return angle_increment_; }
    float get_min_range() const { return min_range_; }
    float get_max_range() const { return max_range_; }
    float get_range_resolution() const { return range_resolution_; }
    const std::string& get_color() const { return color_; }
    float get_point_size() const { return point_size_; }

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

class Lidar3D : public Sensor {
public:
    Lidar3D(
        std::string name,
        std::string frame_id,
        std::string topic,
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
    float get_vertical_resolution() const { return vertical_resolution_; }
    float get_horizontal_resolution() const { return horizontal_resolution_; }
    float get_min_range() const { return min_range_; }
    float get_max_range() const { return max_range_; }
    int get_points_per_second() const { return points_per_second_; }
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

