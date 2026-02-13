#include "horus/robot/sensors.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace horus {
namespace robot {

namespace {
std::string trim(std::string value) {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
    value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
    return value;
}

std::string normalize_lower(std::string value) {
    value = trim(std::move(value));
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}
} // namespace

Sensor::Sensor(std::string name, core::SensorType sensor_type, std::string frame_id, std::string topic)
    : name_(std::move(name)),
      sensor_type_(sensor_type),
      frame_id_(std::move(frame_id)),
      topic_(std::move(topic)) {
    if (name_.empty()) {
        throw std::invalid_argument("Sensor name cannot be empty");
    }
    if (frame_id_.empty()) {
        throw std::invalid_argument("Sensor frame_id cannot be empty");
    }
    if (topic_.empty()) {
        throw std::invalid_argument("Sensor topic cannot be empty");
    }
}

void Sensor::add_metadata(const std::string& key, const std::any& value) {
    metadata_[key] = value;
}

std::optional<std::any> Sensor::get_metadata(const std::string& key) const {
    auto it = metadata_.find(key);
    if (it == metadata_.end()) {
        return std::nullopt;
    }
    return it->second;
}

Camera::Camera(
    std::string name,
    std::string frame_id,
    std::string topic,
    bool is_stereo,
    std::pair<int, int> resolution,
    int fps,
    float fov,
    std::string encoding,
    std::string streaming_type,
    std::string minimap_streaming_type,
    std::string teleop_streaming_type,
    std::string startup_mode)
    : Sensor(std::move(name), core::SensorType::CAMERA, std::move(frame_id), std::move(topic)),
      is_stereo_(is_stereo),
      resolution_(resolution),
      fps_(fps),
      fov_(fov),
      encoding_(std::move(encoding)) {
    set_streaming_type(streaming_type);
    set_minimap_streaming_type(minimap_streaming_type);
    set_teleop_streaming_type(teleop_streaming_type);
    set_startup_mode(startup_mode);
}

bool Camera::is_valid_transport(const std::string& value) {
    const std::string normalized = normalize_lower(value);
    return normalized == "ros" || normalized == "webrtc";
}

bool Camera::is_valid_startup_mode(const std::string& value) {
    const std::string normalized = normalize_lower(value);
    return normalized == "minimap" || normalized == "teleop";
}

void Camera::set_streaming_type(const std::string& value) {
    const auto normalized = normalize_lower(value);
    if (normalized.empty()) {
        streaming_type_.clear();
        return;
    }
    if (!is_valid_transport(normalized)) {
        throw std::invalid_argument("Camera streaming_type must be 'ros' or 'webrtc'");
    }
    streaming_type_ = normalized;
}

void Camera::set_minimap_streaming_type(const std::string& value) {
    const auto normalized = normalize_lower(value);
    if (normalized.empty()) {
        minimap_streaming_type_.clear();
        return;
    }
    if (!is_valid_transport(normalized)) {
        throw std::invalid_argument("Camera minimap_streaming_type must be 'ros' or 'webrtc'");
    }
    minimap_streaming_type_ = normalized;
}

void Camera::set_teleop_streaming_type(const std::string& value) {
    const auto normalized = normalize_lower(value);
    if (normalized.empty()) {
        teleop_streaming_type_.clear();
        return;
    }
    if (!is_valid_transport(normalized)) {
        throw std::invalid_argument("Camera teleop_streaming_type must be 'ros' or 'webrtc'");
    }
    teleop_streaming_type_ = normalized;
}

void Camera::set_startup_mode(const std::string& value) {
    const auto normalized = normalize_lower(value);
    if (normalized.empty()) {
        startup_mode_.clear();
        return;
    }
    if (!is_valid_startup_mode(normalized)) {
        throw std::invalid_argument("Camera startup_mode must be 'minimap' or 'teleop'");
    }
    startup_mode_ = normalized;
}

std::string Camera::get_camera_type() const {
    return is_stereo_ ? "stereo" : "mono";
}

std::string Camera::get_resolution_str() const {
    std::ostringstream stream;
    stream << resolution_.first << "x" << resolution_.second;
    return stream.str();
}

LaserScan::LaserScan(
    std::string name,
    std::string frame_id,
    std::string topic,
    float min_angle,
    float max_angle,
    float angle_increment,
    float min_range,
    float max_range,
    float range_resolution,
    std::string color,
    float point_size)
    : Sensor(std::move(name), core::SensorType::LASER_SCAN, std::move(frame_id), std::move(topic)),
      min_angle_(min_angle),
      max_angle_(max_angle),
      angle_increment_(angle_increment),
      min_range_(min_range),
      max_range_(max_range),
      range_resolution_(range_resolution),
      color_(std::move(color)),
      point_size_(point_size) {}

float LaserScan::get_scan_range_degrees() const {
    return (max_angle_ - min_angle_) * 180.0f / 3.14159f;
}

int LaserScan::get_num_points() const {
    return static_cast<int>((max_angle_ - min_angle_) / angle_increment_);
}

Lidar3D::Lidar3D(
    std::string name,
    std::string frame_id,
    std::string topic,
    float vertical_fov,
    float horizontal_fov,
    float vertical_resolution,
    float horizontal_resolution,
    float min_range,
    float max_range,
    int points_per_second,
    int num_layers)
    : Sensor(std::move(name), core::SensorType::LIDAR_3D, std::move(frame_id), std::move(topic)),
      vertical_fov_(vertical_fov),
      horizontal_fov_(horizontal_fov),
      vertical_resolution_(vertical_resolution),
      horizontal_resolution_(horizontal_resolution),
      min_range_(min_range),
      max_range_(max_range),
      points_per_second_(points_per_second),
      num_layers_(num_layers) {}

int Lidar3D::get_point_cloud_size() const {
    const auto h_points = static_cast<int>(horizontal_fov_ / horizontal_resolution_);
    const auto v_points = static_cast<int>(vertical_fov_ / vertical_resolution_);
    return h_points * v_points;
}

std::string Lidar3D::get_lidar_type() const {
    return std::to_string(num_layers_) + "-layer 3D LiDAR";
}

} // namespace robot
} // namespace horus
