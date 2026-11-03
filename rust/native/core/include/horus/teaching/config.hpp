#pragma once

#include <array>
#include <optional>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

namespace horus::teaching {
using Json = nlohmann::json;

struct TeachStream {
    std::string stream_id, topic, message_type;
    std::string role = "observation";
    bool required = true;
    double max_gap_seconds = 1.0;
    std::string clock_id = "ros", frame_id, source_identity = "unknown",
                stamp_semantics = "unknown";
    Json calibration = Json::object();
    std::string reliability = "best_effort", durability = "volatile";
    int queue_depth = 5;
    Json to_dict() const;
};
struct TeachPortal {
    std::string mode = "stereo";
    double edge_fade = 0.06, corner_radius = 0.08, crop = 0.02;
    bool swap_eyes = false;
    double max_skew_ms = 50.0;
    Json to_dict() const;
};
struct TeachCamera {
    std::string sensor_id, role = "front";
    std::array<double, 3> position{0.0, 1.35, 1.2};
    double width_m = 0.7;
    std::string stream_id;
    bool required_for_control = false;
    std::array<double, 3> rotation_deg{0.0, 0.0, 0.0};
    std::optional<TeachPortal> portal;
    Json to_dict() const;
};
struct TeachPointCloud {
    std::string stream_id;
    double point_size_m = 0.004, max_age_seconds = 1.0;
    int max_points = 1000000;
    Json to_dict() const;
};

// The normalized JSON is also the on-disk profile. Unknown keys are rejected.
class TeachConfig {
  public:
    explicit TeachConfig(Json config);
    explicit TeachConfig(const std::string &recording_root, Json options = Json::object());
    explicit TeachConfig(const char *recording_root, Json options = Json::object())
        : TeachConfig(std::string(recording_root ? recording_root : ""), std::move(options)) {}
    const Json &to_dict() const noexcept { return config_; }
    const std::string &digest() const noexcept { return digest_; }
    Json registration(const std::string &robot_name) const;
    TeachConfig with_options(const Json &) const;
    TeachConfig with_stream(const TeachStream &) const;
    TeachConfig with_camera(const TeachCamera &) const;
    TeachConfig with_spatial_view(const TeachPointCloud &) const;

  private:
    Json config_;
    std::string digest_;
};

std::string canonical(const Json &value);
std::string sha256(const std::string &value);
void validate_identifier(const std::string &value);
void validate_pose(const Json &value);
double monotonic_seconds();
std::uint64_t monotonic_ns();
std::uint64_t unix_ns();
std::string new_uuid();
void validate_uuid(const std::string &value);
bool retained_calibration(const Json &stream);
} // namespace horus::teaching
