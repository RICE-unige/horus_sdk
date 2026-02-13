#ifndef HORUS_BRIDGE_ROBOT_REGISTRY_HPP
#define HORUS_BRIDGE_ROBOT_REGISTRY_HPP

#include "horus/robot/dataviz.hpp"
#include "horus/robot/robot.hpp"
#include <any>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace horus {
namespace bridge {

struct Vector3Payload {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

struct ResolutionPayload {
    int width{640};
    int height{480};
};

struct CameraConfigPayload {
    std::string streaming_type{"ros"};
    std::string minimap_streaming_type{"ros"};
    std::string teleop_streaming_type{"webrtc"};
    std::string startup_mode{"minimap"};
    std::string image_type{"raw"};
    std::string display_mode{"projected"};
    bool use_tf{true};
    std::string projection_target_frame;
    std::string webrtc_client_signal_topic{"/horus/webrtc/client_signal"};
    std::string webrtc_server_signal_topic{"/horus/webrtc/server_signal"};
    int webrtc_bitrate_kbps{2000};
    int webrtc_framerate{30};
    std::string webrtc_stun_server_url{"stun:stun.l.google.com:19302"};
    std::string webrtc_turn_server_url;
    std::string webrtc_turn_username;
    std::string webrtc_turn_credential;
    float image_scale{1.0f};
    float focal_length_scale{0.5f};
    Vector3Payload view_position_offset;
    Vector3Payload view_rotation_offset;
    bool show_frustum{true};
    std::string frustum_color{"#FFFF00"};
    float overhead_size{1.0f};
    Vector3Payload overhead_position_offset{0.0f, 2.0f, 0.0f};
    bool overhead_face_camera{true};
    Vector3Payload overhead_rotation_offset{90.0f, 0.0f, 0.0f};
    ResolutionPayload resolution;
    int fps{30};
    float fov{60.0f};
    std::string encoding{"bgr8"};
};

struct VizConfigPayload {
    std::string color{"white"};
    float point_size{0.05f};
};

struct SensorPayload {
    std::string name;
    std::string type;
    std::string topic;
    std::string frame;
    core::Metadata metadata;
    VizConfigPayload viz_config;
    std::optional<CameraConfigPayload> camera_config;
};

struct OccupancyPayload {
    bool show_unknown_space{true};
    std::optional<float> position_scale;
    std::optional<Vector3Payload> position_offset;
    std::optional<Vector3Payload> rotation_offset_euler;
};

struct VisualizationPayload {
    std::string type;
    std::string topic;
    std::string scope;
    bool enabled{true};
    std::optional<std::string> frame;
    std::optional<std::string> color;
    std::optional<OccupancyPayload> occupancy;
};

struct RobotManagerSectionsPayload {
    bool status{true};
    bool data_viz{true};
    bool teleop{true};
    bool tasks{true};
};

struct RobotManagerConfigPayload {
    bool enabled{true};
    std::string prefab_asset_path{"Assets/Prefabs/UI/RobotManager.prefab"};
    std::string prefab_resource_path;
    RobotManagerSectionsPayload sections;
};

struct DimensionsPayload {
    float length{0.0f};
    float width{0.0f};
    float height{0.0f};
};

struct WorkspaceConfigPayload {
    float position_scale{1.0f};
};

struct ControlPayload {
    std::string drive_topic;
};

struct RobotRegistrationPayload {
    std::string action{"register"};
    std::string robot_name;
    std::string robot_type;
    std::vector<SensorPayload> sensors;
    std::vector<VisualizationPayload> visualizations;
    std::vector<VisualizationPayload> global_visualizations;
    ControlPayload control;
    RobotManagerConfigPayload robot_manager_config;
    double timestamp{0.0};
    std::optional<DimensionsPayload> dimensions;
    std::optional<WorkspaceConfigPayload> workspace_config;
};

class RobotRegistryClient {
public:
    RobotRegistryClient();

    std::pair<bool, std::map<std::string, std::any>> register_robot(
        robot::Robot& robot,
        const robot::DataViz& dataviz,
        double timeout_sec = 10.0,
        bool keep_alive = false,
        bool show_dashboard = true,
        std::optional<double> workspace_scale = std::nullopt);

    std::pair<bool, std::map<std::string, std::any>> register_robots(
        std::vector<robot::Robot*>& robots,
        std::vector<robot::DataViz> datavizs = {},
        double timeout_sec = 10.0,
        bool keep_alive = true,
        bool show_dashboard = true,
        std::optional<double> workspace_scale = std::nullopt);

    std::pair<bool, std::map<std::string, std::any>> unregister_robot(
        const std::string& robot_id,
        double timeout_sec = 5.0);

    bool check_backend_availability() const;

    RobotRegistrationPayload build_robot_config_dict(
        const robot::Robot& robot,
        const robot::DataViz& dataviz,
        std::optional<std::vector<VisualizationPayload>> global_visualizations = std::nullopt,
        std::optional<double> workspace_scale = std::nullopt) const;

    std::vector<VisualizationPayload> build_global_visualizations_payload(
        const std::vector<robot::DataViz>& datavizs) const;

    std::unordered_map<std::string, std::unordered_map<std::string, std::string>>
    extract_camera_topic_profiles(const RobotRegistrationPayload& payload) const;

private:
    bool registration_in_progress_{false};
};

RobotRegistryClient& get_robot_registry_client();

std::vector<VisualizationPayload> build_global_visualizations_payload(
    const std::vector<robot::DataViz>& datavizs);

RobotRegistrationPayload build_robot_config_dict(
    const robot::Robot& robot,
    const robot::DataViz& dataviz,
    std::optional<double> workspace_scale = std::nullopt);

std::optional<std::string> queued_reason_from_ack(const std::map<std::string, std::any>& ack);

} // namespace bridge
} // namespace horus

#endif // HORUS_BRIDGE_ROBOT_REGISTRY_HPP

