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
    std::string minimap_topic;
    std::string teleop_topic;
    std::string minimap_image_type{"raw"};
    std::string teleop_image_type{"raw"};
    int minimap_max_fps{30};
    std::string teleop_stereo_layout{"mono"};
    std::string teleop_right_topic;
    std::string startup_mode{"minimap"};
    bool is_stereo{false};
    std::string stereo_layout{"mono"};
    std::string right_topic;
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
    bool immersive_ros_flip_x{false};
    bool immersive_ros_flip_y{false};
    Vector3Payload view_position_offset;
    Vector3Payload view_rotation_offset;
    Vector3Payload projected_position_offset;
    Vector3Payload projected_scale_multiplier{1.0f, 1.0f, 1.0f};
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
    std::map<std::string, std::any> path;
    std::map<std::string, std::any> velocity;
    std::map<std::string, std::any> trail;
    std::map<std::string, std::any> collision;
    std::map<std::string, std::any> point_cloud;
    std::map<std::string, std::any> mesh;
    std::map<std::string, std::any> octomap;
    std::map<std::string, std::any> semantic_box;
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
    std::optional<float> position_scale;
    std::map<std::string, std::any> compass;
    std::map<std::string, std::any> tutorial;
};

struct TeleopControlPayload {
    bool enabled{true};
    std::string command_topic;
    std::string raw_input_topic;
    std::string head_pose_topic;
    std::string robot_profile{"wheeled"};
    std::string response_mode{"analog"};
    double publish_rate_hz{30.0};
    bool custom_passthrough_only{false};
    std::map<std::string, std::any> deadman;
    std::map<std::string, std::any> axes;
    std::map<std::string, std::any> discrete;
};

struct TaskControlPayload {
    std::map<std::string, std::any> go_to_point;
    std::map<std::string, std::any> waypoint;
};

struct ControlPayload {
    std::string drive_topic;
    TeleopControlPayload teleop;
    TaskControlPayload tasks;
};

struct RosBindingPayload {
    std::string logical_name;
    std::string tf_mode{"prefixed"};
    std::string topic_mode{"prefixed"};
    std::string base_frame{"base_link"};
    std::string tf_prefix;
    std::string topic_prefix;
};

struct RobotRegistrationPayload {
    std::string action{"register"};
    std::string robot_name;
    std::string robot_type;
    RosBindingPayload ros_binding;
    std::vector<SensorPayload> sensors;
    std::vector<VisualizationPayload> visualizations;
    std::vector<VisualizationPayload> global_visualizations;
    ControlPayload control;
    RobotManagerConfigPayload robot_manager_config;
    double timestamp{0.0};
    std::optional<DimensionsPayload> dimensions;
    std::optional<WorkspaceConfigPayload> workspace_config;
    std::optional<std::string> robot_model_id;
    std::optional<bool> has_visual_mesh_model;
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
