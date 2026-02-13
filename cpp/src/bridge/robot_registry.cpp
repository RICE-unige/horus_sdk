#include "horus/bridge/robot_registry.hpp"

#include "horus/topics.hpp"
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <limits>
#include <mutex>
#include <set>
#include <tuple>

namespace horus {
namespace bridge {

namespace {

template <typename T>
std::optional<T> any_cast_optional(const std::any& value) {
    if (value.type() == typeid(T)) {
        return std::any_cast<T>(value);
    }
    return std::nullopt;
}

std::optional<std::string> any_to_string(const std::any& value) {
    if (value.type() == typeid(std::string)) {
        return std::any_cast<std::string>(value);
    }
    if (value.type() == typeid(const char*)) {
        return std::string(std::any_cast<const char*>(value));
    }
    return std::nullopt;
}

bool coerce_bool(const std::optional<std::any>& value, bool default_value) {
    if (!value) {
        return default_value;
    }
    if (value->type() == typeid(bool)) {
        return std::any_cast<bool>(*value);
    }
    if (value->type() == typeid(int)) {
        return std::any_cast<int>(*value) != 0;
    }
    if (value->type() == typeid(double)) {
        return std::any_cast<double>(*value) != 0.0;
    }
    if (value->type() == typeid(float)) {
        return std::any_cast<float>(*value) != 0.0f;
    }
    if (value->type() == typeid(std::string)) {
        std::string normalized = std::any_cast<std::string>(*value);
        std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on") {
            return true;
        }
        if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off") {
            return false;
        }
    }
    return default_value;
}

float coerce_float(const std::optional<std::any>& value, float default_value) {
    if (!value) {
        return default_value;
    }
    if (value->type() == typeid(float)) {
        return std::any_cast<float>(*value);
    }
    if (value->type() == typeid(double)) {
        return static_cast<float>(std::any_cast<double>(*value));
    }
    if (value->type() == typeid(int)) {
        return static_cast<float>(std::any_cast<int>(*value));
    }
    if (value->type() == typeid(std::string)) {
        try {
            return std::stof(std::any_cast<std::string>(*value));
        } catch (...) {
            return default_value;
        }
    }
    return default_value;
}

int coerce_int(const std::optional<std::any>& value, int default_value) {
    if (!value) {
        return default_value;
    }
    if (value->type() == typeid(int)) {
        return std::any_cast<int>(*value);
    }
    if (value->type() == typeid(double)) {
        return static_cast<int>(std::any_cast<double>(*value));
    }
    if (value->type() == typeid(float)) {
        return static_cast<int>(std::any_cast<float>(*value));
    }
    if (value->type() == typeid(std::string)) {
        try {
            return std::stoi(std::any_cast<std::string>(*value));
        } catch (...) {
            return default_value;
        }
    }
    return default_value;
}

Vector3Payload coerce_vec3(const std::optional<std::any>& value, const Vector3Payload& default_value) {
    if (!value) {
        return default_value;
    }
    if (value->type() == typeid(Vector3Payload)) {
        return std::any_cast<Vector3Payload>(*value);
    }
    if (value->type() == typeid(std::vector<float>)) {
        const auto vector = std::any_cast<std::vector<float>>(*value);
        if (vector.size() == 3) {
            return Vector3Payload{vector[0], vector[1], vector[2]};
        }
    }
    if (value->type() == typeid(std::vector<double>)) {
        const auto vector = std::any_cast<std::vector<double>>(*value);
        if (vector.size() == 3) {
            return Vector3Payload{
                static_cast<float>(vector[0]),
                static_cast<float>(vector[1]),
                static_cast<float>(vector[2])};
        }
    }
    return default_value;
}

std::string normalize_transport(std::string value, const std::string& fallback) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (value == "ros" || value == "webrtc") {
        return value;
    }
    return fallback;
}

std::string to_lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

double now_sec() {
    using clock = std::chrono::system_clock;
    const auto now = clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now).count() / 1000.0;
}

std::vector<std::string> collect_topics(const robot::DataViz& dataviz) {
    std::vector<std::string> topics;
    for (const auto& visualization : dataviz.get_enabled_visualizations()) {
        if (visualization.data_source.topic.empty()) {
            continue;
        }
        if (std::find(topics.begin(), topics.end(), visualization.data_source.topic) == topics.end()) {
            topics.push_back(visualization.data_source.topic);
        }
    }
    return topics;
}

} // namespace

RobotRegistryClient::RobotRegistryClient() = default;

std::pair<bool, std::map<std::string, std::any>> RobotRegistryClient::register_robot(
    robot::Robot& robot,
    const robot::DataViz& dataviz,
    double timeout_sec,
    bool keep_alive,
    bool show_dashboard,
    std::optional<double> workspace_scale) {
    if (registration_in_progress_) {
        return {false, {{"error", std::string("Registration already in progress")}}};
    }
    registration_in_progress_ = true;

    const auto global_visualizations = build_global_visualizations_payload({dataviz});
    auto payload =
        build_robot_config_dict(robot, dataviz, global_visualizations, workspace_scale);

    std::map<std::string, std::any> ack;
    ack["success"] = true;
    ack["robot_name"] = robot.get_name();
    ack["robot_id"] = robot.get_name();
    ack["assigned_color"] = std::string("#00FF00");
    ack["payload"] = payload;
    ack["timeout_sec"] = timeout_sec;
    ack["keep_alive"] = keep_alive;
    ack["show_dashboard"] = show_dashboard;

    robot.add_metadata("horus_robot_id", robot.get_name());
    robot.add_metadata("horus_color", std::string("#00FF00"));
    robot.add_metadata("horus_registered", true);
    robot.add_metadata("horus_topics", collect_topics(dataviz));

    registration_in_progress_ = false;
    return {true, ack};
}

std::pair<bool, std::map<std::string, std::any>> RobotRegistryClient::register_robots(
    std::vector<robot::Robot*>& robots,
    std::vector<robot::DataViz> datavizs,
    double timeout_sec,
    bool keep_alive,
    bool show_dashboard,
    std::optional<double> workspace_scale) {
    if (registration_in_progress_) {
        return {false, {{"error", std::string("Registration already in progress")}}};
    }
    registration_in_progress_ = true;

    if (datavizs.empty()) {
        datavizs.reserve(robots.size());
        for (const auto* robot : robots) {
            if (!robot) {
                registration_in_progress_ = false;
                return {false, {{"error", std::string("Null robot pointer")}}};
            }
            datavizs.push_back(*robot->create_dataviz());
        }
    }

    if (datavizs.size() != robots.size()) {
        registration_in_progress_ = false;
        return {false, {{"error", std::string("Robot/dataviz length mismatch")}}};
    }

    const auto global_visualizations = build_global_visualizations_payload(datavizs);
    std::vector<std::map<std::string, std::any>> results;
    results.reserve(robots.size());

    for (std::size_t i = 0; i < robots.size(); ++i) {
        auto* robot = robots[i];
        if (!robot) {
            registration_in_progress_ = false;
            return {false, {{"error", std::string("Null robot pointer")}}};
        }
        auto payload =
            build_robot_config_dict(*robot, datavizs[i], global_visualizations, workspace_scale);

        std::map<std::string, std::any> ack;
        ack["success"] = true;
        ack["robot_name"] = robot->get_name();
        ack["robot_id"] = robot->get_name();
        ack["assigned_color"] = std::string("#00FF00");
        ack["payload"] = payload;
        ack["timeout_sec"] = timeout_sec;
        ack["keep_alive"] = keep_alive;
        ack["show_dashboard"] = show_dashboard;
        results.push_back(ack);

        robot->add_metadata("horus_robot_id", robot->get_name());
        robot->add_metadata("horus_color", std::string("#00FF00"));
        robot->add_metadata("horus_registered", true);
        robot->add_metadata("horus_topics", collect_topics(datavizs[i]));
    }

    registration_in_progress_ = false;
    return {true, {{"success", true}, {"results", results}}};
}

std::pair<bool, std::map<std::string, std::any>> RobotRegistryClient::unregister_robot(
    const std::string& robot_id,
    double timeout_sec) {
    return {
        true,
        {
            {"message", std::string("Unregistered (Local cleanup only)")},
            {"robot_id", robot_id},
            {"timeout_sec", timeout_sec},
        }
    };
}

bool RobotRegistryClient::check_backend_availability() const {
    return true;
}

RobotRegistrationPayload RobotRegistryClient::build_robot_config_dict(
    const robot::Robot& robot,
    const robot::DataViz& dataviz,
    std::optional<std::vector<VisualizationPayload>> global_visualizations,
    std::optional<double> workspace_scale) const {
    RobotRegistrationPayload payload;
    payload.robot_name = robot.get_name();
    payload.robot_type = robot.get_type_str();
    payload.control.drive_topic = "/" + robot.get_name() + "/cmd_vel";
    payload.timestamp = now_sec();

    if (robot.get_dimensions()) {
        const auto& dimensions = *robot.get_dimensions();
        payload.dimensions = DimensionsPayload{
            static_cast<float>(dimensions.length),
            static_cast<float>(dimensions.width),
            static_cast<float>(dimensions.height)};
    }

    for (const auto& sensor : robot.get_sensors()) {
        if (!sensor) {
            continue;
        }

        SensorPayload sensor_payload;
        sensor_payload.name = sensor->get_name();
        sensor_payload.type = core::sensor_type_to_string(sensor->get_sensor_type());
        sensor_payload.topic = sensor->get_topic();
        sensor_payload.frame = sensor->get_frame_id();
        sensor_payload.metadata = sensor->get_all_metadata();

        if (auto scan = std::dynamic_pointer_cast<robot::LaserScan>(sensor)) {
            sensor_payload.viz_config.color = scan->get_color();
            sensor_payload.viz_config.point_size = scan->get_point_size();
        } else {
            sensor_payload.viz_config.color = "white";
            sensor_payload.viz_config.point_size = 0.05f;
        }

        if (auto camera = std::dynamic_pointer_cast<robot::Camera>(sensor)) {
            CameraConfigPayload camera_payload;
            const auto metadata = camera->get_all_metadata();
            const auto streaming_metadata = any_to_string(metadata.contains("streaming_type")
                                                             ? metadata.at("streaming_type")
                                                             : std::any{});

            auto legacy_streaming = normalize_transport(streaming_metadata.value_or(""), "");
            if (legacy_streaming.empty()) {
                legacy_streaming = normalize_transport(camera->get_streaming_type(), "ros");
            }
            if (legacy_streaming.empty()) {
                legacy_streaming = "ros";
            }

            auto minimap = normalize_transport(
                metadata.contains("minimap_streaming_type")
                    ? any_to_string(metadata.at("minimap_streaming_type")).value_or("")
                    : std::string(),
                "");
            if (minimap.empty()) {
                minimap = normalize_transport(camera->get_minimap_streaming_type(), "");
            }
            if (minimap.empty()) {
                minimap = normalize_transport(legacy_streaming, "ros");
            }

            auto teleop = normalize_transport(
                metadata.contains("teleop_streaming_type")
                    ? any_to_string(metadata.at("teleop_streaming_type")).value_or("")
                    : std::string(),
                "");
            if (teleop.empty()) {
                teleop = normalize_transport(camera->get_teleop_streaming_type(), "");
            }
            if (teleop.empty()) {
                teleop = normalize_transport(legacy_streaming, "webrtc");
            }
            auto startup = to_lower(metadata.contains("startup_mode")
                                        ? any_to_string(metadata.at("startup_mode")).value_or("")
                                        : camera->get_startup_mode());
            if (startup != "minimap" && startup != "teleop") {
                startup = "minimap";
            }

            camera_payload.streaming_type = legacy_streaming;
            camera_payload.minimap_streaming_type = minimap;
            camera_payload.teleop_streaming_type = teleop;
            camera_payload.startup_mode = startup;
            camera_payload.image_type =
                to_lower(metadata.contains("image_type")
                             ? any_to_string(metadata.at("image_type")).value_or("raw")
                             : std::string("raw"));
            camera_payload.display_mode =
                to_lower(metadata.contains("display_mode")
                             ? any_to_string(metadata.at("display_mode")).value_or("projected")
                             : std::string("projected"));
            camera_payload.use_tf =
                coerce_bool(metadata.contains("use_tf")
                                ? std::optional<std::any>(metadata.at("use_tf"))
                                : std::nullopt,
                            true);
            camera_payload.projection_target_frame =
                metadata.contains("projection_target_frame")
                    ? any_to_string(metadata.at("projection_target_frame")).value_or("")
                    : "";
            camera_payload.webrtc_client_signal_topic =
                metadata.contains("webrtc_client_signal_topic")
                    ? any_to_string(metadata.at("webrtc_client_signal_topic"))
                          .value_or("/horus/webrtc/client_signal")
                    : "/horus/webrtc/client_signal";
            camera_payload.webrtc_server_signal_topic =
                metadata.contains("webrtc_server_signal_topic")
                    ? any_to_string(metadata.at("webrtc_server_signal_topic"))
                          .value_or("/horus/webrtc/server_signal")
                    : "/horus/webrtc/server_signal";
            camera_payload.webrtc_bitrate_kbps = coerce_int(
                metadata.contains("webrtc_bitrate_kbps")
                    ? std::optional<std::any>(metadata.at("webrtc_bitrate_kbps"))
                    : std::nullopt,
                2000);
            camera_payload.webrtc_framerate = coerce_int(
                metadata.contains("webrtc_framerate")
                    ? std::optional<std::any>(metadata.at("webrtc_framerate"))
                    : std::nullopt,
                camera->get_fps());
            camera_payload.webrtc_stun_server_url =
                metadata.contains("webrtc_stun_server_url")
                    ? any_to_string(metadata.at("webrtc_stun_server_url"))
                          .value_or("stun:stun.l.google.com:19302")
                    : "stun:stun.l.google.com:19302";
            camera_payload.webrtc_turn_server_url =
                metadata.contains("webrtc_turn_server_url")
                    ? any_to_string(metadata.at("webrtc_turn_server_url")).value_or("")
                    : "";
            camera_payload.webrtc_turn_username =
                metadata.contains("webrtc_turn_username")
                    ? any_to_string(metadata.at("webrtc_turn_username")).value_or("")
                    : "";
            camera_payload.webrtc_turn_credential =
                metadata.contains("webrtc_turn_credential")
                    ? any_to_string(metadata.at("webrtc_turn_credential")).value_or("")
                    : "";
            camera_payload.image_scale = coerce_float(
                metadata.contains("image_scale")
                    ? std::optional<std::any>(metadata.at("image_scale"))
                    : std::nullopt,
                1.0f);
            camera_payload.focal_length_scale = coerce_float(
                metadata.contains("focal_length_scale")
                    ? std::optional<std::any>(metadata.at("focal_length_scale"))
                    : std::nullopt,
                0.5f);
            camera_payload.view_position_offset = coerce_vec3(
                metadata.contains("view_position_offset")
                    ? std::optional<std::any>(metadata.at("view_position_offset"))
                    : std::nullopt,
                {0.0f, 0.0f, 0.0f});
            camera_payload.view_rotation_offset = coerce_vec3(
                metadata.contains("view_rotation_offset")
                    ? std::optional<std::any>(metadata.at("view_rotation_offset"))
                    : std::nullopt,
                {0.0f, 0.0f, 0.0f});
            camera_payload.show_frustum = coerce_bool(
                metadata.contains("show_frustum")
                    ? std::optional<std::any>(metadata.at("show_frustum"))
                    : std::nullopt,
                true);
            camera_payload.frustum_color =
                metadata.contains("frustum_color")
                    ? any_to_string(metadata.at("frustum_color")).value_or("#FFFF00")
                    : "#FFFF00";
            camera_payload.overhead_size = coerce_float(
                metadata.contains("overhead_size")
                    ? std::optional<std::any>(metadata.at("overhead_size"))
                    : std::nullopt,
                1.0f);
            camera_payload.overhead_position_offset = coerce_vec3(
                metadata.contains("overhead_position_offset")
                    ? std::optional<std::any>(metadata.at("overhead_position_offset"))
                    : std::nullopt,
                {0.0f, 2.0f, 0.0f});
            camera_payload.overhead_face_camera = coerce_bool(
                metadata.contains("overhead_face_camera")
                    ? std::optional<std::any>(metadata.at("overhead_face_camera"))
                    : std::nullopt,
                true);
            camera_payload.overhead_rotation_offset = coerce_vec3(
                metadata.contains("overhead_rotation_offset")
                    ? std::optional<std::any>(metadata.at("overhead_rotation_offset"))
                    : std::nullopt,
                {90.0f, 0.0f, 0.0f});
            camera_payload.resolution = ResolutionPayload{
                camera->get_resolution().first,
                camera->get_resolution().second};
            camera_payload.fps = camera->get_fps();
            camera_payload.fov = camera->get_fov();
            camera_payload.encoding = camera->get_encoding();
            sensor_payload.camera_config = camera_payload;
        }

        payload.sensors.push_back(sensor_payload);
    }

    std::vector<VisualizationPayload> robot_visualizations;
    std::vector<VisualizationPayload> fallback_global;
    for (const auto& visualization : dataviz.get_visualizations()) {
        VisualizationPayload visualization_payload;
        visualization_payload.type = core::visualization_type_to_string(visualization.viz_type);
        visualization_payload.topic = visualization.data_source.topic;
        visualization_payload.scope = visualization.is_robot_specific() ? "robot" : "global";
        visualization_payload.enabled = visualization.enabled;
        if (!visualization.data_source.frame_id.empty()) {
            visualization_payload.frame = visualization.data_source.frame_id;
        }
        if (visualization.render_options.contains("color")) {
            const auto color = any_to_string(visualization.render_options.at("color"));
            if (color && !color->empty()) {
                visualization_payload.color = *color;
            }
        }

        if (visualization_payload.type == "occupancy_grid") {
            OccupancyPayload occupancy;
            bool has_occupancy = false;
            if (visualization.render_options.contains("show_unknown_space")) {
                occupancy.show_unknown_space = coerce_bool(
                    std::optional<std::any>(visualization.render_options.at("show_unknown_space")),
                    true);
                has_occupancy = true;
            }
            if (visualization.render_options.contains("position_scale")) {
                occupancy.position_scale = coerce_float(
                    std::optional<std::any>(visualization.render_options.at("position_scale")),
                    1.0f);
                has_occupancy = true;
            }
            if (visualization.render_options.contains("position_offset")) {
                occupancy.position_offset = coerce_vec3(
                    std::optional<std::any>(visualization.render_options.at("position_offset")),
                    {0.0f, 0.0f, 0.0f});
                has_occupancy = true;
            }
            if (visualization.render_options.contains("rotation_offset_euler")) {
                occupancy.rotation_offset_euler = coerce_vec3(
                    std::optional<std::any>(visualization.render_options.at("rotation_offset_euler")),
                    {0.0f, 0.0f, 0.0f});
                has_occupancy = true;
            }
            if (has_occupancy) {
                visualization_payload.occupancy = occupancy;
            }
        }

        if (visualization_payload.scope == "global") {
            fallback_global.push_back(visualization_payload);
        } else {
            robot_visualizations.push_back(visualization_payload);
        }
    }

    payload.visualizations = std::move(robot_visualizations);
    if (global_visualizations) {
        payload.global_visualizations = *global_visualizations;
    } else {
        std::vector<VisualizationPayload> deduped;
        std::set<std::tuple<std::string, std::string, std::string>> seen;
        for (const auto& visualization : fallback_global) {
            const auto key = std::make_tuple(
                visualization.type,
                visualization.topic,
                visualization.frame.value_or(""));
            if (seen.insert(key).second) {
                deduped.push_back(visualization);
            }
        }
        payload.global_visualizations = std::move(deduped);
    }

    const auto manager_metadata = robot.get_metadata("robot_manager_config");
    if (manager_metadata && manager_metadata->type() == typeid(std::map<std::string, std::any>)) {
        const auto manager_cfg = std::any_cast<std::map<std::string, std::any>>(*manager_metadata);
        payload.robot_manager_config.enabled = coerce_bool(
            manager_cfg.contains("enabled") ? std::optional<std::any>(manager_cfg.at("enabled")) : std::nullopt,
            true);
        payload.robot_manager_config.prefab_asset_path =
            manager_cfg.contains("prefab_asset_path")
                ? any_to_string(manager_cfg.at("prefab_asset_path"))
                      .value_or("Assets/Prefabs/UI/RobotManager.prefab")
                : "Assets/Prefabs/UI/RobotManager.prefab";
        payload.robot_manager_config.prefab_resource_path =
            manager_cfg.contains("prefab_resource_path")
                ? any_to_string(manager_cfg.at("prefab_resource_path")).value_or("")
                : "";
        if (manager_cfg.contains("sections") &&
            manager_cfg.at("sections").type() == typeid(std::map<std::string, std::any>)) {
            const auto sections = std::any_cast<std::map<std::string, std::any>>(manager_cfg.at("sections"));
            payload.robot_manager_config.sections.status = coerce_bool(
                sections.contains("status") ? std::optional<std::any>(sections.at("status")) : std::nullopt,
                true);
            payload.robot_manager_config.sections.data_viz = coerce_bool(
                sections.contains("data_viz") ? std::optional<std::any>(sections.at("data_viz")) : std::nullopt,
                true);
            payload.robot_manager_config.sections.teleop = coerce_bool(
                sections.contains("teleop") ? std::optional<std::any>(sections.at("teleop")) : std::nullopt,
                true);
            payload.robot_manager_config.sections.tasks = coerce_bool(
                sections.contains("tasks") ? std::optional<std::any>(sections.at("tasks")) : std::nullopt,
                true);
        }
    }

    if (workspace_scale) {
        const double value = *workspace_scale;
        if (std::isfinite(value) && value > 0.0) {
            payload.workspace_config = WorkspaceConfigPayload{static_cast<float>(value)};
        }
    }

    return payload;
}

std::vector<VisualizationPayload> RobotRegistryClient::build_global_visualizations_payload(
    const std::vector<robot::DataViz>& datavizs) const {
    std::vector<VisualizationPayload> global;
    std::set<std::tuple<std::string, std::string, std::string>> seen;

    for (const auto& dataviz : datavizs) {
        for (const auto& visualization : dataviz.get_enabled_visualizations()) {
            if (visualization.is_robot_specific()) {
                continue;
            }
            VisualizationPayload payload;
            payload.type = core::visualization_type_to_string(visualization.viz_type);
            payload.topic = visualization.data_source.topic;
            payload.scope = "global";
            payload.enabled = visualization.enabled;
            if (!visualization.data_source.frame_id.empty()) {
                payload.frame = visualization.data_source.frame_id;
            }
            if (visualization.render_options.contains("color")) {
                const auto color = any_to_string(visualization.render_options.at("color"));
                if (color && !color->empty()) {
                    payload.color = *color;
                }
            }
            const auto key =
                std::make_tuple(payload.type, payload.topic, payload.frame.value_or(""));
            if (seen.insert(key).second) {
                global.push_back(payload);
            }
        }
    }

    return global;
}

std::unordered_map<std::string, std::unordered_map<std::string, std::string>>
RobotRegistryClient::extract_camera_topic_profiles(const RobotRegistrationPayload& payload) const {
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> profiles;
    for (const auto& sensor : payload.sensors) {
        if (sensor.type != "camera" || !sensor.camera_config) {
            continue;
        }
        const auto& config = *sensor.camera_config;
        auto minimap_streaming = normalize_transport(config.minimap_streaming_type, "");
        if (minimap_streaming.empty()) {
            minimap_streaming = normalize_transport(config.streaming_type, "ros");
        }
        if (minimap_streaming.empty()) {
            minimap_streaming = "ros";
        }

        auto teleop_streaming = normalize_transport(config.teleop_streaming_type, "");
        if (teleop_streaming.empty()) {
            teleop_streaming = normalize_transport(config.streaming_type, "webrtc");
        }
        if (teleop_streaming.empty()) {
            teleop_streaming = "webrtc";
        }

        auto startup_mode = to_lower(config.startup_mode);
        if (startup_mode != "minimap" && startup_mode != "teleop") {
            startup_mode = "minimap";
        }

        profiles[sensor.topic] = {
            {"minimap", minimap_streaming},
            {"teleop", teleop_streaming},
            {"startup", startup_mode},
        };
    }
    return profiles;
}

RobotRegistryClient& get_robot_registry_client() {
    static RobotRegistryClient instance;
    return instance;
}

std::vector<VisualizationPayload> build_global_visualizations_payload(
    const std::vector<robot::DataViz>& datavizs) {
    return get_robot_registry_client().build_global_visualizations_payload(datavizs);
}

RobotRegistrationPayload build_robot_config_dict(
    const robot::Robot& robot,
    const robot::DataViz& dataviz,
    std::optional<double> workspace_scale) {
    const auto global = get_robot_registry_client().build_global_visualizations_payload({dataviz});
    return get_robot_registry_client().build_robot_config_dict(
        robot,
        dataviz,
        global,
        workspace_scale);
}

std::optional<std::string> queued_reason_from_ack(const std::map<std::string, std::any>& ack) {
    const auto it = ack.find("robot_id");
    if (it == ack.end()) {
        return std::nullopt;
    }
    const auto status_opt = any_to_string(it->second);
    if (!status_opt) {
        return std::nullopt;
    }
    auto status = *status_opt;
    if (status.rfind("Queued", 0) != 0) {
        return std::nullopt;
    }
    auto detail = status.substr(std::string("Queued").size());
    while (!detail.empty() && std::isspace(static_cast<unsigned char>(detail.front()))) {
        detail.erase(detail.begin());
    }
    while (!detail.empty() && std::isspace(static_cast<unsigned char>(detail.back()))) {
        detail.pop_back();
    }
    if (detail.size() >= 2 && detail.front() == '(' && detail.back() == ')') {
        detail = detail.substr(1, detail.size() - 2);
    }
    if (!detail.empty() && (detail.front() == ':' || detail.front() == '-')) {
        detail.erase(detail.begin());
    }
    while (!detail.empty() && std::isspace(static_cast<unsigned char>(detail.front()))) {
        detail.erase(detail.begin());
    }
    if (detail.empty()) {
        return std::string("Waiting for Workspace");
    }
    return detail;
}

} // namespace bridge
} // namespace horus
