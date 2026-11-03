#include "registration_json.hpp"
#include "horus/bridge/robot_registry.hpp"
#include <cmath>
#include <stdexcept>

namespace horus::bridge {
using Json = nlohmann::json;

Json metadata_json(const std::map<std::string, std::any>& value) {
    Json out = Json::object();
    for (const auto& [key, item] : value) out[key] = any_json(item);
    return out;
}
template<class T> Json vector_json(const std::any& value) {
    Json out = Json::array();
    for (const auto& item : std::any_cast<const std::vector<T>&>(value)) out.push_back(any_json(std::any(item)));
    return out;
}
Json any_json(const std::any& value) {
    if (!value.has_value() || value.type() == typeid(std::nullptr_t)) return nullptr;
    if (value.type() == typeid(Json)) return std::any_cast<const Json&>(value);
    if (value.type() == typeid(std::string)) return std::any_cast<const std::string&>(value);
    if (value.type() == typeid(const char*)) return std::any_cast<const char*>(value);
    if (value.type() == typeid(bool)) return std::any_cast<bool>(value);
#define INTEGER(T) if (value.type() == typeid(T)) return std::any_cast<T>(value);
    INTEGER(int) INTEGER(unsigned) INTEGER(long) INTEGER(unsigned long) INTEGER(long long) INTEGER(unsigned long long)
#undef INTEGER
    if (value.type() == typeid(double) || value.type() == typeid(float)) {
        const auto number = value.type() == typeid(double) ? std::any_cast<double>(value) : std::any_cast<float>(value);
        if (!std::isfinite(number)) throw std::invalid_argument("Non-finite registration value");
        return number;
    }
    if (value.type() == typeid(std::map<std::string, std::any>)) return metadata_json(std::any_cast<const std::map<std::string, std::any>&>(value));
    if (value.type() == typeid(std::map<std::string, std::string>)) return std::any_cast<const std::map<std::string, std::string>&>(value);
#define VECTOR(T) if (value.type() == typeid(std::vector<T>)) return vector_json<T>(value);
    VECTOR(std::any) VECTOR(std::string) VECTOR(double) VECTOR(float) VECTOR(int)
    using AnyMap = std::map<std::string, std::any>;
    VECTOR(AnyMap)
#undef VECTOR
    if (value.type() == typeid(Vector3Payload)) {
        const auto v = std::any_cast<Vector3Payload>(value); return {{"x", v.x}, {"y", v.y}, {"z", v.z}};
    }
    if (value.type() == typeid(dataviz::Vector3PayloadLike)) {
        const auto v = std::any_cast<dataviz::Vector3PayloadLike>(value); return {{"x", v.x}, {"y", v.y}, {"z", v.z}};
    }
    throw std::invalid_argument(std::string("Unsupported registration metadata type: ") + value.type().name());
}

std::any json_any(const Json& value) {
    if (value.is_boolean()) return value.get<bool>();
    if (value.is_string()) return value.get<std::string>();
    if (value.is_number_integer()) return value.get<long long>();
    if (value.is_number()) return value.get<double>();
    if (value.is_object()) {
        std::map<std::string, std::any> out;
        for (const auto& [key, item] : value.items()) out[key] = json_any(item);
        return out;
    }
    if (value.is_array()) {
        std::vector<std::any> out;
        for (const auto& item : value) out.push_back(json_any(item));
        return out;
    }
    return std::any{};
}

// Emit optional fields only when present, matching Python's wire contract.
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Vector3Payload, x, y, z)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ResolutionPayload, width, height)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VizConfigPayload, color, point_size)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DimensionsPayload, length, width, height)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RosBindingPayload, logical_name, tf_mode, topic_mode, base_frame, tf_prefix, topic_prefix)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(EntityCapabilitiesPayload, controllable, teleoperable, taskable, guidable, observable, communicative)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RobotManagerSectionsPayload, status, data_viz, teleop, tasks)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RobotManagerConfigPayload, enabled, prefab_asset_path, prefab_resource_path, sections)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(CameraConfigPayload,
    streaming_type, minimap_streaming_type, teleop_streaming_type, minimap_topic, teleop_topic,
    minimap_image_type, teleop_image_type, minimap_max_fps, teleop_stereo_layout, teleop_right_topic,
    startup_mode, is_stereo, stereo_layout, right_topic, image_type, display_mode, use_tf,
    projection_target_frame, webrtc_client_signal_topic, webrtc_server_signal_topic, webrtc_bitrate_kbps,
    webrtc_framerate, webrtc_minimap_priority, webrtc_teleop_priority,
    webrtc_minimap_bitrate_kbps, webrtc_minimap_framerate,
    webrtc_teleop_bitrate_kbps, webrtc_teleop_framerate,
    webrtc_stun_server_url, webrtc_turn_server_url, webrtc_turn_username,
    webrtc_turn_credential, image_scale, focal_length_scale, immersive_ros_flip_x, immersive_ros_flip_y,
    view_position_offset, view_rotation_offset, projected_position_offset, projected_scale_multiplier,
    show_frustum, frustum_color, overhead_size, overhead_position_offset, overhead_face_camera,
    overhead_rotation_offset, resolution, fps, fov, encoding)

namespace {
void validate_finite(const Json& value) {
    if (value.is_number_float() && !std::isfinite(value.get<double>()))
        throw std::invalid_argument("Non-finite registration value");
    if (value.is_structured()) for (const auto& item : value) validate_finite(item);
}
}

Json payload_json(const RobotRegistrationPayload& p) {
    Json out = {{"action",p.action},{"robot_name",p.robot_name},{"robot_type",p.robot_type},
        {"ros_binding",p.ros_binding},{"entity_kind",p.entity_kind},{"capabilities",p.capabilities},
        {"robot_manager_config",p.robot_manager_config},{"timestamp",p.timestamp}};
    const auto& t = p.control.teleop;
    out["control"] = {{"drive_topic",p.control.drive_topic}, {"teleop", {
        {"enabled",t.enabled},{"command_topic",t.command_topic},{"raw_input_topic",t.raw_input_topic},
        {"head_pose_topic",t.head_pose_topic},{"robot_profile",t.robot_profile},{"response_mode",t.response_mode},
        {"publish_rate_hz",t.publish_rate_hz},{"custom_passthrough_only",t.custom_passthrough_only},
        {"deadman",metadata_json(t.deadman)},{"axes",metadata_json(t.axes)},{"discrete",metadata_json(t.discrete)}}},
        {"tasks",{{"go_to_point",metadata_json(p.control.tasks.go_to_point)},{"waypoint",metadata_json(p.control.tasks.waypoint)}}}};
    if (p.entity_kind == "workspace_visualization") {
        out["control"] = {{"drive_topic", ""}, {"teleop", {{"enabled", false}}},
            {"tasks", {{"go_to_point", {{"enabled", false}}}, {"waypoint", {{"enabled", false}}}}}};
    }
    out["sensors"] = Json::array();
    for (const auto& s : p.sensors) {
        Json sensor = {{"name",s.name},{"type",s.type},{"topic",s.topic},{"frame",s.frame},
                       {"metadata",metadata_json(s.metadata)},{"viz_config",s.viz_config}};
        if (s.camera_config) sensor["camera_config"] = *s.camera_config;
        out["sensors"].push_back(std::move(sensor));
    }
    auto viz = [](const std::vector<VisualizationPayload>& values) {
        Json result = Json::array();
        for (const auto& v : values) {
            Json item = {{"type",v.type},{"topic",v.topic},{"scope",v.scope},{"enabled",v.enabled}};
            if (v.frame) item["frame"] = *v.frame;
            if (v.color) item["color"] = *v.color;
            if (v.transport_lane) item["transport_lane"] = *v.transport_lane;
            if (v.occupancy) {
                const auto& o = *v.occupancy;
                item["occupancy"] = {{"show_unknown_space",o.show_unknown_space}};
                if (o.position_scale) item["occupancy"]["position_scale"] = *o.position_scale;
                if (o.position_offset) item["occupancy"]["position_offset"] = *o.position_offset;
                if (o.rotation_offset_euler) item["occupancy"]["rotation_offset_euler"] = *o.rotation_offset_euler;
            }
#define CONFIG(name) if (!v.name.empty()) item[#name] = metadata_json(v.name);
            CONFIG(path) CONFIG(velocity) CONFIG(trail) CONFIG(collision) CONFIG(point_cloud)
            CONFIG(gaussian_splat) CONFIG(mesh) CONFIG(octomap) CONFIG(semantic_box) CONFIG(costmap)
            CONFIG(remote_render)
#undef CONFIG
            result.push_back(std::move(item));
        }
        return result;
    };
    out["visualizations"] = viz(p.visualizations);
    out["global_visualizations"] = viz(p.global_visualizations);
    if (p.dimensions) out["dimensions"] = *p.dimensions;
    if (p.workspace_config) {
        Json workspace = Json::object();
        if (p.workspace_config->position_scale) workspace["position_scale"] = *p.workspace_config->position_scale;
        if (!p.workspace_config->compass.empty()) workspace["compass"] = metadata_json(p.workspace_config->compass);
        if (!p.workspace_config->tutorial.empty()) workspace["tutorial"] = metadata_json(p.workspace_config->tutorial);
        if (!p.workspace_config->experiment.empty()) workspace["experiment"] = metadata_json(p.workspace_config->experiment);
        out["workspace_config"] = std::move(workspace);
    }
    if (p.robot_model_id) out["robot_model_id"] = *p.robot_model_id;
    if (p.has_visual_mesh_model) out["has_visual_mesh_model"] = *p.has_visual_mesh_model;
    if (!p.robot_description_manifest.empty()) out["robot_description_manifest"] = metadata_json(p.robot_description_manifest);
    if (p.robot_description_payload_json) out["robot_description_payload_json"] = *p.robot_description_payload_json;
    if (p.field_teammate_config) out["field_teammate_config"] = metadata_json(*p.field_teammate_config);
    if (p.teaching_json) out["teaching"] = Json::parse(*p.teaching_json);
    if (p.controller_hints_json) out["controller_hints"] = Json::parse(*p.controller_hints_json);
    validate_finite(out);
    return out;
}
std::string registration_json(const RobotRegistrationPayload& payload) { return payload_json(payload).dump(); }
} // namespace horus::bridge
