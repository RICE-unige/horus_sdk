#include "horus/bridge/robot_registry.hpp"

#include "horus/topics.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <regex>
#include <set>
#include <sstream>
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

std::optional<std::map<std::string, std::any>> any_to_map(const std::any& value) {
    if (value.type() == typeid(std::map<std::string, std::any>)) {
        return std::any_cast<std::map<std::string, std::any>>(value);
    }
    return std::nullopt;
}

std::optional<std::any> map_get(const std::map<std::string, std::any>& values, const std::string& key) {
    const auto it = values.find(key);
    if (it == values.end()) {
        return std::nullopt;
    }
    return it->second;
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
    if (value->type() == typeid(dataviz::Vector3PayloadLike)) {
        const auto vector = std::any_cast<dataviz::Vector3PayloadLike>(*value);
        return Vector3Payload{vector.x, vector.y, vector.z};
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

std::string normalize_image_type(std::string value, const std::string& fallback) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (value == "raw" || value == "compressed") {
        return value;
    }
    return fallback;
}

std::string normalize_layout(std::string value, const std::string& fallback) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (value == "sbs" || value == "side_by_side" || value == "side-by-side") {
        return "side_by_side";
    }
    if (value == "dual_topic" || value == "dual" || value == "two_topics" || value == "two_topic") {
        return "dual_topic";
    }
    if (value == "mono") {
        return "mono";
    }
    return fallback;
}

std::string coerce_text(const std::optional<std::any>& value, const std::string& default_value) {
    if (!value) {
        return default_value;
    }
    const auto text = any_to_string(*value);
    if (!text) {
        return default_value;
    }
    auto result = *text;
    while (!result.empty() && std::isspace(static_cast<unsigned char>(result.front()))) {
        result.erase(result.begin());
    }
    while (!result.empty() && std::isspace(static_cast<unsigned char>(result.back()))) {
        result.pop_back();
    }
    return result.empty() ? default_value : result;
}

float clamp_float(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(max_value, value));
}

int clamp_int(int value, int min_value, int max_value) {
    return std::max(min_value, std::min(max_value, value));
}

std::string read_text_file(const std::string& path) {
    std::ifstream stream(path);
    if (!stream) {
        return "";
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    return buffer.str();
}

int count_token(const std::string& text, const std::string& token) {
    int count = 0;
    std::size_t pos = 0;
    while ((pos = text.find(token, pos)) != std::string::npos) {
        ++count;
        pos += token.size();
    }
    return count;
}

std::string to_lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::uint32_t right_rotate(std::uint32_t value, std::uint32_t bits) {
    return (value >> bits) | (value << (32U - bits));
}

std::string sha256_hex(const std::string& input) {
    static constexpr std::array<std::uint32_t, 64> k{
        0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
        0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
        0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
        0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
        0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
        0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
        0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
        0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U, 0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

    std::array<std::uint32_t, 8> h{
        0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
        0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};

    std::vector<std::uint8_t> data(input.begin(), input.end());
    const auto bit_len = static_cast<std::uint64_t>(data.size()) * 8ULL;
    data.push_back(0x80U);
    while ((data.size() % 64U) != 56U) {
        data.push_back(0U);
    }
    for (int shift = 56; shift >= 0; shift -= 8) {
        data.push_back(static_cast<std::uint8_t>((bit_len >> shift) & 0xffU));
    }

    for (std::size_t chunk = 0; chunk < data.size(); chunk += 64U) {
        std::array<std::uint32_t, 64> w{};
        for (std::size_t i = 0; i < 16U; ++i) {
            const auto offset = chunk + i * 4U;
            w[i] = (static_cast<std::uint32_t>(data[offset]) << 24U) |
                   (static_cast<std::uint32_t>(data[offset + 1U]) << 16U) |
                   (static_cast<std::uint32_t>(data[offset + 2U]) << 8U) |
                   static_cast<std::uint32_t>(data[offset + 3U]);
        }
        for (std::size_t i = 16U; i < 64U; ++i) {
            const auto s0 = right_rotate(w[i - 15U], 7U) ^ right_rotate(w[i - 15U], 18U) ^ (w[i - 15U] >> 3U);
            const auto s1 = right_rotate(w[i - 2U], 17U) ^ right_rotate(w[i - 2U], 19U) ^ (w[i - 2U] >> 10U);
            w[i] = w[i - 16U] + s0 + w[i - 7U] + s1;
        }

        auto a = h[0];
        auto b = h[1];
        auto c = h[2];
        auto d = h[3];
        auto e = h[4];
        auto f = h[5];
        auto g = h[6];
        auto hh = h[7];
        for (std::size_t i = 0; i < 64U; ++i) {
            const auto s1 = right_rotate(e, 6U) ^ right_rotate(e, 11U) ^ right_rotate(e, 25U);
            const auto ch = (e & f) ^ ((~e) & g);
            const auto temp1 = hh + s1 + ch + k[i] + w[i];
            const auto s0 = right_rotate(a, 2U) ^ right_rotate(a, 13U) ^ right_rotate(a, 22U);
            const auto maj = (a & b) ^ (a & c) ^ (b & c);
            const auto temp2 = s0 + maj;
            hh = g;
            g = f;
            f = e;
            e = d + temp1;
            d = c;
            c = b;
            b = a;
            a = temp1 + temp2;
        }
        h[0] += a;
        h[1] += b;
        h[2] += c;
        h[3] += d;
        h[4] += e;
        h[5] += f;
        h[6] += g;
        h[7] += hh;
    }

    std::ostringstream out;
    for (const auto word : h) {
        out << std::hex << std::setw(8) << std::setfill('0') << word;
    }
    return out.str();
}

std::string json_escape(const std::string& value) {
    std::ostringstream out;
    for (const auto ch : value) {
        switch (ch) {
        case '"':
            out << "\\\"";
            break;
        case '\\':
            out << "\\\\";
            break;
        case '\b':
            out << "\\b";
            break;
        case '\f':
            out << "\\f";
            break;
        case '\n':
            out << "\\n";
            break;
        case '\r':
            out << "\\r";
            break;
        case '\t':
            out << "\\t";
            break;
        default:
            if (static_cast<unsigned char>(ch) < 0x20U) {
                out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                    << static_cast<int>(static_cast<unsigned char>(ch));
            } else {
                out << ch;
            }
        }
    }
    return out.str();
}

std::optional<std::string> extract_xml_attribute(const std::string& tag, const std::string& attribute) {
    const std::regex pattern("\\b" + attribute + "\\s*=\\s*[\"']([^\"']+)[\"']");
    std::smatch match;
    if (std::regex_search(tag, match, pattern) && match.size() > 1U) {
        return match[1].str();
    }
    return std::nullopt;
}

std::vector<std::string> extract_urdf_link_names(const std::string& urdf) {
    std::vector<std::string> names;
    const std::regex pattern("<link\\b[^>]*>");
    for (auto it = std::sregex_iterator(urdf.begin(), urdf.end(), pattern);
         it != std::sregex_iterator();
         ++it) {
        if (const auto name = extract_xml_attribute(it->str(), "name")) {
            names.push_back(*name);
        }
    }
    return names;
}

struct NativeJoint {
    std::string name;
    std::string type;
    std::string parent_link;
    std::string child_link;
};

std::vector<NativeJoint> extract_urdf_joints(const std::string& urdf) {
    std::vector<NativeJoint> joints;
    const std::regex joint_pattern("<joint\\b[\\s\\S]*?</joint>");
    const std::regex parent_pattern("<parent\\b[^>]*>");
    const std::regex child_pattern("<child\\b[^>]*>");
    for (auto it = std::sregex_iterator(urdf.begin(), urdf.end(), joint_pattern);
         it != std::sregex_iterator();
         ++it) {
        const auto block = it->str();
        const auto name = extract_xml_attribute(block, "name").value_or("");
        if (name.empty()) {
            continue;
        }
        NativeJoint joint;
        joint.name = name;
        joint.type = extract_xml_attribute(block, "type").value_or("fixed");
        std::smatch parent_match;
        if (std::regex_search(block, parent_match, parent_pattern)) {
            joint.parent_link = extract_xml_attribute(parent_match.str(), "link").value_or("");
        }
        std::smatch child_match;
        if (std::regex_search(block, child_match, child_pattern)) {
            joint.child_link = extract_xml_attribute(child_match.str(), "link").value_or("");
        }
        joints.push_back(joint);
    }
    return joints;
}

std::string build_native_robot_description_payload_json(
    const robot::Robot& robot,
    const std::string& urdf,
    const std::string& base_frame) {
    const auto links = extract_urdf_link_names(urdf);
    const auto joints = extract_urdf_joints(urdf);

    std::ostringstream out;
    out << "{\"base_frame\":\"" << json_escape(base_frame) << "\",\"joints\":[";
    for (std::size_t i = 0; i < joints.size(); ++i) {
        if (i > 0U) {
            out << ",";
        }
        const auto& joint = joints[i];
        out << "{\"axis_xyz\":[0.0,0.0,1.0],\"child_link\":\"" << json_escape(joint.child_link)
            << "\",\"name\":\"" << json_escape(joint.name)
            << "\",\"origin_rpy\":[0.0,0.0,0.0],\"origin_xyz\":[0.0,0.0,0.0],\"parent_link\":\""
            << json_escape(joint.parent_link) << "\",\"type\":\"" << json_escape(joint.type) << "\"}";
    }
    out << "],\"links\":[";
    for (std::size_t i = 0; i < links.size(); ++i) {
        if (i > 0U) {
            out << ",";
        }
        out << "{\"collisions\":[],\"name\":\"" << json_escape(links[i]) << "\"}";
    }
    out << "],\"robot_name\":\"" << json_escape(robot.get_name()) << "\",\"version\":\"v2\"}";
    return out.str();
}

std::string normalize_point_shape(const std::map<std::string, std::any>& render_options) {
    auto point_shape = to_lower(coerce_text(map_get(render_options, "point_shape"), "square"));
    if (point_shape == "disc") {
        point_shape = "circle";
    }
    return point_shape == "circle" ? std::string("circle") : std::string("square");
}

std::string normalize_octomap_render_mode(const std::map<std::string, std::any>& render_options) {
    auto render_mode = to_lower(coerce_text(map_get(render_options, "render_mode"), "surface_mesh"));
    return render_mode == "voxel_cubes" ? std::string("voxel_cubes") : std::string("surface_mesh");
}

std::string normalize_map_coordinate_space(
    const std::map<std::string, std::any>& render_options,
    const std::string& default_space) {
    auto coordinate_space = to_lower(coerce_text(map_get(render_options, "source_coordinate_space"), default_space));
    return coordinate_space == "optical" ? std::string("optical") : std::string("enu");
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

RosBindingPayload build_ros_binding(const robot::Robot& robot) {
    const auto binding = robot.get_ros_binding();
    RosBindingPayload payload;
    payload.logical_name = coerce_text(map_get(binding, "logical_name"), robot.get_name());
    payload.tf_mode = coerce_text(map_get(binding, "tf_mode"), "prefixed");
    payload.topic_mode = coerce_text(map_get(binding, "topic_mode"), "prefixed");
    payload.base_frame = coerce_text(map_get(binding, "base_frame"), "base_link");
    payload.tf_prefix = coerce_text(map_get(binding, "tf_prefix"), robot.get_name());
    payload.topic_prefix = coerce_text(map_get(binding, "topic_prefix"), "/" + robot.get_name());
    return payload;
}

TeleopControlPayload build_teleop_control(const robot::Robot& robot) {
    std::map<std::string, std::any> metadata;
    if (const auto value = robot.get_metadata("teleop_config")) {
        metadata = any_to_map(*value).value_or(std::map<std::string, std::any>{});
    }

    auto nested_map = [&](const std::string& key) {
        const auto value = map_get(metadata, key);
        return value ? any_to_map(*value).value_or(std::map<std::string, std::any>{})
                     : std::map<std::string, std::any>{};
    };

    const auto deadman = nested_map("deadman");
    const auto axes = nested_map("axes");
    const auto discrete = nested_map("discrete");

    std::set<std::string> allowed_profiles{"wheeled", "legged", "aerial", "drone", "custom"};
    std::string default_profile = robot.get_type_str();
    if (!allowed_profiles.contains(default_profile)) {
        default_profile = "wheeled";
    }
    std::string robot_profile = to_lower(coerce_text(map_get(metadata, "robot_profile"), default_profile));
    if (!allowed_profiles.contains(robot_profile)) {
        robot_profile = default_profile;
    }

    std::string response_mode = to_lower(coerce_text(map_get(metadata, "response_mode"), "analog"));
    if (response_mode != "analog" && response_mode != "discrete") {
        response_mode = "analog";
    }

    std::set<std::string> allowed_deadman{
        "either_index_trigger",
        "left_index_trigger",
        "right_index_trigger",
        "either_grip_trigger",
    };
    std::string deadman_policy = to_lower(coerce_text(map_get(deadman, "policy"), "either_grip_trigger"));
    if (!allowed_deadman.contains(deadman_policy)) {
        deadman_policy = "either_grip_trigger";
    }

    TeleopControlPayload payload;
    payload.enabled = coerce_bool(map_get(metadata, "enabled"), true);
    payload.command_topic = coerce_text(map_get(metadata, "command_topic"), robot.resolve_topic("cmd_vel"));
    payload.raw_input_topic = coerce_text(
        map_get(metadata, "raw_input_topic"),
        "/horus/teleop/" + robot.get_name() + "/joy");
    payload.head_pose_topic = coerce_text(
        map_get(metadata, "head_pose_topic"),
        "/horus/teleop/" + robot.get_name() + "/head_pose");
    payload.robot_profile = robot_profile;
    payload.response_mode = response_mode;
    payload.publish_rate_hz = clamp_float(coerce_float(map_get(metadata, "publish_rate_hz"), 30.0f), 5.0f, 120.0f);
    payload.custom_passthrough_only = coerce_bool(map_get(metadata, "custom_passthrough_only"), false);
    payload.deadman = {
        {"policy", deadman_policy},
        {"timeout_ms", clamp_int(coerce_int(map_get(deadman, "timeout_ms"), 200), 50, 2000)},
    };
    payload.axes = {
        {"deadzone", clamp_float(coerce_float(map_get(axes, "deadzone"), 0.15f), 0.0f, 0.5f)},
        {"expo", clamp_float(coerce_float(map_get(axes, "expo"), 1.7f), 1.0f, 3.0f)},
        {"linear_xy_max_mps", clamp_float(coerce_float(map_get(axes, "linear_xy_max_mps"), 1.0f), 0.0f, 5.0f)},
        {"linear_z_max_mps", clamp_float(coerce_float(map_get(axes, "linear_z_max_mps"), 0.8f), 0.0f, 5.0f)},
        {"angular_z_max_rps", clamp_float(coerce_float(map_get(axes, "angular_z_max_rps"), 1.2f), 0.0f, 6.0f)},
        {"invert_linear_x", coerce_bool(map_get(axes, "invert_linear_x"), false)},
        {"invert_linear_y", coerce_bool(map_get(axes, "invert_linear_y"), false)},
        {"invert_linear_z", coerce_bool(map_get(axes, "invert_linear_z"), false)},
        {"invert_angular_z", coerce_bool(map_get(axes, "invert_angular_z"), false)},
    };
    payload.discrete = {
        {"threshold", clamp_float(coerce_float(map_get(discrete, "threshold"), 0.6f), 0.1f, 1.0f)},
        {"linear_xy_step_mps", clamp_float(coerce_float(map_get(discrete, "linear_xy_step_mps"), 0.6f), 0.0f, 5.0f)},
        {"linear_z_step_mps", clamp_float(coerce_float(map_get(discrete, "linear_z_step_mps"), 0.4f), 0.0f, 5.0f)},
        {"angular_z_step_rps", clamp_float(coerce_float(map_get(discrete, "angular_z_step_rps"), 0.9f), 0.0f, 6.0f)},
    };
    return payload;
}

TaskControlPayload build_task_control(const robot::Robot& robot) {
    std::map<std::string, std::any> metadata;
    if (const auto value = robot.get_metadata("task_config")) {
        metadata = any_to_map(*value).value_or(std::map<std::string, std::any>{});
    }
    const auto go = map_get(metadata, "go_to_point")
                        ? any_to_map(*map_get(metadata, "go_to_point")).value_or(std::map<std::string, std::any>{})
                        : std::map<std::string, std::any>{};
    const auto waypoint = map_get(metadata, "waypoint")
                              ? any_to_map(*map_get(metadata, "waypoint")).value_or(std::map<std::string, std::any>{})
                              : std::map<std::string, std::any>{};
    const auto min_altitude = clamp_float(coerce_float(map_get(go, "min_altitude_m"), 0.0f), 0.0f, 100.0f);
    const auto max_altitude = std::max(
        min_altitude + 0.1f,
        std::min(100.0f, coerce_float(map_get(go, "max_altitude_m"), 10.0f)));

    TaskControlPayload payload;
    payload.go_to_point = {
        {"enabled", coerce_bool(map_get(go, "enabled"), true)},
        {"goal_topic", coerce_text(map_get(go, "goal_topic"), robot.resolve_topic("goal_pose"))},
        {"cancel_topic", coerce_text(map_get(go, "cancel_topic"), robot.resolve_topic("goal_cancel"))},
        {"status_topic", coerce_text(map_get(go, "status_topic"), robot.resolve_topic("goal_status"))},
        {"frame_id", coerce_text(map_get(go, "frame_id"), "map")},
        {"position_tolerance_m", clamp_float(coerce_float(map_get(go, "position_tolerance_m"), 0.20f), 0.01f, 10.0f)},
        {"yaw_tolerance_deg", clamp_float(coerce_float(map_get(go, "yaw_tolerance_deg"), 12.0f), 0.1f, 180.0f)},
        {"min_altitude_m", min_altitude},
        {"max_altitude_m", max_altitude},
    };
    payload.waypoint = {
        {"enabled", coerce_bool(map_get(waypoint, "enabled"), true)},
        {"path_topic", coerce_text(map_get(waypoint, "path_topic"), robot.resolve_topic("waypoint_path"))},
        {"status_topic", coerce_text(map_get(waypoint, "status_topic"), robot.resolve_topic("waypoint_status"))},
        {"frame_id", coerce_text(map_get(waypoint, "frame_id"), "map")},
        {"position_tolerance_m", clamp_float(coerce_float(map_get(waypoint, "position_tolerance_m"), 0.20f), 0.01f, 10.0f)},
        {"yaw_tolerance_deg", clamp_float(coerce_float(map_get(waypoint, "yaw_tolerance_deg"), 12.0f), 0.1f, 180.0f)},
    };
    return payload;
}

std::map<std::string, std::any> build_robot_description_manifest(
    const robot::Robot& robot,
    const std::map<std::string, std::any>& config,
    std::optional<std::string>* payload_json = nullptr) {
    if (!coerce_bool(map_get(config, "enabled"), true)) {
        return {};
    }
    const auto urdf_path = coerce_text(map_get(config, "urdf_path"), "");
    if (urdf_path.empty()) {
        return {};
    }

    const auto urdf = read_text_file(urdf_path);
    if (urdf.empty()) {
        return {};
    }
    const auto base_frame = coerce_text(map_get(config, "base_frame"), "base_link");
    const auto source = coerce_text(map_get(config, "source"), "ros");
    const auto chunk_size = clamp_int(coerce_int(map_get(config, "chunk_size_bytes"), 12000), 1024, 64000);
    const auto description_payload = build_native_robot_description_payload_json(robot, urdf, base_frame);
    const auto links = extract_urdf_link_names(urdf);
    const auto joints = extract_urdf_joints(urdf);

    if (payload_json != nullptr && description_payload.size() <= 250000U) {
        *payload_json = description_payload;
    }

    return {
        {"version", std::string("v2")},
        {"description_id", std::string("sha256:") + sha256_hex(description_payload)},
        {"source", source},
        {"base_frame", base_frame},
        {"link_count", static_cast<int>(links.size())},
        {"joint_count", static_cast<int>(joints.size())},
        {"collision_count", 0},
        {"supports_collision", false},
        {"supports_joints", !joints.empty()},
        {"supports_visual_meshes", false},
        {"mesh_asset_count", 0},
        {"mesh_asset_encoded_bytes", 0},
        {"is_transparent", coerce_bool(map_get(config, "is_transparent"), false)},
        {"encoding", std::string("json+gzip+base64")},
        {"chunk_size_bytes", chunk_size},
    };
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

    registration_in_progress_ = false;
    return {
        false,
        {
            {"success", false},
            {"error", std::string("Native HORUS registration transport is not implemented yet; build_robot_config_dict provides payload parity only.")},
            {"unsupported_feature", std::string("bridge_registration")},
            {"payload", payload},
            {"timeout_sec", timeout_sec},
            {"keep_alive", keep_alive},
            {"show_dashboard", show_dashboard},
        }
    };
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
    std::vector<RobotRegistrationPayload> payloads;
    payloads.reserve(robots.size());

    for (std::size_t i = 0; i < robots.size(); ++i) {
        auto* robot = robots[i];
        if (!robot) {
            registration_in_progress_ = false;
            return {false, {{"error", std::string("Null robot pointer")}}};
        }
        auto payload =
            build_robot_config_dict(*robot, datavizs[i], global_visualizations, workspace_scale);
        payloads.push_back(std::move(payload));
    }

    registration_in_progress_ = false;
    return {
        false,
        {
            {"success", false},
            {"error", std::string("Native HORUS registration transport is not implemented yet; build_robot_config_dict provides payload parity only.")},
            {"unsupported_feature", std::string("bridge_registration")},
            {"payloads", payloads},
            {"timeout_sec", timeout_sec},
            {"keep_alive", keep_alive},
            {"show_dashboard", show_dashboard},
        }
    };
}

std::pair<bool, std::map<std::string, std::any>> RobotRegistryClient::unregister_robot(
    const std::string& robot_id,
    double timeout_sec) {
    return {
        false,
        {
            {"success", false},
            {"error", std::string("Native HORUS unregister transport is not implemented yet.")},
            {"unsupported_feature", std::string("bridge_registration")},
            {"robot_id", robot_id},
            {"timeout_sec", timeout_sec},
        }
    };
}

bool RobotRegistryClient::check_backend_availability() const {
    return false;
}

RobotRegistrationPayload RobotRegistryClient::build_robot_config_dict(
    const robot::Robot& robot,
    const robot::DataViz& dataviz,
    std::optional<std::vector<VisualizationPayload>> global_visualizations,
    std::optional<double> workspace_scale) const {
    RobotRegistrationPayload payload;
    payload.robot_name = robot.get_name();
    payload.robot_type = robot.get_type_str();
    payload.ros_binding = build_ros_binding(robot);
    payload.control.teleop = build_teleop_control(robot);
    payload.control.tasks = build_task_control(robot);
    payload.control.drive_topic = payload.control.teleop.command_topic;
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

            const auto is_stereo = camera->is_stereo();
            auto stereo_layout = is_stereo
                                     ? normalize_layout(
                                           metadata.contains("stereo_layout")
                                               ? any_to_string(metadata.at("stereo_layout")).value_or("")
                                               : camera->get_stereo_layout(),
                                           "side_by_side")
                                     : std::string("mono");
            if (is_stereo && stereo_layout != "dual_topic") {
                stereo_layout = "side_by_side";
            }
            auto right_topic = metadata.contains("right_topic")
                                   ? any_to_string(metadata.at("right_topic")).value_or("")
                                   : camera->get_right_topic();
            auto minimap_topic = metadata.contains("minimap_topic")
                                     ? any_to_string(metadata.at("minimap_topic")).value_or("")
                                     : camera->get_minimap_topic();
            if (minimap_topic.empty()) {
                minimap_topic = camera->get_topic();
            }
            auto teleop_topic = metadata.contains("teleop_topic")
                                    ? any_to_string(metadata.at("teleop_topic")).value_or("")
                                    : camera->get_teleop_topic();
            if (teleop_topic.empty()) {
                teleop_topic = minimap_topic;
            }
            auto image_type = normalize_image_type(
                metadata.contains("image_type") ? any_to_string(metadata.at("image_type")).value_or("") : "",
                "raw");
            auto minimap_image_type = normalize_image_type(
                metadata.contains("minimap_image_type")
                    ? any_to_string(metadata.at("minimap_image_type")).value_or("")
                    : camera->get_minimap_image_type(),
                image_type);
            auto teleop_image_type = normalize_image_type(
                metadata.contains("teleop_image_type")
                    ? any_to_string(metadata.at("teleop_image_type")).value_or("")
                    : camera->get_teleop_image_type(),
                image_type);
            auto minimap_max_fps = clamp_int(
                coerce_int(
                    metadata.contains("minimap_max_fps")
                        ? std::optional<std::any>(metadata.at("minimap_max_fps"))
                        : std::optional<std::any>(camera->get_minimap_max_fps()),
                    30),
                1,
                30);
            auto teleop_stereo_layout = normalize_layout(
                metadata.contains("teleop_stereo_layout")
                    ? any_to_string(metadata.at("teleop_stereo_layout")).value_or("")
                    : camera->get_teleop_stereo_layout(),
                stereo_layout);
            auto teleop_right_topic = metadata.contains("teleop_right_topic")
                                          ? any_to_string(metadata.at("teleop_right_topic")).value_or("")
                                          : camera->get_teleop_right_topic();
            if (teleop_right_topic.empty()) {
                teleop_right_topic = right_topic;
            }
            if (!is_stereo) {
                teleop_stereo_layout = "mono";
                teleop_right_topic.clear();
            }

            camera_payload.streaming_type = legacy_streaming;
            camera_payload.minimap_streaming_type = minimap;
            camera_payload.teleop_streaming_type = teleop;
            camera_payload.minimap_topic = minimap_topic;
            camera_payload.teleop_topic = teleop_topic;
            camera_payload.minimap_image_type = minimap_image_type;
            camera_payload.teleop_image_type = teleop_image_type;
            camera_payload.minimap_max_fps = minimap_max_fps;
            camera_payload.teleop_stereo_layout = teleop_stereo_layout;
            camera_payload.teleop_right_topic = teleop_right_topic;
            camera_payload.startup_mode = startup;
            camera_payload.is_stereo = is_stereo;
            camera_payload.stereo_layout = stereo_layout;
            camera_payload.right_topic = right_topic;
            camera_payload.image_type = image_type;
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
            camera_payload.immersive_ros_flip_x = coerce_bool(
                metadata.contains("immersive_ros_flip_x")
                    ? std::optional<std::any>(metadata.at("immersive_ros_flip_x"))
                    : std::nullopt,
                false);
            camera_payload.immersive_ros_flip_y = coerce_bool(
                metadata.contains("immersive_ros_flip_y")
                    ? std::optional<std::any>(metadata.at("immersive_ros_flip_y"))
                    : std::nullopt,
                false);
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
            camera_payload.projected_position_offset = coerce_vec3(
                metadata.contains("projected_position_offset")
                    ? std::optional<std::any>(metadata.at("projected_position_offset"))
                    : std::nullopt,
                {0.0f, 0.0f, 0.0f});
            camera_payload.projected_scale_multiplier = coerce_vec3(
                metadata.contains("projected_scale_multiplier")
                    ? std::optional<std::any>(metadata.at("projected_scale_multiplier"))
                    : std::nullopt,
                {1.0f, 1.0f, 1.0f});
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

        if (visualization_payload.type == "path") {
            const auto source_type = core::data_source_type_to_string(visualization.data_source.source_type);
            if (source_type == "robot_global_path") {
                visualization_payload.path["role"] = std::string("global");
            } else if (source_type == "robot_local_path") {
                visualization_payload.path["role"] = std::string("local");
            } else if (source_type == "robot_trajectory") {
                visualization_payload.path["role"] = std::string("trajectory");
            }
            if (visualization.render_options.contains("line_width")) {
                visualization_payload.path["line_width"] = coerce_float(
                    std::optional<std::any>(visualization.render_options.at("line_width")),
                    1.0f);
            }
            if (visualization.render_options.contains("line_style")) {
                const auto line_style = any_to_string(visualization.render_options.at("line_style"));
                if (line_style && !line_style->empty()) {
                    visualization_payload.path["line_style"] = *line_style;
                }
            }
        }

        if (visualization_payload.type == "velocity_data") {
            visualization_payload.velocity = {
                {"units", coerce_text(map_get(visualization.render_options, "units"), "m/s")},
                {"text_back_offset_m", coerce_float(map_get(visualization.render_options, "text_back_offset_m"), 0.36f)},
                {"floor_offset_m", coerce_float(map_get(visualization.render_options, "floor_offset_m"), 0.01f)},
                {"update_hz", coerce_float(map_get(visualization.render_options, "update_hz"), 10.0f)},
            };
        }

        if (visualization_payload.type == "odometry_trail") {
            visualization_payload.trail = {
                {"max_points", coerce_int(map_get(visualization.render_options, "max_points"), 48)},
                {"history_seconds", coerce_float(map_get(visualization.render_options, "history_seconds"), 3.2f)},
                {"min_spacing_m", coerce_float(map_get(visualization.render_options, "min_spacing_m"), 0.07f)},
                {"line_width_m", coerce_float(map_get(visualization.render_options, "line_width_m"), 0.0048f)},
                {"trail_back_offset_m", coerce_float(map_get(visualization.render_options, "trail_back_offset_m"), 0.44f)},
            };
        }

        if (visualization_payload.type == "collision_risk") {
            visualization_payload.collision = {
                {"threshold_m", coerce_float(map_get(visualization.render_options, "threshold_m"), 1.2f)},
                {"radius_m", coerce_float(map_get(visualization.render_options, "radius_m"), 1.2f)},
                {"source", coerce_text(map_get(visualization.render_options, "source"), "laser_scan")},
                {"alpha_min", coerce_float(map_get(visualization.render_options, "alpha_min"), 0.0f)},
                {"alpha_max", coerce_float(map_get(visualization.render_options, "alpha_max"), 0.55f)},
            };
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

        if (visualization_payload.type == "point_cloud") {
            const auto min_point_size = std::max(0.0001f, coerce_float(map_get(visualization.render_options, "min_point_size"), 0.002f));
            const auto max_point_size = std::max(min_point_size, coerce_float(map_get(visualization.render_options, "max_point_size"), 0.04f));
            const auto max_visible_budget = std::max(1000, coerce_int(map_get(visualization.render_options, "max_visible_points_budget"), 200000));
            auto render_mode = to_lower(coerce_text(map_get(visualization.render_options, "render_mode"), "opaque_fast"));
            if (render_mode != "transparent_hq") {
                render_mode = "opaque_fast";
            }
            auto point_shape = normalize_point_shape(visualization.render_options);
            const auto max_points_raw = coerce_int(map_get(visualization.render_options, "max_points_per_frame"), 0);
            visualization_payload.point_cloud = {
                {"point_size", std::max(0.001f, coerce_float(map_get(visualization.render_options, "point_size"), 0.05f))},
                {"max_points_per_frame", max_points_raw <= 0 ? 0 : std::max(1, max_points_raw)},
                {"base_sample_stride", std::max(1, coerce_int(map_get(visualization.render_options, "base_sample_stride"), 1))},
                {"min_update_interval", std::max(0.0f, coerce_float(map_get(visualization.render_options, "min_update_interval"), 0.0f))},
                {"enable_adaptive_quality", coerce_bool(map_get(visualization.render_options, "enable_adaptive_quality"), false)},
                {"target_framerate", std::max(30.0f, coerce_float(map_get(visualization.render_options, "target_framerate"), 72.0f))},
                {"min_quality_multiplier", clamp_float(coerce_float(map_get(visualization.render_options, "min_quality_multiplier"), 0.6f), 0.25f, 1.0f)},
                {"min_distance", std::max(0.0f, coerce_float(map_get(visualization.render_options, "min_distance"), 0.0f))},
                {"max_distance", std::max(0.0f, coerce_float(map_get(visualization.render_options, "max_distance"), 0.0f))},
                {"replace_latest", coerce_bool(map_get(visualization.render_options, "replace_latest"), true)},
                {"render_all_points", coerce_bool(map_get(visualization.render_options, "render_all_points"), true)},
                {"auto_point_size_by_workspace_scale", coerce_bool(map_get(visualization.render_options, "auto_point_size_by_workspace_scale"), true)},
                {"min_point_size", min_point_size},
                {"max_point_size", max_point_size},
                {"render_mode", render_mode},
                {"point_shape", point_shape},
                {"enable_view_frustum_culling", coerce_bool(map_get(visualization.render_options, "enable_view_frustum_culling"), true)},
                {"frustum_padding", clamp_float(coerce_float(map_get(visualization.render_options, "frustum_padding"), 0.03f), 0.0f, 0.5f)},
                {"enable_subpixel_culling", coerce_bool(map_get(visualization.render_options, "enable_subpixel_culling"), true)},
                {"min_screen_radius_px", std::max(0.0f, coerce_float(map_get(visualization.render_options, "min_screen_radius_px"), 0.8f))},
                {"visible_points_budget", clamp_int(coerce_int(map_get(visualization.render_options, "visible_points_budget"), 120000), 1000, max_visible_budget)},
                {"max_visible_points_budget", max_visible_budget},
                {"map_static_mode", coerce_bool(map_get(visualization.render_options, "map_static_mode"), true)},
            };
        }

        if (visualization_payload.type == "gaussian_splat") {
            auto asset_format = to_lower(coerce_text(map_get(visualization.render_options, "asset_format"), "3dgs_ply"));
            if (asset_format != "3dgs_ply") {
                asset_format = "3dgs_ply";
            }
            auto coordinate_space = to_lower(coerce_text(map_get(visualization.render_options, "source_coordinate_space"), "colmap"));
            if (coordinate_space != "3dgs" && coordinate_space != "opencv" && coordinate_space != "unity") {
                coordinate_space = "colmap";
            }
            auto render_mode = to_lower(coerce_text(map_get(visualization.render_options, "render_mode"), "splats"));
            if (render_mode != "debug_points" && render_mode != "mono_center_eye" && render_mode != "no_covariance") {
                render_mode = "splats";
            }
            visualization_payload.gaussian_splat = {
                {"manifest_topic", coerce_text(map_get(visualization.render_options, "manifest_topic"), visualization.data_source.topic)},
                {"chunk_begin_topic", coerce_text(map_get(visualization.render_options, "chunk_begin_topic"), "/horus/gaussian_splat/chunk_begin")},
                {"chunk_item_topic", coerce_text(map_get(visualization.render_options, "chunk_item_topic"), "/horus/gaussian_splat/chunk_item")},
                {"chunk_end_topic", coerce_text(map_get(visualization.render_options, "chunk_end_topic"), "/horus/gaussian_splat/chunk_end")},
                {"asset_format", asset_format},
                {"source_coordinate_space", coordinate_space},
                {"render_mode", render_mode},
                {"max_splats", std::max(1000, coerce_int(map_get(visualization.render_options, "max_splats"), 350000))},
                {"render_scale", clamp_float(coerce_float(map_get(visualization.render_options, "render_scale"), 0.5f), 0.25f, 1.0f)},
                {"sh_order", clamp_int(coerce_int(map_get(visualization.render_options, "sh_order"), 2), 0, 3)},
                {"half_precision_sh", coerce_bool(map_get(visualization.render_options, "half_precision_sh"), true)},
                {"adaptive_sort", coerce_bool(map_get(visualization.render_options, "adaptive_sort"), true)},
                {"sort_passes", clamp_int(coerce_int(map_get(visualization.render_options, "sort_passes"), 2), 2, 4)},
                {"opacity_scale", clamp_float(coerce_float(map_get(visualization.render_options, "opacity_scale"), 1.0f), 0.05f, 20.0f)},
                {"splat_scale", clamp_float(coerce_float(map_get(visualization.render_options, "splat_scale"), 1.0f), 0.05f, 4.0f)},
                {"contribution_cull_threshold", clamp_float(coerce_float(map_get(visualization.render_options, "contribution_cull_threshold"), 0.1f), 0.0f, 1.0f)},
                {"high_precision_rt", coerce_bool(map_get(visualization.render_options, "high_precision_rt"), false)},
                {"pointcloud_fallback", coerce_bool(map_get(visualization.render_options, "pointcloud_fallback"), true)},
                {"fallback_topic", coerce_text(map_get(visualization.render_options, "fallback_topic"), "/map_gaussian_splat_preview")},
                {"fallback_frame", coerce_text(map_get(visualization.render_options, "fallback_frame"), visualization.data_source.frame_id.empty() ? std::string("map") : visualization.data_source.frame_id)},
            };
            if (const auto fallback = map_get(visualization.render_options, "fallback_point_cloud")) {
                if (const auto fallback_map = any_to_map(*fallback)) {
                    visualization_payload.gaussian_splat["fallback_point_cloud"] = *fallback_map;
                }
            }
        }

        if (visualization_payload.type == "mesh") {
            auto coordinate_space = normalize_map_coordinate_space(visualization.render_options, "enu");
            visualization_payload.mesh = {
                {"use_vertex_colors", coerce_bool(map_get(visualization.render_options, "use_vertex_colors"), true)},
                {"alpha", clamp_float(coerce_float(map_get(visualization.render_options, "alpha"), 1.0f), 0.0f, 1.0f)},
                {"double_sided", coerce_bool(map_get(visualization.render_options, "double_sided"), true)},
                {"max_triangles", std::max(1000, coerce_int(map_get(visualization.render_options, "max_triangles"), 200000))},
                {"source_coordinate_space", coordinate_space},
            };
        }

        if (visualization_payload.type == "octomap") {
            auto coordinate_space = normalize_map_coordinate_space(visualization.render_options, "enu");
            auto render_mode = normalize_octomap_render_mode(visualization.render_options);
            visualization_payload.octomap = {
                {"render_mode", render_mode},
                {"use_vertex_colors", coerce_bool(map_get(visualization.render_options, "use_vertex_colors"), true)},
                {"alpha", clamp_float(coerce_float(map_get(visualization.render_options, "alpha"), 1.0f), 0.0f, 1.0f)},
                {"double_sided", coerce_bool(map_get(visualization.render_options, "double_sided"), false)},
                {"max_triangles", std::max(1000, coerce_int(map_get(visualization.render_options, "max_triangles"), 60000))},
                {"source_coordinate_space", coordinate_space},
                {"native_topic", coerce_text(map_get(visualization.render_options, "native_topic"), "/map_3d_octomap")},
                {"native_frame", coerce_text(map_get(visualization.render_options, "native_frame"), "map")},
                {"native_binary_only", coerce_bool(map_get(visualization.render_options, "native_binary_only"), true)},
            };
        }

        if (visualization_payload.type == "semantic_box") {
            const auto semantic_id = coerce_text(map_get(visualization.render_options, "id"), "");
            const auto label = coerce_text(map_get(visualization.render_options, "label"), "");
            if (!semantic_id.empty() && !label.empty()) {
                visualization_payload.semantic_box = {
                    {"id", semantic_id},
                    {"label", label},
                    {"center", coerce_vec3(map_get(visualization.render_options, "center"), {0.0f, 0.0f, 0.0f})},
                    {"size", coerce_vec3(map_get(visualization.render_options, "size"), {1.0f, 1.0f, 1.0f})},
                    {"rotation_offset_euler", coerce_vec3(map_get(visualization.render_options, "rotation_offset_euler"), {0.0f, 0.0f, 0.0f})},
                };
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
            WorkspaceConfigPayload workspace;
            workspace.position_scale = static_cast<float>(value);
            payload.workspace_config = workspace;
        }
    }

    if (const auto compass_value = robot.get_metadata("workspace_compass_config")) {
        const auto compass_cfg = any_to_map(*compass_value).value_or(std::map<std::string, std::any>{});
        WorkspaceConfigPayload workspace = payload.workspace_config.value_or(WorkspaceConfigPayload{});
        auto voice_mode = to_lower(coerce_text(map_get(compass_cfg, "voice_mode"), "auto"));
        if (voice_mode != "batch" && voice_mode != "realtime") {
            voice_mode = "auto";
        }
        auto gateway_port = coerce_int(map_get(compass_cfg, "gateway_port"), 8088);
        if (gateway_port <= 0 || gateway_port > 65535) {
            gateway_port = 8088;
        }
        workspace.compass = {
            {"enabled", coerce_bool(map_get(compass_cfg, "enabled"), false)},
            {"gateway_port", gateway_port},
            {"voice_mode", voice_mode},
            {"autonomy", std::string("approve_actions")},
            {"contract_version", coerce_text(map_get(compass_cfg, "contract_version"), "compass.v1")},
        };
        payload.workspace_config = workspace;
    }

    if (const auto tutorial_value = robot.get_metadata("workspace_tutorial_config")) {
        const auto tutorial_cfg = any_to_map(*tutorial_value).value_or(std::map<std::string, std::any>{});
        const auto preset_id = coerce_text(map_get(tutorial_cfg, "preset_id"), "");
        if (!preset_id.empty()) {
            WorkspaceConfigPayload workspace = payload.workspace_config.value_or(WorkspaceConfigPayload{});
            workspace.tutorial = {
                {"enabled", coerce_bool(map_get(tutorial_cfg, "enabled"), true)},
                {"preset_id", preset_id},
            };
            payload.workspace_config = workspace;
        }
    }

    if (const auto body_value = robot.get_metadata("local_body_model_config")) {
        const auto body_cfg = any_to_map(*body_value).value_or(std::map<std::string, std::any>{});
        const auto body_enabled = coerce_bool(map_get(body_cfg, "enabled"), false);
        auto model_id = to_lower(coerce_text(map_get(body_cfg, "robot_model_id"), ""));
        if (body_enabled && !model_id.empty()) {
            payload.robot_model_id = model_id;
            payload.has_visual_mesh_model = true;
        }
    }

    if (const auto description_value = robot.get_metadata("robot_description_config")) {
        const auto description_cfg = any_to_map(*description_value).value_or(std::map<std::string, std::any>{});
        std::optional<std::string> description_payload_json;
        auto manifest = build_robot_description_manifest(robot, description_cfg, &description_payload_json);
        if (!manifest.empty()) {
            payload.robot_description_manifest = std::move(manifest);
            payload.robot_description_payload_json = std::move(description_payload_json);
            if (coerce_bool(map_get(payload.robot_description_manifest, "supports_visual_meshes"), false)) {
                payload.has_visual_mesh_model = true;
            }
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
            if (payload.type == "occupancy_grid") {
                OccupancyPayload occupancy;
                bool has_occupancy = false;
                if (visualization.render_options.contains("show_unknown_space")) {
                    occupancy.show_unknown_space = coerce_bool(map_get(visualization.render_options, "show_unknown_space"), true);
                    has_occupancy = true;
                }
                if (visualization.render_options.contains("position_scale")) {
                    occupancy.position_scale = coerce_float(map_get(visualization.render_options, "position_scale"), 1.0f);
                    has_occupancy = true;
                }
                if (visualization.render_options.contains("position_offset")) {
                    occupancy.position_offset = coerce_vec3(map_get(visualization.render_options, "position_offset"), {0.0f, 0.0f, 0.0f});
                    has_occupancy = true;
                }
                if (visualization.render_options.contains("rotation_offset_euler")) {
                    occupancy.rotation_offset_euler = coerce_vec3(map_get(visualization.render_options, "rotation_offset_euler"), {0.0f, 0.0f, 0.0f});
                    has_occupancy = true;
                }
                if (has_occupancy) {
                    payload.occupancy = occupancy;
                }
            }
            if (payload.type == "point_cloud") {
                const auto min_point_size = std::max(0.0001f, coerce_float(map_get(visualization.render_options, "min_point_size"), 0.002f));
                const auto max_point_size = std::max(min_point_size, coerce_float(map_get(visualization.render_options, "max_point_size"), 0.04f));
                const auto max_visible_budget = std::max(1000, coerce_int(map_get(visualization.render_options, "max_visible_points_budget"), 200000));
                auto render_mode = to_lower(coerce_text(map_get(visualization.render_options, "render_mode"), "opaque_fast"));
                if (render_mode != "transparent_hq") {
                    render_mode = "opaque_fast";
                }
                auto point_shape = normalize_point_shape(visualization.render_options);
                payload.point_cloud = {
                    {"point_size", std::max(0.001f, coerce_float(map_get(visualization.render_options, "point_size"), 0.05f))},
                    {"max_points_per_frame", std::max(0, coerce_int(map_get(visualization.render_options, "max_points_per_frame"), 0))},
                    {"base_sample_stride", std::max(1, coerce_int(map_get(visualization.render_options, "base_sample_stride"), 1))},
                    {"min_update_interval", std::max(0.0f, coerce_float(map_get(visualization.render_options, "min_update_interval"), 0.0f))},
                    {"enable_adaptive_quality", coerce_bool(map_get(visualization.render_options, "enable_adaptive_quality"), false)},
                    {"target_framerate", std::max(30.0f, coerce_float(map_get(visualization.render_options, "target_framerate"), 72.0f))},
                    {"min_quality_multiplier", clamp_float(coerce_float(map_get(visualization.render_options, "min_quality_multiplier"), 0.6f), 0.25f, 1.0f)},
                    {"min_distance", std::max(0.0f, coerce_float(map_get(visualization.render_options, "min_distance"), 0.0f))},
                    {"max_distance", std::max(0.0f, coerce_float(map_get(visualization.render_options, "max_distance"), 0.0f))},
                    {"replace_latest", coerce_bool(map_get(visualization.render_options, "replace_latest"), true)},
                    {"render_all_points", coerce_bool(map_get(visualization.render_options, "render_all_points"), true)},
                    {"auto_point_size_by_workspace_scale", coerce_bool(map_get(visualization.render_options, "auto_point_size_by_workspace_scale"), true)},
                    {"min_point_size", min_point_size},
                    {"max_point_size", max_point_size},
                    {"render_mode", render_mode},
                    {"point_shape", point_shape},
                    {"enable_view_frustum_culling", coerce_bool(map_get(visualization.render_options, "enable_view_frustum_culling"), true)},
                    {"frustum_padding", clamp_float(coerce_float(map_get(visualization.render_options, "frustum_padding"), 0.03f), 0.0f, 0.5f)},
                    {"enable_subpixel_culling", coerce_bool(map_get(visualization.render_options, "enable_subpixel_culling"), true)},
                    {"min_screen_radius_px", std::max(0.0f, coerce_float(map_get(visualization.render_options, "min_screen_radius_px"), 0.8f))},
                    {"visible_points_budget", clamp_int(coerce_int(map_get(visualization.render_options, "visible_points_budget"), 120000), 1000, max_visible_budget)},
                    {"max_visible_points_budget", max_visible_budget},
                    {"map_static_mode", coerce_bool(map_get(visualization.render_options, "map_static_mode"), true)},
                };
            }
            if (payload.type == "gaussian_splat") {
                auto asset_format = to_lower(coerce_text(map_get(visualization.render_options, "asset_format"), "3dgs_ply"));
                if (asset_format != "3dgs_ply") {
                    asset_format = "3dgs_ply";
                }
                auto coordinate_space = to_lower(coerce_text(map_get(visualization.render_options, "source_coordinate_space"), "colmap"));
                if (coordinate_space != "3dgs" && coordinate_space != "opencv" && coordinate_space != "unity") {
                    coordinate_space = "colmap";
                }
                auto render_mode = to_lower(coerce_text(map_get(visualization.render_options, "render_mode"), "splats"));
                if (render_mode != "debug_points" && render_mode != "mono_center_eye" && render_mode != "no_covariance") {
                    render_mode = "splats";
                }
                payload.gaussian_splat = {
                    {"manifest_topic", coerce_text(map_get(visualization.render_options, "manifest_topic"), visualization.data_source.topic)},
                    {"chunk_begin_topic", coerce_text(map_get(visualization.render_options, "chunk_begin_topic"), "/horus/gaussian_splat/chunk_begin")},
                    {"chunk_item_topic", coerce_text(map_get(visualization.render_options, "chunk_item_topic"), "/horus/gaussian_splat/chunk_item")},
                    {"chunk_end_topic", coerce_text(map_get(visualization.render_options, "chunk_end_topic"), "/horus/gaussian_splat/chunk_end")},
                    {"asset_format", asset_format},
                    {"source_coordinate_space", coordinate_space},
                    {"render_mode", render_mode},
                    {"max_splats", std::max(1000, coerce_int(map_get(visualization.render_options, "max_splats"), 350000))},
                    {"render_scale", clamp_float(coerce_float(map_get(visualization.render_options, "render_scale"), 0.5f), 0.25f, 1.0f)},
                    {"sh_order", clamp_int(coerce_int(map_get(visualization.render_options, "sh_order"), 2), 0, 3)},
                    {"half_precision_sh", coerce_bool(map_get(visualization.render_options, "half_precision_sh"), true)},
                    {"adaptive_sort", coerce_bool(map_get(visualization.render_options, "adaptive_sort"), true)},
                    {"sort_passes", clamp_int(coerce_int(map_get(visualization.render_options, "sort_passes"), 2), 2, 4)},
                    {"opacity_scale", clamp_float(coerce_float(map_get(visualization.render_options, "opacity_scale"), 1.0f), 0.05f, 20.0f)},
                    {"splat_scale", clamp_float(coerce_float(map_get(visualization.render_options, "splat_scale"), 1.0f), 0.05f, 4.0f)},
                    {"contribution_cull_threshold", clamp_float(coerce_float(map_get(visualization.render_options, "contribution_cull_threshold"), 0.1f), 0.0f, 1.0f)},
                    {"high_precision_rt", coerce_bool(map_get(visualization.render_options, "high_precision_rt"), false)},
                    {"pointcloud_fallback", coerce_bool(map_get(visualization.render_options, "pointcloud_fallback"), true)},
                    {"fallback_topic", coerce_text(map_get(visualization.render_options, "fallback_topic"), "/map_gaussian_splat_preview")},
                    {"fallback_frame", coerce_text(map_get(visualization.render_options, "fallback_frame"), visualization.data_source.frame_id.empty() ? std::string("map") : visualization.data_source.frame_id)},
                };
                if (const auto fallback = map_get(visualization.render_options, "fallback_point_cloud")) {
                    if (const auto fallback_map = any_to_map(*fallback)) {
                        payload.gaussian_splat["fallback_point_cloud"] = *fallback_map;
                    }
                }
            }
            if (payload.type == "mesh") {
                auto coordinate_space = normalize_map_coordinate_space(visualization.render_options, "enu");
                payload.mesh = {
                    {"use_vertex_colors", coerce_bool(map_get(visualization.render_options, "use_vertex_colors"), true)},
                    {"alpha", clamp_float(coerce_float(map_get(visualization.render_options, "alpha"), 1.0f), 0.0f, 1.0f)},
                    {"double_sided", coerce_bool(map_get(visualization.render_options, "double_sided"), true)},
                    {"max_triangles", std::max(1000, coerce_int(map_get(visualization.render_options, "max_triangles"), 200000))},
                    {"source_coordinate_space", coordinate_space},
                };
            }
            if (payload.type == "octomap") {
                auto render_mode = normalize_octomap_render_mode(visualization.render_options);
                auto coordinate_space = normalize_map_coordinate_space(visualization.render_options, "enu");
                payload.octomap = {
                    {"render_mode", render_mode},
                    {"use_vertex_colors", coerce_bool(map_get(visualization.render_options, "use_vertex_colors"), true)},
                    {"alpha", clamp_float(coerce_float(map_get(visualization.render_options, "alpha"), 1.0f), 0.0f, 1.0f)},
                    {"double_sided", coerce_bool(map_get(visualization.render_options, "double_sided"), false)},
                    {"max_triangles", std::max(1000, coerce_int(map_get(visualization.render_options, "max_triangles"), 60000))},
                    {"source_coordinate_space", coordinate_space},
                    {"native_topic", coerce_text(map_get(visualization.render_options, "native_topic"), "/map_3d_octomap")},
                    {"native_frame", coerce_text(map_get(visualization.render_options, "native_frame"), "map")},
                    {"native_binary_only", coerce_bool(map_get(visualization.render_options, "native_binary_only"), true)},
                };
            }
            if (payload.type == "semantic_box") {
                const auto semantic_id = coerce_text(map_get(visualization.render_options, "id"), "");
                const auto label = coerce_text(map_get(visualization.render_options, "label"), "");
                if (!semantic_id.empty() && !label.empty()) {
                    payload.semantic_box = {
                        {"id", semantic_id},
                        {"label", label},
                        {"center", coerce_vec3(map_get(visualization.render_options, "center"), {0.0f, 0.0f, 0.0f})},
                        {"size", coerce_vec3(map_get(visualization.render_options, "size"), {1.0f, 1.0f, 1.0f})},
                        {"rotation_offset_euler", coerce_vec3(map_get(visualization.render_options, "rotation_offset_euler"), {0.0f, 0.0f, 0.0f})},
                    };
                }
            }
            const auto key =
                payload.type == "semantic_box"
                    ? std::make_tuple(
                          payload.type,
                          coerce_text(map_get(payload.semantic_box, "id"), payload.topic),
                          payload.frame.value_or(""))
                    : std::make_tuple(payload.type, payload.topic, payload.frame.value_or(""));
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
    const auto merge_profile = [&](
                                   const std::string& topic,
                                   const std::string& source_mode,
                                   const std::string& minimap_streaming,
                                   const std::string& teleop_streaming,
                                   const std::string& startup_mode) {
        if (topic.empty()) {
            return;
        }

        auto& profile = profiles[topic];
        if (profile.empty()) {
            profile = {
                {"minimap", minimap_streaming},
                {"teleop", teleop_streaming},
                {"startup", startup_mode},
                {"source_mode", source_mode},
            };
            return;
        }

        profile["minimap"] = normalize_transport(profile["minimap"], minimap_streaming);
        profile["teleop"] = normalize_transport(profile["teleop"], teleop_streaming);
        auto startup = to_lower(profile["startup"]);
        if (startup != "minimap" && startup != "teleop") {
            startup = startup_mode;
        }
        profile["startup"] = startup;

        auto merged_source = profile["source_mode"];
        if (merged_source != source_mode) {
            merged_source = "both";
        }
        if (merged_source != "minimap" && merged_source != "teleop" && merged_source != "both") {
            merged_source = "both";
        }
        profile["source_mode"] = merged_source;
    };

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

        const auto minimap_topic = config.minimap_topic.empty() ? sensor.topic : config.minimap_topic;
        const auto teleop_topic = config.teleop_topic.empty() ? sensor.topic : config.teleop_topic;
        merge_profile(minimap_topic, "minimap", minimap_streaming, teleop_streaming, startup_mode);
        merge_profile(teleop_topic, "teleop", minimap_streaming, teleop_streaming, startup_mode);
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
