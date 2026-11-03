#include "horus/monitoring/dashboard.hpp"
#include <algorithm>
#include <iomanip>
#include <map>
#include <regex>
#include <set>
#include <sstream>

namespace horus::monitoring {
using Json = nlohmann::json;
namespace {
std::string text(const Json &j, const char *key, const std::string &fallback = "") {
    auto it = j.find(key);
    return it != j.end() && it->is_string() ? it->get<std::string>() : fallback;
}
std::string clean(std::string value) {
    for (auto &c : value)
        if (static_cast<unsigned char>(c) < 32 || c == 127)
            c = ' ';
    return value;
}
std::string fit(std::string s, unsigned width) {
    s = clean(s);
    if (s.size() > width)
        s = s.substr(0, width > 3 ? width - 3 : 0) + "...";
    s.resize(width, ' ');
    return s;
}
} // namespace
bool is_bridge_node(const std::string &name) {
    static const std::regex pattern("^/*horus_(unity_bridge|backend(_node)?)(_[a-zA-Z0-9]+)*$");
    const auto base =
        name.substr(name.find_last_of('/') == std::string::npos ? 0 : name.find_last_of('/') + 1);
    return std::regex_match(base, pattern) || base.ends_with("_RosSubscriber") ||
           base.ends_with("_RosPublisher");
}
Json registration_topics(const Json &payloads) {
    std::map<std::string, Json> rows;
    auto add = [&](const std::string &topic, const std::string &robot, const std::string &role,
                   Json profile = Json::object()) {
        if (topic.empty() || topic[0] != '/')
            return;
        if (rows.size() >= 8192 && !rows.count(topic))
            throw std::invalid_argument("dashboard exceeds 8192 distinct registered topics");
        auto it = rows.find(topic);
        if (it != rows.end()) {
            if (it->second["robot"] != robot)
                it->second["robot"] = "shared";
            if (role == "backend_pub")
                it->second["role"] = role;
            if (!profile.empty())
                it->second["camera_profile"] = std::move(profile);
            return;
        }
        rows[topic] = {{"topic", topic},
                       {"robot", robot},
                       {"role", role},
                       {"camera_profile", std::move(profile)}};
    };
    const std::map<std::string, std::string> core = {
        {"/horus/registration", "sdk_pub"},
        {"/horus/registration_ack", "sdk_sub"},
        {"/horus/heartbeat", "sdk_sub"},
        {"/horus/multi_operator_presence", "sdk_sub"},
        {"/horus/teleop/runtime_state", "sdk_sub"},
        {"/horus/multi_operator/sdk_registration_replay_request", "sdk_sub"},
        {"/horus/multi_operator/sdk_registry_replay_begin", "sdk_pub"},
        {"/horus/multi_operator/sdk_registry_replay_item", "sdk_pub"},
        {"/horus/multi_operator/sdk_registry_replay_end", "sdk_pub"},
        {"/horus/robot_description/request", "sdk_sub"}};
    for (const auto &[topic, role] : core)
        add(topic, "sdk", role);
    for (const auto &robot : payloads) {
        const auto name = text(robot, "robot_name", "shared");
        for (const auto &sensor : robot.value("sensors", Json::array())) {
            const auto topic = text(sensor, "topic");
            add(topic, name, "backend_sub");
            if (text(sensor, "type") != "camera")
                continue;
            const auto cfg = sensor.value("camera_config", Json::object());
            const auto minimap =
                text(cfg, "minimap_topic").empty() ? topic : text(cfg, "minimap_topic");
            const auto teleop =
                text(cfg, "teleop_topic").empty() ? topic : text(cfg, "teleop_topic");
            Json profile = {{"minimap", text(cfg, "minimap_streaming_type",
                                             text(cfg, "streaming_type", "ros"))},
                            {"teleop", text(cfg, "teleop_streaming_type",
                                            text(cfg, "streaming_type", "webrtc"))},
                            {"startup", text(cfg, "startup_mode", "minimap")}};
            profile["source_mode"] = minimap == teleop ? "both" : "minimap";
            add(minimap, name, "backend_sub", profile);
            profile["source_mode"] = minimap == teleop ? "both" : "teleop";
            add(teleop, name, "backend_sub", profile);
            for (const auto *key : {"right_topic", "teleop_right_topic"})
                add(text(cfg, key), name, "backend_sub", profile);
        }
        for (const auto *key : {"visualizations", "global_visualizations"}) {
            for (const auto &viz : robot.value(key, Json::array())) {
                const auto group = std::string(key) == "global_visualizations" ? "shared" : name;
                const auto topic = text(viz, "topic");
                add(topic, group, "backend_sub");
                if (text(viz, "type") == "tf" || text(viz, "type") == "transform")
                    add(topic == "/tf" ? "/tf_static" : topic + "_static", group, "backend_sub");
                if (text(viz, "type") == "mesh" &&
                    viz.value("mesh", Json::object()).value("use_indexed_binary", false)) {
                    add(topic + "/indexed", group, "backend_sub");
                    add(topic + "/indexed_manifest", group, "backend_sub");
                    add(topic + "/indexed_request", group, "backend_pub");
                }
            }
        }
        // Control topic names are explicit contract fields, never inferred from frame prefixes.
        if (robot.contains("control")) {
            const auto &control = robot["control"];
            add(text(control, "drive_topic"), name, "backend_pub");
            for (const auto &[key, val] : control.items()) {
                if (!val.is_object())
                    continue;
                for (const auto &[child_key, child] : val.items()) {
                    if (child.is_string() && child_key.ends_with("topic"))
                        add(child.get<std::string>(), name, "backend_pub");
                    if (key == "tasks" && child.is_object()) {
                        for (const auto *topic_key :
                             {"topic", "goal_topic", "command_topic", "cancel_topic"})
                            add(text(child, topic_key), name, "backend_pub");
                    }
                }
            }
        }
    }
    Json result = Json::array();
    for (auto &[_, row] : rows)
        result.push_back(std::move(row));
    return result;
}
std::string graph_link(const std::string &role, const Json &pubs, const Json &subs,
                       const std::string &local_name, const std::string &local_namespace) {
    if (!pubs.is_array() || !subs.is_array())
        return "UNKNOWN";
    auto bridge = [](const Json &list) {
        return std::any_of(list.begin(), list.end(),
                           [](const Json &e) { return is_bridge_node(text(e, "name")); });
    };
    if (role == "sdk_pub" || role == "sdk_sub") {
        const auto &local = role == "sdk_pub" ? pubs : subs;
        bool found = std::any_of(local.begin(), local.end(), [&](const Json &e) {
            return text(e, "name") == local_name && text(e, "namespace") == local_namespace;
        });
        if (!found)
            return "NO_SDK";
        return bridge(role == "sdk_pub" ? subs : pubs) ? "OK" : "NO_BRIDGE";
    }
    if (role == "backend_pub")
        return !bridge(pubs) ? "NO_BRIDGE" : subs.empty() ? "NO_CONSUMER" : "OK";
    return pubs.empty() ? "NO_SOURCE" : bridge(subs) ? "OK" : "NO_BRIDGE";
}
std::string render_dashboard(const Json &state, unsigned width, unsigned height, unsigned page,
                             bool color) {
    width = std::clamp(width, 48u, 180u);
    height = std::clamp(height, 12u, 100u);
    std::ostringstream out;
    auto line = [&](const std::string &s) { out << "| " << fit(s, width - 4) << " |\n"; };
    const auto border = "+" + std::string(width - 2, '-') + "+\n";
    out << border;
    line("HORUS | Native SDK");
    line("App: " + std::string(state.value("app_connected", false) ? "CONNECTED"
                               : state.value("heartbeat_seen", false)
                                   ? "DISCONNECTED (heartbeat expired)"
                                   : "WAITING"));
    line("Bridge: " + text(state, "bridge_state", "Unknown"));
    line("Registration: " + text(state, "registration_state", "Idle") +
         " | Accepted: " + std::to_string(state.value("registration_count", std::size_t(0))));
    line("Operators: " + text(state, "operator_summary", "No fresh presence report"));
    if (!text(state, "error").empty())
        line("Error: " + text(state, "error"));
    out << border;
    const auto topics = state.value("topics", Json::array());
    const unsigned count = std::max(1u, height - 11);
    const unsigned pages = std::max(1u, static_cast<unsigned>((topics.size() + count - 1) / count));
    page %= pages;
    unsigned link_width = width >= 80 ? 12 : 10;
    const unsigned topic_width = width - link_width - 13;
    for (std::size_t i = page * count; i < std::min(topics.size(), std::size_t((page + 1) * count));
         ++i) {
        const auto &row = topics[i];
        const auto transport = text(row, "transport", "ros");
        const bool media = transport == "webrtc";
        auto badge = fit(media ? "WebRTC" : "ROS", 7);
        if (color)
            badge = std::string(media ? "\033[34m" : "\033[36m") + badge + "\033[0m";
        out << "| " << fit(text(row, "topic"), topic_width) << " " << badge << " "
            << fit(text(row, "link", "UNKNOWN"), link_width) << " |\n";
    }
    line("Topics " + std::to_string(page + 1) + "/" + std::to_string(pages) +
         " | Graph only; sensor delivery not measured");
    out << border;
    return out.str();
}
} // namespace horus::monitoring
