#include "dashboard.hpp"
#include <cstdlib>
#include <iostream>
#include <sys/ioctl.h>
#include <unistd.h>

namespace horus::native {
using Json = nlohmann::json;
namespace {
std::string text(const Json &j, const char *key, const std::string &fallback = "") {
    auto it = j.find(key);
    return it != j.end() && it->is_string() ? it->get<std::string>() : fallback;
}
bool yes(const Json &j, const char *key) { return j.contains(key) && j[key] == true; }
} // namespace
StatusMonitor::StatusMonitor(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    using String = std_msgs::msg::String;
    presence_sub_ = node_->create_subscription<String>(
        "/horus/multi_operator_presence", 32, [this](String::ConstSharedPtr msg) {
            if (msg->data.size() > 16384)
                return;
            auto j = Json::parse(msg->data, nullptr, false);
            if (!j.is_object())
                return;
            const auto id = text(j, "app_id", text(j, "ip", "unknown"));
            std::lock_guard lock(mutex_);
            const auto now = Clock::now();
            std::erase_if(presence_, [&](const auto &e) {
                return now - e.second.time > std::chrono::seconds(5);
            });
            if (presence_.size() >= 256 && !presence_.count(id))
                return;
            presence_[id] = {now, std::move(j)};
            receipts_["/horus/multi_operator_presence"] = now;
            cache_at_ = {};
        });
    runtime_sub_ = node_->create_subscription<String>(
        "/horus/teleop/runtime_state", 32, [this](String::ConstSharedPtr msg) {
            if (msg->data.size() > 16384)
                return;
            auto j = Json::parse(msg->data, nullptr, false);
            if (!j.is_object() || text(j, "camera_topic").empty())
                return;
            const auto topic = text(j, "camera_topic");
            const auto transport = text(j, "transport");
            if (transport != "ros" && transport != "webrtc")
                return;
            std::lock_guard lock(mutex_);
            const auto now = Clock::now();
            std::erase_if(runtime_, [&](const auto &e) {
                return now - e.second.time > std::chrono::seconds(5);
            });
            if (runtime_.size() >= 2048 && !runtime_.count(topic))
                return;
            runtime_[topic] = {now, std::move(j)};
            receipts_["/horus/teleop/runtime_state"] = now;
            cache_at_ = {};
        });
}
void StatusMonitor::configure(const Json &payloads) {
    std::lock_guard lock(mutex_);
    auto next = registrations_;
    for (const auto &payload : payloads) {
        const auto name = text(payload, "robot_name");
        if (text(payload, "action") == "unregister")
            next.erase(name);
        else {
            // Do not retain mesh JSON or credentials in dashboard snapshots.
            Json compact;
            for (const auto *key :
                 {"robot_name", "sensors", "visualizations", "global_visualizations", "control"})
                if (payload.contains(key))
                    compact[key] = payload[key];
            next[name] = std::move(compact);
        }
    }
    Json list = Json::array();
    for (const auto &[_, payload] : next)
        list.push_back(payload);
    auto rows = monitoring::registration_topics(list);
    registrations_ = std::move(next);
    rows_ = std::move(rows);
    cache_at_ = {};
}
void StatusMonitor::receipt(const std::string &topic) {
    std::lock_guard lock(mutex_);
    receipts_[topic] = Clock::now();
}
Json StatusMonitor::snapshot() {
    const auto now = Clock::now();
    Json rows;
    std::map<std::string, Timed> presence, runtime;
    std::map<std::string, Clock::time_point> receipts;
    {
        std::lock_guard lock(mutex_);
        if (now - cache_at_ < std::chrono::seconds(1))
            return cached_;
        rows = rows_;
        presence = presence_;
        runtime = runtime_;
        receipts = receipts_;
    }
    bool app = receipts.count("/horus/heartbeat") &&
               now - receipts["/horus/heartbeat"] <= std::chrono::seconds(3);
    auto endpoints = [](const auto &values) {
        Json result = Json::array();
        for (const auto &value : values)
            result.push_back({{"name", value.node_name()}, {"namespace", value.node_namespace()}});
        return result;
    };
    for (auto &row : rows) {
        const auto topic = text(row, "topic"), role = text(row, "role");
        row["link"] = "UNKNOWN";
        row["pubs"] = nullptr;
        row["subs"] = nullptr;
        row["data"] = "Not measured";
        row["transport"] = "ros";
        row["transport_evidence"] = "configured";
        auto profile = row["camera_profile"];
        if (!profile.empty()) {
            auto mode = text(profile, "source_mode", "minimap");
            if (mode == "both")
                mode = text(profile, "startup", "minimap");
            row["transport"] = text(profile, mode.c_str(), "ros");
        }
        auto active = runtime.find(topic);
        if (app && active != runtime.end() && now - active->second.time < std::chrono::seconds(5) &&
            yes(active->second.data, "active")) {
            row["transport"] = text(active->second.data, "transport", "ros");
            row["transport_evidence"] = "app_state";
        }
        try {
            const auto pubs = endpoints(node_->get_publishers_info_by_topic(topic));
            const auto subs = endpoints(node_->get_subscriptions_info_by_topic(topic));
            row["pubs"] = pubs.size();
            row["subs"] = subs.size();
            row["link"] =
                monitoring::graph_link(role, pubs, subs, node_->get_name(), node_->get_namespace());
            if (row["transport"] == "webrtc")
                row["link"] = "MEDIA";
        } catch (const std::exception &) { /* Unknown is not evidence of disconnection. */
        }
        if (role == "sdk_sub") {
            auto receipt = receipts.find(topic);
            row["data"] = receipt == receipts.end() ? "Waiting" : "Received";
            if (receipt != receipts.end()) {
                row["receipt_age_s"] = std::chrono::duration<double>(now - receipt->second).count();
                if (topic == "/horus/heartbeat" && !app)
                    row["data"] = "Stale";
            }
        }
    }
    Json operators = Json::array();
    unsigned ready = 0;
    for (const auto &[id, value] : presence) {
        if (now - value.time > std::chrono::seconds(5))
            continue;
        const auto state = text(value.data, "state");
        const bool workspace_ready =
            yes(value.data, "workspace_active") &&
            (state.rfind("workspace_ready", 0) == 0 || state == "private_workspace_ready" ||
             state == "workspace_synced" || state == "registry_synced");
        ready += workspace_ready;
        operators.push_back({{"app_id", id},
                             {"role", text(value.data, "role")},
                             {"state", state},
                             {"workspace_ready", workspace_ready}});
    }
    Json result = {{"topics", rows},
                   {"operators", operators},
                   {"operator_summary", operators.empty() ? std::string("No fresh presence report")
                                                          : std::to_string(operators.size()) +
                                                                " reporting | workspace-ready: " +
                                                                std::to_string(ready)}};
    try {
        const auto names = node_->get_node_names();
        result["bridge_state"] =
            app ? "Responding (app heartbeat)"
            : std::any_of(names.begin(), names.end(), monitoring::is_bridge_node)
                ? "Discovered (ROS graph)"
                : "Not discovered";
    } catch (const std::exception &) {
        result["bridge_state"] = "Unknown (ROS graph unavailable)";
    }
    std::lock_guard lock(mutex_);
    cached_ = result;
    cache_at_ = now;
    return result;
}
TerminalDashboard::TerminalDashboard(bool enabled)
    : enabled_(enabled), terminal_(isatty(STDOUT_FILENO)),
      color_(terminal_ && !std::getenv("NO_COLOR")) {}
TerminalDashboard::~TerminalDashboard() {
    if (enabled_ && terminal_)
        std::cout << "\033[0m" << std::flush;
}
void TerminalDashboard::update(const Json &state) {
    if (!enabled_)
        return;
    if (!terminal_) {
        Json summary = state;
        summary.erase("topics");
        summary.erase("models");
        summary.erase("bridge");
        auto serialized = summary.dump();
        if (serialized != last_summary_) {
            std::cout << "[HORUS] " << serialized << std::endl;
            last_summary_ = std::move(serialized);
        }
        return;
    }
    winsize size{};
    unsigned width = 100, height = 24;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) == 0) {
        if (size.ws_col)
            width = size.ws_col;
        if (size.ws_row)
            height = size.ws_row;
    }
    if (previous_lines_)
        std::cout << "\033[" << previous_lines_ << "A\r";
    auto output = monitoring::render_dashboard(state, width, height, tick_++ / 4, color_);
    std::size_t start = 0;
    unsigned lines = 0;
    while (start < output.size()) {
        auto end = output.find('\n', start);
        if (end == std::string::npos)
            end = output.size();
        std::cout << "\033[2K\r" << output.substr(start, end - start) << '\n';
        start = end + 1;
        ++lines;
    }
    for (unsigned i = lines; i < previous_lines_; ++i)
        std::cout << "\033[2K\n";
    if (previous_lines_ > lines)
        std::cout << "\033[" << previous_lines_ - lines << "A";
    previous_lines_ = lines;
    std::cout << std::flush;
}
} // namespace horus::native
