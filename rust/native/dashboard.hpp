#pragma once
#include "core/include/horus/monitoring/dashboard.hpp"
#include <chrono>
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace horus::native {
class StatusMonitor {
  public:
    explicit StatusMonitor(rclcpp::Node::SharedPtr node);
    void configure(const nlohmann::json &payloads);
    void receipt(const std::string &topic);
    nlohmann::json snapshot();

  private:
    using Clock = std::chrono::steady_clock;
    using Json = nlohmann::json;
    struct Timed {
        Clock::time_point time;
        Json data;
    };
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr presence_sub_, runtime_sub_;
    std::mutex mutex_;
    std::map<std::string, Json> registrations_;
    std::map<std::string, Timed> presence_, runtime_;
    std::map<std::string, Clock::time_point> receipts_;
    Json rows_ = Json::array(), cached_ = Json::object();
    Clock::time_point cache_at_{};
};

class TerminalDashboard {
  public:
    explicit TerminalDashboard(bool enabled);
    ~TerminalDashboard();
    void update(const nlohmann::json &state);

  private:
    bool enabled_, terminal_, color_;
    unsigned previous_lines_{0}, tick_{0};
    std::string last_summary_;
};
} // namespace horus::native
