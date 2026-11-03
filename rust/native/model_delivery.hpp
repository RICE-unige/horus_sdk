#pragma once
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace horus::native {
// Request-driven model delivery runs one bounded chunk per timer tick. Large
// bodies never sleep in the registry callback or block heartbeat processing.
class ModelDelivery {
  public:
    explicit ModelDelivery(rclcpp::Node::SharedPtr node);
    void prepare(nlohmann::json &payload);
    void stop();
    nlohmann::json status() const;

  private:
    using Json = nlohmann::json;
    using String = std_msgs::msg::String;
    using Publisher = rclcpp::Publisher<String>::SharedPtr;
    struct Model {
        Json manifest;
        std::string encoded;
    };
    struct Transfer {
        Json request;
        std::string name;
        std::shared_ptr<const Model> model;
        Publisher begin, item, end;
        std::size_t next = 0, count = 0, chunk = 0;
        bool begun = false;
        std::chrono::steady_clock::time_point due;
        std::chrono::steady_clock::time_point discovery_deadline;
    };
    void request(const Json &value);
    void tick();
    Publisher publisher(const std::string &topic);
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    mutable std::mutex mutex_;
    std::map<std::string, std::shared_ptr<const Model>> models_;
    std::map<std::string, Publisher> publishers_;
    std::map<std::string, Transfer> transfers_;
    std::string cursor_, error_;
    bool stopped_ = false;
    std::size_t bytes_ = 0, sent_ = 0;
};
} // namespace horus::native
