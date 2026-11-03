#include "description_runtime.h"
#include "horus/description/compiler.hpp"
#include <chrono>
#include <cmath>
#include <cstring>
#include <future>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

namespace {
using Json = nlohmann::json;
Json resolve(Json options) {
    const auto source = options.value("source", "ros");
    const auto parameter_node = options.value("ros_param_node", "");
    if (source == "local")
        throw std::invalid_argument("No URDF path configured for source='local'");
    if (source != "ros" && source != "topic")
        throw std::invalid_argument("Description source must be local, ros or topic");
    if (source == "ros" && parameter_node.empty())
        throw std::invalid_argument("No ros_param_node configured for source='ros'");
    const double timeout = options.value("source_timeout_s", 5.0);
    if (!std::isfinite(timeout) || timeout <= 0 || timeout > 60)
        throw std::invalid_argument("Description source timeout must be in (0,60]");
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    struct Shutdown {
        std::shared_ptr<rclcpp::Context> context;
        ~Shutdown() { context->shutdown("Description source resolved"); }
    } shutdown{context};
    auto node = std::make_shared<rclcpp::Node>(
        "horus_description_source_" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()),
        rclcpp::NodeOptions().context(context).use_global_arguments(false));
    rclcpp::ExecutorOptions eo;
    eo.context = context;
    rclcpp::executors::SingleThreadedExecutor executor(eo);
    executor.add_node(node);
    using String = std_msgs::msg::String;
    std::string xml;
    auto receive = [&](String::ConstSharedPtr msg) {
        if (xml.empty() && !msg->data.empty()) {
            if (msg->data.size() > 16 * 1024 * 1024)
                throw std::length_error("Oversized ROS robot description");
            xml = msg->data;
        }
    };
    const auto topic = options.value("robot_description_topic", "/robot_description");
    rclcpp::Subscription<String>::SharedPtr retained, live;
    if (source == "topic") {
        retained = node->create_subscription<String>(
            topic, rclcpp::QoS(1).reliable().transient_local(), receive);
        live = node->create_subscription<String>(
            topic, rclcpp::QoS(1).best_effort().durability_volatile(), receive);
    }
    using Service = rcl_interfaces::srv::GetParameters;
    rclcpp::Client<Service>::SharedPtr client;
    std::optional<rclcpp::Client<Service>::FutureAndRequestId> pending;
    if (source == "ros")
        client = node->create_client<Service>(parameter_node + "/get_parameters");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    while (xml.empty() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some(std::chrono::milliseconds(10));
        if (client && !pending && client->service_is_ready()) {
            auto req = std::make_shared<Service::Request>();
            req->names = {options.value("ros_param_name", "robot_description")};
            pending = client->async_send_request(req);
        }
        if (pending && pending->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto response = pending->get();
            pending.reset();
            if (!response->values.empty() &&
                response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
                xml = response->values[0].string_value;
            client.reset();
        }
        if (xml.empty())
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (pending)
        client->remove_pending_request(pending->request_id);
    if (xml.empty())
        throw std::runtime_error("Timed out waiting for robot description topic/parameter");
    return horus::description::compile_urdf(xml, options).to_dict();
}
char *copy(const std::string &s) {
    auto *p = new char[s.size() + 1];
    std::memcpy(p, s.c_str(), s.size() + 1);
    return p;
}
} // namespace
extern "C" char *horus_native_description_resolve(const char *options) {
    try {
        if (!options)
            throw std::invalid_argument("Missing description options");
        return copy(Json{{"ok", true}, {"value", resolve(Json::parse(options))}}.dump());
    } catch (const std::exception &e) {
        try {
            return copy(Json{{"ok", false}, {"error", e.what()}}.dump());
        } catch (...) {
            return nullptr;
        }
    } catch (...) {
        return nullptr;
    }
}
