#include "mesh_transport.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <nlohmann/json.hpp>
#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>

namespace {
using Json = nlohmann::json;
using String = std_msgs::msg::String;
using Bytes = std_msgs::msg::UInt8MultiArray;
char* text(const std::string& value) {
    auto* result = new char[value.size() + 1];
    std::memcpy(result, value.c_str(), value.size() + 1);
    return result;
}
char* error_text(const char* value) noexcept {
    try { return text(value); } catch (...) { return nullptr; }
}
class MeshTransport {
public:
    MeshTransport(const std::string& topic, std::size_t limit) : limit_(limit) {
        if (topic.size() < 2 || topic.front() != '/' || topic.back() == '/' ||
            topic.find('~') != std::string::npos || topic.find('{') != std::string::npos ||
            limit < 52 || limit > 64 * 1024 * 1024)
            throw std::invalid_argument("Use an absolute map topic and a payload limit of 52 bytes..64 MiB");
        context_ = std::make_shared<rclcpp::Context>();
        context_->init(0, nullptr);
        static std::atomic<unsigned> sequence{0};
        node_ = std::make_shared<rclcpp::Node>("horus_native_mesh_" + std::to_string(sequence++) + "_" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()),
            rclcpp::NodeOptions().context(context_).use_global_arguments(false));
        pages_ = node_->create_publisher<Bytes>(topic + "/indexed", rclcpp::QoS(8).reliable());
        manifest_ = node_->create_publisher<String>(topic + "/indexed_manifest", rclcpp::QoS(1).reliable().transient_local());
        request_ = node_->create_subscription<String>(topic + "/indexed_request", rclcpp::QoS(16).reliable(),
            [this](String::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> guard(mutex_);
                if (msg->data.size() > 256 * 1024 || requests_.size() >= 16) { ++dropped_; return; }
                requests_.push_back(msg->data);
            });
        rclcpp::ExecutorOptions options;
        options.context = context_;
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(options);
        executor_->add_node(node_);
        thread_ = std::thread([this] {
            try { executor_->spin(); }
            catch (const std::exception& error) { std::lock_guard<std::mutex> guard(mutex_); error_ = error.what(); }
        });
    }
    ~MeshTransport() {
        // A later spin() can reset cancel(); a shut-down private context cannot.
        try { context_->shutdown("Mesh transport destroyed"); } catch (...) {}
        try { executor_->cancel(); } catch (...) {}
        if (thread_.joinable()) thread_.join();
    }
    void publish(const std::uint8_t* data, std::size_t size) {
        if (!data || size < 52 || size > limit_) throw std::invalid_argument("Invalid mesh payload size");
        Bytes message;
        message.data.assign(data, data + size);
        // rclcpp owns serialization; never hold the repair queue lock during I/O.
        std::lock_guard<std::mutex> guard(publish_mutex_);
        pages_->publish(message);
    }
    void manifest(const std::string& value) {
        if (value.size() > 1024 * 1024 || !Json::parse(value, nullptr, false).is_object())
            throw std::invalid_argument("Invalid map manifest JSON or size");
        String message;
        message.data = value;
        std::lock_guard<std::mutex> guard(publish_mutex_);
        manifest_->publish(message);
    }
    Json poll() {
        const auto subscribers = pages_->get_subscription_count();
        std::lock_guard<std::mutex> guard(mutex_);
        Json request = nullptr;
        if (!requests_.empty()) { request = std::move(requests_.front()); requests_.pop_front(); }
        return {{"subscribers",subscribers}, {"request",request}, {"dropped_requests",dropped_}, {"error",error_}};
    }
private:
    std::size_t limit_;
    rclcpp::Context::SharedPtr context_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<Bytes>::SharedPtr pages_;
    rclcpp::Publisher<String>::SharedPtr manifest_;
    rclcpp::Subscription<String>::SharedPtr request_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread thread_;
    std::mutex mutex_, publish_mutex_;
    std::deque<std::string> requests_;
    std::uint64_t dropped_{0};
    std::string error_;
};
}

extern "C" {
void* horus_native_mesh_create(const char* topic, std::size_t limit, char** error) {
    if (error) *error = nullptr;
    try { return new MeshTransport(topic ? topic : "", limit); }
    catch (const std::exception& exc) { if (error) *error = error_text(exc.what()); }
    catch (...) { if (error) *error = error_text("Unknown native mesh creation error"); }
    return nullptr;
}
int horus_native_mesh_publish(void* handle, const std::uint8_t* data, std::size_t size, char** error) {
    if (error) *error = nullptr;
    try {
        if (!handle) throw std::invalid_argument("Missing mesh transport");
        static_cast<MeshTransport*>(handle)->publish(data, size);
        return 1;
    } catch (const std::exception& exc) { if (error) *error = error_text(exc.what()); }
    catch (...) { if (error) *error = error_text("Unknown native mesh publication error"); }
    return 0;
}
int horus_native_mesh_manifest(void* handle, const char* json, char** error) {
    if (error) *error = nullptr;
    try {
        if (!handle || !json) throw std::invalid_argument("Missing mesh transport or manifest");
        static_cast<MeshTransport*>(handle)->manifest(json);
        return 1;
    } catch (const std::exception& exc) { if (error) *error = error_text(exc.what()); }
    catch (...) { if (error) *error = error_text("Unknown native mesh manifest error"); }
    return 0;
}
char* horus_native_mesh_poll(void* handle) {
    try {
        if (!handle) throw std::invalid_argument("Missing mesh transport");
        return text(static_cast<MeshTransport*>(handle)->poll().dump());
    } catch (const std::exception& exc) {
        try { return text(Json{{"error",exc.what()}}.dump()); } catch (...) { return nullptr; }
    } catch (...) { return nullptr; }
}
void horus_native_mesh_destroy(void* handle) { delete static_cast<MeshTransport*>(handle); }
}
