#include "tf_runtime.h"
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <tf2_msgs/msg/tf_message.hpp>

namespace {
using Message = tf2_msgs::msg::TFMessage;
using Transform = geometry_msgs::msg::TransformStamped;
constexpr size_t capacity = 4096;
std::string frame(const char *text) {
    if (!text || strnlen(text, 257) == 0 || strnlen(text, 257) > 256)
        throw std::invalid_argument("TF frame must contain 1..256 characters");
    std::string value(text);
    if (value.front() == '/' || value.find_first_of(" \t\r\n") != std::string::npos)
        throw std::invalid_argument("TF frames cannot start with '/' or contain whitespace");
    return value;
}
struct Runtime {
    rclcpp::Context::SharedPtr context;
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    rclcpp::Publisher<Message>::SharedPtr dynamic, fixed;
    std::map<std::string, Transform> retained;

    explicit Runtime(const char *config) {
        if (!config || strnlen(config, 4097) > 4096)
            throw std::invalid_argument("Invalid TF publisher configuration");
        const auto o = nlohmann::json::parse(config);
        const auto topic = o.value("topic", std::string("/tf"));
        const auto static_topic = o.value("static_topic", std::string("/tf_static"));
        if (topic.empty() || static_topic.empty() || topic == static_topic)
            throw std::invalid_argument("TF and static TF topics must be distinct and nonempty");
        context = std::make_shared<rclcpp::Context>();
        rclcpp::InitOptions init;
        init.shutdown_on_signal = false;
        context->init(0, nullptr, init);
        try {
            rclcpp::NodeOptions options;
            options.context(context);
            options.use_global_arguments(false);
            options.append_parameter_override("use_sim_time", o.value("use_sim_time", false));
            static std::atomic<uint64_t> next{};
            node = std::make_shared<rclcpp::Node>("horus_tf_publisher_" + std::to_string(++next),
                                                  options);
            dynamic = node->create_publisher<Message>(topic, rclcpp::QoS(100).reliable());
            fixed = node->create_publisher<Message>(static_topic,
                                                    rclcpp::QoS(1).reliable().transient_local());
            rclcpp::ExecutorOptions exec;
            exec.context = context;
            executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(exec);
            executor->add_node(node);
        } catch (...) {
            context->shutdown("TF initialization failed");
            throw;
        }
    }
    ~Runtime() { context->shutdown("TF publisher destroyed"); }

    void publish(const HorusNativeTransform *input, size_t count, bool is_static) {
        if (count > capacity || (!input && count))
            throw std::invalid_argument("TF batch exceeds 4096 transforms or is null");
        if (!count)
            return;
        executor->spin_some();
        const auto now = node->now().nanoseconds();
        Message message;
        message.transforms.reserve(count);
        std::set<std::string> children;
        // Validate the complete batch before publishing or mutating retained static TF.
        for (size_t i = 0; i < count; ++i) {
            const auto &source = input[i];
            Transform out;
            out.header.frame_id = frame(source.parent);
            out.child_frame_id = frame(source.child);
            if (out.header.frame_id == out.child_frame_id ||
                !children.insert(out.child_frame_id).second)
                throw std::invalid_argument("TF has a self-parent or duplicate child in one batch");
            for (double value : source.translation)
                if (!std::isfinite(value))
                    throw std::invalid_argument("TF translation must be finite");
            double norm = 0;
            for (double value : source.rotation)
                norm = std::hypot(norm, value);
            if (!std::isfinite(norm) || std::abs(norm - 1) > 1e-6)
                throw std::invalid_argument("TF quaternion must be finite and normalized (xyzw)");
            const int64_t stamp = source.stamp_ns == -1 ? now : source.stamp_ns;
            if (stamp < 0 || stamp / 1000000000LL > std::numeric_limits<int32_t>::max())
                throw std::invalid_argument("TF stamp outside builtin_interfaces/Time range");
            out.header.stamp.sec = static_cast<int32_t>(stamp / 1000000000LL);
            out.header.stamp.nanosec = static_cast<uint32_t>(stamp % 1000000000LL);
            out.transform.translation.x = source.translation[0];
            out.transform.translation.y = source.translation[1];
            out.transform.translation.z = source.translation[2];
            out.transform.rotation.x = source.rotation[0];
            out.transform.rotation.y = source.rotation[1];
            out.transform.rotation.z = source.rotation[2];
            out.transform.rotation.w = source.rotation[3];
            message.transforms.push_back(std::move(out));
        }
        if (!is_static) {
            dynamic->publish(message);
            return;
        }
        auto updated = retained;
        for (auto &item : message.transforms)
            updated[item.child_frame_id] = std::move(item);
        if (updated.size() > capacity)
            throw std::invalid_argument("Retained static TF exceeds 4096 frames");
        message.transforms.clear();
        message.transforms.reserve(updated.size());
        for (const auto &[child, transform] : updated)
            message.transforms.push_back(transform);
        fixed->publish(message);
        retained.swap(updated);
    }
};
void set_error(char **error, const char *what) {
    if (error)
        *error = strdup(what);
}
} // namespace

extern "C" void *horus_native_tf_create(const char *options, char **error) {
    if (error)
        *error = nullptr;
    try {
        return new Runtime(options);
    } catch (const std::exception &e) {
        set_error(error, e.what());
        return nullptr;
    } catch (...) {
        set_error(error, "Unknown TF initialization error");
        return nullptr;
    }
}
extern "C" int horus_native_tf_publish(void *handle, const HorusNativeTransform *transforms,
                                       size_t count, int is_static, char **error) {
    if (error)
        *error = nullptr;
    try {
        if (!handle)
            throw std::invalid_argument("Missing TF publisher");
        static_cast<Runtime *>(handle)->publish(transforms, count, is_static != 0);
        return 1;
    } catch (const std::exception &e) {
        set_error(error, e.what());
        return 0;
    } catch (...) {
        set_error(error, "Unknown TF publication error");
        return 0;
    }
}
extern "C" void horus_native_tf_destroy(void *handle) { delete static_cast<Runtime *>(handle); }
extern "C" int64_t horus_native_tf_subscribers(void *handle, int is_static, char **error) {
    if (error)
        *error = nullptr;
    try {
        if (!handle)
            throw std::invalid_argument("Missing TF publisher");
        auto *runtime = static_cast<Runtime *>(handle);
        return (is_static ? runtime->fixed : runtime->dynamic)->get_subscription_count();
    } catch (const std::exception &e) {
        set_error(error, e.what());
        return -1;
    } catch (...) {
        set_error(error, "Unknown TF discovery error");
        return -1;
    }
}
