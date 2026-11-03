#include "collision_runtime.h"
#include "horus/dataviz/collision_risk.hpp"
#include <bit>
#include <cstring>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

namespace {
using namespace horus::dataviz;
using Json = nlohmann::json;
struct Runtime {
    rclcpp::Context::SharedPtr context;
    rclcpp::Node::SharedPtr node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    rclcpp::SubscriptionBase::SharedPtr subscription;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    std::thread thread;
    std::mutex shutdown;
    CollisionRiskOptions options;
    CollisionObservation observation;
    std::string frame;
    double smoothed = 0;
    explicit Runtime(const char *text) {
        if (!text || strnlen(text, 65537) > 65536)
            throw std::invalid_argument("Invalid collision configuration");
        const auto j = Json::parse(text);
        options.robot_name = j.at("robot_name");
        options.source_type = j.value("source_type", "laser_scan");
        options.source_topic = j.at("source_topic");
        options.output_topic = j.value("output_topic", std::string{});
        options.threshold_m = j.value("threshold_m", 1.2);
        options.publish_hz = j.value("publish_hz", 10.);
        options.smoothing_alpha = j.value("smoothing_alpha", .35);
        options.z_window_m = j.value("z_window_m", .35);
        if (options.robot_name.empty() || options.source_topic.empty() ||
            (options.source_type != "laser_scan" && options.source_type != "point_cloud") ||
            !std::isfinite(options.threshold_m) || !std::isfinite(options.publish_hz) ||
            !std::isfinite(options.smoothing_alpha) || !std::isfinite(options.z_window_m))
            throw std::invalid_argument("Invalid collision risk options");
        options.threshold_m = std::max(.001, options.threshold_m);
        options.publish_hz = std::clamp(options.publish_hz, 1., 100.);
        options.smoothing_alpha = std::clamp(options.smoothing_alpha, 0., 1.);
        options.z_window_m = std::max(0., options.z_window_m);
        if (options.output_topic.empty())
            options.output_topic = "/" + options.robot_name + "/collision_risk";
        frame = options.robot_name + "/base_link";
        context = std::make_shared<rclcpp::Context>();
        context->init(0, nullptr);
        node = std::make_shared<rclcpp::Node>("horus_collision_risk",
                                              rclcpp::NodeOptions().context(context));
        publisher = node->create_publisher<std_msgs::msg::String>(options.output_topic,
                                                                  rclcpp::QoS(10).reliable());
        // Best-effort accepts both reliable and best-effort sensor publishers.
        if (options.source_type == "laser_scan")
            subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
                options.source_topic, rclcpp::SensorDataQoS().keep_last(1),
                [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
                    try {
                        if (m->ranges.size() > 2000000)
                            throw std::invalid_argument("Scan exceeds bounded sample budget");
                        observation = analyze_laser_scan<float>(
                            m->ranges, m->angle_min, m->angle_increment,
                            std::max(0.f, m->range_min), std::max(m->range_min, m->range_max));
                        if (!m->header.frame_id.empty())
                            frame = m->header.frame_id;
                    } catch (const std::exception &e) {
                        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                                             "Rejected scan: %s", e.what());
                    }
                });
        else
            subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                options.source_topic, rclcpp::SensorDataQoS().keep_last(1),
                [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
                    try {
                        cloud(*m);
                    } catch (const std::exception &e) {
                        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                                             "Rejected cloud: %s", e.what());
                    }
                });
        timer = node->create_wall_timer(std::chrono::duration<double>(1. / options.publish_hz),
                                        [this] { publish(); });
        rclcpp::ExecutorOptions eo;
        eo.context = context;
        executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
        executor->add_node(node);
        thread = std::thread([this] {
            try {
                executor->spin();
            } catch (const std::exception &e) {
                RCLCPP_ERROR(node->get_logger(), "Collision analyzer stopped: %s", e.what());
            }
        });
    }
    void cloud(const sensor_msgs::msg::PointCloud2 &m) {
        if (!m.point_step || uint64_t(m.width) * m.height > 20000000 ||
            uint64_t(m.width) * m.point_step > m.row_step ||
            uint64_t(m.row_step) * m.height > m.data.size())
            throw std::invalid_argument("Invalid cloud layout");
        std::array<sensor_msgs::msg::PointField, 3> fields;
        for (size_t i = 0; i < 3; i++) {
            auto it = std::find_if(m.fields.begin(), m.fields.end(), [i](const auto &f) {
                return f.name == std::array<const char *, 3>{"x", "y", "z"}[i];
            });
            if (it == m.fields.end() || it->count != 1 ||
                (it->datatype != 7 && it->datatype != 8) ||
                uint64_t(it->offset) + (it->datatype == 7 ? 4 : 8) > m.point_step)
                throw std::invalid_argument("Cloud requires scalar float XYZ");
            fields[i] = *it;
        }
        auto value = [&](const uint8_t *p, const auto &f) {
            std::array<uint8_t, 8> raw{};
            const size_t n = f.datatype == 7 ? 4 : 8;
            std::memcpy(raw.data(), p + f.offset, n);
            if (m.is_bigendian != (std::endian::native == std::endian::big))
                std::reverse(raw.begin(), raw.begin() + n);
            if (n == 4) {
                float v;
                std::memcpy(&v, raw.data(), 4);
                return double(v);
            }
            double v;
            std::memcpy(&v, raw.data(), 8);
            return v;
        };
        CollisionObservation next;
        for (uint32_t y = 0; y < m.height; y++)
            for (uint32_t x = 0; x < m.width; x++) {
                const auto *p = m.data.data() + size_t(y) * m.row_step + size_t(x) * m.point_step;
                include_point(next, value(p, fields[0]), value(p, fields[1]), value(p, fields[2]),
                              options.z_window_m);
            }
        observation = next;
        if (!m.header.frame_id.empty())
            frame = m.header.frame_id;
    }
    void publish() {
        smoothed = options.smoothing_alpha *
                       risk_from_distance(observation.min_distance_m, options.threshold_m) +
                   (1 - options.smoothing_alpha) * smoothed;
        const auto d = smoothed <= 1e-4 ? std::array<double, 3>{} : observation.direction;
        auto rounded = [](double x) { return std::round(x * 10000) / 10000; };
        const auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();
        Json j{{"robot", options.robot_name},
               {"frame", frame},
               {"source", options.source_type},
               {"stamp_ms", stamp},
               {"threshold_m", rounded(options.threshold_m)},
               {"min_distance_m", observation.min_distance_m
                                      ? Json(rounded(*observation.min_distance_m))
                                      : Json(nullptr)},
               {"risk", rounded(smoothed)},
               {"direction", {{"x", rounded(d[0])}, {"y", rounded(d[1])}, {"z", 0.}}}};
        std_msgs::msg::String message;
        message.data = j.dump();
        publisher->publish(message);
    }
    void stop() {
        std::lock_guard lock(shutdown);
        if (executor)
            executor->cancel();
        if (context && context->is_valid())
            context->shutdown("Collision analyzer stopped");
        if (thread.joinable())
            thread.join();
    }
    ~Runtime() { stop(); }
};
} // namespace
extern "C" {
void *horus_collision_create(const char *options, char **error) {
    if (error)
        *error = nullptr;
    try {
        return new Runtime(options);
    } catch (const std::exception &e) {
        if (error)
            *error = strdup(e.what());
    } catch (...) {
        if (error)
            *error = strdup("Collision analyzer failed");
    }
    return nullptr;
}
void horus_collision_stop(void *p) {
    if (p)
        static_cast<Runtime *>(p)->stop();
}
void horus_collision_destroy(void *p) { delete static_cast<Runtime *>(p); }
}
