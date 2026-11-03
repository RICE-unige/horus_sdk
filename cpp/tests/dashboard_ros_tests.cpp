#include "registry.h"
#include <cassert>
#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

using Json = nlohmann::json;
using String = std_msgs::msg::String;
using namespace std::chrono_literals;
Json take(char *raw) {
    std::unique_ptr<char, decltype(&horus_native_free)> owned(raw, horus_native_free);
    if (!raw)
        throw std::runtime_error("empty result");
    return Json::parse(raw);
}
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto bridge = std::make_shared<rclcpp::Node>("horus_unity_bridge_dashboard_test");
    auto source = std::make_shared<rclcpp::Node>("native_dashboard_camera");
    auto ack = bridge->create_publisher<String>("/horus/registration_ack", 8);
    auto hb = bridge->create_publisher<String>("/horus/heartbeat", rclcpp::QoS(1).best_effort());
    auto presence = bridge->create_publisher<String>("/horus/multi_operator_presence", 8);
    auto runtime = bridge->create_publisher<String>("/horus/teleop/runtime_state", 8);
    auto camera = source->create_publisher<String>("/native/dashboard/camera", 1);
    auto reg = bridge->create_subscription<String>(
        "/horus/registration", rclcpp::QoS(256).transient_local(), [&](String::ConstSharedPtr m) {
            auto p = Json::parse(m->data);
            String reply;
            reply.data =
                Json{{"robot_name", p["robot_name"]}, {"success", true}, {"robot_id", "test"}}
                    .dump();
            ack->publish(reply);
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    // Let rclcpp release an executor-held subscription after the owner resets it.
    auto wake = bridge->create_wall_timer(20ms, [] {});
    executor.add_node(bridge);
    executor.add_node(source);
    std::jthread spin([&] { executor.spin(); });
    auto finish = [&] {
        executor.cancel();
        if (spin.joinable())
            spin.join();
        rclcpp::shutdown();
    };
    try {
        char *error = nullptr;
        auto raw = horus_native_create_with_options(R"({"auto_launch":false})", &error);
        if (!raw) {
            auto message = std::string(error ? error : "create failed");
            horus_native_free(error);
            throw std::runtime_error(message);
        }
        std::unique_ptr<void, decltype(&horus_native_destroy)> handle(raw, horus_native_destroy);
        const auto payload =
            R"([{"robot_name":"dashboard_test","sensors":[{"type":"camera","topic":"/native/dashboard/camera","camera_config":{"minimap_streaming_type":"ros","teleop_streaming_type":"webrtc"}}]}])";
        assert(take(horus_native_register(raw, payload, 5, 0, 0))["success"] == true);
        auto status = [&] { return take(horus_native_status(raw)); };
        auto row = [&](const Json &s) {
            for (const auto &item : s["topics"])
                if (item["topic"] == "/native/dashboard/camera")
                    return item;
            return Json();
        };
        auto until = [&](const char *label, auto condition) {
            const auto deadline = std::chrono::steady_clock::now() + 5s;
            while (!condition()) {
                if (std::chrono::steady_clock::now() > deadline)
                    throw std::runtime_error(std::string(label) + ": " + status().dump());
                std::this_thread::sleep_for(50ms);
            }
        };
        until("source without bridge", [&] { return row(status())["link"] == "NO_BRIDGE"; });
        assert(row(status())["data"] == "Not measured");
        // Monitoring must not add a payload subscriber or advertise data delivery.
        assert(camera->get_subscription_count() == 0);
        auto cam_sub = bridge->create_subscription<String>("/native/dashboard/camera", 1,
                                                           [](String::ConstSharedPtr) {});
        until("bridge subscription", [&] { return row(status())["link"] == "OK"; });
        until("monitor discovery", [&] {
            return hb->get_subscription_count() && presence->get_subscription_count() &&
                   runtime->get_subscription_count();
        });
        String m;
        m.data = "Heartbeat: nan";
        hb->publish(m);
        std::this_thread::sleep_for(100ms);
        assert(status()["app_connected"] == false);
        m.data = "Heartbeat: 1.5 | IP: 192.0.2.1";
        hb->publish(m);
        m.data =
            R"({"app_id":"one","role":"private","state":"private_workspace_ready","workspace_active":true})";
        presence->publish(m);
        m.data =
            R"({"camera_topic":"/native/dashboard/camera","active":true,"transport":"webrtc"})";
        runtime->publish(m);
        until("active report", [&] {
            auto s = status();
            return s["app_connected"] == true && s["operators"].size() == 1 &&
                   row(s)["transport"] == "webrtc";
        });
        auto active = status();
        assert(active["operators"][0]["workspace_ready"] == true);
        assert(row(active)["link"] == "MEDIA" && row(active)["transport_evidence"] == "app_state");
        until("heartbeat expiry", [&] { return status()["app_connected"] == false; });
        until("transport expiry", [&] { return row(status())["transport"] == "ros"; });
        assert(status()["registration_state"] == "Accepted");
        until("operator expiry", [&] { return status()["operators"].empty(); });
        cam_sub.reset();
        until("bridge removed", [&] { return row(status())["link"] == "NO_BRIDGE"; });
        camera.reset();
        until("source removed", [&] { return row(status())["link"] == "NO_SOURCE"; });
    } catch (...) {
        finish();
        throw;
    }
    finish();
}
