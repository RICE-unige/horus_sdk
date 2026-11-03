#include "registry.h"
#include "bridge_process.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <thread>

using Json = nlohmann::json;
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
Json take(char* value) {
    std::unique_ptr<char, decltype(&horus_native_free)> guard(value, horus_native_free);
    check(value != nullptr, "No native result");
    return Json::parse(value);
}
struct Handle {
    void* value;
    explicit Handle(const Json& options) {
        char* error = nullptr;
        value = horus_native_create_with_options(options.dump().c_str(), &error);
        std::unique_ptr<char, decltype(&horus_native_free)> guard(error, horus_native_free);
        if (!value) throw std::runtime_error(error ? error : "Creation failed");
    }
    ~Handle() { horus_native_destroy(value); }
    Json run(double timeout = 5) {
        return take(horus_native_register(value, R"([{"robot_name":"owned_bridge","action":"register"}])", timeout, 0, 0));
    }
};

int main(int argc, char** argv) {
    // A test peer, not a bridge or a robot controller. It only ACKs registration.
    if (argc > 1 && std::string(argv[1]) == "--ack-peer") {
        rclcpp::init(0, nullptr);
        auto node = std::make_shared<rclcpp::Node>("horus_native_spawn_test_peer");
        using String = std_msgs::msg::String;
        auto publisher = node->create_publisher<String>("/horus/registration_ack", rclcpp::QoS(256).reliable());
        auto sub = node->create_subscription<String>("/horus/registration", rclcpp::QoS(256).reliable().transient_local(),
            [&](String::ConstSharedPtr message) {
                const auto payload = Json::parse(message->data);
                String ack;
                ack.data = Json{{"success",true},{"robot_name",payload["robot_name"]},{"robot_id","spawn-test"}}.dump();
                publisher->publish(ack);
            });
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }
    if (argc > 1 && std::string(argv[1]) == "--log-failure") {
        std::cerr << "deliberate startup failure\n";
        return 17;
    }
    try {
        const auto executable = std::filesystem::canonical("/proc/self/exe").string();
        BridgeProcess unrelated;
        unrelated.start("/bin/sleep", {"30"});
        {
            Handle handle({{"bridge_executable",executable},{"bridge_arguments",{"--log-failure"}}});
            auto result = handle.run();
            check(result["success"] == false, "Exited child accepted as connected");
            check(result["error"].get<std::string>().find("deliberate startup failure") != std::string::npos,
                  "Startup logs not returned");
        }
        {
            Handle handle({{"bridge_executable",executable},{"bridge_arguments",{"--ack-peer"}}});
            check(handle.run()["success"] == true, "Spawned peer did not complete registration");
            auto status = take(horus_native_status(handle.value));
            check(status["bridge"]["owned"] == true && status["bridge"]["running"] == true,
                  "Owned bridge not tracked");
        }
        unrelated.poll();
        check(unrelated.running(), "Registry shutdown killed unrelated process");
        unrelated.stop();
        check(!unrelated.running(), "Owned process not reaped");
        std::cout << "Native startup, logs and process ownership checks passed\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
