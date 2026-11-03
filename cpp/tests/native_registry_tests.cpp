#include "horus/bridge/robot_registry.hpp"
#include "registry.h"
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

using Json = nlohmann::json;
using String = std_msgs::msg::String;
using namespace std::chrono_literals;
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
Json take(char* data) {
    std::unique_ptr<char, decltype(&horus_native_free)> owned(data, horus_native_free);
    require(data != nullptr, "Missing native result");
    return Json::parse(data);
}
struct Handle {
    void* value;
    Handle() {
        char* error = nullptr;
        value = horus_native_create_with_options(R"({"auto_launch":false})", &error);
        std::unique_ptr<char, decltype(&horus_native_free)> owned(error, horus_native_free);
        if (!value) throw std::runtime_error(error ? error : "Create failed");
    }
    ~Handle() { horus_native_destroy(value); }
    Json run(const std::string& name, double timeout = 3, bool keep = false) {
        return take(horus_native_register(value, Json::array({{{"action","register"},{"robot_name",name}}}).dump().c_str(), timeout, keep, 0));
    }
};

class Peer {
public:
    Peer() {
        node = std::make_shared<rclcpp::Node>("horus_native_test_peer");
        ack = node->create_publisher<String>("/horus/registration_ack", rclcpp::QoS(256).reliable());
        heartbeat = node->create_publisher<String>("/horus/heartbeat", rclcpp::QoS(10).best_effort());
        request = node->create_publisher<String>("/horus/multi_operator/sdk_registration_replay_request", rclcpp::QoS(10).reliable());
        sub = node->create_subscription<String>("/horus/registration", rclcpp::QoS(256).reliable().transient_local(),
            [this](String::ConstSharedPtr msg) {
                auto payload = Json::parse(msg->data);
                const auto name = payload.at("robot_name").get<std::string>();
                unsigned count;
                { std::lock_guard<std::mutex> guard(mutex); count = ++counts[name]; last = payload; }
                if (name == "silent" || name == "cancel" || name == "concurrent") return;
                String out;
                out.data = Json{{"success",true},{"robot_name","not_this_robot"},{"robot_id","wrong"}}.dump();
                ack->publish(out);
                out.data = Json{{"success",true},{"robot_name",name},{"robot_id",name == "queued" && count == 1 ? "Queued (Waiting for Workspace)" : "id-"+name},
                    {"assigned_color","#12AB34"}}.dump();
                if (name == "reject") out.data = Json{{"success",false},{"robot_name",name},{"error","deliberate rejection"}}.dump();
                ack->publish(out);
            });
        replay = node->create_subscription<String>("/horus/multi_operator/sdk_registry_replay_item", rclcpp::QoS(256).reliable(),
            [this](String::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> guard(mutex);
                replay_items.push_back(Json::parse(msg->data));
            });
        exec.add_node(node);
        thread = std::thread([this] { exec.spin(); });
    }
    ~Peer() { exec.cancel(); thread.join(); }
    unsigned count(const std::string& name) { std::lock_guard<std::mutex> guard(mutex); return counts[name]; }
    void request_replay() {
        String msg;
        msg.data = R"({"request_id":"native-test","join_attempt_id":"join-test"})";
        request->publish(msg);
    }
    std::mutex mutex;
    Json last;
    std::vector<Json> replay_items;
    rclcpp::Publisher<String>::SharedPtr ack, heartbeat, request;
private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<String>::SharedPtr sub, replay;
    rclcpp::executors::SingleThreadedExecutor exec;
    std::thread thread;
    std::map<std::string,unsigned> counts;
};

template<class Predicate> void eventually(Predicate&& predicate) {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!predicate()) {
        if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("Condition did not arrive");
        std::this_thread::sleep_for(20ms);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        Peer peer;
        {
            Handle h;
            require(h.run("accepted")["success"] == true, "Accepted registration failed");
            require(h.run("queued")["success"] == true && peer.count("queued") >= 2, "Queued ACK counted as success");
            require(h.run("reject")["success"] == false, "Rejected registration counted as success");
            require(take(horus_native_status(h.value))["registration_state"] == "Rejected: deliberate rejection",
                "Dashboard retained an obsolete registration state after rejection");
            auto started = std::chrono::steady_clock::now();
            require(h.run("silent", .15)["success"] == false, "Silent peer counted as success");
            require(take(horus_native_status(h.value))["registration_state"] == "Timed out (no accepted acknowledgement)",
                "Dashboard did not retain the final timeout state");
            require(std::chrono::steady_clock::now() - started < 700ms, "Timeout not enforced");
            require(h.run("silent", -1)["success"] == false, "Negative timeout accepted");
            require(take(horus_native_register(h.value,"not-json",1,0,0))["success"] == false, "Invalid JSON accepted");
            require(take(horus_native_register(h.value,"[]",1,0,0))["success"] == false, "Empty batch accepted");
            require(take(horus_native_register(h.value,R"([{"robot_name":"model","robot_description_manifest":{}}])",1,0,0))["success"] == false,
                "Incomplete native robot body silently registered");
            auto batch = Json::array({{{"robot_name","a"}},{{"robot_name","b"}}}).dump();
            auto result = take(horus_native_register(h.value,batch.c_str(),3,0,0));
            require(result["success"] == true && result["registrations"].size()==2, "Fleet ACKs not retained");
            peer.request_replay();
            eventually([&] { std::lock_guard<std::mutex> guard(peer.mutex); return !peer.replay_items.empty(); });
            require(take(horus_native_unregister(h.value,"a",3))["success"] == true, "Unregister failed");
            { std::lock_guard<std::mutex> guard(peer.mutex); require(peer.last["action"]=="unregister", "Wrong unregister action"); }
            auto pending = std::async(std::launch::async,[&]{return h.run("concurrent",3);});
            eventually([&]{return peer.count("concurrent")>0;});
            require(h.run("another")["error"] == "Registration already in progress", "Concurrent operation not rejected");
            horus_native_stop(h.value);
            require(pending.get()["cancelled"]==true, "Cancellation not delivered");
        }
        require(rclcpp::ok(), "Native destruction shut down the caller's ROS context");
        {
            Handle h;
            const auto batch = Json::array({{{"robot_name","partial"}},{{"robot_name","cancel"}}}).dump();
            auto pending = std::async(std::launch::async,[&]{return take(horus_native_register(h.value,batch.c_str(),3,0,0));});
            eventually([&]{return take(horus_native_status(h.value))["registration_count"] == 1;});
            horus_native_stop(h.value);
            const auto result = pending.get();
            require(result["success"] == false && result["cancelled"] == true &&
                    result["registrations"].contains("partial"), "Cancelled fleet lost its accepted registrations");
        }
        {
            Handle h;
            auto pending = std::async(std::launch::async,[&]{return h.run("keepalive",3,true);});
            eventually([&]{return take(horus_native_status(h.value))["registration_count"] == 1;});
            horus_native_stop(h.value);
            auto result = pending.get();
            require(result["success"] == true && result["cancelled"] == true, "Stopping a successful monitor reported failed registration");
        }
        {
            horus::bridge::RobotRegistryClient client({.auto_launch = false});
            horus::robot::Robot robot("cpp_api", horus::core::RobotType::WHEELED);
            const auto viz = robot.create_dataviz();
            require(client.register_robot(robot,*viz,3,false,false).first,"C++ public API failed");
            require(robot.is_registered_with_horus() && robot.get_horus_id()=="id-cpp_api", "Robot metadata not applied");
            require(client.unregister_robot(robot.get_name(),3).first,"C++ public unregister failed");
        }
        {
            horus::bridge::RobotRegistryClient client({.auto_launch = false});
            client.stop();
            horus::robot::Robot robot("stopped", horus::core::RobotType::WHEELED);
            require(!client.register_robot(robot,*robot.create_dataviz(),.1,false,false).first,
                    "Stop before initialization was forgotten");
            require(Json::parse(client.status_json())["stopped"] == true, "Stopped status is wrong");
        }
        std::cout << "Native DDS registration lifecycle checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
}
