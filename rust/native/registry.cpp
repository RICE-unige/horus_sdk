#include "registry.h"
#include "bridge_process.hpp"
#include "model_delivery.hpp"
#include "dashboard.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <thread>

namespace {
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using String = std_msgs::msg::String;
using Publisher = rclcpp::Publisher<String>::SharedPtr;

Json failure(const std::string& error) { return {{"success", false}, {"error", error}}; }
char* copy(const std::string& text) {
    auto* result = new char[text.size() + 1];
    std::memcpy(result, text.c_str(), text.size() + 1);
    return result;
}
bool queued(const Json& ack) {
    const auto it = ack.find("robot_id");
    return it != ack.end() && it->is_string() && it->get_ref<const std::string&>().rfind("Queued", 0) == 0;
}
void publish(const Publisher& pub, const Json& payload) {
    String msg;
    msg.data = payload.dump();
    pub->publish(msg);
}

class Registry {
public:
    explicit Registry(const Json& options) {
        if (!options.is_object()) throw std::invalid_argument("Runtime options must be an object");
        auto_launch_ = options.value("auto_launch", true);
        bridge_executable_ = options.value("bridge_executable", std::string{});
        bridge_arguments_ = options.value("bridge_arguments", std::vector<std::string>{});
        context_ = std::make_shared<rclcpp::Context>();
        context_->init(0, nullptr);
        static std::atomic<unsigned> counter{0};
        node_ = std::make_shared<rclcpp::Node>(
            "horus_native_registry_" + std::to_string(counter.fetch_add(1)) + "_" +
                std::to_string(Clock::now().time_since_epoch().count()),
            rclcpp::NodeOptions().context(context_).use_global_arguments(false));
        monitor_ = std::make_unique<horus::native::StatusMonitor>(node_);
        const auto durable = rclcpp::QoS(256).reliable().transient_local();
        publisher_ = node_->create_publisher<String>("/horus/registration", durable);
        const std::string replay_prefix = "/horus/multi_operator/sdk_registry_replay_";
        replay_begin_ = node_->create_publisher<String>(replay_prefix + "begin", durable);
        replay_item_ = node_->create_publisher<String>(replay_prefix + "item", durable);
        replay_end_ = node_->create_publisher<String>(replay_prefix + "end", durable);
        ack_sub_ = node_->create_subscription<String>("/horus/registration_ack", rclcpp::QoS(256).reliable(),
            [this](String::ConstSharedPtr msg) {
                auto ack = Json::parse(msg->data, nullptr, false);
                if (!ack.is_object() || !ack.contains("success") || !ack["success"].is_boolean() ||
                    !ack.contains("robot_name") || !ack["robot_name"].is_string()) return;
                const auto name = ack["robot_name"].get<std::string>();
                monitor_->receipt("/horus/registration_ack");
                std::lock_guard<std::mutex> guard(mutex_);
                if (pending_.count(name)) { acks_[name] = std::move(ack); changed_.notify_all(); }
            });
        heartbeat_sub_ = node_->create_subscription<String>("/horus/heartbeat", rclcpp::QoS(1).best_effort(),
            [this](String::ConstSharedPtr msg) {
                if (msg->data.rfind("Heartbeat:", 0) != 0) return;
                try {
                    const auto suffix = msg->data.find('|');
                    const auto value = msg->data.substr(10, suffix == std::string::npos ? suffix : suffix - 10);
                    std::size_t used = 0;
                    const double uptime = std::stod(value, &used);
                    if (!std::isfinite(uptime) || uptime < 0 || value.find_first_not_of(" \t", used) != std::string::npos) return;
                    const auto ip = msg->data.find("IP:");
                    const auto app = ip == std::string::npos ? std::string("unknown") : msg->data.substr(ip + 3);
                    std::lock_guard<std::mutex> guard(mutex_);
                    auto previous = apps_.find(app);
                    const auto now = Clock::now();
                    if (previous == apps_.end() || now - previous->second.first > std::chrono::seconds(3) ||
                        uptime + 0.75 < previous->second.second) replay_pending_ = true;
                    apps_[app] = {now, uptime};
                    heartbeat_seen_ = true;
                    monitor_->receipt("/horus/heartbeat");
                    for (auto it = apps_.begin(); it != apps_.end();) {
                        if (now - it->second.first > std::chrono::seconds(10)) it = apps_.erase(it);
                        else ++it;
                    }
                    changed_.notify_all();
                } catch (const std::exception&) { /* Ignore malformed heartbeat text. */ }
            });
        replay_sub_ = node_->create_subscription<String>(
            "/horus/multi_operator/sdk_registration_replay_request", rclcpp::QoS(10).reliable(),
            [this](String::ConstSharedPtr msg) {
                auto request = Json::parse(msg->data, nullptr, false);
                if (!request.is_object()) return;
                std::lock_guard<std::mutex> guard(mutex_);
                replay_request_ = std::move(request);
                monitor_->receipt("/horus/multi_operator/sdk_registration_replay_request");
                replay_pending_ = true;
                changed_.notify_all();
            });
        replay_timer_ = node_->create_wall_timer(std::chrono::milliseconds(200), [this] {
            { std::lock_guard<std::mutex> guard(bridge_mutex_); bridge_.poll(); }
            replay();
        });
        models_ = std::make_unique<horus::native::ModelDelivery>(node_);
        rclcpp::ExecutorOptions executor_options;
        executor_options.context = context_;
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
        executor_->add_node(node_);
        thread_ = std::thread([this] {
            try { executor_->spin(); }
            catch (const std::exception& error) {
                std::lock_guard<std::mutex> guard(mutex_);
                executor_error_ = error.what();
                changed_.notify_all();
            }
        });
    }

    ~Registry() {
        stop();
        // cancel() before spin() is not latched by rclcpp. Closing our private
        // context first also covers destruction before the worker starts.
        try { context_->shutdown("HORUS native registry destroyed"); } catch (...) {}
        try { executor_->cancel(); } catch (...) {}
        if (thread_.joinable()) thread_.join();
    }

    void stop() {
        if (models_) models_->stop();
        {
            std::lock_guard<std::mutex> guard(mutex_);
            stopped_ = true;
            changed_.notify_all();
        }
        std::lock_guard<std::mutex> guard(bridge_mutex_);
        bridge_.stop();
    }

    Json status() {
        auto result = monitor_->snapshot();
        Json bridge_status;
        {
            std::lock_guard<std::mutex> guard(bridge_mutex_);
            bridge_.poll();
            bridge_status = {{"owned", bridge_.started()}, {"running", bridge_.running()},
                             {"error", bridge_.error()}, {"log_tail", bridge_.log_tail()}};
        }
        std::lock_guard<std::mutex> guard(mutex_);
        const auto now = Clock::now();
        bool connected = false;
        for (const auto& app : apps_) connected |= now - app.second.first <= std::chrono::seconds(3);
        result.update({{"app_connected", connected && !stopped_}, {"registration_count", retained_.size()},
                {"heartbeat_seen", heartbeat_seen_}, {"registration_state", registration_state_},
                {"stopped", stopped_}, {"error", executor_error_},
                {"bridge", bridge_status}, {"models", models_->status()},
                {"registration_subscribers", publisher_->get_subscription_count()}});
        return result;
    }

    Json run(Json payloads, double timeout, bool keep_alive, bool show_status) {
        std::unique_lock<std::mutex> operation(operation_, std::try_to_lock);
        if (!operation.owns_lock()) return failure("Registration already in progress");
        if (!std::isfinite(timeout) || timeout <= 0 || timeout > 86400) return failure("Timeout must be finite and in (0, 86400] seconds");
        if (!payloads.is_array() || payloads.empty() || payloads.size() > 256) return failure("Expected 1-256 registration payloads");
        std::set<std::string> names;
        for (const auto& payload : payloads) {
            if (!payload.is_object() || !payload.contains("robot_name") || !payload["robot_name"].is_string() ||
                payload["robot_name"].get_ref<const std::string&>().empty()) return failure("Every registration needs a robot_name");
            if (!names.insert(payload["robot_name"].get<std::string>()).second) return failure("Duplicate robot_name in registration batch");
            const auto action = payload.value("action", std::string("register"));
            if (action != "register" && action != "unregister") return failure("Unsupported registration action");
        }
        for (auto& payload : payloads) models_->prepare(payload);
        monitor_->configure(payloads);
        horus::native::TerminalDashboard dashboard(show_status);
        auto next_dashboard = Clock::time_point::min();
        const auto start = Clock::now();
        const auto deadline = start + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout));
        std::unique_lock<std::mutex> lock(mutex_);
        if (stopped_) return failure("Registry stopped; create a new client to restart");
        std::set<std::string> projected;
        for (const auto& entry : retained_) projected.insert(entry.first);
        for (const auto& payload : payloads) {
            const auto name = payload["robot_name"].get<std::string>();
            if (payload.value("action", std::string("register")) == "unregister") projected.erase(name);
            else projected.insert(name);
        }
        if (projected.size() > 256) return failure("Registry capacity is 256 retained robots per client");
        acks_.clear();
        pending_ = names;
        registration_state_ = "Waiting for acknowledgement";
        for (const auto& payload : payloads) {
            if (payload.value("action", std::string("register")) == "unregister")
                retained_.erase(payload["robot_name"].get<std::string>());
        }
        // Clear attempt-local ACKs on every exit, including exceptions.
        struct Cleanup {
            Registry& self;
            ~Cleanup() { self.pending_.clear(); self.acks_.clear(); }
        } cleanup{*this};
        Json accepted = Json::object();
        auto update_dashboard = [&] {
            if (!show_status) return;
            lock.unlock();
            try { dashboard.update(status()); } catch (...) { lock.lock(); throw; }
            lock.lock();
        };
        auto failed = [&](const std::string& error) {
            registration_state_ = error;
            auto value = failure(error);
            value["registrations"] = accepted;
            value["pending"] = pending_;
            update_dashboard();
            return value;
        };
        auto next_publish = Clock::time_point::min();
        bool completed = false;
        while (true) {
            if (!executor_error_.empty()) return failed("ROS executor failed: " + executor_error_);
            if (stopped_) {
                Json result = completed ? Json{{"success", true}, {"registrations", accepted}} : failed("Registration cancelled");
                result["cancelled"] = true;
                return result;
            }
            for (auto it = acks_.begin(); it != acks_.end();) {
                const auto name = it->first;
                const auto ack = it->second;
                it = acks_.erase(it);
                if (!ack["success"].get<bool>()) {
                    const auto reason = ack.find("error");
                    registration_state_ = reason != ack.end() && reason->is_string()
                        ? "Rejected: " + reason->get<std::string>() : "Rejected by HORUS app";
                    auto rejected = ack;
                    rejected["registrations"] = accepted;
                    rejected["pending"] = pending_;
                    update_dashboard();
                    return rejected;
                }
                if (queued(ack)) {
                    registration_state_ = "Queued (waiting for workspace)";
                    continue;
                }
                accepted[name] = ack;
                pending_.erase(name);
                for (const auto& payload : payloads) {
                    if (payload["robot_name"] != name) continue;
                    if (payload.value("action", std::string("register")) == "unregister") retained_.erase(name);
                    else retained_[name] = payload;
                }
            }
            if (!completed && accepted.size() == payloads.size()) {
                completed = true;
                registration_state_ = "Accepted";
                for (const auto& payload : payloads) {
                    const auto name = payload["robot_name"].get<std::string>();
                    if (payload.value("action", std::string("register")) == "unregister") retained_.erase(name);
                    else retained_[name] = payload;
                }
                if (!keep_alive) {
                    update_dashboard();
                    if (payloads.size() == 1) return accepted.begin().value();
                    return {{"success", true}, {"registrations", accepted}};
                }
            }
            const auto now = Clock::now();
            if (show_status && now >= next_dashboard) {
                update_dashboard();
                next_dashboard = now + std::chrono::seconds(1);
            }
            if (!completed && now >= deadline) {
                registration_state_ = "Timed out (no accepted acknowledgement)";
                auto result = failure("Registration timeout waiting for an accepted app acknowledgement");
                result["registrations"] = accepted;
                result["pending"] = pending_;
                update_dashboard();
                return result;
            }
            if (!completed && auto_launch_ && now - start >= std::chrono::milliseconds(500) &&
                std::any_of(payloads.begin(), payloads.end(), [](const Json& payload) {
                    return payload.value("action", std::string("register")) == "register";
                })) {
                lock.unlock();
                std::string error;
                try { error = ensure_bridge(); } catch (...) { lock.lock(); throw; }
                lock.lock();
                if (!error.empty()) return failed(error);
            }
            if (!completed && now >= next_publish) {
                // DDS publishes outside the state lock so callbacks can continue accepting ACKs.
                for (const auto& payload : payloads) {
                    if (!pending_.count(payload["robot_name"].get<std::string>())) continue;
                    lock.unlock();
                    try {
                        std::lock_guard<std::mutex> wire(wire_);
                        publish(publisher_, payload);
                    }
                    catch (...) { lock.lock(); throw; }
                    lock.lock();
                }
                next_publish = now + std::chrono::seconds(1);
            }
            changed_.wait_until(lock, completed ? now + std::chrono::seconds(1) :
                std::min({deadline, next_publish, now + std::chrono::milliseconds(200)}));
        }
    }

private:
    std::string ensure_bridge() {
        std::lock_guard<std::mutex> guard(bridge_mutex_);
        { std::lock_guard<std::mutex> state(mutex_); if (stopped_) return {}; }
        bridge_.poll();
        if (!bridge_.error().empty()) return bridge_.error() + "\n" + bridge_.log_tail();
        if (bridge_.running() || publisher_->get_subscription_count() > 0) return {};
        // A bridge without a connected Quest has no registration subscriber yet.
        for (const auto& name : node_->get_node_names())
            if (name == "/horus_unity_bridge") return {};
        const auto executable = bridge_executable_.empty() ? BridgeProcess::locate() : bridge_executable_;
        if (executable.empty()) return "HORUS bridge executable was not found. Source its install/local_setup.bash, provide bridge_executable, or start a bridge separately with auto_launch=false.";
        bridge_.start(executable, bridge_arguments_);
        return {};
    }

    void replay() {
        std::lock_guard<std::mutex> wire(wire_);
        Json payloads = Json::array(), request;
        {
            std::lock_guard<std::mutex> guard(mutex_);
            if (stopped_ || !replay_pending_ || retained_.empty()) return;
            replay_pending_ = false;
            request = replay_request_;
            replay_request_ = Json::object();
            for (const auto& entry : retained_) {
                if (!pending_.count(entry.first)) payloads.push_back(entry.second);
            }
        }
        auto text = [](const Json& value, const char* key, const std::string& fallback) {
            auto it = value.find(key);
            return it != value.end() && it->is_string() ? it->get<std::string>() : fallback;
        };
        const auto id = text(request, "request_id", std::to_string(Clock::now().time_since_epoch().count()));
        const auto join = text(request, "join_attempt_id", "");
        Json envelope = {{"request_id", id}, {"join_attempt_id", join}, {"expected_count", payloads.size()},
                         {"ts_unix_ms", std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::system_clock::now().time_since_epoch()).count()}};
        publish(replay_begin_, envelope);
        for (std::size_t i = 0; i < payloads.size(); ++i) {
            publish(publisher_, payloads[i]);
            publish(replay_item_, {{"request_id", id}, {"join_attempt_id", join}, {"index", i},
                                   {"registration_json", payloads[i].dump()}});
        }
        envelope["published_count"] = payloads.size();
        publish(replay_end_, envelope);
    }

    std::shared_ptr<rclcpp::Context> context_;
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<horus::native::ModelDelivery> models_;
    std::unique_ptr<horus::native::StatusMonitor> monitor_;
    Publisher publisher_, replay_begin_, replay_item_, replay_end_;
    rclcpp::Subscription<String>::SharedPtr ack_sub_, heartbeat_sub_, replay_sub_;
    rclcpp::TimerBase::SharedPtr replay_timer_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread thread_;
    std::mutex mutex_, operation_, wire_, bridge_mutex_;
    BridgeProcess bridge_;
    bool auto_launch_{true};
    std::string bridge_executable_;
    std::vector<std::string> bridge_arguments_;
    std::condition_variable changed_;
    bool stopped_{false}, replay_pending_{false};
    bool heartbeat_seen_{false};
    std::string registration_state_{"Idle"};
    std::string executor_error_;
    std::set<std::string> pending_;
    std::map<std::string, Json> acks_, retained_;
    std::map<std::string, std::pair<Clock::time_point, double>> apps_;
    Json replay_request_ = Json::object();
};

template <class F> char* result(F&& fn) noexcept {
    try { return copy(fn().dump()); }
    catch (const std::exception& error) {
        try { return copy(failure(error.what()).dump()); } catch (...) { return nullptr; }
    } catch (...) { return nullptr; }
}
Registry& registry(void* handle) {
    if (!handle) throw std::invalid_argument("Native registry handle is null");
    return *static_cast<Registry*>(handle);
}
} // namespace

extern "C" {
void* horus_native_create(char** error) {
    return horus_native_create_with_options("{}", error);
}
void* horus_native_create_with_options(const char* options, char** error) {
    if (error) *error = nullptr;
    try {
        if (!options) throw std::invalid_argument("Runtime options are null");
        return new Registry(Json::parse(options));
    }
    catch (const std::exception& exc) {
        if (error) { try { *error = copy(exc.what()); } catch (...) {} }
        return nullptr;
    } catch (...) { return nullptr; }
}
char* horus_native_register(void* handle, const char* payloads, double timeout, int keep_alive, int show_status) {
    return result([&] {
        if (!payloads) return failure("Registration JSON is null");
        return registry(handle).run(Json::parse(payloads), timeout, keep_alive != 0, show_status != 0);
    });
}
char* horus_native_unregister(void* handle, const char* name, double timeout) {
    return result([&] {
        if (!name || !*name) return failure("Robot name is required");
        return registry(handle).run(Json::array({{{"action", "unregister"}, {"robot_name", name}}}), timeout, false, false);
    });
}
char* horus_native_status(void* handle) { return result([&] { return registry(handle).status(); }); }
void horus_native_stop(void* handle) {
    try { if (handle) static_cast<Registry*>(handle)->stop(); } catch (...) {}
}
void horus_native_destroy(void* handle) { delete static_cast<Registry*>(handle); }
void horus_native_free(char* text) { delete[] text; }
}
