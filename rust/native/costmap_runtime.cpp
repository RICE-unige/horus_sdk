#include "costmap_runtime.h"
#include "horus/mapping/costmap.hpp"
#include <rclcpp/rclcpp.hpp>
#ifdef HORUS_NATIVE_COSTMAP
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_msgs/msg/costmap_update.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#endif
#include <atomic>
#include <cmath>
#include <cstring>
#include <deque>
#include <future>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <thread>
#include <unistd.h>

namespace {
using Json = nlohmann::json;
#ifdef HORUS_NATIVE_COSTMAP
using State = horus::mapping::CostmapState;
using Geometry = horus::mapping::GridGeometry;
using Grid = nav_msgs::msg::OccupancyGrid;
using RawGrid = nav2_msgs::msg::Costmap;
using Patch = map_msgs::msg::OccupancyGridUpdate;
using RawPatch = nav2_msgs::msg::CostmapUpdate;
using Snapshot = nav2_msgs::srv::GetCostmap;
double now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
std::uint64_t stamp(const std_msgs::msg::Header &h) {
    if (h.stamp.sec < 0 || h.stamp.nanosec >= 1000000000)
        throw std::invalid_argument("Invalid costmap timestamp");
    return std::uint64_t(h.stamp.sec) * 1000000000 + h.stamp.nanosec;
}
Geometry geometry(const std::string &frame, std::uint32_t w, std::uint32_t h, double r,
                  const geometry_msgs::msg::Pose &origin) {
    auto &p = origin.position;
    auto &q = origin.orientation;
    return {frame, w, h, r, {p.x, p.y, p.z, q.x, q.y, q.z, q.w}};
}
bool apply_message(State &s, const Grid &m) {
    return s.full(
        geometry(m.header.frame_id, m.info.width, m.info.height, m.info.resolution, m.info.origin),
        m.data, stamp(m.header));
}
bool apply_message(State &s, const RawGrid &m) {
    return s.full_raw(geometry(m.header.frame_id, m.metadata.size_x, m.metadata.size_y,
                               m.metadata.resolution, m.metadata.origin),
                      m.data, stamp(m.header));
}
bool apply_message(State &s, const Patch &m) {
    return s.patch(m.header.frame_id, m.x, m.y, m.width, m.height, m.data, stamp(m.header));
}
bool apply_message(State &s, const RawPatch &m) {
    return s.patch_raw(m.header.frame_id, m.x, m.y, m.size_x, m.size_y, m.data, stamp(m.header));
}

class Runtime {
    rclcpp::Context::SharedPtr context_;
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    rclcpp::Publisher<Grid>::SharedPtr publisher_;
    rclcpp::Client<Snapshot>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::optional<rclcpp::Client<Snapshot>::FutureAndRequestId> pending_;
    std::thread worker_;
    std::atomic<bool> stopped_{false};
    mutable std::mutex mutex_;
    State state_;
    bool ready_ = false, dirty_ = false, journal_lost_ = false;
    double publish_interval_, resync_, requested_ = 0, next_resync_ = 0, last_publish_ = -1e30;
    std::uint64_t received_ = 0, forwarded_ = 0, recoveries_ = 0, invalidations_ = 0;
    std::string error_;
    struct Entry {
        std::uint64_t stamp;
        std::function<bool(State &)> apply;
    };
    std::deque<Entry> journal_;
    std::size_t journal_bytes_ = 0;
    void invalidate(const std::string &reason) {
        ready_ = false;
        dirty_ = false;
        next_resync_ = 0;
        error_ = reason;
        ++invalidations_;
        if (pending_)
            journal_lost_ = true;
    }
    template <class Message> void receive(std::shared_ptr<const Message> message, bool full) {
        std::lock_guard guard(mutex_);
        ++received_;
        try {
            auto time = stamp(message->header);
            if (pending_ && !journal_lost_) {
                auto bytes = message->data.size();
                if (journal_.size() >= 64 || bytes > 8 * 1024 * 1024 - journal_bytes_) {
                    journal_.clear();
                    journal_bytes_ = 0;
                    invalidate("Costmap recovery journal overflow");
                } else {
                    journal_bytes_ += bytes;
                    journal_.push_back(
                        {time, [message](State &s) { return apply_message(s, *message); }});
                }
            }
            if (!ready_ && (client_ || !full))
                return;
            dirty_ = apply_message(state_, *message) || dirty_;
            if (full) {
                ready_ = true;
                error_.clear();
            }
        } catch (const std::exception &e) {
            invalidate(e.what());
        }
    }
    template <class Message> void subscribe(const std::string &topic, rclcpp::QoS qos, bool full) {
        rclcpp::SubscriptionOptions options;
        options.event_callbacks.message_lost_callback = [this](rclcpp::QOSMessageLostInfo &) {
            std::lock_guard guard(mutex_);
            invalidate("DDS reported lost costmap data");
        };
        subscriptions_.push_back(node_->create_subscription<Message>(
            topic, qos,
            [this, full](typename Message::ConstSharedPtr message) {
                receive<Message>(std::move(message), full);
            },
            options));
    }
    void tick() {
        std::lock_guard guard(mutex_);
        auto time = now();
        if (pending_) {
            if (pending_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                try {
                    if (journal_lost_)
                        throw std::runtime_error("Costmap recovery lost an update");
                    State fresh;
                    apply_message(fresh, pending_->get()->map);
                    for (const auto &entry : journal_)
                        if (entry.stamp >= fresh.stamp_ns())
                            entry.apply(fresh);
                    state_ = std::move(fresh);
                    ready_ = dirty_ = true;
                    error_.clear();
                    ++recoveries_;
                    next_resync_ = time + resync_;
                } catch (const std::exception &e) {
                    invalidate(e.what());
                    next_resync_ = time + 0.5;
                }
                client_->remove_pending_request(pending_->request_id);
                pending_.reset();
                journal_.clear();
                journal_bytes_ = 0;
            } else if (time - requested_ >= 2) {
                client_->remove_pending_request(pending_->request_id);
                pending_.reset();
                journal_.clear();
                journal_bytes_ = 0;
                invalidate("Costmap snapshot timed out");
                next_resync_ = time + 0.5;
            }
        }
        if (client_ && !pending_ && time >= next_resync_) {
            if (client_->service_is_ready()) {
                journal_.clear();
                journal_bytes_ = 0;
                journal_lost_ = false;
                pending_.emplace(
                    client_->async_send_request(std::make_shared<Snapshot::Request>()));
                requested_ = time;
            } else if (!ready_) {
                error_ = "Waiting for costmap snapshot service";
                next_resync_ = time + 0.5;
            }
        }
        if (!ready_ || !dirty_ || time - last_publish_ < publish_interval_)
            return;
        const auto &g = *state_.geometry();
        Grid msg;
        msg.header.frame_id = g.frame;
        msg.header.stamp.sec = state_.stamp_ns() / 1000000000;
        msg.header.stamp.nanosec = state_.stamp_ns() % 1000000000;
        msg.info.width = g.width;
        msg.info.height = g.height;
        msg.info.resolution = g.resolution;
        auto &p = msg.info.origin.position;
        auto &q = msg.info.origin.orientation;
        p.x = g.origin[0];
        p.y = g.origin[1];
        p.z = g.origin[2];
        q.x = g.origin[3];
        q.y = g.origin[4];
        q.z = g.origin[5];
        q.w = g.origin[6];
        auto cells = state_.data();
        msg.data.assign(cells.begin(), cells.end());
        publisher_->publish(std::move(msg));
        ++forwarded_;
        dirty_ = false;
        last_publish_ = time;
    }

  public:
    explicit Runtime(const Json &options) {
        const std::set<std::string> allowed{"input_topic",      "input_format", "updates_topic",
                                            "snapshot_service", "output_topic", "publish_hz",
                                            "resync_seconds",   "volatile_full"};
        if (!options.is_object())
            throw std::invalid_argument("Costmap options must be an object");
        for (auto it = options.begin(); it != options.end(); ++it)
            if (!allowed.count(it.key()))
                throw std::invalid_argument("Unknown costmap option: " + it.key());
        auto input = options.at("input_topic").get<std::string>(),
             output = options.value("output_topic", std::string("/horus/costmap"));
        auto updates = options.value("updates_topic", std::string()),
             service = options.value("snapshot_service", std::string());
        auto format = options.value("input_format", std::string("occupancy"));
        if (format != "occupancy" && format != "raw")
            throw std::invalid_argument("Costmap format must be occupancy or raw");
        for (const auto &topic : {input, output, updates, service})
            if ((!topic.empty() && (topic.front() != '/' || topic.back() == '/')) ||
                topic.find_first_of(" \t\n") != std::string::npos)
                throw std::invalid_argument("Costmap topics/services must be absolute");
        if (input.empty() || output.empty() || input == output ||
            (!updates.empty() && (updates == input || updates == output || service.empty())))
            throw std::invalid_argument(
                "Use distinct costmap topics and a snapshot service for incremental input");
        double hz = options.value("publish_hz", 5.0);
        resync_ = options.value("resync_seconds", 2.0);
        if (!std::isfinite(hz) || hz <= 0 || hz > 30 || !std::isfinite(resync_) || resync_ < 0.2)
            throw std::invalid_argument("Invalid costmap publish/resync rate");
        publish_interval_ = 1 / hz;
        try {
            context_ = std::make_shared<rclcpp::Context>();
            context_->init(0, nullptr);
            static std::atomic<unsigned> next_id{0};
            node_ = std::make_shared<rclcpp::Node>(
                "horus_costmap_" + std::to_string(::getpid()) + "_" + std::to_string(next_id.fetch_add(1)),
                rclcpp::NodeOptions().context(context_).use_global_arguments(false));
            publisher_ =
                node_->create_publisher<Grid>(output, rclcpp::QoS(1).reliable().transient_local());
            if (!service.empty())
                client_ = node_->create_client<Snapshot>(service);
            ready_ = !client_;
            auto qos = rclcpp::QoS(1).reliable();
            if (!options.value("volatile_full", false))
                qos.transient_local();
            if (format == "raw")
                subscribe<RawGrid>(input, qos, true);
            else
                subscribe<Grid>(input, qos, true);
            if (!updates.empty()) {
                if (format == "raw")
                    subscribe<RawPatch>(updates, rclcpp::QoS(64).reliable(), false);
                else
                    subscribe<Patch>(updates, rclcpp::QoS(64).reliable(), false);
            }
            timer_ = node_->create_wall_timer(std::chrono::milliseconds(20), [this] { tick(); });
            rclcpp::ExecutorOptions exec;
            exec.context = context_;
            executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(exec);
            executor_->add_node(node_);
            worker_ = std::thread([this] {
                try {
                    executor_->spin();
                } catch (const std::exception &e) {
                    std::lock_guard guard(mutex_);
                    error_ = e.what();
                }
                stopped_ = true;
            });
        } catch (...) {
            shutdown();
            throw;
        }
    }
    ~Runtime() { shutdown(); }
    void shutdown() noexcept {
        stopped_ = true;
        if (context_)
            try {
                context_->shutdown("Costmap adapter closed");
            } catch (...) {
            }
        if (executor_)
            executor_->cancel();
        if (worker_.joinable())
            worker_.join();
        if (pending_ && client_) {
            try {
                client_->remove_pending_request(pending_->request_id);
            } catch (...) {
            }
            pending_.reset();
        }
        timer_.reset();
        subscriptions_.clear();
        executor_.reset();
        publisher_.reset();
        client_.reset();
        node_.reset();
        context_.reset();
    }
    Json status() const {
        std::lock_guard guard(mutex_);
        return {{"running", !stopped_.load()},     {"ready", ready_},
                {"received", received_},           {"forwarded", forwarded_},
                {"recoveries", recoveries_},       {"invalidations", invalidations_},
                {"revision", state_.revision()},   {"snapshot_pending", bool(pending_)},
                {"journal_bytes", journal_bytes_}, {"error", error_}};
    }
};
#else
class Runtime {
  public:
    explicit Runtime(const Json &) {
        throw std::runtime_error(
            "Build native ROS with nav2_msgs and map_msgs for costmap adaptation");
    }
    Json status() const { return {{"running", false}}; }
};
#endif
char *copy(const std::string &value) {
    auto *p = static_cast<char *>(std::malloc(value.size() + 1));
    if (!p)
        throw std::bad_alloc();
    std::memcpy(p, value.c_str(), value.size() + 1);
    return p;
}
} // namespace
extern "C" {
void *horus_native_costmap_create(const char *options, char **error) {
    if (error)
        *error = nullptr;
    try {
        if (!options)
            throw std::invalid_argument("Missing options");
        return new Runtime(Json::parse(options));
    } catch (const std::exception &e) {
        if (error)
            try {
                *error = copy(e.what());
            } catch (...) {
            }
        return nullptr;
    } catch (...) {
        return nullptr;
    }
}
char *horus_native_costmap_status(void *adapter) {
    try {
        if (!adapter)
            throw std::invalid_argument("Missing adapter");
        return copy(
            Json{{"ok", true}, {"value", static_cast<Runtime *>(adapter)->status()}}.dump());
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
void horus_native_costmap_destroy(void *adapter) { delete static_cast<Runtime *>(adapter); }
}
