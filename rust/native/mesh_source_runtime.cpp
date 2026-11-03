#include "mesh_source_runtime.h"
#include "horus/mapping/mesh_page_scheduler.hpp"
#include "horus/mapping/mesh_source.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#ifdef HORUS_NATIVE_NVBLOX
#include <nvblox_msgs/msg/mesh.hpp>
#endif
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <deque>
#include <mutex>
#include <nlohmann/json.hpp>
#include <thread>
#include <unistd.h>

namespace {
using Json = nlohmann::json;
using namespace horus::mapping;
using Clock = std::chrono::steady_clock;
using Marker = visualization_msgs::msg::Marker;
using Markers = visualization_msgs::msg::MarkerArray;
using String = std_msgs::msg::String;
using Bytes = std_msgs::msg::UInt8MultiArray;
char *copy(const std::string &value) {
    auto *p = new char[value.size() + 1];
    std::memcpy(p, value.c_str(), value.size() + 1);
    return p;
}
MeshSourceOptions options(const Json &j) {
    MeshSourceOptions o;
    o.map.frame_id = j.value("frame_id", std::string("map"));
    o.map.max_pages = j.value("max_pages", 1024u);
    o.map.max_payload_bytes = j.value("max_payload_bytes", std::size_t(2 * 1024 * 1024));
    o.map.max_cache_bytes = j.value("max_cache_bytes", std::size_t(256 * 1024 * 1024));
    o.triangles_per_page = j.value("triangles_per_page", std::size_t(4096));
    o.max_input_points = j.value("max_input_points", std::size_t(3000000));
    o.blocks_per_axis = j.value("blocks_per_axis", 2u);
    return o;
}
template <class V> std::array<double, 3> vec(const V &v) { return {v.x, v.y, v.z}; }
template <class Q> std::array<double, 4> quat(const Q &v) { return {v.x, v.y, v.z, v.w}; }
std::array<std::uint8_t, 4> rgba(const std_msgs::msg::ColorRGBA &c) {
    std::array<double, 4> input{c.r, c.g, c.b, c.a};
    std::array<std::uint8_t, 4> out;
    for (std::size_t i = 0; i < 4; ++i) {
        if (!std::isfinite(input[i]) || input[i] < 0 || input[i] > 1)
            throw std::invalid_argument("Invalid mesh color");
        out[i] = static_cast<std::uint8_t>(std::nearbyint(input[i] * 255));
    }
    return out;
}
class SourceAdapter {
    struct Pending {
        std::function<bool()> apply;
        std::size_t bytes;
        Clock::time_point time;
    };
    MeshSourceOptions options_;
    MeshSourceState map_;
    MeshPageScheduler scheduler_;
    std::shared_ptr<rclcpp::Context> context_;
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::unique_ptr<tf2_ros::TransformListener> listener_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::SubscriptionBase::SharedPtr source_;
    rclcpp::Subscription<String>::SharedPtr request_;
    rclcpp::Publisher<Bytes>::SharedPtr output_;
    rclcpp::Publisher<String>::SharedPtr manifest_, status_pub_;
    rclcpp::TimerBase::SharedPtr pump_, discovery_, inventory_;
    std::thread worker_;
    mutable std::mutex mutex_;
    std::string input_topic_, output_topic_, type_, color_mode_, qos_key_, error_;
    std::array<std::uint8_t, 4> fallback_{179, 184, 199, 255};
    std::deque<Pending> pending_;
    std::size_t pending_bytes_{0};
    static constexpr std::size_t max_pending_bytes = 128 * 1024 * 1024;
    Clock::time_point start_ = Clock::now();
    std::map<std::uint32_t, std::tuple<std::uint64_t, std::uint64_t, double>> last_sent_;
    std::uint64_t received_{0}, rejected_{0}, sent_{0}, sent_bytes_{0}, repairs_{0};
    std::size_t source_publishers_{0};
    Clock::time_point last_received_{};
    double seconds() const { return std::chrono::duration<double>(Clock::now() - start_).count(); }
    MeshPose lookup(const std::string &frame, std::int64_t ns) {
        auto t = tf_->lookupTransform(options_.map.frame_id, frame, rclcpp::Time(ns, RCL_ROS_TIME));
        return {vec(t.transform.translation), quat(t.transform.rotation)};
    }
    MeshTransformLookup resolver() {
        return [this](const auto &f, auto t) { return lookup(f, t); };
    }
    void enqueue(const std::vector<MeshPage> &pages) {
        for (const auto &p : pages)
            scheduler_.enqueue(p);
    }
    bool apply(const Markers::ConstSharedPtr &batch, double received_at) {
        try {
            // Resolve every needed transform first; retrying late TF must not decode the map
            // repeatedly.
            std::map<std::pair<std::string, std::int64_t>, MeshPose> transforms;
            for (const auto &m : batch->markers)
                if (m.action == 0 && !m.points.empty() &&
                    m.header.frame_id != options_.map.frame_id) {
                    auto key = std::make_pair(m.header.frame_id,
                                              rclcpp::Time(m.header.stamp).nanoseconds());
                    if (!transforms.count(key))
                        transforms[key] = lookup(key.first, key.second);
                }
            std::vector<TriangleMarker> markers;
            markers.reserve(batch->markers.size());
            for (const auto &m : batch->markers) {
                TriangleMarker item;
                item.name_space = m.ns;
                item.id = m.id;
                item.action = m.action;
                item.type = m.type;
                if (m.action == 0) {
                    item.frame = m.header.frame_id;
                    item.stamp_ns = rclcpp::Time(m.header.stamp).nanoseconds();
                    item.pose = {vec(m.pose.position), quat(m.pose.orientation)};
                    item.scale = vec(m.scale);
                    item.frame_locked = m.frame_locked;
                    item.lifetime_seconds = rclcpp::Duration(m.lifetime).seconds();
                    item.color = rgba(m.color);
                    item.points.reserve(m.points.size());
                    item.colors.reserve(m.colors.size());
                    for (const auto &p : m.points)
                        item.points.push_back(vec(p));
                    for (const auto &c : m.colors)
                        item.colors.push_back(rgba(c));
                }
                markers.push_back(std::move(item));
            }
            auto resolve = [&](const auto &f, auto t) {
                auto it = transforms.find({f, t});
                return it == transforms.end() ? lookup(f, t) : it->second;
            };
            enqueue(map_.apply_markers(markers, received_at, resolve));
            return true;
        } catch (const tf2::TransformException &) {
            return false;
        }
    }
    void receive(Markers::ConstSharedPtr batch) {
        std::lock_guard lock(mutex_);
        ++received_;
        last_received_ = Clock::now();
        std::size_t size = 0, points = 0;
        if (batch->markers.size() > 1024) {
            reject("Marker batch exceeds 1024 markers");
            return;
        }
        for (const auto &m : batch->markers) {
            if (m.points.size() > options_.max_input_points - points) {
                reject("Marker point limit exceeded");
                return;
            }
            points += m.points.size();
            size += m.points.size() * sizeof(geometry_msgs::msg::Point) +
                    m.colors.size() * sizeof(std_msgs::msg::ColorRGBA) + m.ns.size() +
                    m.text.size() + m.mesh_resource.size() + sizeof(Marker);
        }
        const auto received_at = node_->now().seconds();
        defer(
            {[this, batch, received_at] { return apply(batch, received_at); }, size, Clock::now()});
    }
#ifdef HORUS_NATIVE_NVBLOX
    bool apply(const nvblox_msgs::msg::Mesh::ConstSharedPtr &message) {
        try {
            const auto stamp = rclcpp::Time(message->header.stamp).nanoseconds();
            const auto pose = message->header.frame_id == options_.map.frame_id
                                  ? MeshPose{}
                                  : lookup(message->header.frame_id, stamp);
            std::vector<NvbloxBlockUpdate> blocks;
            blocks.reserve(message->blocks.size());
            for (std::size_t i = 0; i < message->blocks.size(); ++i) {
                const auto &src = message->blocks[i];
                const auto &index = message->block_indices[i];
                if (src.vertices.empty() && src.triangles.empty()) {
                    blocks.push_back({{index.x, index.y, index.z}, nullptr});
                    continue;
                }
                if (src.vertices.empty() || src.triangles.empty())
                    throw std::invalid_argument("Incomplete nvblox geometry");
                auto g = std::make_shared<BlockGeometry>();
                g->vertices.reserve(src.vertices.size());
                g->colors.reserve(src.vertices.size());
                for (std::size_t v = 0; v < src.vertices.size(); ++v) {
                    const auto &p = src.vertices[v];
                    g->vertices.push_back({p.x, p.y, p.z});
                    auto color = fallback_;
                    if ((color_mode_ == "auto" || color_mode_ == "source") &&
                        src.colors.size() == src.vertices.size())
                        color = rgba(src.colors[v]);
                    else if ((color_mode_ == "auto" || color_mode_ == "normals") &&
                             src.normals.size() == src.vertices.size()) {
                        const auto &n = src.normals[v];
                        double length =
                            std::sqrt(double(n.x) * n.x + double(n.y) * n.y + double(n.z) * n.z);
                        const std::array<double, 3> components =
                            std::isfinite(length) && length > 1e-8
                                ? std::array<double, 3>{std::abs(n.x) / length,
                                                        std::abs(n.y) / length,
                                                        std::abs(n.z) / length}
                                : std::array<double, 3>{0, 0, 1};
                        for (std::size_t c = 0; c < 3; ++c)
                            color[c] = std::nearbyint((.25 + .75 * components[c]) * 255);
                        color[3] = 255;
                    }
                    g->colors.push_back(color);
                }
                for (auto t : src.triangles) {
                    if (t < 0)
                        throw std::invalid_argument("Negative nvblox index");
                    g->triangles.push_back(t);
                }
                blocks.push_back({{index.x, index.y, index.z}, std::move(g)});
            }
            enqueue(map_.apply_blocks(blocks, message->clear, message->header.frame_id, stamp,
                                      [&](const auto &, auto) { return pose; }));
            return true;
        } catch (const tf2::TransformException &) {
            return false;
        }
    }
    void receive(nvblox_msgs::msg::Mesh::ConstSharedPtr message) {
        std::lock_guard lock(mutex_);
        ++received_;
        last_received_ = Clock::now();
        if (message->blocks.size() != message->block_indices.size() ||
            message->blocks.size() > 65536) {
            reject("Invalid nvblox block count");
            return;
        }
        std::size_t size = 0, points = 0;
        for (const auto &b : message->blocks) {
            if (b.vertices.size() > options_.max_input_points - points) {
                reject("nvblox point limit exceeded");
                return;
            }
            points += b.vertices.size();
            size += (b.vertices.size() + b.normals.size()) * 12 + b.colors.size() * 16 +
                    b.triangles.size() * 4 + sizeof(nvblox_msgs::msg::MeshBlock);
        }
        defer({[this, message] { return apply(message); }, size, Clock::now()});
    }
#endif
    void reject(const std::string &error) {
        ++rejected_;
        error_ = error;
    }
    bool attempt(Pending &p) {
        try {
            return p.apply();
        } catch (const std::exception &e) {
            reject(e.what());
            return true;
        }
    }
    void defer(Pending item) {
        if (item.bytes > max_pending_bytes) {
            reject("Source message exceeds 128 MiB conversion budget");
            return;
        }
        if (pending_.empty() && attempt(item))
            return;
        if (pending_.size() >= 64 || item.bytes > max_pending_bytes - pending_bytes_) {
            reject("TF wait queue capacity reached; replay source snapshot after recovery");
            return;
        }
        pending_bytes_ += item.bytes;
        pending_.push_back(std::move(item));
    }
    void discover() {
        std::lock_guard lock(mutex_);
        try {
            const auto endpoints = node_->get_publishers_info_by_topic(input_topic_);
            source_publishers_ = 0;
            if (endpoints.empty())
                return;
            bool reliable = true, transient = true;
            const auto expected =
                type_ == "nvblox" ? "nvblox_msgs/msg/Mesh" : "visualization_msgs/msg/" + type_;
            for (const auto &e : endpoints) {
                if (e.topic_type() != expected) continue;
                ++source_publishers_;
                reliable &= e.qos_profile().reliability() == rclcpp::ReliabilityPolicy::Reliable;
                transient &=
                    e.qos_profile().durability() == rclcpp::DurabilityPolicy::TransientLocal;
            }
            if (source_publishers_ == 0) {
                error_ = "Expected source type " + expected;
                return;
            }
            const auto key = std::string(reliable ? "reliable" : "best_effort") + "/" +
                             (transient ? "transient_local" : "volatile");
            if (source_ && key == qos_key_)
                return;
            rclcpp::QoS qos(64);
            if (reliable)
                qos.reliable();
            else
                qos.best_effort();
            if (transient)
                qos.transient_local();
            else
                qos.durability_volatile();
            rclcpp::SubscriptionBase::SharedPtr next;
            if (type_ == "Marker")
                next = node_->create_subscription<Marker>(
                    input_topic_, qos, [this](Marker::ConstSharedPtr m) {
                        auto batch = std::make_shared<Markers>();
                        batch->markers.push_back(*m);
                        receive(batch);
                    });
            else if (type_ == "MarkerArray")
                next = node_->create_subscription<Markers>(
                    input_topic_, qos, [this](Markers::ConstSharedPtr m) { receive(m); });
#ifdef HORUS_NATIVE_NVBLOX
            else
                next = node_->create_subscription<nvblox_msgs::msg::Mesh>(
                    input_topic_, qos,
                    [this](nvblox_msgs::msg::Mesh::ConstSharedPtr m) { receive(m); });
#endif
            source_ = std::move(next);
            qos_key_ = key;
            if (!reliable)
                error_ = "Best-effort input may lose map deltas; reliable publication or source "
                         "snapshots are recommended";
        } catch (const std::exception &e) {
            error_ = e.what();
        }
    }
    void pump() {
        std::lock_guard lock(mutex_);
        try {
            for (unsigned i = 0; i < 4 && !pending_.empty(); ++i) {
                auto &p = pending_.front();
                if (Clock::now() - p.time > std::chrono::seconds(5))
                    reject("Mesh TF unavailable for 5s; source snapshot required");
                else if (!attempt(p))
                    break;
                pending_bytes_ -= p.bytes;
                pending_.pop_front();
            }
            enqueue(map_.expire(node_->now().seconds()));
            if (!output_->get_subscription_count())
                return;
            for (unsigned i = 0; i < 4; ++i) {
                auto page = scheduler_.pop_ready(seconds());
                if (!page)
                    break;
                Bytes wire;
                wire.data = page->payload;
                try {
                    output_->publish(wire);
                } catch (...) {
                    scheduler_.enqueue(page);
                    throw;
                }
                last_sent_[page->chunk_id] = {page->epoch, page->revision, seconds()};
                ++sent_;
                sent_bytes_ += page->payload.size();
            }
        } catch (const std::exception &e) {
            error_ = e.what();
        }
    }
    Json status_unlocked() const {
        return {{"source_connected", bool(source_) && source_publishers_ > 0},
                {"source_subscribed", bool(source_)},
                {"source_publishers", source_publishers_},
                {"source_receipt_age_s",
                 received_
                     ? Json(std::chrono::duration<double>(Clock::now() - last_received_).count())
                     : Json(nullptr)},
                {"source_qos", qos_key_},
                {"error", error_},
                {"received", received_},
                {"rejected", rejected_},
                {"sent_pages", sent_},
                {"sent_bytes", sent_bytes_},
                {"repair_pages", repairs_},
                {"tf_wait_messages", pending_.size()},
                {"tf_wait_bytes", pending_bytes_},
                {"page_count", map_.state().page_count()},
                {"cached_bytes", map_.state().cached_bytes()},
                {"source_bytes", map_.source_bytes()},
                {"pending_pages", scheduler_.pending_count()},
                {"pending_bytes", scheduler_.pending_bytes()},
                {"unchanged_pages", map_.state().unchanged_pages()},
                {"suppressed_bytes", map_.state().suppressed_bytes()}};
    }

  public:
    explicit SourceAdapter(const Json &j)
        : options_(options(j)), map_(options_),
          scheduler_({j.value("pages_per_second", 60.0),
                      j.value("bytes_per_second", 4.0 * 1024 * 1024), options_.map.max_pages,
                      options_.map.max_payload_bytes}) {
        input_topic_ = j.at("input_topic").get<std::string>();
        output_topic_ = j.value("output_topic", std::string("/horus/maps/live"));
        type_ = j.value("input_type", std::string("Marker"));
        color_mode_ = j.value("color_mode", std::string("auto"));
        if (input_topic_.empty() || input_topic_[0] != '/' || output_topic_.empty() ||
            output_topic_[0] != '/' || input_topic_ == output_topic_ || output_topic_.back() == '/')
            throw std::invalid_argument(
                "Use distinct absolute map topics without a trailing slash");
        if (type_ != "Marker" && type_ != "MarkerArray" && type_ != "nvblox")
            throw std::invalid_argument("input_type must be Marker, MarkerArray or nvblox");
        if (color_mode_ != "auto" && color_mode_ != "source" && color_mode_ != "normals" &&
            color_mode_ != "fallback")
            throw std::invalid_argument("Invalid nvblox color mode");
#ifndef HORUS_NATIVE_NVBLOX
        if (type_ == "nvblox")
            throw std::runtime_error(
                "This build lacks nvblox_msgs; install it and rebuild native ROS support");
#endif
        if (j.contains("fallback_color")) {
            const auto color = j.at("fallback_color").get<std::array<float, 4>>();
            std_msgs::msg::ColorRGBA c;
            c.r = color[0];
            c.g = color[1];
            c.b = color[2];
            c.a = color[3];
            fallback_ = rgba(c);
        }
        context_ = std::make_shared<rclcpp::Context>();
        context_->init(0, nullptr);
        static std::atomic<unsigned> counter{0};
        node_ = std::make_shared<rclcpp::Node>(
            "horus_mesh_source_" + std::to_string(getpid()) + "_" + std::to_string(counter++),
            rclcpp::NodeOptions().context(context_).use_global_arguments(false).parameter_overrides(
                {rclcpp::Parameter("use_sim_time", j.value("use_sim_time", false))}));
        tf_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_, node_, false);
        output_ = node_->create_publisher<Bytes>(output_topic_ + "/indexed", 64);
        manifest_ = node_->create_publisher<String>(output_topic_ + "/indexed_manifest",
                                                    rclcpp::QoS(1).transient_local());
        status_pub_ = node_->create_publisher<String>(output_topic_ + "/status", 1);
        request_ = node_->create_subscription<String>(
            output_topic_ + "/indexed_request", 16, [this](String::ConstSharedPtr m) {
                if (m->data.size() > 256 * 1024)
                    return;
                std::lock_guard lock(mutex_);
                try {
                    for (const auto &page : map_.state().repair_updates(m->data)) {
                        const auto it = last_sent_.find(page->chunk_id);
                        if (it != last_sent_.end() && std::get<0>(it->second) == page->epoch &&
                            std::get<1>(it->second) == page->revision &&
                            seconds() - std::get<2>(it->second) < 2)
                            continue;
                        if (scheduler_.enqueue(page))
                            ++repairs_;
                    }
                } catch (const std::exception &e) {
                    reject(e.what());
                }
            });
        discovery_ =
            node_->create_wall_timer(std::chrono::milliseconds(500), [this] { discover(); });
        pump_ = node_->create_wall_timer(std::chrono::milliseconds(10), [this] { pump(); });
        inventory_ = node_->create_wall_timer(std::chrono::seconds(2), [this] {
            std::lock_guard lock(mutex_);
            try {
                String msg;
                msg.data = map_.state().manifest_json();
                manifest_->publish(msg);
                msg.data = status_unlocked().dump();
                status_pub_->publish(msg);
            } catch (const std::exception &e) {
                error_ = e.what();
            }
        });
        rclcpp::ExecutorOptions eo;
        eo.context = context_;
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
        executor_->add_node(node_);
        worker_ = std::thread([this] {
            try {
                executor_->spin();
            } catch (const std::exception &e) {
                std::lock_guard lock(mutex_);
                error_ = e.what();
            }
        });
    }
    ~SourceAdapter() {
        try {
            context_->shutdown("HORUS mesh source destroyed");
        } catch (...) {
        }
        try {
            executor_->cancel();
        } catch (...) {
        }
        if (worker_.joinable())
            worker_.join();
    }
    Json status() const {
        std::lock_guard lock(mutex_);
        return status_unlocked();
    }
};
} // namespace
extern "C" {
void *horus_native_mesh_source_create(const char *json, char **error) {
    if (error)
        *error = nullptr;
    try {
        if (!json)
            throw std::invalid_argument("Missing mesh source options");
        return new SourceAdapter(Json::parse(json));
    } catch (const std::exception &e) {
        if (error)
            try {
                *error = copy(e.what());
            } catch (...) {
            }
    } catch (...) {
    }
    return nullptr;
}
char *horus_native_mesh_source_status(void *handle) {
    try {
        if (!handle)
            throw std::invalid_argument("Missing mesh source handle");
        return copy(static_cast<SourceAdapter *>(handle)->status().dump());
    } catch (const std::exception &e) {
        try {
            return copy(Json{{"error", e.what()}}.dump());
        } catch (...) {
        }
    } catch (...) {
    }
    return nullptr;
}
void horus_native_mesh_source_destroy(void *handle) { delete static_cast<SourceAdapter *>(handle); }
}
