#include "remote_runtime.h"
#include "horus/remote_rendering/protocol.hpp"
#include <atomic>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <tuple>

namespace {
using namespace horus::remote_rendering;
using String = std_msgs::msg::String;
using Compressed = sensor_msgs::msg::CompressedImage;
using Image = sensor_msgs::msg::Image;
double monotonic() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
bool contains_idr(std::span<const uint8_t> bytes) {
    // Inspect Annex-B NAL headers; this is recovery gating, not a video decoder.
    for (size_t i = 0; i + 3 < bytes.size(); ++i)
        if (bytes[i] == 0 && bytes[i + 1] == 0 && bytes[i + 2] == 1 && (bytes[i + 3] & 0x1f) == 5)
            return true;
    return false;
}
Json parse(const char *value) {
    if (!value || strnlen(value, maximum_metadata_bytes + 1) > maximum_metadata_bytes)
        throw std::invalid_argument("Missing or oversized remote JSON");
    return Json::parse(value);
}
std::string topic(const Json &o, const char *key, const char *fallback) {
    auto value = o.value(key, std::string(fallback));
    if (value.empty() || value[0] != '/' || value.size() > 256)
        throw std::invalid_argument(std::string("Invalid remote topic: ") + key);
    return value;
}
class Runtime {
    rclcpp::Context::SharedPtr context_;
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::Subscription<String>::SharedPtr view_sub_, keyframe_sub_;
    rclcpp::Publisher<String>::SharedPtr status_pub_;
    rclcpp::Publisher<Image>::SharedPtr image_pub_;
    rclcpp::Publisher<Compressed>::SharedPtr compressed_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread worker_;
    std::mutex stop_mutex_, mutex_, publish_mutex_;
    std::atomic<bool> stopped_{false}, keyframe_{true};
    LatestValueQueue<ViewState> views_;
    ViewStreamSequenceGate gate_;
    std::optional<ViewState> latest_;
    std::string session_filter_, mode_, stream_topic_, view_topic_, error_, last_reject_;
    double received_at_{}, maximum_age_;
    uint64_t received_{}, rejected_{}, published_{}, discarded_{}, last_frame_{};
    using StreamIdentity = std::tuple<std::string, std::string, uint64_t>;
    StreamIdentity published_stream_;
    bool has_published_{};
    bool needs_idr_{true};
    std::map<uint64_t, double> view_receipts_;
    void receive(const String &message) {
        {
            std::lock_guard lock(mutex_);
            ++received_;
        }
        try {
            if (message.data.size() > maximum_metadata_bytes)
                throw std::invalid_argument("View state exceeds metadata budget");
            auto view = ViewState::from_json(message.data);
            std::lock_guard lock(mutex_);
            if (stopped_)
                return;
            auto rejected = (!session_filter_.empty() && session_filter_ != view.session_id)
                                ? std::optional<std::string>("session_id")
                                : gate_.accept(view, monotonic());
            if (rejected) {
                ++rejected_;
                last_reject_ = *rejected;
                return;
            }
            if (!latest_ || latest_->session_id != view.session_id ||
                latest_->view_stream_id != view.view_stream_id ||
                latest_->workspace.revision != view.workspace.revision) {
                view_receipts_.clear();
                if (mode_ == "h264") {
                    needs_idr_ = true;
                    keyframe_ = true;
                }
            }
            received_at_ = monotonic();
            view_receipts_[view.view_sequence] = received_at_;
            while (!view_receipts_.empty() &&
                   (view_receipts_.size() > 8192 ||
                    received_at_ - view_receipts_.begin()->second > maximum_age_))
                view_receipts_.erase(view_receipts_.begin());
            latest_ = view;
            views_.put(std::move(view));
        } catch (const std::exception &e) {
            std::lock_guard lock(mutex_);
            ++rejected_;
            last_reject_ = e.what();
        }
    }

  public:
    explicit Runtime(const Json &o)
        : session_filter_(o.value("session_id", std::string{})),
          mode_(o.value("mode", std::string("diagnostic"))),
          maximum_age_(o.value("maximum_view_age_seconds", 2.0)) {
        if ((mode_ != "diagnostic" && mode_ != "rgb" && mode_ != "h264") ||
            !std::isfinite(maximum_age_) || maximum_age_ <= 0 || maximum_age_ > 60 ||
            session_filter_.size() > 256)
            throw std::invalid_argument("Invalid remote mode/session/view expiry");
        view_topic_ = topic(o, "view_topic", default_view_state_topic);
        stream_topic_ = topic(o, "stream_topic",
                              mode_ == "diagnostic" ? default_ros_diagnostic_topic
                                                    : "/horus/remote_render/image");
        const auto status_topic = topic(o, "status_topic", default_status_topic);
        if (view_topic_ == stream_topic_ || view_topic_ == status_topic ||
            stream_topic_ == status_topic)
            throw std::invalid_argument("Remote input, output and status topics must be distinct");
        try {
            context_ = std::make_shared<rclcpp::Context>();
            rclcpp::InitOptions init;
            init.shutdown_on_signal = false;
            context_->init(0, nullptr, init);
            rclcpp::NodeOptions options;
            options.context(context_);
            options.use_global_arguments(false);
            static std::atomic<uint64_t> next{};
            node_ = std::make_shared<rclcpp::Node>("horus_native_remote_" + std::to_string(++next),
                                                   options);
            status_pub_ = node_->create_publisher<String>(status_topic, rclcpp::QoS(1).reliable());
            if (mode_ == "rgb")
                image_pub_ =
                    node_->create_publisher<Image>(stream_topic_, rclcpp::QoS(1).best_effort());
            else
                compressed_pub_ = node_->create_publisher<Compressed>(
                    stream_topic_, mode_ == "diagnostic" ? rclcpp::QoS(1).reliable()
                                                         : rclcpp::QoS(1).best_effort());
            view_sub_ = node_->create_subscription<String>(
                view_topic_, rclcpp::QoS(1).best_effort(),
                [this](String::ConstSharedPtr m) { receive(*m); });
            if (mode_ == "h264")
                keyframe_sub_ = node_->create_subscription<String>(
                    stream_topic_ + "/request_keyframe", 1, [this](String::ConstSharedPtr m) {
                        if (m->data == "idr")
                            keyframe_ = true;
                    });
            timer_ = node_->create_wall_timer(std::chrono::milliseconds(250), [this] {
                try {
                    String message;
                    message.data = status().dump();
                    status_pub_->publish(message);
                } catch (const std::exception &e) {
                    std::lock_guard lock(mutex_);
                    error_ = e.what();
                }
            });
            rclcpp::ExecutorOptions exec;
            exec.context = context_;
            executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(exec);
            executor_->add_node(node_);
            worker_ = std::thread([this] {
                try {
                    executor_->spin();
                } catch (const std::exception &e) {
                    std::lock_guard lock(mutex_);
                    error_ = e.what();
                }
                stopped_ = true;
                views_.close();
            });
        } catch (...) {
            stop();
            throw;
        }
    }
    ~Runtime() { stop(); }
    void stop() noexcept {
        std::lock_guard lock(stop_mutex_);
        stopped_ = true;
        views_.close();
        try {
            if (executor_)
                executor_->cancel();
        } catch (...) {
        }
        try {
            if (context_ && context_->is_valid())
                context_->shutdown("Remote publisher stopped");
        } catch (...) {
        }
        if (worker_.joinable())
            worker_.join();
    }
    Json next(uint32_t timeout) {
        if (timeout > 60000)
            throw std::invalid_argument("Remote wait is limited to 60 seconds");
        if (stopped_)
            return nullptr;
        auto view = views_.get(std::chrono::milliseconds(timeout));
        if (stopped_ || !view)
            return nullptr;
        return view->to_dict();
    }
    bool take_keyframe() { return keyframe_.exchange(false); }
    Json status() {
        std::lock_guard lock(mutex_);
        const double age = latest_ ? monotonic() - received_at_ : 0;
        const bool current_stream_published =
            latest_ && has_published_ &&
            published_stream_ == StreamIdentity{latest_->session_id, latest_->view_stream_id,
                                                latest_->workspace.revision};
        return {{"state", stopped_                   ? "stopped"
                          : !error_.empty()          ? "error"
                          : !latest_                 ? "waiting_for_view"
                          : age > maximum_age_       ? "stale_view"
                          : current_stream_published ? "streaming_source"
                                                     : "ready"},
                {"running", !stopped_},
                {"protocol_version", protocol_version},
                {"format_version",
                 mode_ == "diagnostic" ? ros_diagnostic_format_version : webrtc_format_version},
                {"transport", mode_ == "diagnostic" ? "ros" : "webrtc"},
                {"webrtc_image_type", mode_},
                {"stream_topic", stream_topic_},
                {"view_state_topic", view_topic_},
                {"views_received", received_},
                {"views_rejected", rejected_},
                {"last_view_reject_reason", last_reject_},
                {"frames_published", published_},
                {"frames_discarded", discarded_},
                {"views_replaced", views_.replaced_count()},
                {"retained_view_receipts", view_receipts_.size()},
                {"waiting_for_keyframe", mode_ == "h264" && needs_idr_},
                {"pose_age_ms", latest_ ? Json(age * 1000) : Json(nullptr)},
                {"error", error_}};
    }
    bool publish(int kind, const Json &raw, std::span<const uint8_t> bytes) {
        const auto metadata = validate_frame_metadata(raw);
        if (kind < 0 || kind > 2 || (kind == 0 && mode_ != "diagnostic") ||
            (kind == 1 && mode_ != "rgb") || (kind == 2 && mode_ != "h264") || bytes.empty() ||
            bytes.size() > maximum_image_bytes + maximum_metadata_bytes + 32)
            throw std::invalid_argument("Invalid remote frame kind, size or configured mode");
        const auto width = metadata.at("width_per_eye").get<uint64_t>() * 2;
        const auto height = metadata.at("video_height").get<uint64_t>();
        if (width > 16384 || !height || height > 16384 || width * height > maximum_image_bytes / 3)
            throw std::invalid_argument("Remote image dimensions exceed the publication budget");
        if (kind == 0) {
            auto packet = unpack_ros_diagnostic_frame(bytes);
            if (packet.metadata != metadata || metadata.at("codec") != "png")
                throw std::invalid_argument("Diagnostic packet metadata mismatch");
        } else if ((kind == 1 &&
                    (bytes.size() != width * height * 3 || metadata.at("codec") != "h264")) ||
                   (kind == 2 && metadata.at("codec") != "h264"))
            throw std::invalid_argument("Video data disagrees with metadata");
        const StreamIdentity identity{metadata.at("session_id").get<std::string>(),
                                      metadata.value("view_stream_id", std::string{}),
                                      metadata.at("workspace_revision").get<uint64_t>()};
        const auto frame = metadata.at("frame_id").get<uint64_t>();
        const bool idr = kind == 2 && contains_idr(bytes);
        std::lock_guard publication(publish_mutex_);
        {
            std::lock_guard lock(mutex_);
            if (stopped_)
                throw std::runtime_error("Remote publisher is stopped");
            if (has_published_ && identity == published_stream_ && frame <= last_frame_) {
                ++discarded_;
                return false;
            }
            const auto receipt =
                view_receipts_.find(metadata.at("source_view_sequence").get<uint64_t>());
            if (!latest_ || receipt == view_receipts_.end() ||
                monotonic() - receipt->second > maximum_age_ ||
                metadata.at("session_id") != latest_->session_id ||
                metadata.value("view_stream_id", std::string{}) != latest_->view_stream_id ||
                metadata.at("workspace_revision") != latest_->workspace.revision ||
                metadata.at("source_view_sequence").get<uint64_t>() > latest_->view_sequence) {
                ++discarded_;
                if (kind == 2) {
                    needs_idr_ = true;
                    keyframe_ = true;
                }
                return false;
            }
            if (kind == 2 && (needs_idr_ || !has_published_ || identity != published_stream_) &&
                !idr) {
                ++discarded_;
                needs_idr_ = true;
                keyframe_ = true;
                return false;
            }
        }
        if (kind == 1) {
            Image image;
            image.header.stamp = node_->now();
            image.header.frame_id = "horus.remote_render.v3:" + metadata.dump();
            image.width = width;
            image.height = height;
            image.step = width * 3;
            image.encoding = "rgb8";
            image.data.assign(bytes.begin(), bytes.end());
            image_pub_->publish(std::move(image));
        } else {
            Compressed image;
            image.header.stamp = node_->now();
            image.header.frame_id = kind == 0 ? "remote_render_v3:" + std::to_string(frame)
                                              : "horus.remote_render.v3:" + metadata.dump();
            image.format = kind == 0 ? ros_diagnostic_format_version
                                     : "h264; stream-format=byte-stream; alignment=au";
            image.data.assign(bytes.begin(), bytes.end());
            compressed_pub_->publish(std::move(image));
        }
        {
            std::lock_guard lock(mutex_);
            ++published_;
            has_published_ = true;
            published_stream_ = identity;
            last_frame_ = frame;
            if (kind == 2 && latest_ &&
                identity == StreamIdentity{latest_->session_id, latest_->view_stream_id,
                                           latest_->workspace.revision})
                needs_idr_ = false;
        }
        return true;
    }
};
template <class F> auto guard(char **error, F &&f) noexcept -> decltype(f()) {
    if (error)
        *error = nullptr;
    try {
        return f();
    } catch (const std::exception &e) {
        if (error)
            *error = strdup(e.what());
    } catch (...) {
        if (error)
            *error = strdup("Remote ROS operation failed");
    }
    return {};
}
Runtime &runtime(void *p) {
    if (!p)
        throw std::invalid_argument("Missing remote publisher");
    return *static_cast<Runtime *>(p);
}
} // namespace
extern "C" {
void *horus_remote_create(const char *options, char **error) {
    return guard(error, [&]() -> void * { return new Runtime(parse(options)); });
}
void horus_remote_destroy(void *p) { delete static_cast<Runtime *>(p); }
void horus_remote_stop(void *p) {
    if (p)
        static_cast<Runtime *>(p)->stop();
}
char *horus_remote_next_view(void *p, uint32_t timeout, char **error) {
    return guard(error, [&] { return strdup(runtime(p).next(timeout).dump().c_str()); });
}
char *horus_remote_status(void *p, char **error) {
    return guard(error, [&] { return strdup(runtime(p).status().dump().c_str()); });
}
int horus_remote_take_keyframe(void *p) {
    return p ? static_cast<Runtime *>(p)->take_keyframe() : 0;
}
int horus_remote_publish(void *p, int kind, const char *json, const uint8_t *data, size_t n,
                         char **error) {
    int result = -1;
    guard(error, [&] {
        if (!data && n)
            throw std::invalid_argument("Missing remote bytes");
        result = runtime(p).publish(kind, parse(json), {data, n});
        return true;
    });
    return result;
}
}
