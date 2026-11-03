#pragma once
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace horus::remote_rendering {
using Json = nlohmann::json;
inline constexpr auto protocol_version = "horus.remote_render.v3";
inline constexpr auto view_state_version = "horus.remote_render.view_state.v3";
inline constexpr auto frame_metadata_version = "horus.remote_render.frame.v3";
inline constexpr auto ros_diagnostic_format_version = "horus.remote_render.v3.ros";
inline constexpr auto webrtc_format_version = "horus.remote_render.v3.webrtc";
inline constexpr auto default_view_state_topic = "/horus/remote_render/v3/view_state";
inline constexpr auto default_ros_diagnostic_topic = "/horus/remote_render/v3/stereo";
inline constexpr auto default_status_topic = "/horus/remote_render/v3/status";
inline constexpr std::size_t maximum_metadata_bytes = 64 * 1024;
inline constexpr std::size_t maximum_image_bytes = 32 * 1024 * 1024;
using Matrix4 = std::array<std::array<double, 4>, 4>;

// Host monotonic clock for render/encode timing, not ROS message header time.
inline std::uint64_t monotonic_time_ns() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

struct Vector3 {
    double x{}, y{}, z{};
};
struct Quaternion {
    double x{}, y{}, z{}, w{1};
};
struct EyeFov {
    double left{-1}, right{1}, up{1}, down{-1};
    std::array<double, 4> renderer_projection() const;
};
struct EyeView {
    Vector3 position;
    Quaternion rotation;
    EyeFov fov;
};
struct WorkspaceState {
    std::uint64_t revision{};
    Vector3 position;
    Quaternion rotation;
    double scale{1};
};
struct ViewState {
    std::string session_id, reference_space_id, view_stream_id;
    std::uint64_t view_sequence{}, sample_time_ns{}, predicted_display_time_ns{},
        predicted_display_period_ns{};
    WorkspaceState workspace;
    EyeView left_eye, right_eye;
    std::uint32_t viewport_width{}, viewport_height{};
    double refresh_rate_hz{};
    static ViewState from_dict(const Json &);
    static ViewState from_json(const std::string &);
    Json to_dict() const;
};

EyeFov eye_fov_from_unity_projection(double m00, double m11, double m02, double m12);
Matrix4 eye_view_matrix(const EyeView &);
Matrix4 eye_projection_matrix(const EyeView &, double near_m, double far_m);
Matrix4 eye_view_projection_matrix(const EyeView &, double near_m, double far_m);
// Normalizes optional fields and rejects malformed values before any frame allocation.
Json validate_frame_metadata(const Json &);
struct DiagnosticFrameView {
    Json metadata;
    // Borrows the caller's packet; keep it alive while using image.
    std::span<const std::uint8_t> image;
};
std::vector<std::uint8_t> pack_ros_diagnostic_frame(const Json &, std::span<const std::uint8_t>);
DiagnosticFrameView unpack_ros_diagnostic_frame(std::span<const std::uint8_t>);

struct ClockSample {
    std::int64_t client_send_ns, server_receive_ns, server_send_ns, client_receive_ns;
    double round_trip_ns() const;
    double server_minus_client_ns() const;
};
class ClockAlignment {
    std::size_t maximum_;
    std::deque<ClockSample> samples_;

  public:
    explicit ClockAlignment(std::size_t maximum_samples = 32);
    void add(const ClockSample &);
    std::size_t sample_count() const { return samples_.size(); }
    double offset_ns() const;
    double round_trip_ns() const;
};
class ViewStreamSequenceGate {
    double idle_timeout_, last_received_{};
    std::optional<std::string> active_;
    std::uint64_t last_sequence_{}, last_sample_{}, revision_{};
    std::deque<std::string> retired_;

  public:
    explicit ViewStreamSequenceGate(double idle_timeout = 2);
    std::optional<std::string> accept(const ViewState &, double monotonic_seconds);
    std::uint64_t revision() const { return revision_; }
};
ViewState select_newest_view(std::span<const ViewState>);

template <class T> class LatestValueQueue {
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::optional<T> value_;
    bool closed_{};
    std::uint64_t replaced_{};

  public:
    void put(T value) {
        std::lock_guard lock(mutex_);
        if (closed_)
            throw std::runtime_error("queue is closed");
        replaced_ += value_.has_value();
        value_ = std::move(value);
        ready_.notify_one();
    }
    std::optional<T> get(std::optional<std::chrono::milliseconds> timeout = std::nullopt) {
        std::unique_lock lock(mutex_);
        const auto predicate = [&] { return value_.has_value() || closed_; };
        if (timeout)
            ready_.wait_for(lock, *timeout, predicate);
        else
            ready_.wait(lock, predicate);
        auto result = std::move(value_);
        value_.reset();
        return result;
    }
    void close() {
        std::lock_guard lock(mutex_);
        closed_ = true;
        ready_.notify_all();
    }
    std::uint64_t replaced_count() const {
        std::lock_guard lock(mutex_);
        return replaced_;
    }
};
} // namespace horus::remote_rendering
