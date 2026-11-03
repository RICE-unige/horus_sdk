#pragma once
#include "horus/remote_rendering/encoder.hpp"
#include "horus/remote_rendering/stereo.hpp"

namespace horus::remote_rendering {
struct RemotePublisherOptions {
    std::string session_id, mode{"diagnostic"}; // diagnostic, rgb or h264
    std::string view_topic{default_view_state_topic}, stream_topic{default_ros_diagnostic_topic},
        status_topic{default_status_topic};
    double maximum_view_age_seconds{2};
};
// ROS endpoint for a renderer owned by the application. One active viewer, a
// newest-only input slot and no sensor subscriptions or hidden scene selection.
class RemotePublisher {
    struct Impl;
    std::unique_ptr<Impl> impl_;
    bool publish(int kind, const Json &, std::span<const std::uint8_t>);

  public:
    explicit RemotePublisher(const RemotePublisherOptions & = {});
    ~RemotePublisher();
    RemotePublisher(const RemotePublisher &) = delete;
    RemotePublisher &operator=(const RemotePublisher &) = delete;
    std::optional<ViewState> next_view(std::uint32_t timeout_ms = 100);
    bool publish_diagnostic(const StereoFrameRender &);
    bool publish_rgb(const StereoFrameRender &);
    bool publish_h264(const EncodedH264Frame &);
    bool take_keyframe_request();
    Json status() const;
    void stop();
};
} // namespace horus::remote_rendering
