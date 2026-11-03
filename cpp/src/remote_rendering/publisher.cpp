#include "horus/remote_rendering/publisher.hpp"
#ifdef HORUS_ROS2_ENABLED
#include "registry.h"
#include "remote_runtime.h"
#endif

namespace horus::remote_rendering {
struct RemotePublisher::Impl {
#ifdef HORUS_ROS2_ENABLED
    std::unique_ptr<void, decltype(&horus_remote_destroy)> runtime{nullptr, horus_remote_destroy};
    static std::string take(char *text) {
        std::unique_ptr<char, decltype(&horus_native_free)> owned(text, horus_native_free);
        return text ? std::string(text) : "Remote runtime returned no result";
    }
    static Json result(char *text, char *error) {
        if (!text)
            throw std::runtime_error(take(error));
        return Json::parse(take(text));
    }
#endif
    explicit Impl(const RemotePublisherOptions &o) {
#ifdef HORUS_ROS2_ENABLED
        auto json = Json{{"session_id", o.session_id},
                         {"mode", o.mode},
                         {"view_topic", o.view_topic},
                         {"stream_topic", o.stream_topic},
                         {"status_topic", o.status_topic},
                         {"maximum_view_age_seconds", o.maximum_view_age_seconds}}
                        .dump();
        char *error = nullptr;
        runtime.reset(horus_remote_create(json.c_str(), &error));
        if (!runtime)
            throw std::runtime_error(take(error));
#else
        (void)o;
        throw std::runtime_error("Enable ROS 2 for remote publication");
#endif
    }
};
RemotePublisher::RemotePublisher(const RemotePublisherOptions &o)
    : impl_(std::make_unique<Impl>(o)) {}
RemotePublisher::~RemotePublisher() = default;
std::optional<ViewState> RemotePublisher::next_view(uint32_t timeout) {
#ifdef HORUS_ROS2_ENABLED
    char *error = nullptr;
    auto text = horus_remote_next_view(impl_->runtime.get(), timeout, &error);
    auto json = Impl::result(text, error);
    if (!json.is_null())
        return ViewState::from_dict(json);
#endif
    return std::nullopt;
}
bool RemotePublisher::publish(int kind, const Json &j, std::span<const uint8_t> bytes) {
#ifdef HORUS_ROS2_ENABLED
    char *error = nullptr;
    const auto json = j.dump();
    int result = horus_remote_publish(impl_->runtime.get(), kind, json.c_str(), bytes.data(),
                                      bytes.size(), &error);
    if (result < 0)
        throw std::runtime_error(Impl::take(error));
    return result != 0;
#else
    throw std::runtime_error("Enable ROS 2 for remote publication");
#endif
}
bool RemotePublisher::publish_diagnostic(const StereoFrameRender &f) {
    const auto packet = pack_stereo_diagnostic(f);
    return publish(0, f.metadata, packet);
}
bool RemotePublisher::publish_rgb(const StereoFrameRender &f) {
    return publish(1, f.metadata, f.video_rgb);
}
bool RemotePublisher::publish_h264(const EncodedH264Frame &f) {
    return publish(2, Json::parse(f.metadata_json), f.data);
}
bool RemotePublisher::take_keyframe_request() {
#ifdef HORUS_ROS2_ENABLED
    return horus_remote_take_keyframe(impl_->runtime.get());
#else
    return false;
#endif
}
Json RemotePublisher::status() const {
#ifdef HORUS_ROS2_ENABLED
    char *error = nullptr;
    auto text = horus_remote_status(impl_->runtime.get(), &error);
    return Impl::result(text, error);
#else
    return {{"running", false}};
#endif
}
void RemotePublisher::stop() {
#ifdef HORUS_ROS2_ENABLED
    horus_remote_stop(impl_->runtime.get());
#endif
}
} // namespace horus::remote_rendering
