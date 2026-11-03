#include "horus/tf.hpp"
#include <stdexcept>
#ifdef HORUS_ROS2_ENABLED
#include "registry.h"
#include "tf_runtime.h"
#include <nlohmann/json.hpp>
#include <vector>
#endif
namespace horus::tf {
struct Publisher::Impl {
#ifdef HORUS_ROS2_ENABLED
    std::unique_ptr<void, decltype(&horus_native_tf_destroy)> handle{nullptr,
                                                                     horus_native_tf_destroy};
    static std::string take(char *error) {
        std::unique_ptr<char, decltype(&horus_native_free)> owned(error, horus_native_free);
        return error ? error : "TF publisher failed without error details";
    }
#endif
    explicit Impl(const PublisherOptions &o) {
#ifdef HORUS_ROS2_ENABLED
        auto text = nlohmann::json{{"topic", o.topic},
                                   {"static_topic", o.static_topic},
                                   {"use_sim_time", o.use_sim_time}}
                        .dump();
        char *error = nullptr;
        handle.reset(horus_native_tf_create(text.c_str(), &error));
        if (!handle)
            throw std::runtime_error(take(error));
#else
        (void)o;
        throw std::runtime_error("Build HORUS with ROS 2 enabled for TF publication");
#endif
    }
    void publish(std::span<const Transform> transforms, bool is_static) {
#ifdef HORUS_ROS2_ENABLED
        if (transforms.size() > 4096)
            throw std::invalid_argument("TF batch exceeds 4096 transforms");
        std::vector<HorusNativeTransform> batch;
        batch.reserve(transforms.size());
        for (const auto &t : transforms) {
            if (t.parent.find('\0') != std::string::npos || t.child.find('\0') != std::string::npos)
                throw std::invalid_argument("TF frame contains NUL");
            if (t.stamp_ns && *t.stamp_ns < 0)
                throw std::invalid_argument("TF stamp must be nonnegative");
            batch.push_back({t.parent.c_str(),
                             t.child.c_str(),
                             {t.translation[0], t.translation[1], t.translation[2]},
                             {t.rotation[0], t.rotation[1], t.rotation[2], t.rotation[3]},
                             t.stamp_ns.value_or(-1)});
        }
        char *error = nullptr;
        if (!horus_native_tf_publish(handle.get(), batch.data(), batch.size(), is_static, &error))
            throw std::runtime_error(take(error));
#else
        (void)transforms;
        (void)is_static;
        throw std::runtime_error("Build HORUS with ROS 2 enabled for TF publication");
#endif
    }
};
Publisher::Publisher(const PublisherOptions &options) : impl_(std::make_unique<Impl>(options)) {}
Publisher::~Publisher() = default;
void Publisher::publish(std::span<const Transform> transforms, bool is_static) {
    impl_->publish(transforms, is_static);
}
size_t Publisher::subscriber_count(bool is_static) const {
#ifdef HORUS_ROS2_ENABLED
    char *error = nullptr;
    const auto count = horus_native_tf_subscribers(impl_->handle.get(), is_static, &error);
    if (count < 0)
        throw std::runtime_error(Impl::take(error));
    return static_cast<size_t>(count);
#else
    (void)is_static;
    throw std::runtime_error("Build HORUS with ROS 2 enabled for TF publication");
#endif
}
} // namespace horus::tf
