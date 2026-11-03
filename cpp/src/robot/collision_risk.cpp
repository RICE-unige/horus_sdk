#include "horus/dataviz/collision_risk.hpp"
#include <stdexcept>
#ifdef HORUS_ROS2_ENABLED
#include "collision_runtime.h"
#include "registry.h"
#include <nlohmann/json.hpp>
#endif
namespace horus::dataviz {
struct CollisionRiskAnalyzer::Impl {
#ifdef HORUS_ROS2_ENABLED
    std::unique_ptr<void, decltype(&horus_collision_destroy)> handle{nullptr,
                                                                     horus_collision_destroy};
#endif
    explicit Impl(const CollisionRiskOptions &o) {
#ifdef HORUS_ROS2_ENABLED
        const auto text =
            nlohmann::json{{"robot_name", o.robot_name},           {"source_topic", o.source_topic},
                           {"source_type", o.source_type},         {"output_topic", o.output_topic},
                           {"threshold_m", o.threshold_m},         {"publish_hz", o.publish_hz},
                           {"smoothing_alpha", o.smoothing_alpha}, {"z_window_m", o.z_window_m}}
                .dump();
        char *error = nullptr;
        handle.reset(horus_collision_create(text.c_str(), &error));
        std::unique_ptr<char, decltype(&horus_native_free)> owned(error, horus_native_free);
        if (!handle)
            throw std::runtime_error(error ? error : "Collision analyzer creation failed");
#else
        (void)o;
        throw std::runtime_error("Build with ROS 2 for CollisionRiskAnalyzer");
#endif
    }
};
CollisionRiskAnalyzer::CollisionRiskAnalyzer(const CollisionRiskOptions &o)
    : impl_(std::make_unique<Impl>(o)) {}
CollisionRiskAnalyzer::~CollisionRiskAnalyzer() = default;
void CollisionRiskAnalyzer::stop() {
#ifdef HORUS_ROS2_ENABLED
    horus_collision_stop(impl_->handle.get());
#endif
}
} // namespace horus::dataviz
