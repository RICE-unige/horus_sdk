#include "horus/mapping/costmap_adapter.hpp"
#include <stdexcept>
#ifdef HORUS_ROS2_ENABLED
#include "costmap_runtime.h"
#include "registry.h"
#endif
namespace horus::mapping {
struct CostmapAdapter::Impl {
#ifdef HORUS_ROS2_ENABLED
    std::unique_ptr<void,decltype(&horus_native_costmap_destroy)> runtime{nullptr,horus_native_costmap_destroy};
    static std::string take(char* text) {
        std::unique_ptr<char,decltype(&horus_native_free)> owned(text,horus_native_free);
        return text ? std::string(text) : "Native costmap returned no result";
    }
#endif
    explicit Impl(const CostmapAdapterOptions& o) {
#ifdef HORUS_ROS2_ENABLED
        auto options=nlohmann::json{{"input_topic",o.input_topic},{"input_format",o.input_format},{"updates_topic",o.updates_topic},
            {"snapshot_service",o.snapshot_service},{"output_topic",o.output_topic},{"publish_hz",o.publish_hz},
            {"resync_seconds",o.resync_seconds},{"volatile_full",o.volatile_full}}.dump();
        char* error=nullptr; runtime.reset(horus_native_costmap_create(options.c_str(),&error));
        if (!runtime) throw std::runtime_error(take(error));
#else
        (void)o; throw std::runtime_error("Build HORUS with ROS 2 enabled for costmap adaptation");
#endif
    }
    nlohmann::json status() const {
#ifdef HORUS_ROS2_ENABLED
        auto result=nlohmann::json::parse(take(horus_native_costmap_status(runtime.get())));
        if (!result.value("ok",false)) throw std::runtime_error(result.value("error","Costmap status failed"));
        return result.at("value");
#else
        return {{"running",false}};
#endif
    }
};
CostmapAdapter::CostmapAdapter(const CostmapAdapterOptions& options):impl_(std::make_unique<Impl>(options)) {}
CostmapAdapter::~CostmapAdapter()=default;
nlohmann::json CostmapAdapter::status() const { return impl_->status(); }
}
