#include "horus/teaching/runtime.hpp"
#ifdef HORUS_ROS2_ENABLED
#include "core/api.h"
#include "teach_runtime.h"
#endif

namespace horus::teaching {
TeachRuntime::TeachRuntime(const TeachConfig &config, const std::string &robot,
                           std::shared_ptr<TeachAdapter> adapter)
    : session_(config, robot, std::move(adapter)) {
#ifdef HORUS_ROS2_ENABLED
    char *error = nullptr;
    auto text = config.to_dict().dump();
    runtime_ = horus_teach_ros_create(&session_, text.c_str(), robot.c_str(), &error);
    std::string reason = error ? error : "Teach ROS runtime initialization failed";
    horus_teach_free(error);
    if (!runtime_)
        throw std::runtime_error(reason);
#else
    throw std::runtime_error("TeachRuntime requires a ROS-enabled SDK build");
#endif
}
TeachRuntime::~TeachRuntime() {
#ifdef HORUS_ROS2_ENABLED
    horus_teach_ros_destroy(runtime_);
#endif
}
Json TeachRuntime::status() const {
#ifdef HORUS_ROS2_ENABLED
    char *value = horus_teach_ros_status(runtime_);
    if (!value)
        throw std::runtime_error("Teach status unavailable");
    Json result;
    try {
        result = Json::parse(value);
    } catch (...) {
        horus_teach_free(value);
        throw;
    }
    horus_teach_free(value);
    if (!result.value("ok", false))
        throw std::runtime_error(result.value("error", "Teach runtime failed"));
    return result.at("value");
#else
    return {{"running", false}, {"error", "ROS disabled"}};
#endif
}
} // namespace horus::teaching
