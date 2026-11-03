#include "horus/description/resolver.hpp"
#if defined(HORUS_ROS2_ENABLED)
#include "description_runtime.h"
#include "registry.h"
#endif
#include <memory>
#include <stdexcept>
namespace horus::description {
Artifact resolve(const nlohmann::json &options) {
    if (!options.value("urdf_path", "").empty())
        return compile_file(options);
#if defined(HORUS_ROS2_ENABLED)
    std::unique_ptr<char, decltype(&horus_native_free)> text(
        horus_native_description_resolve(options.dump().c_str()), horus_native_free);
    if (!text)
        throw std::runtime_error("Description source returned no result");
    const auto result = nlohmann::json::parse(text.get());
    if (!result.value("ok", false))
        throw std::runtime_error(result.value("error", "Description resolution failed"));
    const auto &value = result.at("value");
    return {value.at("manifest"), value.at("payload"), value.at("payload_json"),
            value.at("encoded_payload")};
#else
    throw std::runtime_error("Build with ROS 2 for robot description topic/parameter sources");
#endif
}
} // namespace horus::description
