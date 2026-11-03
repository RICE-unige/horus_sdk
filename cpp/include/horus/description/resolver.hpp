#pragma once
#include "horus/description/compiler.hpp"
namespace horus::description {
// Local files work without ROS; topic/parameter sources require the ROS build.
Artifact resolve(const nlohmann::json &options);
} // namespace horus::description
