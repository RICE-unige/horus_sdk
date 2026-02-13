#ifndef HORUS_PLUGINS_ROSBOT_HPP
#define HORUS_PLUGINS_ROSBOT_HPP

#include "horus/robot/robot.hpp"
#include <memory>
#include <string>

namespace horus {
namespace plugins {

std::shared_ptr<robot::Robot> make_rosbot(const std::string& name = "rosbot");

} // namespace plugins
} // namespace horus

#endif // HORUS_PLUGINS_ROSBOT_HPP

