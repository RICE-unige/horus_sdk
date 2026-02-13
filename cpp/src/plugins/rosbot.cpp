#include "horus/plugins/rosbot.hpp"

#include "horus/robot/sensors.hpp"

namespace horus {
namespace plugins {

std::shared_ptr<robot::Robot> make_rosbot(const std::string& name) {
    auto robot = std::make_shared<robot::Robot>(name, core::RobotType::WHEELED);
    robot->add_sensor(std::make_shared<robot::LaserScan>(
        "front_laser",
        name + "/laser_link",
        "/" + name + "/scan"));
    return robot;
}

} // namespace plugins
} // namespace horus

