#ifndef HORUS_BRIDGE_ROS2_HPP
#define HORUS_BRIDGE_ROS2_HPP

#include <string>

namespace horus {
namespace bridge {

class Ros2Bridge {
public:
    bool available() const;
    std::string transport_name() const { return "ros2"; }
};

} // namespace bridge
} // namespace horus

#endif // HORUS_BRIDGE_ROS2_HPP

