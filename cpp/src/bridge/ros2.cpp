#include "horus/bridge/ros2.hpp"

namespace horus {
namespace bridge {

bool Ros2Bridge::available() const {
#if defined(HORUS_ROS2_ENABLED)
    return true;
#else
    return false;
#endif
}

} // namespace bridge
} // namespace horus

