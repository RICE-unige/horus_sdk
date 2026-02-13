#ifndef HORUS_ROBOT_STATUS_HPP
#define HORUS_ROBOT_STATUS_HPP

#include <string>

namespace horus {
namespace robot {

struct RobotStatus {
    std::string state{"idle"};
    std::string message;
};

} // namespace robot
} // namespace horus

#endif // HORUS_ROBOT_STATUS_HPP

