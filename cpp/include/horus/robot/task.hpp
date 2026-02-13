#ifndef HORUS_ROBOT_TASK_HPP
#define HORUS_ROBOT_TASK_HPP

#include <string>

namespace horus {
namespace robot {

struct RobotTask {
    std::string id;
    std::string type;
    std::string description;
};

} // namespace robot
} // namespace horus

#endif // HORUS_ROBOT_TASK_HPP

