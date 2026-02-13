#ifndef HORUS_ROBOT_TELEOP_HPP
#define HORUS_ROBOT_TELEOP_HPP

namespace horus {
namespace robot {

struct TeleopState {
    bool enabled{false};
    double linear_speed{0.0};
    double angular_speed{0.0};
};

} // namespace robot
} // namespace horus

#endif // HORUS_ROBOT_TELEOP_HPP

