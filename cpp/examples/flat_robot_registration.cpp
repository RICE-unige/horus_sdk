#include "registration_common.hpp"

int main() {
    examples::RegistrationSet set;
    set.robots.push_back(examples::standard_robot(
        "flat_bot",
        horus::core::RobotType::WHEELED,
        {0.8, 0.55, 0.45},
        "wheeled"));
    set.robots.back().configure_ros_binding("flat", "flat", "base_link");
    set.datavizs.push_back(examples::standard_dataviz(set.robots.back()));
    return examples::register_or_fail(set);
}
