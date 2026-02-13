#include "horus/robot/robot.hpp"
#include "horus/robot/sensors.hpp"
#include "horus/utils/logging.hpp"

#include <memory>
#include <optional>
#include <string>

namespace {
std::optional<std::string> get_arg(int argc, char** argv, const std::string& name) {
    for (int i = 1; i < argc - 1; ++i) {
        if (argv[i] == name) {
            return std::string(argv[i + 1]);
        }
    }
    return std::nullopt;
}
} // namespace

int main(int argc, char** argv) {
    const auto robot_name = get_arg(argc, argv, "--robot-name").value_or("SdkBot_E2E");

    horus::robot::Robot robot(robot_name, horus::core::RobotType::WHEELED);
    auto lidar = std::make_shared<horus::robot::LaserScan>(
        "Front Lidar",
        "laser_frame",
        "/scan",
        -3.14159f,
        3.14159f,
        0.005f,
        0.1f,
        12.0f,
        0.01f,
        "#00FFFF",
        0.1f);
    robot.add_sensor(lidar);

    auto dataviz = robot.create_dataviz();
    auto [ok, result] = robot.register_with_horus(dataviz, false, false);
    if (!ok) {
        horus::utils::print_error("Registration failed");
        return 1;
    }
    horus::utils::print_success("Registration OK");

    if (!robot.is_registered_with_horus()) {
        horus::utils::print_error("Robot registration metadata missing");
        return 2;
    }

    const auto robot_id = robot.get_horus_id().value_or("");
    horus::utils::print_info("Robot ID: " + robot_id);
    return 0;
}

