#include "horus/bridge/robot_registry.hpp"
#include "horus/robot/robot.hpp"
#include "horus/robot/sensors.hpp"

#include <chrono>
#include <iostream>
#include <memory>

int main() {
    constexpr int kIterations = 1000;
    horus::bridge::RobotRegistryClient client;

    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < kIterations; ++i) {
        horus::robot::Robot robot("bench_robot", horus::core::RobotType::WHEELED);
        auto camera =
            std::make_shared<horus::robot::Camera>("cam", "camera_link", "/camera/image_raw");
        robot.add_sensor(camera);
        auto dataviz = robot.create_dataviz();
        (void)client.build_robot_config_dict(robot, *dataviz);
    }
    const auto end = std::chrono::steady_clock::now();
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << "payload_micro iterations=" << kIterations
              << " elapsed_ms=" << elapsed_ms << std::endl;
    return 0;
}

