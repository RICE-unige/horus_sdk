#include "horus/bridge/robot_registry.hpp"
#include "horus/robot/robot.hpp"
#include "horus/robot/sensors.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

int main() {
    constexpr int kRobots = 10;
    horus::bridge::RobotRegistryClient client;

    std::vector<horus::robot::Robot> robots;
    std::vector<horus::robot::DataViz> datavizs;
    robots.reserve(kRobots);
    datavizs.reserve(kRobots);

    for (int i = 0; i < kRobots; ++i) {
        robots.emplace_back("robot_" + std::to_string(i), horus::core::RobotType::WHEELED);
        auto camera = std::make_shared<horus::robot::Camera>(
            "camera_" + std::to_string(i),
            "camera_link",
            "/robot_" + std::to_string(i) + "/camera/image_raw");
        robots.back().add_sensor(camera);
        auto dataviz = robots.back().create_dataviz();
        dataviz->add_occupancy_grid("/map", "map");
        datavizs.push_back(*dataviz);
    }

    std::vector<horus::robot::Robot*> robot_ptrs;
    robot_ptrs.reserve(robots.size());
    for (auto& robot : robots) {
        robot_ptrs.push_back(&robot);
    }

    const auto start = std::chrono::steady_clock::now();
    auto [ok, _] = client.register_robots(
        robot_ptrs,
        datavizs,
        10.0,
        false,
        false,
        std::nullopt);
    const auto end = std::chrono::steady_clock::now();
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << "registration_scenario robots=" << kRobots
              << " ok=" << (ok ? "true" : "false")
              << " elapsed_ms=" << elapsed_ms << std::endl;
    return ok ? 0 : 1;
}

