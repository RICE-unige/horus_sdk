// Cross-language serializer throughput benchmark (C++).
//
// Measures how many registration payloads per second the C++ SDK can build,
// single-threaded and across all hardware threads. The native SDKs have no
// global interpreter lock, so payload building scales with cores -- the key
// advantage over the Python reference. Run with an optional payload count:
//
//     ./throughput_benchmark [count]

#include "horus/bridge/robot_registry.hpp"
#include "horus/robot/robot.hpp"
#include "horus/robot/sensors.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Workload {
    horus::robot::Robot robot;
    std::shared_ptr<horus::robot::DataViz> dataviz;
    horus::bridge::RobotRegistryClient client;
};

Workload make_workload(int idx) {
    horus::robot::Robot robot("robot_" + std::to_string(idx), horus::core::RobotType::WHEELED);
    auto camera = std::make_shared<horus::robot::Camera>(
        "camera_" + std::to_string(idx),
        "camera_link",
        "/robot_" + std::to_string(idx) + "/camera/image_raw/compressed");
    robot.add_sensor(camera);
    auto dataviz = robot.create_dataviz();
    return Workload{std::move(robot), dataviz, horus::bridge::RobotRegistryClient{}};
}

double per_sec(std::size_t count, std::chrono::steady_clock::duration elapsed) {
    const double secs = std::chrono::duration<double>(elapsed).count();
    return secs > 0.0 ? static_cast<double>(count) / secs : 0.0;
}

}  // namespace

int main(int argc, char** argv) {
    const std::size_t total = argc > 1 ? std::stoul(argv[1]) : 50000;
    unsigned threads = std::thread::hardware_concurrency();
    if (threads == 0) {
        threads = 4;
    }

    std::uint64_t sink = 0;

    // Single-threaded.
    Workload single_workload = make_workload(0);
    (void)single_workload.client.build_robot_config_dict(single_workload.robot, *single_workload.dataviz);
    const auto s0 = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < total; ++i) {
        auto payload = single_workload.client.build_robot_config_dict(
            single_workload.robot, *single_workload.dataviz);
        sink += payload.sensors.size();
    }
    const double single = per_sec(total, std::chrono::steady_clock::now() - s0);

    // Multi-threaded: per-thread client + workload, total work split across cores.
    std::vector<Workload> workloads;
    workloads.reserve(threads);
    for (unsigned t = 0; t < threads; ++t) {
        workloads.push_back(make_workload(static_cast<int>(t + 1)));
    }
    const std::size_t per_thread = total / threads;
    std::vector<std::uint64_t> partials(threads, 0);

    std::vector<std::thread> pool;
    pool.reserve(threads);
    const auto m0 = std::chrono::steady_clock::now();
    for (unsigned t = 0; t < threads; ++t) {
        pool.emplace_back([&, t]() {
            std::uint64_t local = 0;
            for (std::size_t i = 0; i < per_thread; ++i) {
                auto payload = workloads[t].client.build_robot_config_dict(
                    workloads[t].robot, *workloads[t].dataviz);
                local += payload.sensors.size();
            }
            partials[t] = local;
        });
    }
    for (auto& worker : pool) {
        worker.join();
    }
    const double multi = per_sec(per_thread * threads, std::chrono::steady_clock::now() - m0);
    for (const auto value : partials) {
        sink += value;
    }

    std::cout << "cpp serializer throughput: single=" << static_cast<long long>(single)
              << " payloads/s, multi(" << threads
              << " threads)=" << static_cast<long long>(multi)
              << " payloads/s, speedup=" << (single > 0.0 ? multi / single : 0.0) << "x"
              << " (sink=" << sink << ")" << std::endl;
    return 0;
}
