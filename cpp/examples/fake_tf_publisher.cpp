#include "horus/tf.hpp"
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <map>
#include <thread>

namespace {
volatile std::sig_atomic_t stopped = 0;
void stop(int) { stopped = 1; }
} // namespace
int main(int argc, char **argv) {
    try {
        std::map<std::string, std::string> args{
            {"--robot-name", "test_bot"}, {"--rate-hz", "10"}, {"--cycles", "20"}};
        for (int i = 1; i < argc; i += 2) {
            if (!args.count(argv[i]) || i + 1 == argc)
                throw std::invalid_argument(
                    "Usage: fake_tf_publisher [--robot-name NAME] [--rate-hz HZ] [--cycles N]");
            args[argv[i]] = argv[i + 1];
        }
        const auto name = args.at("--robot-name");
        size_t rate_end = 0, count_end = 0;
        const double rate = std::stod(args.at("--rate-hz"), &rate_end);
        const int cycles = std::stoi(args.at("--cycles"), &count_end);
        if (rate_end != args.at("--rate-hz").size() || count_end != args.at("--cycles").size() ||
            !std::isfinite(rate) || rate < 0.1 || rate > 240 || cycles <= 0 || name.empty() ||
            name.find_first_not_of(
                "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") !=
                std::string::npos)
            throw std::invalid_argument(
                "Use an alphanumeric robot name, rate 0.1..240 and a positive cycle count");
        horus::tf::Publisher publisher;
        std::signal(SIGINT, stop);
        std::signal(SIGTERM, stop);
        horus::tf::Transform camera{name + "/base_link", name + "/camera_link", {0.3, 0, 0.5}};
        horus::tf::Transform lidar{name + "/base_link", name + "/laser_link", {0, 0, 0.4}};
        publisher.publish(std::span(&camera, 1), true);
        publisher.publish(std::span(&lidar, 1), true);
        const auto start = std::chrono::steady_clock::now();
        int published = 0;
        for (int i = 0; i < cycles && !stopped; ++i) {
            const double angle = i / rate * 0.3;
            horus::tf::Transform base{"map",
                                      name + "/base_link",
                                      {std::cos(angle), std::sin(angle), 0.2},
                                      {0, 0, std::sin(angle / 2), std::cos(angle / 2)}};
            publisher.publish(std::span(&base, 1));
            ++published;
            const auto deadline = start + std::chrono::duration<double>((i + 1) / rate);
            while (!stopped && std::chrono::steady_clock::now() < deadline)
                std::this_thread::sleep_for(std::min<std::chrono::duration<double>>(
                    std::chrono::duration<double>(0.02),
                    deadline - std::chrono::steady_clock::now()));
        }
        std::cout << "Published " << published << " TF updates for " << name
                  << " on /tf with two retained frames on /tf_static\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
