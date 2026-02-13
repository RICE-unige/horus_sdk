#include "horus/utils/logging.hpp"

#include <chrono>
#include <optional>
#include <string>
#include <thread>

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
    const auto robot_name = get_arg(argc, argv, "--robot-name").value_or("test_bot");
    const auto rate_hz =
        get_arg(argc, argv, "--rate-hz").has_value() ? std::stod(*get_arg(argc, argv, "--rate-hz")) : 10.0;
    const auto cycles =
        get_arg(argc, argv, "--cycles").has_value() ? std::stoi(*get_arg(argc, argv, "--cycles")) : 20;

    const auto period =
        std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(0.1, rate_hz)));

    horus::utils::print_info(
        "Publishing fake TF for " + robot_name + " at " + std::to_string(rate_hz) + " Hz");
    for (int i = 0; i < cycles; ++i) {
        horus::utils::print_info(
            "[fake_tf] robot=" + robot_name + " frame=" + robot_name + "/base_link seq=" +
            std::to_string(i + 1));
        std::this_thread::sleep_for(period);
    }
    return 0;
}

