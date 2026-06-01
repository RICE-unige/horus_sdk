#include "registration_common.hpp"

int main() {
    auto set = examples::standard_fleet(
        {"splat_rover_1", "splat_rover_2"},
        horus::core::RobotType::WHEELED,
        {0.70, 0.48, 0.30},
        "wheeled");
    for (auto& robot : set.robots) {
        robot.configure_robot_manager({.enabled = true, .status = true, .data_viz = true, .teleop = false, .tasks = false});
    }
    set.datavizs.front().add_gaussian_splat_map(
        "/horus/gaussian_splat/manifest",
        "map",
        {{"max_splats", 350000}, {"render_mode", std::string("splats")}},
        "/map_gaussian_splat_preview");
    return examples::register_or_fail(set);
}
