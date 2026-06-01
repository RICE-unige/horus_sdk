#include "registration_common.hpp"

int main() {
    auto set = examples::robot_description_fleet();
    set.datavizs.front().add_occupancy_grid(
        "/map",
        "map",
        {{"color_free", std::string("#222222")}, {"color_occupied", std::string("#F4F4F4")}, {"alpha", 0.85}});
    return examples::register_or_fail(set);
}
