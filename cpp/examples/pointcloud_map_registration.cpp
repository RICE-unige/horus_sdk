#include "registration_common.hpp"

int main() {
    auto set = examples::robot_description_fleet();
    set.datavizs.front().add_3d_map(
        "/map_3d",
        "map",
        {{"point_size", 0.035}, {"render_all_points", true}, {"point_shape", std::string("circle")}});
    return examples::register_or_fail(set);
}
