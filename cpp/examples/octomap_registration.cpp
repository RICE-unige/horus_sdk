#include "registration_common.hpp"

int main() {
    auto set = examples::robot_description_fleet();
    set.datavizs.front().add_3d_octomap(
        "/map_3d_octomap_mesh",
        "map",
        {{"render_mode", std::string("surface_mesh")}, {"native_topic", std::string("/map_3d_octomap")}});
    return examples::register_or_fail(set);
}
