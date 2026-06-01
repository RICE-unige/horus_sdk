#include "registration_common.hpp"

int main() {
    auto set = examples::robot_description_fleet();
    set.datavizs.front().add_3d_mesh(
        "/map_3d_mesh",
        "map",
        {{"max_triangles", 60000}, {"use_vertex_colors", true}});
    return examples::register_or_fail(set);
}
