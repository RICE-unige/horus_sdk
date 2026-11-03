#include "horus/description/compiler.hpp"
#include "horus/teaching/config.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using Json = nlohmann::json;
bool near(double a, double b) { return std::abs(a - b) < 1e-6; }
template <class F> void rejected(F f) {
    bool bad = false;
    try {
        f();
    } catch (const std::exception &) {
        bad = true;
    }
    assert(bad);
}
int main() {
    const Json config = {{"urdf_path", HORUS_MODEL_FIXTURE},
                         {"base_frame", "base"},
                         {"visual_link_pose_source", "tf"},
                         {"body_mesh_mode", "runtime_high_mesh"}};
    const auto result = horus::description::compile_file(config);
    const auto &p = result.payload;
    assert(p["links"].size() == 3 && p["joints"].size() == 2);
    assert(p["visual_links"].size() == 2 && p["mesh_assets"].size() == 2);
    assert(p["visual_links"][0]["frame_id"] == "base");
    assert(p["mesh_assets"][0]["triangle_count"] == 2);
    assert(near(p["visual_links"][1]["origin_xyz"][0], -1));
    assert(near(p["visual_links"][1]["origin_xyz"][1], 0));
    assert(near(p["visual_links"][1]["origin_rpy"][2], M_PI / 2));
    assert(p["joints"][1]["axis_xyz"] == Json::array({0, 1, 0}));
    assert(near(p["links"][2]["collisions"][0]["origin_xyz"][0], -1));
    assert(near(p["links"][2]["collisions"][0]["origin_xyz"][2], 0.2));
    assert(result.manifest["description_id"] ==
           "sha256:" + horus::teaching::sha256(result.payload_json));
    assert(result.manifest["collision_count"] == 2 && !result.encoded_payload.empty());
    auto from_leg = config;
    from_leg["base_frame"] = "leg";
    const auto rooted = horus::description::compile_file(from_leg);
    assert(near(rooted.payload["visual_links"][1]["origin_xyz"][0], 0));
    assert(near(rooted.payload["visual_links"][0]["origin_xyz"][1], -1));
    auto collision_only = config;
    collision_only["body_mesh_mode"] = "collision_only";
    assert(horus::description::compile_file(collision_only).payload["mesh_assets"].empty());
    rejected([&] {
        auto budget = config;
        budget["max_source_triangles"] = 1;
        horus::description::compile_file(budget);
    });
    rejected([&] {
        horus::description::compile_urdf("<robot><link name='a'/><link name='b'/></robot>",
                                         Json::object());
    });
    rejected([&] {
        horus::description::compile_urdf("<robot><link name='a'/><link name='a'/></robot>",
                                         Json::object());
    });
    rejected([&] {
        horus::description::compile_urdf("<robot><link name='a'/><joint name='bad'><parent "
                                         "link='a'/><child link='a'/></joint></robot>",
                                         Json::object());
    });
    rejected([&] {
        horus::description::compile_urdf("<robot><link name='a'><collision><geometry><sphere "
                                         "radius='nan'/></geometry></collision></link></robot>",
                                         Json::object());
    });
    std::cout << "Native model transforms, rigid grouping, collision poses and validation passed\n";
}
