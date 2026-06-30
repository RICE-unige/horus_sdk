#include "horus/bridge/robot_registry.hpp"
#include "horus/description/mesh_baker.hpp"
#include "horus/robot/robot.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <any>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace desc = horus::description;

namespace {

const float CORNERS[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                             {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};

struct Tri {
    float n[3];
    int idx[3];
};

const Tri TRIS[12] = {
    {{0, 0, -1}, {0, 1, 2}}, {{0, 0, -1}, {0, 2, 3}}, {{0, 0, 1}, {4, 6, 5}},
    {{0, 0, 1}, {4, 7, 6}},  {{0, -1, 0}, {0, 5, 1}}, {{0, -1, 0}, {0, 4, 5}},
    {{0, 1, 0}, {3, 2, 6}},  {{0, 1, 0}, {3, 6, 7}},  {{-1, 0, 0}, {0, 3, 7}},
    {{-1, 0, 0}, {0, 7, 4}}, {{1, 0, 0}, {1, 5, 6}},  {{1, 0, 0}, {1, 6, 2}}};

void push_f32(std::vector<std::uint8_t>& out, float value) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, 4);
    out.push_back(static_cast<std::uint8_t>(bits & 0xFF));
    out.push_back(static_cast<std::uint8_t>((bits >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((bits >> 16) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((bits >> 24) & 0xFF));
}

std::vector<std::uint8_t> cube_binary_stl() {
    std::vector<std::uint8_t> out(80, 0);
    const std::uint32_t count = 12;
    out.push_back(count & 0xFF);
    out.push_back((count >> 8) & 0xFF);
    out.push_back((count >> 16) & 0xFF);
    out.push_back((count >> 24) & 0xFF);
    for (const auto& tri : TRIS) {
        for (int k = 0; k < 3; ++k) {
            push_f32(out, tri.n[k]);
        }
        for (int v = 0; v < 3; ++v) {
            for (int k = 0; k < 3; ++k) {
                push_f32(out, CORNERS[tri.idx[v]][k]);
            }
        }
        out.push_back(0);
        out.push_back(0);
    }
    return out;
}

std::string cube_ascii_stl() {
    std::ostringstream o;
    o << "solid cube\n";
    for (const auto& tri : TRIS) {
        o << "facet normal " << tri.n[0] << " " << tri.n[1] << " " << tri.n[2] << "\nouter loop\n";
        for (int v = 0; v < 3; ++v) {
            o << "vertex " << CORNERS[tri.idx[v]][0] << " " << CORNERS[tri.idx[v]][1] << " "
              << CORNERS[tri.idx[v]][2] << "\n";
        }
        o << "endloop\nendfacet\n";
    }
    o << "endsolid cube\n";
    return o.str();
}

void write_file(const std::filesystem::path& path, const std::string& content) {
    std::ofstream f(path, std::ios::binary);
    f.write(content.data(), static_cast<std::streamsize>(content.size()));
}

void write_file(const std::filesystem::path& path, const std::vector<std::uint8_t>& bytes) {
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

}  // namespace

int main() {
    // base64 reference vectors.
    assert(desc::base64_encode(std::vector<std::uint8_t>{'a', 'b', 'c'}) == "YWJj");
    assert(desc::base64_encode(std::vector<std::uint8_t>{'a', 'b'}) == "YWI=");
    assert(desc::base64_encode(std::vector<std::uint8_t>{'a'}) == "YQ==");

    const auto dir = std::filesystem::temp_directory_path() / "horus_cpp_stl";
    std::filesystem::create_directories(dir);
    const auto bin_path = dir / "cube_bin.stl";
    const auto ascii_path = dir / "cube_ascii.stl";
    write_file(bin_path, cube_binary_stl());
    write_file(ascii_path, cube_ascii_stl());

    const auto bin_asset = desc::bake_stl_file(bin_path.string(), "cube").value();
    const auto ascii_asset = desc::bake_stl_file(ascii_path.string(), "cube").value();
    assert(bin_asset.triangle_count == 12);
    assert(bin_asset.vertex_count == 24);
    assert(bin_asset.bounds_min == (std::array<float, 3>{{0.0f, 0.0f, 0.0f}}));
    assert(bin_asset.bounds_max == (std::array<float, 3>{{1.0f, 1.0f, 1.0f}}));
    // Same geometry, identical baked buffers regardless of STL encoding.
    assert(bin_asset.positions_b64 == ascii_asset.positions_b64);
    assert(bin_asset.indices_b64 == ascii_asset.indices_b64);

    // Baked into the registration manifest via configure_robot_description.
    write_file(dir / "cube.stl", cube_binary_stl());
    const auto urdf_path = dir / "robot.urdf";
    write_file(urdf_path,
               std::string(R"(<robot name="m"><link name="base_link"><visual><geometry>)") +
                   R"(<mesh filename="cube.stl"/></geometry></visual></link></robot>)");

    horus::robot::Robot robot("mesh_bot", horus::core::RobotType::WHEELED);
    robot.configure_robot_description({
        .urdf_path = urdf_path.string(),
        .base_frame = "base_link",
        .body_mesh_mode = "runtime_high_mesh",
    });
    auto dataviz = robot.create_dataviz();
    horus::bridge::RobotRegistryClient client;
    auto payload = client.build_robot_config_dict(robot, *dataviz);
    const auto& manifest = payload.robot_description_manifest;

    assert(std::any_cast<bool>(manifest.at("supports_visual_meshes")) == true);
    assert(std::any_cast<int>(manifest.at("mesh_asset_count")) == 1);
    assert(std::any_cast<int>(manifest.at("mesh_asset_encoded_bytes")) > 0);
    assert(payload.robot_description_payload_json.has_value());
    assert(payload.robot_description_payload_json->find("mesh_assets") != std::string::npos);

    // collision_only skips baking.
    horus::robot::Robot collision_robot("collision_bot", horus::core::RobotType::WHEELED);
    collision_robot.configure_robot_description({
        .urdf_path = urdf_path.string(),
        .base_frame = "base_link",
        .body_mesh_mode = "collision_only",
    });
    auto collision_dataviz = collision_robot.create_dataviz();
    auto collision_payload = client.build_robot_config_dict(collision_robot, *collision_dataviz);
    assert(std::any_cast<bool>(collision_payload.robot_description_manifest.at("supports_visual_meshes")) == false);

    std::filesystem::remove_all(dir);
    std::cout << "cpp_stl_mesh_tests passed" << std::endl;
    return 0;
}
