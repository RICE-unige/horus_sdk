#ifndef HORUS_DESCRIPTION_MESH_BAKER_HPP
#define HORUS_DESCRIPTION_MESH_BAKER_HPP

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace horus {
namespace description {

// A baked, indexed mesh ready for the registration payload. Mirrors the Python
// RobotDescriptionModels.MeshAsset and the Rust description::MeshAsset.
struct MeshAsset {
    std::string mesh_id;
    std::size_t vertex_count{0};
    std::size_t triangle_count{0};
    std::string positions_b64;
    std::string normals_b64;
    std::string indices_b64;
    std::array<float, 3> bounds_min{{0.0f, 0.0f, 0.0f}};
    std::array<float, 3> bounds_max{{0.0f, 0.0f, 0.0f}};

    std::size_t encoded_bytes() const {
        return positions_b64.size() + normals_b64.size() + indices_b64.size();
    }
};

// Standard base64 encoding (matches Python base64.b64encode).
std::string base64_encode(const std::vector<std::uint8_t>& data);

// Parse and bake an STL file (binary or ASCII) into a MeshAsset.
std::optional<MeshAsset> bake_stl_file(const std::string& path, const std::string& mesh_id);

// Bake every resolvable STL mesh referenced by the URDF's visual elements.
std::vector<MeshAsset> bake_visual_meshes(const std::string& urdf, const std::string& urdf_path);

}  // namespace description
}  // namespace horus

#endif  // HORUS_DESCRIPTION_MESH_BAKER_HPP
