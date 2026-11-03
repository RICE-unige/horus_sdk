#pragma once
#include "gpu.hpp"
#include <filesystem>

namespace horus::remote_rendering {
struct SceneLoadOptions {
    double world_scale{1.0};
    std::uint64_t maximum_vertices{100000000}, maximum_triangles{50000000};
    std::uint64_t maximum_texture_pixels{64 * 1024 * 1024};
    bool bake_lighting{true};
};
struct SceneAsset {
    std::vector<Point3f> vertices;
    std::vector<Rgb8> point_colors, face_colors, texture_atlas;
    std::vector<std::array<std::uint32_t, 3>> faces;
    std::vector<std::array<float, 2>> uvs;
    std::vector<std::array<std::uint32_t, 4>> texture_rects;
    std::uint32_t texture_width{1}, texture_height{1};
    MeshInput mesh_input() const;
};
// Full scene import: node transforms, indexed triangles, material textures/tints.
// Point-only PLY is supported. No simplification, implicit units or network fetches.
SceneAsset load_scene(const std::filesystem::path &, const SceneLoadOptions & = {});
} // namespace horus::remote_rendering
