#pragma once
#include "scene.hpp"

namespace horus::remote_rendering {
// Catalog entries retain source/attribution information. Resolving never downloads.
Json scene_catalog(const std::filesystem::path &cache_root = {});
std::filesystem::path resolve_scene_asset(const std::string &scene_id,
                                          const std::filesystem::path &path = {},
                                          const std::filesystem::path &cache_root = {});
Json prepare_eth3d_scene(const std::string &scene_id, const std::filesystem::path &path = {},
                         double dataset_scale = 1.0, const std::filesystem::path &cache_root = {});
SceneAsset prepare_cow_lady_scene(const std::filesystem::path &, double world_scale = 1.0);

// Binary PLY/HRL streaming uses bounded reusable input storage. Each returned
// chunk owns its finite points; originals are never simplified or overwritten.
class PointChunkStream {
    struct Impl;
    std::unique_ptr<Impl> impl_;

  public:
    PointChunkStream(const Json &prepared_scene, std::size_t chunk_size = 1000000);
    PointChunkStream(const std::filesystem::path &ply_or_hrl, std::size_t chunk_size = 1000000);
    ~PointChunkStream();
    PointChunkStream(const PointChunkStream &) = delete;
    std::optional<SceneAsset> next();
};
// Explicit opt-in LOD export; retains the first original colored point per voxel,
// sorts voxel keys deterministically and writes the Python-compatible HRL1 format.
std::uint64_t write_point_lod(PointChunkStream &, const std::filesystem::path &output,
                              float voxel_size_m = .0125f, std::size_t maximum_voxels = 20000000);
} // namespace horus::remote_rendering
