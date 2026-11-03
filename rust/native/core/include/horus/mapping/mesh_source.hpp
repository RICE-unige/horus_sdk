#pragma once
#include "horus/mapping/live_mesh.hpp"
#include <array>
#include <functional>
#include <map>
#include <memory>

namespace horus::mapping {
struct MeshPose {
    std::array<double, 3> translation{0, 0, 0};
    std::array<double, 4> rotation{0, 0, 0, 1}; // xyzw
};
struct TriangleMarker {
    std::string name_space, frame;
    std::int32_t id{0}, action{0}, type{11};
    std::int64_t stamp_ns{0};
    bool frame_locked{false};
    double lifetime_seconds{0};
    MeshPose pose;
    std::array<double, 3> scale{1, 1, 1};
    std::array<std::uint8_t, 4> color{255, 255, 255, 255};
    std::vector<std::array<double, 3>> points;
    std::vector<std::array<std::uint8_t, 4>> colors;
};
using MeshTransformLookup = std::function<MeshPose(const std::string &, std::int64_t)>;
struct MeshSourceOptions {
    LiveMeshOptions map;
    std::size_t triangles_per_page{4096}, max_input_points{3000000};
    unsigned blocks_per_axis{2};
};
struct NvbloxBlockUpdate {
    std::array<std::int32_t, 3> index;
    std::shared_ptr<const BlockGeometry> geometry; // null deletes this block
};
// Caller-serialized transactional ingestion. Missing source keys are unchanged.
// Both sources retain full geometry and feed the same bounded live-map cache.
class MeshSourceState {
  public:
    explicit MeshSourceState(MeshSourceOptions options = {});
    std::vector<MeshPage> apply_markers(std::span<const TriangleMarker> markers, double now,
                                        const MeshTransformLookup &lookup = {});
    std::vector<MeshPage> apply_blocks(std::span<const NvbloxBlockUpdate> blocks, bool clear,
                                       const std::string &frame, std::int64_t stamp_ns,
                                       const MeshTransformLookup &lookup = {});
    std::vector<MeshPage> expire(double now);
    const LiveMeshState &state() const { return map_; }
    std::size_t source_bytes() const { return source_bytes_; }

  private:
    MeshSourceOptions options_;
    LiveMeshState map_;
    using MarkerKey = std::pair<std::string, std::int32_t>;
    using Key = std::array<std::int32_t, 3>;
    std::map<MarkerKey, std::size_t> parts_;
    std::map<MarkerKey, double> expiry_;
    std::map<Key, std::shared_ptr<const BlockGeometry>> blocks_;
    std::map<Key, std::set<Key>> page_blocks_;
    std::string block_frame_;
    std::size_t source_bytes_{0};
};
} // namespace horus::mapping
