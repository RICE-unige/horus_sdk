#pragma once
#include "horus/mapping/mesh_source.hpp"
#include <nlohmann/json.hpp>

namespace horus::mapping {
struct MeshSourceAdapterOptions {
    std::string input_topic, input_type{"Marker"}, output_topic{"/horus/maps/live"};
    MeshSourceOptions source;
    double pages_per_second{60}, bytes_per_second{4 * 1024 * 1024};
    bool use_sim_time{false};
    std::string color_mode{"auto"};
    std::array<float, 4> fallback_color{.70f, .72f, .78f, 1.f};
};
// ROS Marker/MarkerArray/nvblox source, TF normalization, full-geometry spatial
// paging, unchanged-page suppression, bounded queue, rate limiting and repair.
class MeshSourceAdapter {
  public:
    explicit MeshSourceAdapter(const MeshSourceAdapterOptions &);
    ~MeshSourceAdapter();
    MeshSourceAdapter(const MeshSourceAdapter &) = delete;
    MeshSourceAdapter &operator=(const MeshSourceAdapter &) = delete;
    nlohmann::json status() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace horus::mapping
