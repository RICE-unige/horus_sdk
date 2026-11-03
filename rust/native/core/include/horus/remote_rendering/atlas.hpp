#pragma once
#include "datasets.hpp"
#include "stereo.hpp"
namespace horus::remote_rendering {
struct RemoteRenderViewSpec {
    std::string name;
    std::array<double, 3> position{};
    double yaw_degrees = 0, pitch_degrees = 0, atlas_x = 0, atlas_y = 0, atlas_width = 1,
           atlas_height = 1;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(RemoteRenderViewSpec, name, position, yaw_degrees,
                                                pitch_degrees, atlas_x, atlas_y, atlas_width,
                                                atlas_height)
struct AtlasOptions {
    uint32_t width = 640, height = 480;
    int point_radius = 1, hole_fill_iterations = 2;
    double world_scale = 1, near_base = .3, far_base = 20, vertical_fov_degrees = 52;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(AtlasOptions, width, height, point_radius,
                                                hole_fill_iterations, world_scale, near_base,
                                                far_base, vertical_fov_degrees)
RenderedViews render_multiview_atlas(std::span<const Point3f>, std::span<const Rgb8>,
                                     std::span<const RemoteRenderViewSpec>,
                                     const AtlasOptions & = {});
// Streams every source point through a bounded raster target; does not retain the full cloud.
RenderedViews render_multiview_atlas(PointChunkStream &, std::span<const RemoteRenderViewSpec>,
                                     const AtlasOptions & = {});
SceneAsset unproject_multiview_surfels(const RenderedViews &, std::span<const RemoteRenderViewSpec>,
                                       uint32_t columns, uint32_t rows, double world_scale = 1,
                                       double vertical_fov_degrees = 52);
std::vector<uint8_t> pack_atlas_rgbd(const RenderedViews &, double near_m, double far_m);
} // namespace horus::remote_rendering
