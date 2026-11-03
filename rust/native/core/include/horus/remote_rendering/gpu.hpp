#pragma once
#include "protocol.hpp"
#include <memory>

namespace horus::remote_rendering {
using Point3f = std::array<float, 3>;
using Rgb8 = std::array<std::uint8_t, 3>;
struct MeshInput {
    std::span<const Point3f> vertices;
    std::span<const std::array<std::uint32_t, 3>> faces;
    std::span<const Rgb8> colors; // One tint per triangle, as in Python's CUDA backend.
    std::span<const std::array<float, 2>> uvs;
    std::span<const std::array<std::uint32_t, 4>> texture_rects;
    std::span<const Rgb8> texture_atlas;
    std::uint32_t texture_width{1}, texture_height{1};
};
struct RenderedViews {
    std::uint32_t width{}, height{}, view_count{};
    std::vector<std::uint8_t> rgb;
    std::vector<float> depth;
    float cuda_ms{};
};
struct GpuRenderOptions {
    std::uint32_t width{640}, height{480};
    float near_m{0.05f}, far_m{100.0f};
    int point_radius{1};
};
// Load an explicitly selected HORUS CUDA library. No Python process, CUDA
// installation, network access or hidden runtime compilation is performed.
class CudaRenderer {
    friend class GpuVideoEncoder;
    struct Impl;
    std::unique_ptr<Impl> impl_;
    void render_impl(std::span<const EyeView>, const GpuRenderOptions &,
                     std::span<std::uint8_t>, std::span<float>, float &, bool device);

  public:
    CudaRenderer(const std::string &point_library, std::size_t point_count);
    CudaRenderer(const std::string &mesh_library, const MeshInput &);
    ~CudaRenderer();
    CudaRenderer(const CudaRenderer &) = delete;
    CudaRenderer &operator=(const CudaRenderer &) = delete;
    void upload(std::span<const Point3f>, std::span<const Rgb8>, std::size_t offset = 0);
    RenderedViews render_views(std::span<const EyeView>, const GpuRenderOptions & = {});
    void render_into(std::span<const EyeView>, const GpuRenderOptions &,
                     std::span<std::uint8_t> rgb, std::span<float> depth, float &elapsed_ms);
};
} // namespace horus::remote_rendering
