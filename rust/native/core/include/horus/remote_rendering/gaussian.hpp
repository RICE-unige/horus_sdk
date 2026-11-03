#pragma once
#include "gpu.hpp"

namespace horus::remote_rendering {
struct GaussianLoadOptions {
    double world_scale{1};
    bool canonicalize_y_down{false};
    double floor_quantile{0.995};
    std::uint64_t maximum_gaussians{20'000'000};
    std::uint64_t maximum_decoded_bytes{8ull * 1024 * 1024 * 1024};
};
struct GaussianScene {
    std::vector<Point3f> means, scales;
    std::vector<std::array<float, 4>> quaternions; // wxyz, unlike EyeView's xyzw.
    std::vector<float> opacities;
    std::vector<Point3f> spherical_harmonics; // Gaussian, coefficient, RGB.
    std::uint32_t sh_degree{};
    std::size_t discarded_nonfinite{};
    Vector3 source_from_world_translation, world_bounds_min, world_bounds_max;
    Quaternion source_from_world_rotation;
    EyeView world_to_source(const EyeView &) const;
};
GaussianScene load_gaussian_ply(const std::string &, const GaussianLoadOptions & = {});

// Uses an explicit native gsplat plugin; no Python, hidden downloads or point fallback.
class GaussianRenderer {
    friend class GpuVideoEncoder;
    struct Impl;
    std::unique_ptr<Impl> impl_;
    void render_impl(std::span<const EyeView>, const GpuRenderOptions &, std::span<std::uint8_t>,
                     std::span<float>, float &, bool device);

  public:
    GaussianRenderer(const std::string &library, const GaussianScene &);
    ~GaussianRenderer();
    GaussianRenderer(const GaussianRenderer &) = delete;
    GaussianRenderer &operator=(const GaussianRenderer &) = delete;
    RenderedViews render_views(std::span<const EyeView>, const GpuRenderOptions & = {});
    void render_into(std::span<const EyeView>, const GpuRenderOptions &, std::span<std::uint8_t>,
                     std::span<float>, float &elapsed_ms);
};
} // namespace horus::remote_rendering
