#include "horus/remote_rendering/gaussian.hpp"
#include "remote_gaussian_backend.h"
#include <cmath>
#include <dlfcn.h>
#include <filesystem>

namespace horus::remote_rendering {
struct GaussianRenderer::Impl {
    void *library{}, *handle{};
    std::mutex mutex;
    GaussianScene transform;
    decltype(&horus_gaussian_last_error) error{};
    decltype(&horus_gaussian_render) render{};
    decltype(&horus_gaussian_destroy) destroy{};
    ~Impl() {
        if (handle && destroy)
            destroy(handle);
        if (library)
            dlclose(library);
    }
    template <class T> T symbol(const char *name) {
        dlerror();
        auto p = dlsym(library, name);
        const auto e = dlerror();
        if (e)
            throw std::runtime_error(std::string("Gaussian ABI missing ") + name + ": " + e);
        return reinterpret_cast<T>(p);
    }
    void check(int code) {
        if (code)
            throw std::runtime_error(error() ? error() : "Native Gaussian rendering failed");
    }
};
GaussianRenderer::GaussianRenderer(const std::string &library, const GaussianScene &s) {
    const auto n = s.means.size();
    const auto k = (s.sh_degree + 1) * (s.sh_degree + 1);
    if (!n || n > INT32_MAX || s.sh_degree > 4 || s.scales.size() != n ||
        s.quaternions.size() != n || s.opacities.size() != n ||
        s.spherical_harmonics.size() != n * k)
        throw std::invalid_argument("Invalid Gaussian scene shapes");
    for (std::size_t i = 0; i < n; ++i) {
        double norm = 0;
        for (auto x : s.quaternions[i]) {
            if (!std::isfinite(x))
                throw std::invalid_argument("Invalid Gaussian quaternion");
            norm += double(x) * x;
        }
        if (std::abs(norm - 1) > .001 || !std::isfinite(s.opacities[i]) || s.opacities[i] < 0 ||
            s.opacities[i] > 1)
            throw std::invalid_argument("Invalid Gaussian rotation or opacity");
        for (unsigned j = 0; j < 3; ++j)
            if (!std::isfinite(s.means[i][j]) || !std::isfinite(s.scales[i][j]) ||
                s.scales[i][j] <= 0)
                throw std::invalid_argument("Invalid Gaussian position or scale");
    }
    for (const auto &c : s.spherical_harmonics)
        for (auto x : c)
            if (!std::isfinite(x))
                throw std::invalid_argument("Invalid Gaussian SH coefficient");
    eye_view_matrix({s.source_from_world_translation, s.source_from_world_rotation, {}});
    if (!std::filesystem::path(library).is_absolute())
        throw std::invalid_argument("Use an absolute Gaussian plugin path");
    auto p = std::make_unique<Impl>();
    p->library = dlopen(library.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!p->library)
        throw std::runtime_error(std::string("Cannot load Gaussian plugin: ") + dlerror());
    if (p->symbol<decltype(&horus_gaussian_abi_version)>("horus_gaussian_abi_version")() != 1)
        throw std::runtime_error("Unsupported Gaussian plugin ABI");
    p->error = p->symbol<decltype(p->error)>("horus_gaussian_last_error");
    p->render = p->symbol<decltype(p->render)>("horus_gaussian_render");
    p->destroy = p->symbol<decltype(p->destroy)>("horus_gaussian_destroy");
    p->check(p->symbol<decltype(&horus_gaussian_create)>("horus_gaussian_create")(
        n, s.sh_degree, s.means[0].data(), s.quaternions[0].data(), s.scales[0].data(),
        s.opacities.data(), s.spherical_harmonics[0].data(), &p->handle));
    p->transform.source_from_world_rotation = s.source_from_world_rotation;
    p->transform.source_from_world_translation = s.source_from_world_translation;
    impl_ = std::move(p);
}
GaussianRenderer::~GaussianRenderer() = default;
namespace {
std::size_t count(std::size_t eyes, const GpuRenderOptions &o) {
    if (!eyes || eyes > 12 || !o.width || !o.height || o.width > 16384 || o.height > 16384 ||
        !std::isfinite(o.near_m) || !std::isfinite(o.far_m) || o.near_m <= 0 || o.far_m <= o.near_m)
        throw std::invalid_argument("Invalid Gaussian render dimensions or depth range");
    auto n = std::uint64_t(eyes) * o.width * o.height;
    if (n > 256 * 1024 * 1024 / 7)
        throw std::invalid_argument("Gaussian output exceeds 256 MiB frame budget");
    return n;
}
} // namespace
RenderedViews GaussianRenderer::render_views(std::span<const EyeView> eyes,
                                             const GpuRenderOptions &o) {
    auto n = count(eyes.size(), o);
    RenderedViews r{o.width, o.height, static_cast<uint32_t>(eyes.size())};
    r.rgb.resize(n * 3);
    r.depth.resize(n);
    render_into(eyes, o, r.rgb, r.depth, r.cuda_ms);
    return r;
}
void GaussianRenderer::render_into(std::span<const EyeView> eyes, const GpuRenderOptions &o,
                                   std::span<std::uint8_t> rgb, std::span<float> depth,
                                   float &elapsed) {
    render_impl(eyes, o, rgb, depth, elapsed, false);
}
void GaussianRenderer::render_impl(std::span<const EyeView> eyes, const GpuRenderOptions &o,
                                   std::span<std::uint8_t> rgb, std::span<float> depth,
                                   float &elapsed, bool device) {
    auto n = count(eyes.size(), o);
    if (rgb.size() != n * 3 || depth.size() != n)
        throw std::invalid_argument("Gaussian output buffer shape mismatch");
    std::array<float, 12 * 16> views{};
    std::array<float, 12 * 9> intrinsics{};
    for (std::size_t i = 0; i < eyes.size(); ++i) {
        const auto e = impl_->transform.world_to_source(eyes[i]);
        const auto m = eye_view_matrix(e), fov = eye_projection_matrix(e, o.near_m, o.far_m);
        for (unsigned r = 0; r < 4; ++r)
            for (unsigned c = 0; c < 4; ++c)
                views[i * 16 + r * 4 + c] = (r == 1 ? -1 : 1) * m[r][c];
        intrinsics[i * 9] = .5 * o.width * fov[0][0];
        intrinsics[i * 9 + 4] = .5 * o.height * fov[1][1];
        intrinsics[i * 9 + 2] = .5 * o.width * (fov[0][2] + 1);
        intrinsics[i * 9 + 5] = .5 * o.height * (1 - fov[1][2]);
        intrinsics[i * 9 + 8] = 1;
    }
    for (auto x : views)
        if (!std::isfinite(x))
            throw std::invalid_argument("Gaussian view exceeds float32");
    for (auto x : intrinsics)
        if (!std::isfinite(x))
            throw std::invalid_argument("Gaussian intrinsics exceed float32");
    std::lock_guard lock(impl_->mutex);
    const auto render = device
                            ? impl_->symbol<decltype(impl_->render)>("horus_gaussian_render_device")
                            : impl_->render;
    impl_->check(render(impl_->handle, views.data(), intrinsics.data(), eyes.size(), o.width,
                        o.height, o.near_m, o.far_m, rgb.data(), depth.data(), &elapsed));
}
} // namespace horus::remote_rendering
