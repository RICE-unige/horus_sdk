#include "remote_gpu.h"
#include "horus/remote_rendering/gpu.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <filesystem>
#include <map>

namespace horus::remote_rendering {
namespace {
class Library {
    void *handle_{};

  public:
    explicit Library(const std::string &path) {
        if (!std::filesystem::path(path).is_absolute())
            throw std::invalid_argument("Use an absolute CUDA library path");
        handle_ = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!handle_)
            throw std::runtime_error(std::string("Cannot load CUDA backend: ") + dlerror());
    }
    ~Library() {
        if (handle_)
            dlclose(handle_);
    }
    template <class T> T symbol(const char *name) {
        dlerror();
        auto *p = dlsym(handle_, name);
        const auto *error = dlerror();
        if (error)
            throw std::runtime_error(std::string("CUDA ABI missing ") + name + ": " + error);
        return reinterpret_cast<T>(p);
    }
};
std::size_t pixels(std::size_t views, const GpuRenderOptions &o) {
    if (!views || views > 12 || !o.width || !o.height || o.width > 16384 || o.height > 16384 ||
        !std::isfinite(o.near_m) || !std::isfinite(o.far_m) || o.near_m <= 0 ||
        o.far_m <= o.near_m || o.point_radius < 0 || o.point_radius > 8)
        throw std::invalid_argument("Invalid GPU view count, dimensions, depth range or radius");
    const auto n = std::uint64_t(views) * o.width * o.height;
    if (n > 256 * 1024 * 1024 / 7)
        throw std::invalid_argument("GPU output exceeds 256 MiB host frame budget");
    return n;
}
template <class T> void finite_values(std::span<const T> values) {
    for (const auto &p : values)
        for (auto x : p)
            if (!std::isfinite(x))
                throw std::invalid_argument("Geometry must be finite");
}
} // namespace
struct CudaRenderer::Impl {
    Library library;
    std::mutex mutex;
    void *handle{};
    bool mesh{};
    std::size_t capacity{}, uploaded{};
    std::map<std::size_t, std::size_t> ranges;
    const char *(*error)();
    void (*destroy)(void *);
    int (*upload)(void *, std::size_t, std::size_t, const float *, const unsigned char *){};
    int (*points_render)(void *, const float *, const float *, const float *, int, int, int, float,
                         float, float, int, unsigned char *, float *, float *){};
    int (*mesh_render)(void *, const float *, const float *, const float *, int, int, int, float,
                       float, float, unsigned char *, float *, float *){};
    Impl(const std::string &path, bool mesh) : library(path), mesh(mesh) {
        error = library.symbol<decltype(error)>(mesh ? "horus_cuda_mesh_last_error"
                                                     : "horus_cuda_last_error");
        destroy = library.symbol<decltype(destroy)>(mesh ? "horus_cuda_mesh_destroy"
                                                         : "horus_cuda_destroy");
        if (mesh)
            mesh_render = library.symbol<decltype(mesh_render)>("horus_cuda_mesh_render_views");
        else {
            points_render = library.symbol<decltype(points_render)>("horus_cuda_render_views");
            upload = library.symbol<decltype(upload)>("horus_cuda_upload");
        }
    }
    ~Impl() {
        if (handle)
            destroy(handle);
    }
    void check(int code) {
        if (code) {
            const auto *e = error();
            throw std::runtime_error(e ? e : "CUDA backend failed");
        }
    }
    void mark_uploaded(std::size_t first, std::size_t count) {
        auto end = first + count;
        auto it = ranges.lower_bound(first);
        if (it != ranges.begin()) {
            auto prev = std::prev(it);
            if (prev->second >= first)
                it = prev;
        }
        while (it != ranges.end() && it->first <= end) {
            first = std::min(first, it->first);
            end = std::max(end, it->second);
            uploaded -= it->second - it->first;
            it = ranges.erase(it);
        }
        ranges[first] = end;
        uploaded += end - first;
    }
};
CudaRenderer::CudaRenderer(const std::string &library, std::size_t count) {
    if (!count || count > UINT32_MAX)
        throw std::invalid_argument("Invalid GPU point count");
    auto p = std::make_unique<Impl>(library, false);
    const auto create = p->library.symbol<int (*)(std::size_t, void **)>("horus_cuda_create");
    p->check(create(count, &p->handle));
    p->capacity = count;
    impl_ = std::move(p);
}
CudaRenderer::CudaRenderer(const std::string &library, const MeshInput &m) {
    if (m.vertices.empty() || m.faces.empty() || m.vertices.size() > UINT32_MAX ||
        m.faces.size() > UINT32_MAX || m.colors.size() != m.faces.size())
        throw std::invalid_argument("Invalid GPU mesh shapes");
    finite_values(m.vertices);
    for (const auto &face : m.faces)
        for (auto i : face)
            if (i >= m.vertices.size())
                throw std::invalid_argument("Mesh index out of range");
    if (!m.uvs.empty() && m.uvs.size() != m.vertices.size())
        throw std::invalid_argument("UV count must match vertices");
    finite_values(m.uvs);
    if (!m.texture_rects.empty() && m.texture_rects.size() != m.faces.size())
        throw std::invalid_argument("Texture rect count must match faces");
    if (!m.texture_width || !m.texture_height || m.texture_width > 32768 ||
        m.texture_height > 32768)
        throw std::invalid_argument("Invalid texture size");
    if (!m.texture_atlas.empty() &&
        m.texture_atlas.size() != std::size_t(m.texture_width) * m.texture_height)
        throw std::invalid_argument("Texture shape mismatch");
    if (m.texture_atlas.empty() && (m.texture_width != 1 || m.texture_height != 1))
        throw std::invalid_argument("Missing texture pixels");
    for (const auto &r : m.texture_rects)
        if (!r[2] || !r[3] || std::uint64_t(r[0]) + r[2] > m.texture_width ||
            std::uint64_t(r[1]) + r[3] > m.texture_height)
            throw std::invalid_argument("Texture rectangle out of range");
    std::vector<std::array<float, 2>> uv;
    std::vector<std::array<std::uint32_t, 4>> rects;
    if (m.uvs.empty())
        uv.resize(m.vertices.size(), {0, 0});
    if (m.texture_rects.empty())
        rects.resize(m.faces.size(), {0, 0, 1, 1});
    const Rgb8 white{255, 255, 255};
    auto p = std::make_unique<Impl>(library, true);
    const auto create = p->library.symbol<int (*)(
        std::size_t, std::size_t, const float *, const unsigned int *, const float *,
        const unsigned char *, const unsigned int *, const unsigned char *, int, int, void **)>(
        "horus_cuda_mesh_create");
    p->check(create(m.vertices.size(), m.faces.size(), m.vertices[0].data(), m.faces[0].data(),
                    m.uvs.empty() ? uv[0].data() : m.uvs[0].data(), m.colors[0].data(),
                    m.texture_rects.empty() ? rects[0].data() : m.texture_rects[0].data(),
                    m.texture_atlas.empty() ? white.data() : m.texture_atlas[0].data(),
                    m.texture_width, m.texture_height, &p->handle));
    impl_ = std::move(p);
}
CudaRenderer::~CudaRenderer() = default;
void CudaRenderer::upload(std::span<const Point3f> points, std::span<const Rgb8> colors,
                          std::size_t offset) {
    std::lock_guard lock(impl_->mutex);
    if (impl_->mesh || points.size() != colors.size() || offset > impl_->capacity ||
        points.size() > impl_->capacity - offset)
        throw std::invalid_argument("Invalid point upload range");
    finite_values(points);
    if (points.empty())
        return;
    impl_->check(
        impl_->upload(impl_->handle, offset, points.size(), points[0].data(), colors[0].data()));
    impl_->mark_uploaded(offset, points.size());
}
RenderedViews CudaRenderer::render_views(std::span<const EyeView> eyes,
                                         const GpuRenderOptions &options) {
    if (impl_->mesh && eyes.size() > 2)
        throw std::invalid_argument("The CUDA mesh backend supports at most two views");
    const auto n = pixels(eyes.size(), options);
    RenderedViews r{options.width, options.height, static_cast<std::uint32_t>(eyes.size())};
    r.rgb.resize(n * 3);
    r.depth.resize(n);
    render_into(eyes, options, r.rgb, r.depth, r.cuda_ms);
    return r;
}
void CudaRenderer::render_into(std::span<const EyeView> eyes, const GpuRenderOptions &o,
                               std::span<std::uint8_t> rgb, std::span<float> depth,
                               float &elapsed) {
    render_impl(eyes,o,rgb,depth,elapsed,false);
}
void CudaRenderer::render_impl(std::span<const EyeView> eyes, const GpuRenderOptions &o,
                               std::span<std::uint8_t> rgb, std::span<float> depth,
                               float &elapsed, bool device) {
    const auto n = pixels(eyes.size(), o);
    if (rgb.size() != n * 3 || depth.size() != n)
        throw std::invalid_argument("GPU output buffer shape mismatch");
    std::array<float, 12 * 3> positions{};
    std::array<float, 12 * 9> matrices{};
    std::array<float, 12 * 4> projections{};
    for (std::size_t i = 0; i < eyes.size(); ++i) {
        const auto &e = eyes[i];
        const auto m = eye_view_matrix(e);
        const auto p = e.fov.renderer_projection();
        positions[i * 3] = e.position.x;
        positions[i * 3 + 1] = e.position.y;
        positions[i * 3 + 2] = e.position.z;
        for (unsigned r = 0; r < 3; ++r)
            for (unsigned c = 0; c < 3; ++c)
                matrices[i * 9 + r * 3 + c] = m[r][c];
        for (unsigned c = 0; c < 4; ++c)
            projections[i * 4 + c] = p[c];
    }
    for (auto v : positions)
        if (!std::isfinite(v))
            throw std::invalid_argument("GPU eye exceeds float32");
    for (auto v : projections)
        if (!std::isfinite(v))
            throw std::invalid_argument("GPU projection exceeds float32");
    std::lock_guard lock(impl_->mutex);
    if (!impl_->mesh && impl_->uploaded != impl_->capacity)
        throw std::runtime_error("Upload all points before rendering");
    if (impl_->mesh) {
        const auto render=device?impl_->library.symbol<decltype(impl_->mesh_render)>("horus_cuda_mesh_render_views_device"):impl_->mesh_render;
        impl_->check(render(impl_->handle, positions.data(), matrices.data(),
                                        projections.data(), eyes.size(), o.width, o.height, 90,
                                        o.near_m, o.far_m, rgb.data(), depth.data(), &elapsed));
    } else {
        const auto render=device?impl_->library.symbol<decltype(impl_->points_render)>("horus_cuda_render_views_device"):impl_->points_render;
        impl_->check(render(impl_->handle, positions.data(), matrices.data(),
                                          projections.data(), eyes.size(), o.width, o.height, 90,
                                          o.near_m, o.far_m, o.point_radius, rgb.data(),
                                          depth.data(), &elapsed));
    }
}
} // namespace horus::remote_rendering
namespace {
template <class F> int guarded(char **error, F f) noexcept {
    if (error)
        *error = nullptr;
    try {
        f();
        return 0;
    } catch (const std::exception &e) {
        if (error)
            *error = strdup(e.what());
    } catch (...) {
        if (error)
            *error = strdup("Native GPU operation failed");
    }
    return 1;
}
using namespace horus::remote_rendering;
template <class T> std::span<const T> input(const void *p, std::size_t n) {
    if (n && !p)
        throw std::invalid_argument("Missing GPU input buffer");
    return {static_cast<const T *>(p), n};
}
} // namespace
extern "C" {
void *horus_gpu_points_create(const char *path, size_t n, char **error) {
    void *out = nullptr;
    guarded(error, [&] {
        if (!path)
            throw std::invalid_argument("Missing CUDA library");
        out = new CudaRenderer(path, n);
    });
    return out;
}
void *horus_gpu_mesh_create(const char *path, size_t n, size_t f, const float *v, const uint32_t *t,
                            const uint8_t *c, const float *uv, const uint32_t *rect,
                            const uint8_t *atlas, uint32_t w, uint32_t h, char **error) {
    void *out = nullptr;
    guarded(error, [&] {
        if (!path)
            throw std::invalid_argument("Missing CUDA library");
        MeshInput m{input<Point3f>(v, n),
                    input<std::array<std::uint32_t, 3>>(t, f),
                    input<Rgb8>(c, f),
                    input<std::array<float, 2>>(uv, uv ? n : 0),
                    input<std::array<std::uint32_t, 4>>(rect, rect ? f : 0),
                    input<Rgb8>(atlas, atlas ? std::size_t(w) * h : 0),
                    w,
                    h};
        out = new CudaRenderer(path, m);
    });
    return out;
}
int horus_gpu_upload(void *handle, size_t offset, size_t n, const float *p, const uint8_t *c,
                     char **error) {
    return guarded(error, [&] {
        if (!handle)
            throw std::invalid_argument("Missing GPU handle");
        static_cast<CudaRenderer *>(handle)->upload(input<Point3f>(p, n), input<Rgb8>(c, n),
                                                    offset);
    });
}
int horus_gpu_render(void *handle, const horus_gpu_eye *eyes, size_t count, uint32_t w, uint32_t h,
                     float near, float far, int radius, uint8_t *rgb, size_t rgb_size, float *depth,
                     size_t depth_size, float *elapsed, char **error) {
    return guarded(error, [&] {
        if (!handle || !eyes || !rgb || !depth || !elapsed || !count || count > 12)
            throw std::invalid_argument("Missing GPU render inputs");
        std::array<EyeView, 12> views;
        for (std::size_t i = 0; i < count; ++i) {
            const auto &e = eyes[i];
            views[i] = {{e.position[0], e.position[1], e.position[2]},
                        {e.rotation[0], e.rotation[1], e.rotation[2], e.rotation[3]},
                        {e.fov[0], e.fov[1], e.fov[2], e.fov[3]}};
        }
        static_cast<CudaRenderer *>(handle)->render_into(std::span(views).first(count),
                                                         {w, h, near, far, radius}, {rgb, rgb_size},
                                                         {depth, depth_size}, *elapsed);
    });
}
void horus_gpu_destroy(void *handle) { delete static_cast<CudaRenderer *>(handle); }
}
