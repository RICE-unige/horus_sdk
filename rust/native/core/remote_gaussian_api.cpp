#include "horus/remote_rendering/gaussian.hpp"
#include "remote_gaussian.h"
#include <cstring>
using namespace horus::remote_rendering;
namespace {
template <class F> int guarded(char **error, F fn) noexcept {
    if (error)
        *error = nullptr;
    try {
        fn();
        return 0;
    } catch (const std::exception &e) {
        if (error)
            *error = strdup(e.what());
    } catch (...) {
        if (error)
            *error = strdup("Native Gaussian operation failed");
    }
    return -1;
}
} // namespace
extern "C" {
void *horus_gaussian_scene_load(const char *path, double scale, int down, double q, uint64_t n,
                                uint64_t bytes, char **error) {
    void *out = nullptr;
    guarded(error, [&] {
        if (!path)
            throw std::invalid_argument("Missing PLY path");
        out = new GaussianScene(load_gaussian_ply(path, {scale, down != 0, q, n, bytes}));
    });
    return out;
}
void horus_gaussian_scene_destroy(void *p) { delete static_cast<GaussianScene *>(p); }
const void *horus_gaussian_scene_array(const void *p, int kind, size_t *n) {
    if (!p || !n)
        return nullptr;
    const auto &s = *static_cast<const GaussianScene *>(p);
    switch (kind) {
    case 0:
        *n = s.means.size();
        return s.means.data();
    case 1:
        *n = s.scales.size();
        return s.scales.data();
    case 2:
        *n = s.quaternions.size();
        return s.quaternions.data();
    case 3:
        *n = s.opacities.size();
        return s.opacities.data();
    case 4:
        *n = s.spherical_harmonics.size();
        return s.spherical_harmonics.data();
    default:
        *n = 0;
        return nullptr;
    }
}
char *horus_gaussian_scene_metadata(const void *p) {
    if (!p)
        return nullptr;
    try {
        const auto &s = *static_cast<const GaussianScene *>(p);
        const auto vector = [](Vector3 v) { return Json::array({v.x, v.y, v.z}); };
        auto q = s.source_from_world_rotation;
        Json j = {{"sh_degree", s.sh_degree},
                  {"discarded_nonfinite", s.discarded_nonfinite},
                  {"world_bounds_min", vector(s.world_bounds_min)},
                  {"world_bounds_max", vector(s.world_bounds_max)},
                  {"source_from_world_translation", vector(s.source_from_world_translation)},
                  {"source_from_world_rotation", Json::array({q.x, q.y, q.z, q.w})}};
        return strdup(j.dump().c_str());
    } catch (...) {
        return nullptr;
    }
}
void *horus_gaussian_renderer_create(const char *library, const void *scene, char **error) {
    void *out = nullptr;
    guarded(error, [&] {
        if (!library || !scene)
            throw std::invalid_argument("Missing Gaussian plugin/scene");
        out = new GaussianRenderer(library, *static_cast<const GaussianScene *>(scene));
    });
    return out;
}
void horus_gaussian_renderer_destroy(void *p) { delete static_cast<GaussianRenderer *>(p); }
int horus_gaussian_renderer_render(void *p, const horus_gpu_eye *e, size_t count, uint32_t w,
                                   uint32_t h, float near, float far, uint8_t *rgb, size_t nrgb,
                                   float *depth, size_t nd, float *elapsed, char **error) {
    return guarded(error, [&] {
        if (!p || !e || !count || count > 12 || !rgb || !depth || !elapsed)
            throw std::invalid_argument("Invalid Gaussian render arguments");
        std::array<EyeView, 12> eyes;
        for (size_t i = 0; i < count; ++i)
            eyes[i] = {{e[i].position[0], e[i].position[1], e[i].position[2]},
                       {e[i].rotation[0], e[i].rotation[1], e[i].rotation[2], e[i].rotation[3]},
                       {e[i].fov[0], e[i].fov[1], e[i].fov[2], e[i].fov[3]}};
        static_cast<GaussianRenderer *>(p)->render_into({eyes.data(), count}, {w, h, near, far, 0},
                                                        {rgb, nrgb}, {depth, nd}, *elapsed);
    });
}
}
