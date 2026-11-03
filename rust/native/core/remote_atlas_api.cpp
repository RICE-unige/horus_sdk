#include "horus/remote_rendering/atlas.hpp"
#include "remote_atlas.h"
#include <cstring>
using namespace horus::remote_rendering;
namespace {
template <class F> void *guarded(char **error, F f) {
    if (error)
        *error = nullptr;
    try {
        return f();
    } catch (const std::exception &e) {
        if (error)
            *error = strdup(e.what());
    } catch (...) {
        if (error)
            *error = strdup("Atlas operation failed");
    }
    return nullptr;
}
Json parse(const char *s) {
    if (!s || strnlen(s, 65537) > 65536)
        throw std::invalid_argument("Invalid atlas configuration");
    return Json::parse(s);
}
} // namespace
extern "C" {
void *horus_atlas_render(const float *points, const uint8_t *colors, size_t count, void *stream,
                         const char *views, const char *options, char **error) {
    return guarded(error, [&] {
        auto v = parse(views).get<std::vector<RemoteRenderViewSpec>>();
        auto o = parse(options).get<AtlasOptions>();
        if (stream)
            return new RenderedViews(
                render_multiview_atlas(*static_cast<PointChunkStream *>(stream), v, o));
        if (count > 100000000 || (!points && count) || (!colors && count))
            throw std::invalid_argument("Invalid atlas points");
        return new RenderedViews(
            render_multiview_atlas(std::span(reinterpret_cast<const Point3f *>(points), count),
                                   std::span(reinterpret_cast<const Rgb8 *>(colors), count), v, o));
    });
}
const uint8_t *horus_atlas_rgb(const void *p, size_t *n) {
    if (!p || !n)
        return nullptr;
    const auto &a = *static_cast<const RenderedViews *>(p);
    *n = a.rgb.size();
    return a.rgb.data();
}
const float *horus_atlas_depth(const void *p, size_t *n) {
    if (!p || !n)
        return nullptr;
    const auto &a = *static_cast<const RenderedViews *>(p);
    *n = a.depth.size();
    return a.depth.data();
}
void *horus_atlas_unproject(const void *p, const char *views, uint32_t columns, uint32_t rows,
                            double scale, double fov, char **error) {
    return guarded(error, [&] {
        if (!p)
            throw std::invalid_argument("Missing atlas");
        auto v = parse(views).get<std::vector<RemoteRenderViewSpec>>();
        return new SceneAsset(unproject_multiview_surfels(*static_cast<const RenderedViews *>(p), v,
                                                          columns, rows, scale, fov));
    });
}
void horus_atlas_destroy(void *p) { delete static_cast<RenderedViews *>(p); }
}
