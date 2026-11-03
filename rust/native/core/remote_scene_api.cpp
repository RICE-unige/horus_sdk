#include "horus/remote_rendering/scene.hpp"
#include "remote_scene.h"
#include <cstring>
using horus::remote_rendering::SceneAsset;
extern "C" {
void *horus_scene_load(const char *path, double scale, uint64_t vertices, uint64_t triangles,
                       uint64_t pixels, int lighting, char **error) {
    if (error)
        *error = nullptr;
    try {
        if (!path || strnlen(path, 16385) > 16384)
            throw std::invalid_argument("Invalid scene path");
        return new SceneAsset(horus::remote_rendering::load_scene(
            path, {scale, vertices, triangles, pixels, lighting != 0}));
    } catch (const std::exception &e) {
        if (error)
            *error = strdup(e.what());
    } catch (...) {
        if (error)
            *error = strdup("Native scene loading failed");
    }
    return nullptr;
}
void horus_scene_destroy(void *p) { delete static_cast<SceneAsset *>(p); }
const void *horus_scene_array(const void *p, int kind, size_t *n) {
    if (!p || !n)
        return nullptr;
    const auto &s = *static_cast<const SceneAsset *>(p);
    switch (kind) {
    case 0:
        *n = s.vertices.size();
        return s.vertices.data();
    case 1:
        *n = s.point_colors.size();
        return s.point_colors.data();
    case 2:
        *n = s.faces.size();
        return s.faces.data();
    case 3:
        *n = s.face_colors.size();
        return s.face_colors.data();
    case 4:
        *n = s.uvs.size();
        return s.uvs.data();
    case 5:
        *n = s.texture_rects.size();
        return s.texture_rects.data();
    case 6:
        *n = s.texture_atlas.size();
        return s.texture_atlas.data();
    default:
        *n = 0;
        return nullptr;
    }
}
uint32_t horus_scene_texture_size(const void *p, int height) {
    if (!p)
        return 0;
    const auto &s = *static_cast<const SceneAsset *>(p);
    return height ? s.texture_height : s.texture_width;
}
}
