#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void *horus_scene_load(const char *path, double scale, uint64_t vertices, uint64_t triangles,
                      uint64_t texture_pixels, int lighting, char **error);
void horus_scene_destroy(void *scene);
const void *horus_scene_array(const void *scene, int kind, size_t *count);
uint32_t horus_scene_texture_size(const void *scene, int height);
#ifdef __cplusplus
}
#endif
