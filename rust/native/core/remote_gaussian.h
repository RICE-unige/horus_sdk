#pragma once
#include "remote_gpu.h"
#ifdef __cplusplus
extern "C" {
#endif
void *horus_gaussian_scene_load(const char *path, double scale, int y_down, double quantile,
                                uint64_t maximum_count, uint64_t maximum_bytes, char **error);
void horus_gaussian_scene_destroy(void *);
const void *horus_gaussian_scene_array(const void *, int kind, size_t *count);
char *horus_gaussian_scene_metadata(const void *); // Free with horus_teach_free.
void *horus_gaussian_renderer_create(const char *library, const void *scene, char **error);
void horus_gaussian_renderer_destroy(void *);
int horus_gaussian_renderer_render(void *, const horus_gpu_eye *, size_t eye_count, uint32_t width,
                                   uint32_t height, float near_m, float far_m, uint8_t *rgb,
                                   size_t rgb_size, float *depth, size_t depth_size,
                                   float *elapsed_ms, char **error);
#ifdef __cplusplus
}
#endif
