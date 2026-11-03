#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    double position[3], rotation[4], fov[4];
} horus_gpu_eye;
// Returned errors use horus_teach_free. GPU handles own the loaded library.
void *horus_gpu_points_create(const char *library, size_t points, char **error);
void *horus_gpu_mesh_create(const char *library, size_t vertices, size_t faces,
                            const float *positions, const uint32_t *indices, const uint8_t *colors,
                            const float *uvs, const uint32_t *rects, const uint8_t *atlas,
                            uint32_t texture_width, uint32_t texture_height, char **error);
int horus_gpu_upload(void *handle, size_t offset, size_t count, const float *points,
                     const uint8_t *colors, char **error);
int horus_gpu_render(void *handle, const horus_gpu_eye *eyes, size_t eye_count, uint32_t width,
                     uint32_t height, float near_m, float far_m, int radius, uint8_t *rgb,
                     size_t rgb_size, float *depth, size_t depth_size, float *elapsed_ms,
                     char **error);
void horus_gpu_destroy(void *handle);
#ifdef __cplusplus
}
#endif
