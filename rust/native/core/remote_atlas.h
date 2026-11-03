#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void *horus_atlas_render(const float *points, const uint8_t *colors, size_t count, void *stream,
                         const char *views, const char *options, char **error);
const uint8_t *horus_atlas_rgb(const void *, size_t *);
const float *horus_atlas_depth(const void *, size_t *);
void *horus_atlas_unproject(const void *, const char *views, uint32_t columns, uint32_t rows,
                            double scale, double fov, char **error);
void horus_atlas_destroy(void *);
#ifdef __cplusplus
}
#endif
