#pragma once
#include "remote_gpu.h"
#ifdef __cplusplus
extern "C" {
#endif
uint64_t horus_remote_monotonic_time_ns(void);
// The binary result owns its buffers; getters borrow until destroy.
void *horus_stereo_assemble(const uint8_t *rgb, size_t rgb_size, const float *depth,
                            size_t depth_size, uint32_t w, uint32_t h, float near_m, float far_m,
                            const char *context_json, char **error);
const uint8_t *horus_stereo_data(const void *result, int which, size_t *size);
const char *horus_stereo_metadata(const void *result);
double horus_stereo_visibility(const void *result);
void *horus_stereo_diagnostic(const void *result, char **error);
void horus_stereo_destroy(void *result);
int horus_cpu_points(const float *points, const uint8_t *colors, size_t count,
                     const horus_gpu_eye *eyes, size_t eye_count, uint32_t w, uint32_t h,
                     float near_m, float far_m, int radius, uint8_t *rgb, size_t rgb_size,
                     float *depth, size_t depth_size, char **error);
#ifdef __cplusplus
}
#endif
