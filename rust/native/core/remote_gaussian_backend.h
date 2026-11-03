#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// Plugin ABI 1. Inputs and host outputs are borrowed only during each call.
uint32_t horus_gaussian_abi_version(void);
const char *horus_gaussian_last_error(void);
int horus_gaussian_create(size_t count, uint32_t sh_degree, const float *means,
                          const float *quats_wxyz, const float *scales, const float *opacities,
                          const float *sh, void **out);
int horus_gaussian_render(void *, const float *view_matrices, const float *intrinsics,
                          uint32_t views, uint32_t width, uint32_t height, float near_m,
                          float far_m, uint8_t *rgb, float *depth, float *elapsed_ms);
void horus_gaussian_destroy(void *);
#ifdef __cplusplus
}
#endif
