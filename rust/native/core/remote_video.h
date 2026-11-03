#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void *horus_gpu_video_create(const char *library, uint32_t w, uint32_t h, uint32_t dh, uint32_t fps,
                             uint32_t kbps, char **error);
void horus_gpu_video_destroy(void *);
void horus_gpu_video_keyframe(void *);
// Results use the horus_encoder_result_* functions, including destruction.
void *horus_gpu_video_encode(void *, void *renderer, int gaussian, const char *view_json,
                             uint64_t sequence, uint32_t w, uint32_t h, float near_m, float far_m,
                             int point_radius, char **error);
void *horus_gpu_video_close(void *, char **error);
#ifdef __cplusplus
}
#endif
