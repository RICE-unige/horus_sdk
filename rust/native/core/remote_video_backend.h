#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef int (*horus_video_render_callback)(void *, uint8_t *device_rgb, float *device_depth,
                                           float *ms);
uint32_t horus_video_abi_version(void);
const char *horus_video_last_error(void);
int horus_video_create(uint32_t width_per_eye, uint32_t height, uint32_t depth_height, uint32_t fps,
                       uint32_t kbps, void **out);
int horus_video_encode(void *, horus_video_render_callback, void *user, float near_m, float far_m,
                       uint64_t sequence, int64_t index, int keyframe);
int horus_video_flush(void *);
size_t horus_video_packet_count(void *);
const uint8_t *horus_video_packet(void *, size_t index, size_t *bytes, int64_t *pts, int *keyframe);
void horus_video_destroy(void *);
#ifdef __cplusplus
}
#endif
