#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    uint64_t capture_time_ns;
    uint32_t sequence, width, height, color_width, color_height, view_count, mono_atlas;
    float near_m, far_m;
    float eyes[2][11];
} horus_depth_frame_info;
// Output bytes use horus_frame_buffer_destroy. Errors use horus_teach_free.
int horus_depth_quantize(const float *, size_t, float, float, uint16_t *, char **);
int horus_depth_dequantize(const uint16_t *, size_t, float, float, float *, char **);
int horus_depth_reduce(const float *, size_t, uint32_t, uint32_t, uint32_t, uint32_t, int, float,
                       float, float *, size_t, char **);
void *horus_frame_pack(const float *, size_t, const horus_depth_frame_info *, char **);
void *horus_frame_unpack(const uint8_t *, size_t, horus_depth_frame_info *, char **);
const uint8_t *horus_frame_buffer_data(const void *, size_t *);
void horus_frame_buffer_destroy(void *);
int horus_frame_marker(uint8_t *, size_t, uint32_t, uint32_t, uint32_t *, int, char **);
void *horus_luma_encode(const float *, size_t, uint32_t, uint32_t, float, float, int, char **);
int horus_luma_decode(const uint8_t *, size_t, uint32_t, uint32_t, float, float, int, float *,
                      size_t, char **);
int horus_dynamic_pose(uint8_t *, size_t, uint32_t, uint32_t, float *, uint32_t *, float, int,
                       char **);
void *horus_dynamic_pack(const uint8_t *, size_t, const float *, size_t, uint32_t, uint32_t,
                         const float *, uint32_t, float, float, float, char **);
#ifdef __cplusplus
}
#endif
