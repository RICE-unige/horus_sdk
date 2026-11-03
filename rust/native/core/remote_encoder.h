#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
int horus_encoder_compiled(void);
void *horus_encoder_create(uint32_t width, uint32_t height, uint32_t fps, uint32_t kbps,
                           uint32_t gpu, char **error);
void horus_encoder_destroy(void *);
void horus_encoder_keyframe(void *);
void *horus_encoder_encode(void *, const uint8_t *, size_t, const char *metadata, uint64_t sequence,
                           char **error);
void *horus_encoder_close(void *, char **error);
void *horus_async_encoder_create(uint32_t width, uint32_t height, uint32_t fps, uint32_t kbps,
                                 uint32_t gpu, char **error);
void horus_async_encoder_destroy(void *);
int horus_async_encoder_submit(void *,const uint8_t *,size_t,const char *,uint64_t,char **);
void horus_async_encoder_keyframe(void *);
void *horus_async_encoder_poll(void *,size_t maximum,uint32_t timeout,char **error);
void *horus_async_encoder_close(void *,char **error);
uint64_t horus_async_encoder_replaced(const void *);
size_t horus_encoder_result_count(const void *);
const uint8_t *horus_encoder_result_data(const void *, size_t index, size_t *size);
const char *horus_encoder_result_metadata(const void *, size_t index);
int horus_encoder_result_info(const void *, size_t index, uint64_t *sequence,
                              uint64_t *frame_index);
void horus_encoder_result_destroy(void *);
#ifdef __cplusplus
}
#endif
