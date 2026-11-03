#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
void *horus_remote_create(const char *options_json, char **error);
void horus_remote_destroy(void *);
void horus_remote_stop(void *);
char *horus_remote_next_view(void *, uint32_t timeout_ms, char **error);
char *horus_remote_status(void *, char **error);
int horus_remote_take_keyframe(void *);
// kind: 0 = HRV3 PNG diagnostic, 1 = raw RGB video atlas, 2 = H.264 access unit.
// Returns 1 when published, 0 when superseded, -1 on an error.
int horus_remote_publish(void *, int kind, const char *metadata_json, const uint8_t *data,
                         size_t size, char **error);
#ifdef __cplusplus
}
#endif
