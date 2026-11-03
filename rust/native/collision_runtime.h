#pragma once
#ifdef __cplusplus
extern "C" {
#endif
void *horus_collision_create(const char *options, char **error);
void horus_collision_stop(void *handle);
void horus_collision_destroy(void *handle);
#ifdef __cplusplus
}
#endif
