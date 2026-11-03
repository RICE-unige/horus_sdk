#pragma once
#ifdef __cplusplus
extern "C" {
#endif
// JSON configuration/result; returned strings use horus_native_free().
void *horus_native_costmap_create(const char *options, char **error);
char *horus_native_costmap_status(void *adapter);
void horus_native_costmap_destroy(void *adapter);
#ifdef __cplusplus
}
#endif
