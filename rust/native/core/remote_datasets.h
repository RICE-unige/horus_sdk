#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
char *horus_dataset_request(const char *request, char **error);
void *horus_dataset_cow(const char *path, double scale, char **error);
void *horus_dataset_stream(const char *descriptor, size_t chunk_size, char **error);
void *horus_dataset_next(void *stream, int *done, char **error);
uint64_t horus_dataset_write_lod(void *stream, const char *path, float voxel, size_t maximum,
                                 char **error);
void horus_dataset_destroy(void *stream);
#ifdef __cplusplus
}
#endif
