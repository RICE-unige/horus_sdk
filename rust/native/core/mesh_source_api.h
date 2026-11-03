#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    double translation[3], rotation[4];
} horus_mesh_pose;
typedef struct {
    const char *name_space, *frame;
    int32_t id, action, type;
    int64_t stamp_ns;
    int32_t frame_locked;
    double lifetime_seconds;
    horus_mesh_pose pose, transform;
    double scale[3];
    uint8_t color[4];
    const double *points;
    size_t point_count;
    const uint8_t *colors;
    size_t color_count;
} horus_triangle_marker;
typedef struct {
    int32_t index[3], deleted;
    const float *vertices;
    size_t vertex_count;
    const uint32_t *triangles;
    size_t index_count;
    const uint8_t *colors;
    size_t color_count;
} horus_nvblox_block;
void *horus_mesh_source_create(const char *options, char **error);
void horus_mesh_source_destroy(void *);
void *horus_mesh_source_markers(void *, const horus_triangle_marker *, size_t, double now,
                                char **error);
void *horus_mesh_source_blocks(void *, const horus_nvblox_block *, size_t, int clear,
                               const char *frame, int64_t stamp, horus_mesh_pose transform,
                               char **error);
void *horus_mesh_source_pages(void *, const char *operation, const char *request, double now,
                              char **error);
char *horus_mesh_source_status(void *, char **error);
size_t horus_mesh_batch_size(void *);
const uint8_t *horus_mesh_batch_page(void *, size_t, size_t *bytes);
void horus_mesh_batch_destroy(void *);
#ifdef __cplusplus
}
#endif
