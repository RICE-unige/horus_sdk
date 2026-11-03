#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    const char *parent;
    const char *child;
    double translation[3];
    double rotation[4]; // ROS xyzw.
    int64_t stamp_ns;   // -1 uses the publisher's ROS clock; zero is valid sim time.
} HorusNativeTransform;

void *horus_native_tf_create(const char *options, char **error);
int64_t horus_native_tf_subscribers(void *handle, int is_static, char **error);
int horus_native_tf_publish(void *handle, const HorusNativeTransform *transforms, size_t count,
                            int is_static, char **error);
void horus_native_tf_destroy(void *handle);
#ifdef __cplusplus
}
#endif
