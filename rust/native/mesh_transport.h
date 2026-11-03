#ifndef HORUS_NATIVE_MESH_TRANSPORT_H
#define HORUS_NATIVE_MESH_TRANSPORT_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Private ROS context. No robot control or automatic bridge startup.
 * Operations are synchronized; destroy requires all callers to have returned.
 * Returned JSON/error strings must be released with horus_native_free.
 * Publication returns 1 on success, 0 on failure (even if error allocation fails).
 */
void* horus_native_mesh_create(const char* topic, size_t max_payload_bytes, char** error);
int horus_native_mesh_publish(void* handle, const uint8_t* data, size_t size, char** error);
int horus_native_mesh_manifest(void* handle, const char* json, char** error);
/* Returns subscriber count, one queued repair request, and overflow diagnostics. */
char* horus_native_mesh_poll(void* handle);
void horus_native_mesh_destroy(void* handle);
#ifdef __cplusplus
}
#endif
#endif
