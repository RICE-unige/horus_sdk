#pragma once
#ifdef __cplusplus
extern "C" {
#endif
// Owns a private ROS context, source/TF subscriptions and paced map publication.
// Does not auto-launch the bridge or acquire robot control. Destroy joins workers.
// Returned JSON/errors use horus_native_free. No exceptions cross the ABI.
void *horus_native_mesh_source_create(const char *options, char **error);
char *horus_native_mesh_source_status(void *handle);
void horus_native_mesh_source_destroy(void *handle);
#ifdef __cplusplus
}
#endif
