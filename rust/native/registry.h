#ifndef HORUS_NATIVE_REGISTRY_H
#define HORUS_NATIVE_REGISTRY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Native in-process ROS 2 registration. No global ROS init/shutdown or signal handlers.
 * An operation may run concurrently with stop/status, but destroy requires all callers
 * to have returned. Returned UTF-8 JSON strings must be freed with horus_native_free.
 * NULL create result has an error in *error. No C++ exception crosses this boundary.
 */
void* horus_native_create(char** error);
/* options: auto_launch (default true), bridge_executable (absolute path or empty
 * to resolve from AMENT_PREFIX_PATH), bridge_arguments (array, no shell).
 * The registration timeout includes discovery and bridge startup.
 */
void* horus_native_create_with_options(const char* options_json, char** error);
char* horus_native_register(void* handle, const char* payloads_json, double timeout_seconds,
                           int keep_alive, int show_status);
char* horus_native_unregister(void* handle, const char* robot_name, double timeout_seconds);
char* horus_native_status(void* handle);
void horus_native_stop(void* handle);
void horus_native_destroy(void* handle);
void horus_native_free(char* text);

#ifdef __cplusplus
}
#endif
#endif
