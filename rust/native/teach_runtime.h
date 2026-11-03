#pragma once
#ifdef __cplusplus
extern "C" {
#endif
// session is an existing native TeachSession and must outlive this runtime.
void *horus_teach_ros_create(void *session, const char *config, const char *robot, char **error);
char *horus_teach_ros_status(void *runtime);
void horus_teach_ros_destroy(void *runtime);
#ifdef __cplusplus
}
#endif
