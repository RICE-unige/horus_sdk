#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef const char *(*horus_teach_callback)(void *, const char *, const char *);
typedef void (*horus_teach_callback_free)(void *, const char *);
// Every JSON result is {ok:true,value:...} or {ok:false,error:"..."}.
// Callback results follow the same envelope. Neither callback may unwind.
char *horus_teach_config(const char *config, const char *robot);
char *horus_description_compile(const char *xml, const char *options);
char *horus_remote_contract(const char *operation, const char *value);
char *horus_dashboard_render(const char *snapshot, unsigned width, unsigned rows, unsigned page,
                             int color);
uint64_t horus_teach_monotonic_ns(void);
void *horus_teach_session_create(const char *config, const char *robot, void *context,
                                 horus_teach_callback callback, horus_teach_callback_free release,
                                 char **error);
char *horus_teach_session_call(void *session, const char *operation, const char *value);
char *horus_teach_session_append(void *session, const char *metadata, const uint8_t *data,
                                 size_t size);
void horus_teach_session_destroy(void *session);
void *horus_teach_cartesian_create(const char *capabilities, const char *limits, const char *hands,
                                   const char *offsets, void *context,
                                   horus_teach_callback callback, horus_teach_callback_free release,
                                   char **error);
char *horus_teach_cartesian_call(void *adapter, const char *operation, const char *value);
void horus_teach_cartesian_destroy(void *adapter);
void *horus_teach_recorder_create(const char *config, const char *robot, const char *episode,
                                  const char *task, char **error);
char *horus_teach_recorder_call(void *recorder, const char *operation, const char *value);
char *horus_teach_recorder_append(void *recorder, const char *metadata, const uint8_t *data,
                                  size_t size);
void horus_teach_recorder_destroy(void *recorder);
char *horus_teach_episode_validate(const char *path);
char *horus_teach_episode_recover(const char *path);
typedef int (*horus_teach_sample_callback)(void *, const char *, const uint8_t *, size_t);
char *horus_teach_episode_read(const char *path, const char *options, void *context,
                               horus_teach_sample_callback visitor);
void horus_teach_free(char *value);
#ifdef __cplusplus
}
#endif
