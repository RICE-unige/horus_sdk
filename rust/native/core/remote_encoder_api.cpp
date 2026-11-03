#include "horus/remote_rendering/encoder.hpp"
#include "remote_encoder.h"
#include <cstring>

namespace {
using horus::remote_rendering::AsyncH264Encoder;
using horus::remote_rendering::H264Encoder;
using Result = std::vector<horus::remote_rendering::EncodedH264Frame>;
template <class F> void *guard(char **error, F &&fn) noexcept {
    if (error)
        *error = nullptr;
    try {
        return fn();
    } catch (const std::exception &e) {
        if (error)
            *error = strdup(e.what());
    } catch (...) {
        if (error)
            *error = strdup("Native encoder failed");
    }
    return nullptr;
}
const horus::remote_rendering::EncodedH264Frame *frame(const void *p, size_t i) {
    if (!p || i >= static_cast<const Result *>(p)->size())
        return nullptr;
    return &(*static_cast<const Result *>(p))[i];
}
} // namespace
extern "C" {
void *horus_async_encoder_create(uint32_t w, uint32_t h, uint32_t fps, uint32_t kbps, uint32_t gpu,
                                 char **error) {
    return guard(error, [&]() -> void * { return new AsyncH264Encoder({w, h, fps, kbps, gpu}); });
}
void horus_async_encoder_destroy(void *p) { delete static_cast<AsyncH264Encoder *>(p); }
int horus_async_encoder_submit(void *p, const uint8_t *rgb, size_t n, const char *json,
                               uint64_t seq, char **error) {
    auto result = guard(error, [&]() -> void * {
        if (!p || !rgb || !json || n > 16 * 1024 * 1024 * 3 || strnlen(json, 65537) > 65536)
            throw std::invalid_argument("Invalid async encoder input");
        static_cast<AsyncH264Encoder *>(p)->submit_rgb({rgb, rgb + n}, json, seq);
        return p;
    });
    return result ? 0 : -1;
}
void horus_async_encoder_keyframe(void *p) {
    if (p)
        static_cast<AsyncH264Encoder *>(p)->request_keyframe();
}
void *horus_async_encoder_poll(void *p, size_t n, uint32_t timeout, char **error) {
    return guard(error, [&]() -> void * {
        if (!p)
            throw std::invalid_argument("Missing async encoder");
        return new Result(static_cast<AsyncH264Encoder *>(p)->poll_encoded(n, timeout));
    });
}
void *horus_async_encoder_close(void *p, char **error) {
    return guard(error, [&]() -> void * {
        if (!p)
            throw std::invalid_argument("Missing async encoder");
        return new Result(static_cast<AsyncH264Encoder *>(p)->close());
    });
}
uint64_t horus_async_encoder_replaced(const void *p) {
    return p ? static_cast<const AsyncH264Encoder *>(p)->replaced_count() : 0;
}
int horus_encoder_compiled(void) { return H264Encoder::compiled(); }
void *horus_encoder_create(uint32_t w, uint32_t h, uint32_t fps, uint32_t kbps, uint32_t gpu,
                           char **error) {
    return guard(error, [&]() -> void * { return new H264Encoder({w, h, fps, kbps, gpu}); });
}
void horus_encoder_destroy(void *p) { delete static_cast<H264Encoder *>(p); }
void horus_encoder_keyframe(void *p) {
    if (p)
        static_cast<H264Encoder *>(p)->request_keyframe();
}
void *horus_encoder_encode(void *p, const uint8_t *rgb, size_t n, const char *json, uint64_t seq,
                           char **error) {
    return guard(error, [&]() -> void * {
        if (!p || !rgb || !json || n > 16 * 1024 * 1024 * 3 || strnlen(json, 65537) > 65536)
            throw std::invalid_argument("Invalid encoder input");
        return new Result(static_cast<H264Encoder *>(p)->encode_rgb({rgb, n}, json, seq));
    });
}
void *horus_encoder_close(void *p, char **error) {
    return guard(error, [&]() -> void * {
        if (!p)
            throw std::invalid_argument("Missing encoder");
        return new Result(static_cast<H264Encoder *>(p)->close());
    });
}
size_t horus_encoder_result_count(const void *p) {
    return p ? static_cast<const Result *>(p)->size() : 0;
}
const uint8_t *horus_encoder_result_data(const void *p, size_t i, size_t *n) {
    const auto *f = frame(p, i);
    if (!f || !n)
        return nullptr;
    *n = f->data.size();
    return f->data.data();
}
const char *horus_encoder_result_metadata(const void *p, size_t i) {
    const auto *f = frame(p, i);
    return f ? f->metadata_json.c_str() : nullptr;
}
int horus_encoder_result_info(const void *p, size_t i, uint64_t *seq, uint64_t *index) {
    const auto *f = frame(p, i);
    if (!f || !seq || !index)
        return -1;
    *seq = f->source_sequence;
    *index = f->frame_index;
    return f->keyframe;
}
void horus_encoder_result_destroy(void *p) { delete static_cast<Result *>(p); }
}
