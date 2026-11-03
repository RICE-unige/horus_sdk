#include "horus/remote_rendering/video.hpp"
#include "remote_video.h"
#include <cstring>
using namespace horus::remote_rendering;
namespace {
template <class F> void *guarded(char **error, F f) noexcept {
    if (error)
        *error = nullptr;
    try {
        return f();
    } catch (const std::exception &e) {
        if (error)
            *error = strdup(e.what());
    } catch (...) {
        if (error)
            *error = strdup("Native CUDA video failed");
    }
    return nullptr;
}
using Result = std::vector<EncodedH264Frame>;
} // namespace
extern "C" {
void *horus_gpu_video_create(const char *path, uint32_t w, uint32_t h, uint32_t dh, uint32_t fps,
                             uint32_t kbps, char **error) {
    return guarded(error, [&]() -> void * {
        if (!path)
            throw std::invalid_argument("Missing CUDA video library");
        return new GpuVideoEncoder(path, w, h, dh, fps, kbps);
    });
}
void horus_gpu_video_destroy(void *p) { delete static_cast<GpuVideoEncoder *>(p); }
void horus_gpu_video_keyframe(void *p) {
    if (p)
        static_cast<GpuVideoEncoder *>(p)->request_keyframe();
}
void *horus_gpu_video_encode(void *p, void *renderer, int gaussian, const char *json, uint64_t seq,
                             uint32_t w, uint32_t h, float near, float far, int radius,
                             char **error) {
    return guarded(error, [&]() -> void * {
        if (!p || !renderer || !json ||
            strnlen(json, maximum_metadata_bytes + 1) > maximum_metadata_bytes ||
            (gaussian != 0 && gaussian != 1))
            throw std::invalid_argument("Invalid GPU video input");
        auto view = ViewState::from_json(json);
        auto c = StereoFrameContext::from_view(view, seq);
        const auto eyes = std::array{view.left_eye, view.right_eye};
        GpuRenderOptions o{w, h, near, far, radius};
        if (gaussian)
            return new Result(static_cast<GpuVideoEncoder *>(p)->encode(
                *static_cast<GaussianRenderer *>(renderer), eyes, o, c));
        return new Result(static_cast<GpuVideoEncoder *>(p)->encode(
            *static_cast<CudaRenderer *>(renderer), eyes, o, c));
    });
}
void *horus_gpu_video_close(void *p, char **error) {
    return guarded(error, [&]() -> void * {
        if (!p)
            throw std::invalid_argument("Missing CUDA video encoder");
        return new Result(static_cast<GpuVideoEncoder *>(p)->close());
    });
}
}
