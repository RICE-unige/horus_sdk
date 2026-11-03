#include "horus/remote_rendering/video.hpp"
#include "remote_video_backend.h"
#include <atomic>
#include <dlfcn.h>
#include <filesystem>
#include <map>

namespace horus::remote_rendering {
struct GpuVideoEncoder::Impl {
    void *library{}, *handle{};
    uint32_t width, height, depth_height;
    std::mutex mutex;
    std::atomic<bool> keyframe{true};
    bool closed{};
    int64_t index{};
    std::optional<Json> identity;
    std::map<int64_t, std::pair<std::string, uint64_t>> pending;
    decltype(&horus_video_last_error) error{};
    decltype(&horus_video_encode) encode{};
    decltype(&horus_video_flush) flush{};
    decltype(&horus_video_packet_count) count{};
    decltype(&horus_video_packet) packet{};
    decltype(&horus_video_destroy) destroy{};
    Impl(uint32_t w, uint32_t h, uint32_t dh) : width(w), height(h), depth_height(dh) {}
    ~Impl() {
        if (handle && destroy)
            destroy(handle);
        if (library)
            dlclose(library);
    }
    template <class T> T symbol(const char *name) {
        dlerror();
        auto p = dlsym(library, name);
        auto e = dlerror();
        if (e)
            throw std::runtime_error(std::string("CUDA video ABI missing ") + name + ": " + e);
        return reinterpret_cast<T>(p);
    }
    void check(int status) {
        if (status)
            throw std::runtime_error(error() ? error() : "CUDA video failed");
    }
    std::vector<EncodedH264Frame> receive() {
        std::vector<EncodedH264Frame> out;
        const auto n = count(handle);
        if (n > 8)
            throw std::runtime_error("CUDA video output bound exceeded");
        for (size_t i = 0; i < n; ++i) {
            size_t bytes = 0;
            int64_t pts = -1;
            int key = 0;
            const auto data = packet(handle, i, &bytes, &pts, &key);
            const auto found = pending.find(pts);
            if (!data || !bytes || bytes > maximum_image_bytes || found == pending.end())
                throw std::runtime_error("Uncorrelated CUDA video packet");
            out.push_back({{data, data + bytes},
                           std::move(found->second.first),
                           found->second.second,
                           uint64_t(pts),
                           key != 0});
            pending.erase(found);
        }
        return out;
    }
};
GpuVideoEncoder::GpuVideoEncoder(const std::string &library, uint32_t w, uint32_t h, uint32_t dh,
                                 uint32_t fps, uint32_t kbps) {
    if (!std::filesystem::path(library).is_absolute())
        throw std::invalid_argument("Use an absolute CUDA video library path");
    auto p = std::make_unique<Impl>(w, h, dh);
    p->library = dlopen(library.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!p->library)
        throw std::runtime_error(std::string("Cannot load CUDA video plugin: ") + dlerror());
    if (p->symbol<decltype(&horus_video_abi_version)>("horus_video_abi_version")() != 1)
        throw std::runtime_error("Unsupported CUDA video ABI");
    p->error = p->symbol<decltype(p->error)>("horus_video_last_error");
    p->encode = p->symbol<decltype(p->encode)>("horus_video_encode");
    p->flush = p->symbol<decltype(p->flush)>("horus_video_flush");
    p->destroy = p->symbol<decltype(p->destroy)>("horus_video_destroy");
    p->packet = p->symbol<decltype(p->packet)>("horus_video_packet");
    p->count = p->symbol<decltype(p->count)>("horus_video_packet_count");
    p->check(p->symbol<decltype(&horus_video_create)>("horus_video_create")(w, h, dh, fps, kbps,
                                                                            &p->handle));
    impl_ = std::move(p);
}
GpuVideoEncoder::~GpuVideoEncoder() = default;
void GpuVideoEncoder::request_keyframe() { impl_->keyframe.store(true); }
std::vector<EncodedH264Frame>
GpuVideoEncoder::encode_impl(const GpuRenderOptions &o, StereoFrameContext c,
                             const std::function<void(uint8_t *, float *, float &)> &render) {
    if (o.width != impl_->width || o.height != impl_->height)
        throw std::invalid_argument("CUDA video image dimensions changed");
    c.codec = "h264";
    c.marker_height = 16;
    c.depth_strip_height = impl_->depth_height;
    c.render_start_ns = c.render_complete_ns = c.capture_time_ns = monotonic_time_ns();
    stereo_frame_metadata(o, c);
    std::lock_guard lock(impl_->mutex);
    if (impl_->closed)
        throw std::runtime_error("CUDA video encoder is closed");
    if (impl_->pending.size() >= 8 || impl_->index == INT64_MAX)
        throw std::runtime_error("CUDA video pending frame bound exceeded");
    const Json identity = Json::array({c.session_id, c.view_stream_id, c.workspace_revision});
    bool force =
        impl_->keyframe.exchange(false) || !impl_->identity || *impl_->identity != identity;
    struct Call {
        const std::function<void(uint8_t *, float *, float &)> &fn;
        StereoFrameContext &context;
        std::exception_ptr error;
    };
    Call call{render, c, {}};
    auto callback = +[](void *u, uint8_t *rgb, float *d, float *ms) -> int {
        auto &s = *static_cast<Call *>(u);
        try {
            s.context.render_start_ns = monotonic_time_ns();
            s.fn(rgb, d, *ms);
            s.context.render_complete_ns = s.context.capture_time_ns = monotonic_time_ns();
            return 0;
        } catch (...) {
            s.error = std::current_exception();
            return -1;
        }
    };
    const auto code = impl_->encode(impl_->handle, callback, &call, o.near_m, o.far_m, c.sequence,
                                    impl_->index, force);
    if (code) {
        impl_->keyframe.store(true);
        if (call.error)
            std::rethrow_exception(call.error);
        impl_->closed = true;
        impl_->check(code);
    }
    impl_->pending.emplace(impl_->index++,
                           std::pair{stereo_frame_metadata(o, c).dump(), c.sequence});
    impl_->identity = identity;
    try {
        return impl_->receive();
    } catch (...) {
        impl_->closed = true;
        throw;
    }
}
std::vector<EncodedH264Frame> GpuVideoEncoder::encode(CudaRenderer &r,
                                                      std::span<const EyeView> eyes,
                                                      const GpuRenderOptions &o,
                                                      StereoFrameContext c) {
    if (eyes.size() != 2)
        throw std::invalid_argument("CUDA video requires two eyes");
    return encode_impl(o, std::move(c), [&](uint8_t *rgb, float *depth, float &ms) {
        auto n = size_t(o.width) * o.height * 2;
        r.render_impl(eyes, o, {rgb, n * 3}, {depth, n}, ms, true);
    });
}
std::vector<EncodedH264Frame> GpuVideoEncoder::encode(GaussianRenderer &r,
                                                      std::span<const EyeView> eyes,
                                                      const GpuRenderOptions &o,
                                                      StereoFrameContext c) {
    if (eyes.size() != 2)
        throw std::invalid_argument("CUDA video requires two eyes");
    return encode_impl(o, std::move(c), [&](uint8_t *rgb, float *depth, float &ms) {
        auto n = size_t(o.width) * o.height * 2;
        r.render_impl(eyes, o, {rgb, n * 3}, {depth, n}, ms, true);
    });
}
std::vector<EncodedH264Frame> GpuVideoEncoder::close() {
    std::lock_guard lock(impl_->mutex);
    if (impl_->closed)
        return {};
    impl_->closed = true;
    impl_->check(impl_->flush(impl_->handle));
    auto result = impl_->receive();
    if (!impl_->pending.empty())
        throw std::runtime_error("Device encoder did not flush all accepted frames");
    return result;
}
} // namespace horus::remote_rendering
