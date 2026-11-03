#include "../core/remote_video_backend.h"
#include <cuda_runtime.h>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_cuda.h>
#include <libavutil/opt.h>
}
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
thread_local std::string error;
void check(int code, const char *what) {
    if (code < 0) {
        char text[AV_ERROR_MAX_STRING_SIZE]{};
        av_strerror(code, text, sizeof(text));
        throw std::runtime_error(std::string(what) + ": " + text);
    }
}
void cuda_check(cudaError_t code, const char *what) {
    if (code != cudaSuccess)
        throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(code));
}
template <class F> int guarded(F f) noexcept {
    try {
        f();
        error.clear();
        return 0;
    } catch (const std::exception &e) {
        error = e.what();
    } catch (...) {
        error = "CUDA video failed";
    }
    return -1;
}
struct DeviceGuard {
    int old{};
    explicit DeviceGuard(int device) {
        cuda_check(cudaGetDevice(&old), "Get CUDA device");
        cuda_check(cudaSetDevice(device), "Set CUDA device");
    }
    ~DeviceGuard() { cudaSetDevice(old); }
};
__device__ uchar4 atlas_pixel(const uchar3 *rgb, const float *depth, unsigned x, unsigned y,
                              unsigned w, unsigned h, unsigned dh, float near_m, float far_m,
                              unsigned sequence) {
    uchar4 value = make_uchar4(0, 0, 0, 255);
    if (y < h) {
        auto c = rgb[((x / w) * h + y) * w + x % w];
        value = make_uchar4(c.x, c.y, c.z, 255);
    } else if (y < h + dh) {
        unsigned pw = w / 4, local = x % w, group = local / pw, px = local % pw;
        unsigned sy = ((uint64_t(y - h) * 2 + 1) * h) / (uint64_t(dh) * 2);
        float d = INFINITY;
        for (unsigned k = 0; k < 4; ++k) {
            float z = depth[((x / w) * h + sy) * w + px * 4 + k];
            if (isfinite(z) && z >= near_m && z <= far_m)
                d = fminf(d, z);
        }
        if (isfinite(d)) {
            auto code =
                __float2uint_rn(fminf(1.f, fmaxf(0.f, (d - near_m) / (far_m - near_m))) * 65535.f);
            auto luma =
                __float2uint_rn(32.f + float((code >> (12 - 4 * group)) & 15) * 192.f / 15.f);
            value = make_uchar4(luma, luma, luma, 255);
        }
    } else if (x < 384) {
        unsigned cell = x / 8, bit = cell / 2, complement = cell % 2;
        unsigned level =
            ((((0xa50000 | (sequence & 65535)) >> (23 - bit)) & 1) ^ complement) ? 224 : 32;
        value = make_uchar4(level, level, level, 255);
    }
    return value;
}
__device__ unsigned char byte_value(float x) {
    return __float2uint_rn(fminf(255.f, fmaxf(0.f, x)));
}
__global__ void pack_video(const uchar3 *rgb, const float *depth, unsigned char *luma, int y_pitch,
                           unsigned char *chroma, int uv_pitch, unsigned w, unsigned h, unsigned dh,
                           float near_m, float far_m, unsigned sequence) {
    const unsigned x = blockIdx.x * blockDim.x + threadIdx.x,
                   y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= 2 * w || y >= h + dh + 16)
        return;
    auto p = atlas_pixel(rgb, depth, x, y, w, h, dh, near_m, far_m, sequence);
    // Explicit BT.709 limited-range conversion avoids driver-dependent RGB
    // conversion matrices disagreeing with the H.264 color metadata.
    luma[size_t(y) * y_pitch + x] =
        byte_value(16.f + (219.f / 255.f) * (.2126f * p.x + .7152f * p.y + .0722f * p.z));
    if (!(x & 1) && !(y & 1)) {
        float r = 0, g = 0, b = 0;
        for (unsigned dy = 0; dy < 2; ++dy)
            for (unsigned dx = 0; dx < 2; ++dx) {
                p = atlas_pixel(rgb, depth, x + dx, y + dy, w, h, dh, near_m, far_m, sequence);
                r += p.x;
                g += p.y;
                b += p.z;
            }
        r *= .25f;
        g *= .25f;
        b *= .25f;
        chroma[size_t(y / 2) * uv_pitch + x] =
            byte_value(128.f + (224.f / 255.f) * (-.114572f * r - .385428f * g + .5f * b));
        chroma[size_t(y / 2) * uv_pitch + x + 1] =
            byte_value(128.f + (224.f / 255.f) * (.5f * r - .454153f * g - .045847f * b));
    }
}
struct Packet {
    std::vector<uint8_t> bytes;
    int64_t pts;
    int key;
};
struct Encoder {
    unsigned w{}, h{}, dh{};
    int gpu{};
    bool closed{};
    uint8_t *rgb{};
    float *depth{};
    AVBufferRef *device{}, *frames{};
    AVCodecContext *codec{};
    AVFrame *frame{};
    AVPacket *packet{};
    std::vector<Packet> result;
    ~Encoder() {
        int previous = 0;
        cudaGetDevice(&previous);
        cudaSetDevice(gpu);
        av_packet_free(&packet);
        av_frame_free(&frame);
        avcodec_free_context(&codec);
        av_buffer_unref(&frames);
        av_buffer_unref(&device);
        cudaFree(rgb);
        cudaFree(depth);
        cudaSetDevice(previous);
    }
    void init(unsigned width, unsigned height, unsigned dh_, unsigned fps, unsigned kbps) {
        w = width;
        h = height;
        dh = dh_;
        cuda_check(cudaGetDevice(&gpu), "Get renderer device");
        if (w < 192 || w > 4096 || w % 4 || h < 16 || h > 4096 || h % 2 || dh > 4096 || dh % 2 ||
            !fps || fps > 120 || kbps < 100 || kbps > 500000 ||
            uint64_t(w) * 2 * (h + dh + 16) > 16 * 1024 * 1024)
            throw std::invalid_argument("Invalid CUDA video dimensions, FPS or bitrate");
        cuda_check(cudaMalloc(&rgb, size_t(w) * h * 2 * 3), "Allocate device RGB");
        cuda_check(cudaMalloc(&depth, size_t(w) * h * 2 * sizeof(float)), "Allocate device depth");
        // Reuse the runtime context without changing flags after a renderer has
        // initialized it. FFmpeg borrows this context instead of owning it.
        check(av_hwdevice_ctx_create(&device, AV_HWDEVICE_TYPE_CUDA, std::to_string(gpu).c_str(),
                                     nullptr, AV_CUDA_USE_CURRENT_CONTEXT),
              "Create CUDA encoder device");
        frames = av_hwframe_ctx_alloc(device);
        if (!frames)
            throw std::bad_alloc();
        auto *pool = reinterpret_cast<AVHWFramesContext *>(frames->data);
        pool->format = AV_PIX_FMT_CUDA;
        pool->sw_format = AV_PIX_FMT_NV12;
        pool->width = w * 2;
        pool->height = h + dh + 16;
        pool->initial_pool_size = 2;
        check(av_hwframe_ctx_init(frames), "Initialize CUDA frame pool");
        auto *encoder = avcodec_find_encoder_by_name("h264_nvenc");
        if (!encoder)
            throw std::runtime_error("FFmpeg lacks NVENC");
        codec = avcodec_alloc_context3(encoder);
        frame = av_frame_alloc();
        packet = av_packet_alloc();
        if (!codec || !frame || !packet)
            throw std::bad_alloc();
        codec->width = pool->width;
        codec->height = pool->height;
        codec->pix_fmt = AV_PIX_FMT_CUDA;
        codec->hw_frames_ctx = av_buffer_ref(frames);
        if (!codec->hw_frames_ctx)
            throw std::bad_alloc();
        codec->time_base = {1, int(fps)};
        codec->framerate = {int(fps), 1};
        codec->gop_size = fps;
        codec->max_b_frames = 0;
        codec->bit_rate = int64_t(kbps) * 1000;
        codec->rc_max_rate = codec->bit_rate;
        codec->rc_buffer_size = std::max<int64_t>(100000, codec->bit_rate / fps);
        codec->color_primaries = AVCOL_PRI_BT709;
        codec->color_trc = AVCOL_TRC_BT709;
        codec->colorspace = AVCOL_SPC_BT709;
        codec->color_range = AVCOL_RANGE_MPEG;
        for (auto item :
             std::vector<std::pair<const char *, const char *>>{{"preset", "p1"},
                                                                {"tune", "ull"},
                                                                {"profile", "baseline"},
                                                                {"rc", "cbr"},
                                                                {"multipass", "disabled"}})
            check(av_opt_set(codec->priv_data, item.first, item.second, 0), item.first);
        for (auto item : std::vector<std::pair<const char *, int>>{{"rc-lookahead", 0},
                                                                   {"delay", 0},
                                                                   {"zerolatency", 1},
                                                                   {"forced-idr", 1},
                                                                   {"surfaces", 2},
                                                                   {"gpu", gpu}})
            check(av_opt_set_int(codec->priv_data, item.first, item.second, 0), item.first);
        check(avcodec_open2(codec, encoder, nullptr), "Open device NVENC");
    }
    void receive() {
        result.clear();
        while (true) {
            int status = avcodec_receive_packet(codec, packet);
            if (status == AVERROR(EAGAIN) || status == AVERROR_EOF)
                break;
            check(status, "Receive device H264");
            result.push_back({{packet->data, packet->data + packet->size},
                              packet->pts,
                              (packet->flags & AV_PKT_FLAG_KEY) != 0});
            av_packet_unref(packet);
            if (result.size() > 8)
                throw std::runtime_error("Device encoder output bound exceeded");
        }
    }
};
} // namespace
extern "C" {
uint32_t horus_video_abi_version() { return 1; }
const char *horus_video_last_error() { return error.c_str(); }
int horus_video_create(uint32_t w, uint32_t h, uint32_t dh, uint32_t fps, uint32_t kbps,
                       void **out) {
    if (out)
        *out = nullptr;
    return guarded([&] {
        if (!out)
            throw std::invalid_argument("Missing video output handle");
        auto e = std::make_unique<Encoder>();
        e->init(w, h, dh, fps, kbps);
        *out = e.release();
    });
}
int horus_video_encode(void *p, horus_video_render_callback render, void *user, float near,
                       float far, uint64_t sequence, int64_t index, int keyframe) {
    return guarded([&] {
        if (!p || !render || !std::isfinite(near) || !std::isfinite(far) || near <= 0 ||
            far <= near || index < 0)
            throw std::invalid_argument("Invalid device frame");
        auto &e = *static_cast<Encoder *>(p);
        if (e.closed)
            throw std::runtime_error("Device encoder is closed");
        DeviceGuard guard(e.gpu);
        float ms = 0;
        if (render(user, e.rgb, e.depth, &ms))
            throw std::runtime_error("Device renderer callback failed");
        av_frame_unref(e.frame);
        check(av_hwframe_get_buffer(e.frames, e.frame, 0), "Acquire device encoder surface");
        pack_video<<<dim3((2 * e.w + 15) / 16, (e.h + e.dh + 31) / 16), dim3(16, 16)>>>(
            reinterpret_cast<uchar3 *>(e.rgb), e.depth, e.frame->data[0], e.frame->linesize[0],
            e.frame->data[1], e.frame->linesize[1], e.w, e.h, e.dh, near, far, sequence);
        cuda_check(cudaGetLastError(), "Pack device video");
        cuda_check(cudaStreamSynchronize(nullptr), "Finish device packing");
        e.frame->pts = index;
        e.frame->pict_type = keyframe ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_NONE;
        e.frame->color_primaries = e.codec->color_primaries;
        e.frame->color_trc = e.codec->color_trc;
        e.frame->colorspace = e.codec->colorspace;
        e.frame->color_range = e.codec->color_range;
        check(avcodec_send_frame(e.codec, e.frame), "Encode device frame");
        e.receive();
    });
}
int horus_video_flush(void *p) {
    return guarded([&] {
        if (!p)
            throw std::invalid_argument("Missing encoder");
        auto &e = *static_cast<Encoder *>(p);
        DeviceGuard guard(e.gpu);
        e.result.clear();
        if (e.closed)
            return;
        e.closed = true;
        check(avcodec_send_frame(e.codec, nullptr), "Flush device encoder");
        e.receive();
    });
}
size_t horus_video_packet_count(void *p) {
    return p ? static_cast<Encoder *>(p)->result.size() : 0;
}
const uint8_t *horus_video_packet(void *p, size_t i, size_t *n, int64_t *pts, int *key) {
    if (!p || !n || !pts || !key)
        return nullptr;
    const auto &v = static_cast<Encoder *>(p)->result;
    if (i >= v.size())
        return nullptr;
    *n = v[i].bytes.size();
    *pts = v[i].pts;
    *key = v[i].key;
    return v[i].bytes.data();
}
void horus_video_destroy(void *p) { delete static_cast<Encoder *>(p); }
}
