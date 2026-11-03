#include "horus/remote_rendering/frame.hpp"
#include "remote_frame.h"
#include <algorithm>
#include <cstring>
#include <memory>
#include <stdexcept>

namespace {
using namespace horus::remote_rendering;
template <class F> int checked(char **error, F &&f) noexcept {
    if (error)
        *error = nullptr;
    try {
        f();
        return 1;
    } catch (const std::exception &e) {
        if (error)
            *error = strdup(e.what());
    } catch (...) {
        if (error)
            *error = strdup("Native frame operation failed");
    }
    return 0;
}
template <class T> std::span<T> view(T *data, size_t size) {
    if ((!data && size) || size > 256 * 1024 * 1024 / sizeof(T))
        throw std::invalid_argument("Null or oversized frame buffer");
    return {data, size};
}
DepthFrameInfo from(const horus_depth_frame_info *source) {
    if (!source)
        throw std::invalid_argument("Missing frame metadata");
    const auto &s = *source;
    if (s.mono_atlas > 1)
        throw std::invalid_argument("Invalid mono atlas flag");
    DepthFrameInfo f;
    f.capture_time_ns = s.capture_time_ns;
    f.sequence = s.sequence;
    f.width = s.width;
    f.height = s.height;
    f.color_width = s.color_width;
    f.color_height = s.color_height;
    f.view_count = s.view_count;
    f.mono_atlas = s.mono_atlas;
    f.near_m = s.near_m;
    f.far_m = s.far_m;
    for (size_t i = 0; i < 2; ++i) {
        std::copy_n(s.eyes[i], 4, f.eyes[i].projection.begin());
        std::copy_n(s.eyes[i] + 4, 3, f.eyes[i].position.begin());
        std::copy_n(s.eyes[i] + 7, 4, f.eyes[i].rotation.begin());
    }
    return f;
}
horus_depth_frame_info to(const DepthFrameInfo &f) {
    horus_depth_frame_info s{};
    s.capture_time_ns = f.capture_time_ns;
    s.sequence = f.sequence;
    s.width = f.width;
    s.height = f.height;
    s.color_width = f.color_width;
    s.color_height = f.color_height;
    s.view_count = f.view_count;
    s.mono_atlas = f.mono_atlas;
    s.near_m = f.near_m;
    s.far_m = f.far_m;
    for (size_t i = 0; i < 2; ++i) {
        std::copy(f.eyes[i].projection.begin(), f.eyes[i].projection.end(), s.eyes[i]);
        std::copy(f.eyes[i].position.begin(), f.eyes[i].position.end(), s.eyes[i] + 4);
        std::copy(f.eyes[i].rotation.begin(), f.eyes[i].rotation.end(), s.eyes[i] + 7);
    }
    return s;
}
} // namespace
extern "C" {
int horus_depth_quantize(const float *in, size_t n, float near_m, float far_m, uint16_t *out,
                         char **error) {
    return checked(error, [&] {
        auto target = view(out, n);
        auto result = quantize_depth(view(in, n), near_m, far_m);
        std::copy(result.begin(), result.end(), target.begin());
    });
}
int horus_depth_dequantize(const uint16_t *in, size_t n, float near_m, float far_m, float *out,
                           char **error) {
    return checked(error, [&] {
        auto target = view(out, n);
        auto result = dequantize_depth(view(in, n), near_m, far_m);
        std::copy(result.begin(), result.end(), target.begin());
    });
}
int horus_depth_reduce(const float *in, size_t n, uint32_t sw, uint32_t sh, uint32_t w, uint32_t h,
                       int mode, float absolute, float relative, float *out, size_t count,
                       char **error) {
    return checked(error, [&] {
        if (count != uint64_t(w) * h)
            throw std::invalid_argument("Reduced depth shape mismatch");
        auto target = view(out, count);
        auto result = reduce_depth(view(in, n), sw, sh, w, h, static_cast<DepthReduction>(mode),
                                   absolute, relative);
        std::copy(result.begin(), result.end(), target.begin());
    });
}
void *horus_frame_pack(const float *in, size_t n, const horus_depth_frame_info *info,
                       char **error) {
    std::unique_ptr<std::vector<uint8_t>> result;
    checked(error, [&] {
        result = std::make_unique<std::vector<uint8_t>>(pack_depth_frame(view(in, n), from(info)));
    });
    return result.release();
}
void *horus_frame_unpack(const uint8_t *in, size_t n, horus_depth_frame_info *info, char **error) {
    std::unique_ptr<std::vector<uint8_t>> result;
    checked(error, [&] {
        if (!info)
            throw std::invalid_argument("Missing output metadata");
        auto f = unpack_depth_frame(view(in, n));
        result = std::make_unique<std::vector<uint8_t>>(f.depth_codes.size() * 2);
        for (size_t i = 0; i < f.depth_codes.size(); ++i) {
            (*result)[2 * i] = uint8_t(f.depth_codes[i]);
            (*result)[2 * i + 1] = uint8_t(f.depth_codes[i] >> 8);
        }
        *info = to(f.info);
    });
    return result.release();
}
const uint8_t *horus_frame_buffer_data(const void *ptr, size_t *size) {
    if (!ptr || !size)
        return nullptr;
    auto &data = *static_cast<const std::vector<uint8_t> *>(ptr);
    *size = data.size();
    return data.data();
}
void horus_frame_buffer_destroy(void *ptr) { delete static_cast<std::vector<uint8_t> *>(ptr); }
void *horus_luma_encode(const float *in, size_t n, uint32_t w, uint32_t h, float near_m,
                        float far_m, int format, char **error) {
    std::unique_ptr<std::vector<uint8_t>> result;
    checked(error, [&] {
        result = std::make_unique<std::vector<uint8_t>>(encode_luma_depth(
            view(in, n), w, h, near_m, far_m, static_cast<LumaDepthFormat>(format)));
    });
    return result.release();
}
int horus_luma_decode(const uint8_t *in, size_t n, uint32_t w, uint32_t h, float near_m,
                      float far_m, int format, float *out, size_t count, char **error) {
    return checked(error, [&] {
        if (uint64_t(w) * h != count)
            throw std::invalid_argument("Decoded luma shape mismatch");
        auto target = view(out, count);
        auto result = decode_luma_depth(view(in, n), w, h, near_m, far_m,
                                        static_cast<LumaDepthFormat>(format));
        std::copy(result.begin(), result.end(), target.begin());
    });
}
int horus_dynamic_pose(uint8_t *in, size_t n, uint32_t w, uint32_t h, float *pose,
                       uint32_t *sequence, float pr, int encode, char **error) {
    return checked(error, [&] {
        if (!pose || !sequence || (encode != 0 && encode != 1))
            throw std::invalid_argument("Invalid dynamic pose output");
        if (encode) {
            DynamicCameraPose p;
            std::copy_n(pose, 3, p.position.begin());
            std::copy_n(pose + 3, 4, p.rotation.begin());
            encode_dynamic_pose_metadata(view(in, n), w, h, p, *sequence, pr);
        } else {
            auto p = decode_dynamic_pose_metadata(view(in, n), w, h, pr);
            std::copy(p.pose.position.begin(), p.pose.position.end(), pose);
            std::copy(p.pose.rotation.begin(), p.pose.rotation.end(), pose + 3);
            *sequence = p.sequence;
        }
    });
}
void *horus_dynamic_pack(const uint8_t *color, size_t nc, const float *depth, size_t nd, uint32_t w,
                         uint32_t h, const float *pose, uint32_t sequence, float near_m,
                         float far_m, float pr, char **error) {
    std::unique_ptr<std::vector<uint8_t>> result;
    checked(error, [&] {
        if (!pose)
            throw std::invalid_argument("Missing dynamic pose");
        DynamicCameraPose p;
        std::copy_n(pose, 3, p.position.begin());
        std::copy_n(pose + 3, 4, p.rotation.begin());
        result = std::make_unique<std::vector<uint8_t>>(pack_dynamic_rgbd(
            view(color, nc), view(depth, nd), w, h, p, sequence, near_m, far_m, pr));
    });
    return result.release();
}
int horus_frame_marker(uint8_t *in, size_t n, uint32_t w, uint32_t h, uint32_t *sequence,
                       int encode, char **error) {
    return checked(error, [&] {
        if (!sequence || (encode != 0 && encode != 1))
            throw std::invalid_argument("Invalid marker operation");
        if (encode)
            embed_frame_marker(view(in, n), w, h, *sequence);
        else
            *sequence = decode_frame_marker(view(in, n), w, h);
    });
}
}
