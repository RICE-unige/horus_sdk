#include "horus/remote_rendering/frame.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace horus::remote_rendering {
namespace {
void require(bool condition, const char *message) {
    if (!condition)
        throw std::invalid_argument(message);
}
size_t shape(uint32_t width, uint32_t height) {
    auto n = uint64_t(width) * height;
    require(width && height && n <= 32 * 1024 * 1024, "Invalid dynamic image dimensions");
    return n;
}
void range(float near_m, float far_m) {
    require(std::isfinite(near_m) && std::isfinite(far_m) && near_m > 0 && far_m > near_m,
            "Invalid dynamic depth range");
}
uint32_t round_even(double x) {
    const auto base = uint32_t(std::floor(x));
    const auto part = x - base;
    return base + (part > .5 || (part == .5 && base % 2));
}
uint8_t luma(uint32_t nibble) { return uint8_t(round_even(32.f + float(nibble) * (223.f / 15.f))); }
float level(std::span<const uint8_t> rgb, size_t pixel) {
    return float(uint32_t(rgb[pixel * 3]) + rgb[pixel * 3 + 1] + rgb[pixel * 3 + 2]) / 3.f;
}
uint32_t nibble(float v) { return round_even(std::clamp((v - 32.f) * 15.f / 223.f, 0.f, 15.f)); }
uint32_t samples(LumaDepthFormat f) {
    require(f == LumaDepthFormat::Static8 || f == LumaDepthFormat::Dynamic16,
            "Invalid luma format");
    return f == LumaDepthFormat::Static8 ? 2 : 4;
}
DynamicCameraPose normalized(DynamicCameraPose p) {
    for (auto v : p.position)
        require(std::isfinite(v), "Non-finite camera position");
    float sum = 0;
    for (auto v : p.rotation) {
        require(std::isfinite(v), "Non-finite camera rotation");
        sum += v * v;
    }
    const auto norm = std::sqrt(sum);
    require(std::isfinite(norm), "Camera rotation exceeds range");
    if (norm < 1e-8f)
        p.rotation = {0, 0, 0, 1};
    else
        for (auto &v : p.rotation)
            v /= norm;
    return p;
}
float pose_range(float value) {
    require(std::isfinite(value), "Position range must be finite");
    return std::max(1.f, value);
}
} // namespace
std::vector<uint8_t> encode_luma_depth(std::span<const float> depth, uint32_t w, uint32_t h,
                                       float near_m, float far_m, LumaDepthFormat format) {
    const auto count = samples(format);
    require(w % count == 0 && depth.size() == shape(w, h), "Luma depth shape mismatch");
    range(near_m, far_m);
    const auto plane_width = w / count;
    std::vector<uint8_t> rgb(depth.size() * 3, 0);
    for (size_t y = 0; y < h; ++y)
        for (size_t x = 0; x < plane_width; ++x) {
            float d = depth[y * w + x * count];
            if (format == LumaDepthFormat::Dynamic16) {
                d = std::numeric_limits<float>::infinity();
                for (size_t k = 0; k < count; ++k) {
                    const auto v = depth[y * w + x * count + k];
                    if (std::isfinite(v) && v >= near_m && v <= far_m)
                        d = std::min(d, v);
                }
            }
            if (!std::isfinite(d) || d < near_m || d > far_m)
                continue;
            const float code_value = format == LumaDepthFormat::Static8
                                         ? (d - near_m) * (255.f / (far_m - near_m))
                                         : ((d - near_m) / (far_m - near_m)) * 65535.f;
            auto code = round_even(std::clamp(code_value, 0.f, count == 2 ? 255.f : 65535.f));
            for (size_t k = 0; k < count; ++k) {
                auto pixel = y * w + (count == 2 ? 2 * x + k : k * plane_width + x);
                auto value = luma((code >> ((count - 1 - k) * 4)) & 15);
                std::fill_n(rgb.data() + pixel * 3, 3, value);
            }
        }
    return rgb;
}
std::vector<float> decode_luma_depth(std::span<const uint8_t> rgb, uint32_t w, uint32_t h,
                                     float near_m, float far_m, LumaDepthFormat format) {
    const auto count = samples(format);
    require(w % count == 0 && rgb.size() == shape(w, h) * 3, "Encoded luma shape mismatch");
    range(near_m, far_m);
    const auto plane_width = w / count;
    std::vector<float> depth(rgb.size() / 3, std::numeric_limits<float>::infinity());
    for (size_t y = 0; y < h; ++y)
        for (size_t x = 0; x < plane_width; ++x) {
            auto first = y * w + (count == 2 ? 2 * x : x);
            if (level(rgb, first) < (count == 2 ? 16.f : 24.f))
                continue;
            uint32_t code = 0;
            for (size_t k = 0; k < count; ++k) {
                auto pixel = y * w + (count == 2 ? 2 * x + k : k * plane_width + x);
                code = (code << 4) | nibble(level(rgb, pixel));
            }
            auto d = near_m + ((far_m - near_m) * float(code)) / (count == 2 ? 255.f : 65535.f);
            std::fill_n(depth.data() + y * w + x * count, count, d);
        }
    return depth;
}
uint32_t metadata_rows(uint32_t width) {
    require(width >= 8, "Dynamic metadata requires an image at least 8 pixels wide");
    auto blocks = width / 8;
    return ((36 + blocks - 1) / blocks) * 8;
}
uint32_t encode_dynamic_pose_metadata(std::span<uint8_t> rgb, uint32_t w, uint32_t h,
                                      const DynamicCameraPose &source, uint32_t sequence,
                                      float position_range_m) {
    const auto rows = metadata_rows(w), blocks = w / 8;
    require(rgb.size() == shape(w, h) * 3 && rows <= h, "Dynamic metadata image too small");
    const auto p = normalized(source);
    const auto pr = pose_range(position_range_m);
    std::array<uint32_t, 9> values{0xa35c, sequence & 65535};
    for (size_t i = 0; i < 3; ++i)
        values[i + 2] =
            round_even(std::clamp(double(p.position[i]) / (2. * pr) + .5, 0., 1.) * 65535.);
    for (size_t i = 0; i < 4; ++i)
        values[i + 5] = round_even(std::clamp(double(p.rotation[i]) * .5 + .5, 0., 1.) * 65535.);
    for (size_t i = 0; i < 36; ++i) {
        auto n = (values[i / 4] >> (12 - 4 * (i % 4))) & 15;
        auto level = uint8_t(32 + round_even(double(n) * 223. / 15.));
        auto x0 = (i % blocks) * 8, y0 = (i / blocks) * 8;
        for (size_t y = y0; y < y0 + 8; ++y)
            std::fill_n(rgb.data() + (y * w + x0) * 3, 8 * 3, level);
    }
    return rows;
}
DynamicPoseMetadata decode_dynamic_pose_metadata(std::span<const uint8_t> rgb, uint32_t w,
                                                 uint32_t h, float position_range_m) {
    const auto rows = metadata_rows(w), blocks = w / 8;
    require(rgb.size() == shape(w, h) * 3 && rows <= h, "Dynamic metadata image too small");
    const auto pr = pose_range(position_range_m);
    std::array<uint32_t, 9> values{};
    for (size_t i = 0; i < 36; ++i) {
        auto x = (i % blocks) * 8 + 4, y = (i / blocks) * 8 + 4;
        values[i / 4] = (values[i / 4] << 4) | nibble(level(rgb, y * w + x));
    }
    require(values[0] == 0xa35c, "Invalid dynamic pose marker");
    DynamicCameraPose p;
    for (size_t i = 0; i < 3; ++i)
        p.position[i] = ((values[i + 2] / 65535.) - .5) * 2. * pr;
    for (size_t i = 0; i < 4; ++i)
        p.rotation[i] = (values[i + 5] / 65535.) * 2. - 1.;
    return {uint16_t(values[1]), normalized(p)};
}
std::vector<uint8_t> pack_dynamic_rgbd(std::span<const uint8_t> color, std::span<const float> depth,
                                       uint32_t w, uint32_t h, const DynamicCameraPose &pose,
                                       uint32_t sequence, float near_m, float far_m,
                                       float position_range_m) {
    require(color.size() == shape(w, h) * 3, "Dynamic color shape mismatch");
    auto encoded = encode_luma_depth(depth, w, h, near_m, far_m, LumaDepthFormat::Dynamic16);
    encode_dynamic_pose_metadata(encoded, w, h, pose, sequence, position_range_m);
    std::vector<uint8_t> out(color.size() * 2);
    for (size_t y = 0; y < h; ++y) {
        std::copy_n(color.data() + y * w * 3, w * 3, out.data() + y * w * 6);
        std::copy_n(encoded.data() + y * w * 3, w * 3, out.data() + y * w * 6 + w * 3);
    }
    return out;
}
} // namespace horus::remote_rendering
