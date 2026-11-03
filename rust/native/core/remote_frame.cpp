#include "horus/remote_rendering/frame.hpp"
#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <zlib.h>

namespace horus::remote_rendering {
namespace {
constexpr size_t payload_limit = 4 * 1024 * 1024;
constexpr size_t pixel_limit = 64 * 1024 * 1024;
constexpr float invalid_depth = std::numeric_limits<float>::infinity();
void require(bool condition, const char *message) {
    if (!condition)
        throw std::invalid_argument(message);
}
void depth_range(float near_m, float far_m) {
    require(std::isfinite(near_m) && std::isfinite(far_m) && near_m > 0 && far_m > near_m,
            "Depth range must be finite and satisfy 0 < near < far");
}
size_t pixels(uint32_t width, uint32_t height) {
    auto n = uint64_t(width) * height;
    require(width && height && n <= pixel_limit, "Invalid or oversized image dimensions");
    return static_cast<size_t>(n);
}
void put(std::vector<uint8_t> &out, uint64_t value, unsigned count) {
    for (unsigned i = 0; i < count; ++i)
        out.push_back(uint8_t(value >> (8 * i)));
}
uint64_t get(std::span<const uint8_t> in, size_t &offset, unsigned count) {
    require(offset <= in.size() && count <= in.size() - offset, "Truncated frame");
    uint64_t result = 0;
    for (unsigned i = 0; i < count; ++i)
        result |= uint64_t(in[offset++]) << (8 * i);
    return result;
}
void put_float(std::vector<uint8_t> &out, float value) {
    put(out, std::bit_cast<uint32_t>(value), 4);
}
float get_float(std::span<const uint8_t> in, size_t &offset) {
    return std::bit_cast<float>(uint32_t(get(in, offset, 4)));
}
void validate(const DepthFrameInfo &f) {
    depth_range(f.near_m, f.far_m);
    require(f.view_count == 1 || f.view_count == 2, "Expected one or two views");
    require(!f.mono_atlas || f.view_count == 2, "Shared mono atlas needs stereo metadata");
    require(f.width <= 65535 && f.height <= 65535 && f.color_width && f.color_height &&
                f.color_width <= 65535 && f.color_height <= 65535,
            "Invalid frame dimensions");
    require(pixels(f.width, f.height) * (f.mono_atlas ? 1 : f.view_count) * 2 <= payload_limit,
            "Depth frame exceeds 4 MiB");
    for (size_t eye = 0; eye < f.view_count; ++eye) {
        for (auto v : f.eyes[eye].projection)
            require(std::isfinite(v), "Non-finite projection");
        for (auto v : f.eyes[eye].position)
            require(std::isfinite(v), "Non-finite position");
        for (auto v : f.eyes[eye].rotation)
            require(std::isfinite(v), "Non-finite rotation");
    }
}
std::vector<uint8_t> gzip(std::span<const uint8_t> raw) {
    z_stream stream{};
    if (deflateInit2(&stream, 1, Z_DEFLATED, 31, 8, Z_DEFAULT_STRATEGY) != Z_OK)
        throw std::runtime_error("Could not initialize gzip compressor");
    struct End {
        z_stream *s;
        ~End() { deflateEnd(s); }
    } end{&stream};
    std::vector<uint8_t> out(deflateBound(&stream, raw.size()));
    stream.next_in = const_cast<Bytef *>(raw.data());
    stream.avail_in = static_cast<uInt>(raw.size());
    stream.next_out = out.data();
    stream.avail_out = static_cast<uInt>(out.size());
    require(deflate(&stream, Z_FINISH) == Z_STREAM_END, "Gzip compression failed");
    out.resize(stream.total_out);
    require(out.size() <= payload_limit, "Compressed depth exceeds 4 MiB");
    return out;
}
std::vector<uint8_t> gunzip(std::span<const uint8_t> encoded, size_t expected) {
    z_stream stream{};
    if (inflateInit2(&stream, 31) != Z_OK)
        throw std::runtime_error("Could not initialize gzip decoder");
    struct End {
        z_stream *s;
        ~End() { inflateEnd(s); }
    } end{&stream};
    // One extra byte lets zlib consume the trailer without permitting unbounded output.
    std::vector<uint8_t> out(expected + 1);
    stream.next_in = const_cast<Bytef *>(encoded.data());
    stream.avail_in = static_cast<uInt>(encoded.size());
    stream.next_out = out.data();
    stream.avail_out = static_cast<uInt>(out.size());
    require(inflate(&stream, Z_FINISH) == Z_STREAM_END && stream.avail_in == 0 &&
                stream.total_out == expected,
            "Invalid gzip data or depth size");
    out.resize(expected);
    return out;
}
} // namespace
std::vector<uint16_t> quantize_depth(std::span<const float> depth, float near_m, float far_m) {
    depth_range(near_m, far_m);
    require(depth.size() <= pixel_limit, "Depth buffer exceeds limit");
    std::vector<uint16_t> out(depth.size(), 0);
    for (size_t i = 0; i < depth.size(); ++i) {
        auto d = depth[i];
        if (!std::isfinite(d) || d < near_m || d > far_m)
            continue;
        const float normalized = (d - near_m) / (far_m - near_m);
        const float scaled = std::clamp(normalized, 0.f, 1.f) * 65534.f;
        auto low = uint32_t(std::floor(scaled));
        const float fraction = scaled - float(low);
        if (fraction > .5f || (fraction == .5f && low % 2))
            ++low;
        out[i] = uint16_t(low + 1);
    }
    return out;
}
std::vector<float> dequantize_depth(std::span<const uint16_t> codes, float near_m, float far_m) {
    depth_range(near_m, far_m);
    require(codes.size() <= pixel_limit, "Depth buffer exceeds limit");
    std::vector<float> out(codes.size(), invalid_depth);
    for (size_t i = 0; i < codes.size(); ++i)
        if (codes[i])
            out[i] = near_m + (far_m - near_m) * ((float(codes[i]) - 1.f) / 65534.f);
    return out;
}
std::vector<float> reduce_depth(std::span<const float> depth, uint32_t sw, uint32_t sh, uint32_t w,
                                uint32_t h, DepthReduction mode, float absolute, float relative) {
    require(depth.size() == pixels(sw, sh), "Source depth shape mismatch");
    const auto n = pixels(w, h);
    require(w <= sw && h <= sh, "Depth reduction cannot increase dimensions");
    require(mode == DepthReduction::Minimum || mode == DepthReduction::Surfel ||
                mode == DepthReduction::Reprojection,
            "Invalid depth reduction mode");
    require(std::isfinite(absolute) && std::isfinite(relative) && absolute >= 0 && relative >= 0,
            "Invalid depth edge thresholds");
    std::vector<float> out(n, invalid_depth);
    if (mode == DepthReduction::Minimum) {
        require(sw % w == 0 && sh % h == 0, "Minimum depth output must divide source dimensions");
        const auto dx = sw / w, dy = sh / h;
        for (uint32_t y = 0; y < sh; ++y)
            for (uint32_t x = 0; x < sw; ++x) {
                const auto d = depth[size_t(y) * sw + x];
                if (std::isfinite(d)) {
                    auto &target = out[size_t(y / dy) * w + x / dx];
                    target = std::min(target, d);
                }
            }
        return out;
    }
    for (uint32_t y = 0; y < h; ++y)
        for (uint32_t x = 0; x < w; ++x) {
            const auto sx = ((uint64_t(x) * 2 + 1) * sw) / (2 * uint64_t(w));
            const auto sy = ((uint64_t(y) * 2 + 1) * sh) / (2 * uint64_t(h));
            out[size_t(y) * w + x] = depth[sy * sw + sx];
        }
    if (mode == DepthReduction::Surfel)
        return out;
    std::vector<bool> invalid(n);
    auto edge = [&](size_t a, size_t b) {
        if (std::isfinite(out[a]) && std::isfinite(out[b]) &&
            std::abs(out[a] - out[b]) > std::max(absolute, std::min(out[a], out[b]) * relative))
            invalid[out[a] > out[b] ? a : b] = true;
    };
    // Keep comparisons against the original samples, not already invalidated neighbors.
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            size_t i = size_t(y) * w + x;
            if (x + 1 < w)
                edge(i, i + 1);
            if (y + 1 < h) {
                edge(i, i + w);
                if (x + 1 < w) {
                    edge(i, i + w + 1);
                    edge(i + 1, i + w);
                }
            }
        }
    }
    for (size_t i = 0; i < n; ++i)
        if (invalid[i] || !std::isfinite(out[i]))
            out[i] = invalid_depth;
    return out;
}
std::vector<uint8_t> pack_depth_frame(std::span<const float> depth, const DepthFrameInfo &f) {
    validate(f);
    const size_t n = pixels(f.width, f.height);
    require(depth.size() == n * f.view_count, "Depth frame shape mismatch");
    auto codes = quantize_depth(depth, f.near_m, f.far_m);
    if (f.mono_atlas)
        require(std::equal(codes.begin(), codes.begin() + n, codes.begin() + n),
                "Mono atlas requires identical quantized depth views");
    const auto atlas_views = f.mono_atlas ? 1 : f.view_count;
    std::vector<uint8_t> raw;
    raw.reserve(n * atlas_views * 2);
    for (size_t y = 0; y < f.height; ++y)
        for (size_t eye = 0; eye < atlas_views; ++eye)
            for (size_t x = 0; x < f.width; ++x) {
                auto row = f.view_count == 2 ? f.height - 1 - y : y;
                put(raw, codes[eye * n + row * f.width + x], 2);
            }
    auto encoded = gzip(raw);
    std::vector<uint8_t> out{'H', 'R', 'R', 'F'};
    const bool stereo = f.view_count == 2;
    out.reserve((stereo ? 140 : 92) + encoded.size());
    put(out, stereo ? 2 : 1, 1);
    put(out, stereo ? f.mono_atlas : 1, 1);
    put(out, stereo ? 140 : 92, 2);
    put(out, f.sequence, 4);
    put(out, f.capture_time_ns, 8);
    if (stereo) {
        put(out, 2, 1);
        put(out, 0, 3);
    }
    put(out, f.width, 2);
    put(out, f.height, 2);
    put(out, f.color_width, 2);
    put(out, f.color_height, 2);
    put_float(out, f.near_m);
    put_float(out, f.far_m);
    for (size_t eye = 0; eye < f.view_count; ++eye) {
        for (auto v : f.eyes[eye].projection)
            put_float(out, v);
        for (auto v : f.eyes[eye].position)
            put_float(out, v);
        for (auto v : f.eyes[eye].rotation)
            put_float(out, v);
    }
    put(out, 2, 1);
    put(out, 0, 3);
    put(out, encoded.size(), 4);
    put(out, crc32(0, encoded.data(), encoded.size()), 4);
    out.insert(out.end(), encoded.begin(), encoded.end());
    return out;
}
DepthFrame unpack_depth_frame(std::span<const uint8_t> in) {
    size_t p = 0;
    require(get(in, p, 4) == 0x46525248, "Invalid HRRF magic");
    auto version = get(in, p, 1), flags = get(in, p, 1), header_size = get(in, p, 2);
    require((version == 1 && header_size == 92 && flags == 1) ||
                (version == 2 && header_size == 140 && flags <= 1),
            "Unsupported HRRF header");
    DepthFrame result;
    auto &f = result.info;
    f.sequence = get(in, p, 4);
    f.capture_time_ns = get(in, p, 8);
    f.view_count = version;
    f.mono_atlas = version == 2 && flags == 1;
    if (version == 2) {
        require(get(in, p, 1) == 2, "Invalid stereo view count");
        get(in, p, 3);
    }
    f.width = get(in, p, 2);
    f.height = get(in, p, 2);
    f.color_width = get(in, p, 2);
    f.color_height = get(in, p, 2);
    f.near_m = get_float(in, p);
    f.far_m = get_float(in, p);
    for (size_t eye = 0; eye < f.view_count; ++eye) {
        for (auto &v : f.eyes[eye].projection)
            v = get_float(in, p);
        for (auto &v : f.eyes[eye].position)
            v = get_float(in, p);
        for (auto &v : f.eyes[eye].rotation)
            v = get_float(in, p);
    }
    auto encoding = get(in, p, 1);
    get(in, p, 3);
    auto length = get(in, p, 4), checksum = get(in, p, 4);
    validate(f);
    require(length && length <= payload_limit && p == header_size && in.size() == p + length,
            "Invalid depth payload size");
    require(encoding == 2 || (version == 1 && encoding == 1), "Invalid depth encoding");
    auto encoded = in.subspan(p);
    require(crc32(0, encoded.data(), encoded.size()) == checksum, "Depth CRC mismatch");
    const auto n = pixels(f.width, f.height);
    const auto views = f.mono_atlas ? 1 : f.view_count;
    const size_t expected = n * views * 2;
    std::vector<uint8_t> decoded;
    if (encoding == 2)
        decoded = gunzip(encoded, expected);
    auto raw = encoding == 2 ? std::span<const uint8_t>(decoded) : encoded;
    require(raw.size() == expected, "Raw depth size mismatch");
    result.depth_codes.resize(n * f.view_count);
    for (size_t eye = 0; eye < f.view_count; ++eye)
        for (size_t y = 0; y < f.height; ++y)
            for (size_t x = 0; x < f.width; ++x) {
                auto row = f.view_count == 2 ? f.height - 1 - y : y;
                size_t cursor =
                    (row * f.width * views + (f.mono_atlas ? 0 : eye) * f.width + x) * 2;
                result.depth_codes[eye * n + y * f.width + x] = get(raw, cursor, 2);
            }
    return result;
}
void embed_frame_marker(std::span<uint8_t> rgb, uint32_t w, uint32_t h, uint32_t sequence) {
    require(rgb.size() == pixels(w, h) * 3 && w >= frame_marker_width && h >= frame_marker_height,
            "Invalid RGB frame marker dimensions");
    const uint32_t word = 0xa50000 | (sequence & 65535);
    for (uint32_t bit = 0; bit < 24; ++bit)
        for (uint32_t complement = 0; complement < 2; ++complement)
            for (uint32_t y = 0; y < 8; ++y)
                for (uint32_t x = (bit * 2 + complement) * 8; x < (bit * 2 + complement + 1) * 8;
                     ++x) {
                    const uint8_t level = (((word >> (23 - bit)) & 1) ^ complement) ? 224 : 32;
                    std::fill_n(rgb.data() + (size_t(y) * w + x) * 3, 3, level);
                    std::fill_n(rgb.data() + (size_t(h - 1 - y) * w + x) * 3, 3, level);
                }
}
uint16_t decode_frame_marker(std::span<const uint8_t> rgb, uint32_t w, uint32_t h) {
    require(rgb.size() == pixels(w, h) * 3 && w >= frame_marker_width && h >= frame_marker_height,
            "Invalid RGB frame marker dimensions");
    uint32_t word = 0;
    for (uint32_t bit = 0; bit < 24; ++bit) {
        uint32_t sums[2]{};
        for (uint32_t c = 0; c < 2; ++c)
            for (uint32_t y = 0; y < 8; ++y)
                for (uint32_t x = (bit * 2 + c) * 8; x < (bit * 2 + c + 1) * 8; ++x)
                    for (uint32_t channel = 0; channel < 3; ++channel)
                        sums[c] += rgb[(size_t(y) * w + x) * 3 + channel];
        word = (word << 1) | (sums[0] > sums[1]);
    }
    require((word >> 16) == 0xa5, "Invalid remote frame marker");
    return uint16_t(word);
}
std::vector<uint8_t> pack_ros_debug_frame(std::span<const uint8_t> jpeg,
                                          std::span<const uint8_t> frame) {
    require(!jpeg.empty() && !frame.empty() && jpeg.size() <= payload_limit &&
                frame.size() <= payload_limit,
            "Invalid ROS debug frame components");
    std::vector<uint8_t> out{'H', 'R', 'R', 'D', 1, 0, 0, 0};
    out.reserve(16 + jpeg.size() + frame.size());
    put(out, frame.size(), 4);
    put(out, jpeg.size(), 4);
    out.insert(out.end(), frame.begin(), frame.end());
    out.insert(out.end(), jpeg.begin(), jpeg.end());
    return out;
}
DebugFrameView unpack_ros_debug_frame(std::span<const uint8_t> in) {
    size_t p = 0;
    require(get(in, p, 4) == 0x44525248 && get(in, p, 1) == 1, "Invalid HRRD header");
    get(in, p, 3);
    auto f = get(in, p, 4), j = get(in, p, 4);
    require(f && j && f <= payload_limit && j <= payload_limit && in.size() == p + f + j,
            "Invalid ROS debug frame lengths");
    return {in.subspan(p + f, j), in.subspan(p, f)};
}
} // namespace horus::remote_rendering
