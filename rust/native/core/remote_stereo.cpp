#include "horus/remote_rendering/frame.hpp"
#include "horus/remote_rendering/stereo.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <png.h>

namespace horus::remote_rendering {
namespace {
void require(bool condition, const char *message) {
    if (!condition)
        throw std::invalid_argument(message);
}
size_t pixels(const GpuRenderOptions &o, size_t views) {
    require(views && views <= 12 && o.width && o.height && o.width <= 16384 && o.height <= 16384,
            "Invalid render dimensions");
    const auto n = uint64_t(o.width) * o.height * views;
    require(n <= 256 * 1024 * 1024 / 7, "CPU output exceeds 256 MiB frame budget");
    require(std::isfinite(o.near_m) && std::isfinite(o.far_m) && o.near_m > 0 && o.far_m > o.near_m,
            "Invalid depth range");
    require(o.point_radius >= 0 && o.point_radius <= 8, "Invalid point radius");
    return n;
}
uint32_t round_even(float v) {
    const auto base = uint32_t(std::floor(v));
    const auto fraction = v - float(base);
    return base + (fraction > .5f || (fraction == .5f && base % 2));
}
} // namespace
RenderedViews render_colored_points(std::span<const Point3f> points, std::span<const Rgb8> colors,
                                    std::span<const EyeView> eyes, const GpuRenderOptions &o) {
    const auto n = pixels(o, eyes.size());
    require(points.size() == colors.size(), "Point/color count mismatch");
    RenderedViews result{o.width,
                         o.height,
                         uint32_t(eyes.size()),
                         std::vector<uint8_t>(n * 3),
                         std::vector<float>(n, std::numeric_limits<float>::infinity()),
                         0};
    struct Projected {
        int x, y;
        float z;
        Rgb8 color;
    };
    std::vector<Projected> projected;
    projected.reserve(points.size());
    for (size_t eye = 0; eye < eyes.size(); ++eye) {
        const auto matrix = eye_view_matrix(eyes[eye]);
        const auto projection = eyes[eye].fov.renderer_projection();
        projected.clear();
        for (size_t i = 0; i < points.size(); ++i) {
            const auto &p = points[i];
            if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]))
                continue;
            std::array<float, 3> camera{};
            for (size_t k = 0; k < 3; ++k)
                camera[k] = float(matrix[k][0] * p[0] + matrix[k][1] * p[1] + matrix[k][2] * p[2] +
                                  matrix[k][3]);
            if (camera[2] < o.near_m || camera[2] > o.far_m)
                continue;
            const auto x =
                (projection[0] * camera[0] / camera[2] + projection[2] + 1.) * .5 * o.width;
            const auto y =
                (1. - projection[1] * camera[1] / camera[2] - projection[3]) * .5 * o.height;
            const auto margin = o.point_radius + 1;
            if (!std::isfinite(x) || !std::isfinite(y) || x < -margin || y < -margin ||
                x >= double(o.width) + margin || y >= double(o.height) + margin)
                continue;
            projected.push_back({int(std::floor(x)), int(std::floor(y)), camera[2], colors[i]});
        }
        // Offset order and strict comparison preserve the Python tie-breaking contract.
        for (int dy = -o.point_radius; dy <= o.point_radius; ++dy)
            for (int dx = -o.point_radius; dx <= o.point_radius; ++dx) {
                if (dx * dx + dy * dy > o.point_radius * o.point_radius + 1)
                    continue;
                for (const auto &p : projected) {
                    const auto x = p.x + dx, y = p.y + dy;
                    if (x < 0 || y < 0 || x >= int(o.width) || y >= int(o.height))
                        continue;
                    const auto at = (eye * o.height + y) * o.width + x;
                    if (p.z >= result.depth[at])
                        continue;
                    result.depth[at] = p.z;
                    std::copy(p.color.begin(), p.color.end(), result.rgb.begin() + at * 3);
                }
            }
    }
    return result;
}
StereoFrameContext StereoFrameContext::from_view(const ViewState &state, uint64_t sequence) {
    StereoFrameContext out;
    out.session_id = state.session_id;
    out.view_stream_id = state.view_stream_id;
    out.sequence = sequence;
    out.source_view_sequence = state.view_sequence;
    out.workspace_revision = state.workspace.revision;
    out.predicted_display_time_ns = state.predicted_display_time_ns;
    return out;
}
Json stereo_frame_metadata(const GpuRenderOptions &o, const StereoFrameContext &c) {
    pixels(o,2);
    const auto width = uint64_t(o.width) * 2;
    const auto video_height = uint64_t(o.height) + c.depth_strip_height + c.marker_height;
    require(width * video_height <= 256 * 1024 * 1024 / 7, "Stereo atlas exceeds frame budget");
    require(c.depth_strip_height == 0 || o.width % 4 == 0,
            "Stereo depth width must be divisible by four");
    require(c.marker_height == 0 || (c.marker_height >= 8 && width >= 384),
            "Frame marker strip too small");
    require(c.render_complete_ns >= c.render_start_ns && c.capture_time_ns >= c.render_complete_ns,
            "Non-monotonic frame timestamps");
    return validate_frame_metadata(Json{
        {"version", frame_metadata_version},
        {"session_id", c.session_id},
        {"frame_id", c.sequence},
        {"source_view_sequence", c.source_view_sequence},
        {"view_stream_id", c.view_stream_id},
        {"predicted_display_time_ns", c.predicted_display_time_ns},
        {"workspace_revision", c.workspace_revision},
        {"render_start_ns", c.render_start_ns},
        {"render_complete_ns", c.render_complete_ns},
        {"encode_start_ns", c.capture_time_ns},
        {"capture_time_ns", c.capture_time_ns},
        {"stereo_layout", c.depth_strip_height ? "side_by_side_with_depth" : "side_by_side"},
        {"width_per_eye", o.width},
        {"height_per_eye", o.height},
        {"codec", c.codec},
        {"color_space", "srgb"},
        {"color_range", c.codec == "h264" ? "limited" : "full"},
        {"depth_representation", c.depth_strip_height ? "linear_u16_nibbles_in_video" : "none"},
        {"visibility_mask_height", c.depth_strip_height},
        {"frame_marker_height", c.marker_height},
        {"frame_marker", c.sequence & 65535},
        {"video_height", video_height},
        {"depth_near_m", o.near_m},
        {"depth_far_m", o.far_m}});
}
StereoFrameRender assemble_stereo_frame(std::span<const uint8_t> rgb, std::span<const float> depth,
                                        const GpuRenderOptions &o, const StereoFrameContext &c) {
    const auto n=pixels(o,2), width=uint64_t(o.width)*2;
    require(rgb.size()==n*3 && depth.size()==n,"Stereo input shape mismatch");
    const auto video_height=uint64_t(o.height)+c.depth_strip_height+c.marker_height;
    StereoFrameRender result;
    result.metadata=stereo_frame_metadata(o,c);
    result.stereo_rgba.resize(n * 4);
    result.video_rgb.resize(width * video_height * 3);
    size_t visible = 0;
    for (size_t eye = 0; eye < 2; ++eye)
        for (size_t y = 0; y < o.height; ++y)
            for (size_t x = 0; x < o.width; ++x) {
                const auto source = (eye * o.height + y) * o.width + x,
                           target = y * width + eye * o.width + x;
                std::copy_n(rgb.data() + source * 3, 3, result.stereo_rgba.data() + target * 4);
                std::copy_n(rgb.data() + source * 3, 3, result.video_rgb.data() + target * 3);
                const auto valid = std::isfinite(depth[source]);
                result.stereo_rgba[target * 4 + 3] = valid ? 255 : 0;
                visible += valid;
            }
    result.visibility_fraction = double(visible) / n;
    if (c.depth_strip_height) {
        visible = 0;
        const auto pw = o.width / 4;
        for (size_t eye = 0; eye < 2; ++eye)
            for (size_t y = 0; y < c.depth_strip_height; ++y) {
                const auto sy =
                    ((uint64_t(y) * 2 + 1) * o.height) / (uint64_t(c.depth_strip_height) * 2);
                for (size_t x = 0; x < pw; ++x) {
                    float d = std::numeric_limits<float>::infinity();
                    for (size_t k = 0; k < 4; ++k) {
                        auto v = depth[(eye * o.height + sy) * o.width + x * 4 + k];
                        if (std::isfinite(v) && v >= o.near_m && v <= o.far_m)
                            d = std::min(d, v);
                    }
                    if (!std::isfinite(d))
                        continue;
                    ++visible;
                    const auto code = round_even(
                        std::clamp((d - o.near_m) / (o.far_m - o.near_m), 0.f, 1.f) * 65535.f);
                    for (size_t k = 0; k < 4; ++k) {
                        auto luma = uint8_t(
                            round_even(32.f + float((code >> (12 - 4 * k)) & 15) * 192.f / 15.f));
                        const auto at = ((o.height + y) * width + eye * o.width + k * pw + x) * 3;
                        std::fill_n(result.video_rgb.data() + at, 3, luma);
                    }
                }
            }
        result.visibility_fraction = double(visible) / (2 * size_t(pw) * c.depth_strip_height);
    }
    if (c.marker_height) {
        auto strip =
            std::span(result.video_rgb).subspan((video_height - c.marker_height) * width * 3);
        embed_frame_marker(strip, uint32_t(width), c.marker_height, uint32_t(c.sequence));
    }
    return result;
}
std::vector<uint8_t> encode_stereo_png(const StereoFrameRender &frame) {
    auto metadata = validate_frame_metadata(frame.metadata);
    require(metadata.at("codec") == "png", "PNG encoding requires PNG metadata");
    const auto w = metadata.at("width_per_eye").get<uint64_t>() * 2;
    const auto h = metadata.at("video_height").get<uint64_t>();
    require(w && h && w * h <= 256 * 1024 * 1024 / 7 && frame.video_rgb.size() == w * h * 3,
            "Invalid PNG atlas");
    png_image image{};
    struct End {
        png_image* image;
        ~End() { png_image_free(image); }
    } end{&image};
    image.version = PNG_IMAGE_VERSION;
    image.width = w;
    image.height = h;
    image.format = PNG_FORMAT_RGB;
    png_alloc_size_t size = 0;
    require(
        png_image_write_to_memory(&image, nullptr, &size, 0, frame.video_rgb.data(), 0, nullptr),
        "PNG size query failed");
    require(size <= maximum_image_bytes, "PNG exceeds diagnostic carrier budget");
    std::vector<uint8_t> result(size);
    require(png_image_write_to_memory(&image, result.data(), &size, 0, frame.video_rgb.data(), 0,
                                      nullptr),
            "PNG encoding failed");
    result.resize(size);
    return result;
}
std::vector<uint8_t> pack_stereo_diagnostic(const StereoFrameRender &frame) {
    return pack_ros_diagnostic_frame(frame.metadata, encode_stereo_png(frame));
}
} // namespace horus::remote_rendering
