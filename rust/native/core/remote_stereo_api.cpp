#include "horus/remote_rendering/stereo.hpp"
#include "remote_stereo.h"
#include <algorithm>
#include <cstring>
#include <memory>

namespace {
using namespace horus::remote_rendering;
struct Result {
    StereoFrameRender frame;
    std::string metadata;
};
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
            *error = strdup("Native stereo operation failed");
    }
    return 0;
}
template <class T> std::span<T> view(T *p, size_t n) {
    if ((!p && n) || n > 256 * 1024 * 1024 / sizeof(T))
        throw std::invalid_argument("Invalid frame buffer");
    return {p, n};
}
} // namespace
extern "C" {
uint64_t horus_remote_monotonic_time_ns(void) {
    return horus::remote_rendering::monotonic_time_ns();
}
void *horus_stereo_assemble(const uint8_t *rgb, size_t rn, const float *depth, size_t dn,
                            uint32_t w, uint32_t h, float near_m, float far_m, const char *context,
                            char **error) {
    std::unique_ptr<Result> result;
    checked(error, [&] {
        if (!context || strnlen(context, maximum_metadata_bytes + 1) > maximum_metadata_bytes)
            throw std::invalid_argument("Missing or oversized stereo context");
        auto j = Json::parse(context);
        StereoFrameContext c;
        c.session_id = j.at("session_id");
        c.view_stream_id = j.value("view_stream_id", std::string{});
        c.codec = j.value("codec", std::string("png"));
        for (const auto &[key, target] : std::initializer_list<std::pair<const char *, uint64_t *>>{
                 {"sequence", &c.sequence},
                 {"source_view_sequence", &c.source_view_sequence},
                 {"workspace_revision", &c.workspace_revision},
                 {"predicted_display_time_ns", &c.predicted_display_time_ns},
                 {"render_start_ns", &c.render_start_ns},
                 {"render_complete_ns", &c.render_complete_ns},
                 {"capture_time_ns", &c.capture_time_ns}}) {
            if (j.contains(key)) {
                if (!j[key].is_number_unsigned())
                    throw std::invalid_argument("Invalid unsigned timestamp/sequence");
                *target = j[key].get<uint64_t>();
            }
        }
        for (const auto &[key, target] : std::initializer_list<std::pair<const char *, uint32_t *>>{
                 {"depth_strip_height", &c.depth_strip_height},
                 {"marker_height", &c.marker_height}}) {
            if (j.contains(key)) {
                if (!j[key].is_number_unsigned() || j[key].get<uint64_t>() > UINT32_MAX)
                    throw std::invalid_argument("Invalid auxiliary height");
                *target = j[key].get<uint32_t>();
            }
        }
        auto r = std::make_unique<Result>();
        r->frame =
            assemble_stereo_frame(view(rgb, rn), view(depth, dn), {w, h, near_m, far_m, 0}, c);
        r->metadata = r->frame.metadata.dump();
        result = std::move(r);
    });
    return result.release();
}
const uint8_t *horus_stereo_data(const void *p, int which, size_t *n) {
    if (!p || !n || (which != 0 && which != 1))
        return nullptr;
    auto &r = static_cast<const Result *>(p)->frame;
    auto &b = which == 0 ? r.stereo_rgba : r.video_rgb;
    *n = b.size();
    return b.data();
}
const char *horus_stereo_metadata(const void *p) {
    return p ? static_cast<const Result *>(p)->metadata.c_str() : nullptr;
}
double horus_stereo_visibility(const void *p) {
    return p ? static_cast<const Result *>(p)->frame.visibility_fraction : 0;
}
void *horus_stereo_diagnostic(const void *p, char **error) {
    std::unique_ptr<std::vector<uint8_t>> out;
    checked(error, [&] {
        if (!p)
            throw std::invalid_argument("Missing stereo frame");
        out = std::make_unique<std::vector<uint8_t>>(
            pack_stereo_diagnostic(static_cast<const Result *>(p)->frame));
    });
    return out.release();
}
void horus_stereo_destroy(void *p) { delete static_cast<Result *>(p); }
int horus_cpu_points(const float *points, const uint8_t *colors, size_t n,
                     const horus_gpu_eye *eyes, size_t count, uint32_t w, uint32_t h, float near_m,
                     float far_m, int radius, uint8_t *rgb, size_t rn, float *depth, size_t dn,
                     char **error) {
    return checked(error, [&] {
        if (!eyes || !count || count > 12 || uint64_t(count) * w * h != dn || dn > SIZE_MAX / 3 ||
            rn != dn * 3)
            throw std::invalid_argument("Invalid CPU output shape");
        auto rgb_target = view(rgb, rn);
        auto depth_target = view(depth, dn);
        std::array<EyeView, 12> e;
        for (size_t i = 0; i < count; ++i)
            e[i] = {{eyes[i].position[0], eyes[i].position[1], eyes[i].position[2]},
                    {eyes[i].rotation[0], eyes[i].rotation[1], eyes[i].rotation[2],
                     eyes[i].rotation[3]},
                    {eyes[i].fov[0], eyes[i].fov[1], eyes[i].fov[2], eyes[i].fov[3]}};
        auto result =
            render_colored_points(view(reinterpret_cast<const Point3f *>(points), n),
                                  view(reinterpret_cast<const Rgb8 *>(colors), n),
                                  std::span(e).first(count), {w, h, near_m, far_m, radius});
        std::copy(result.rgb.begin(), result.rgb.end(), rgb_target.begin());
        std::copy(result.depth.begin(), result.depth.end(), depth_target.begin());
    });
}
}
