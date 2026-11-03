#pragma once
#include "gpu.hpp"

namespace horus::remote_rendering {
// CPU reference renderer. Every point participates; there is no implicit decimation.
RenderedViews render_colored_points(std::span<const Point3f>, std::span<const Rgb8>,
                                    std::span<const EyeView>, const GpuRenderOptions & = {});
struct StereoFrameContext {
    std::string session_id{"preview"}, view_stream_id, codec{"png"};
    std::uint64_t sequence{}, source_view_sequence{}, workspace_revision{},
        predicted_display_time_ns{}, render_start_ns{}, render_complete_ns{}, capture_time_ns{};
    std::uint32_t depth_strip_height{}, marker_height{};
    static StereoFrameContext from_view(const ViewState &, std::uint64_t frame_sequence);
};
struct StereoFrameRender {
    Json metadata;
    std::vector<std::uint8_t> stereo_rgba, video_rgb;
    double visibility_fraction{};
};
Json stereo_frame_metadata(const GpuRenderOptions &, const StereoFrameContext &);
// rgb/depth are [left, right] top-left row-major images, not interleaved.
StereoFrameRender assemble_stereo_frame(std::span<const std::uint8_t> rgb,
                                        std::span<const float> depth, const GpuRenderOptions &,
                                        const StereoFrameContext &);
std::vector<std::uint8_t> encode_stereo_png(const StereoFrameRender &);
std::vector<std::uint8_t> pack_stereo_diagnostic(const StereoFrameRender &);
} // namespace horus::remote_rendering
