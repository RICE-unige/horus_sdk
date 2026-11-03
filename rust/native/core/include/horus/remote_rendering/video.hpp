#pragma once
#include "encoder.hpp"
#include "gaussian.hpp"
#include "stereo.hpp"
#include <functional>

namespace horus::remote_rendering {
// Synchronous, bounded device pipeline. Geometry, RGB, depth and video packing
// stay on the GPU; only compressed H.264 access units return to host memory.
class GpuVideoEncoder {
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::vector<EncodedH264Frame>
    encode_impl(const GpuRenderOptions &, StereoFrameContext,
                const std::function<void(std::uint8_t *, float *, float &)> &);

  public:
    GpuVideoEncoder(const std::string &video_library, std::uint32_t width_per_eye,
                    std::uint32_t height, std::uint32_t depth_height, std::uint32_t fps = 30,
                    std::uint32_t bitrate_kbps = 12000);
    ~GpuVideoEncoder();
    GpuVideoEncoder(const GpuVideoEncoder &) = delete;
    GpuVideoEncoder &operator=(const GpuVideoEncoder &) = delete;
    std::vector<EncodedH264Frame> encode(CudaRenderer &, std::span<const EyeView>,
                                         const GpuRenderOptions &, StereoFrameContext);
    std::vector<EncodedH264Frame> encode(GaussianRenderer &, std::span<const EyeView>,
                                         const GpuRenderOptions &, StereoFrameContext);
    void request_keyframe();
    std::vector<EncodedH264Frame> close();
};
} // namespace horus::remote_rendering
