#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace horus::remote_rendering {
inline constexpr uint32_t frame_marker_width = 384, frame_marker_height = 8;

struct FrameEye {
    std::array<float, 4> projection{1, 1, 0, 0};
    std::array<float, 3> position{};
    std::array<float, 4> rotation{0, 0, 0, 1};
};
struct DepthFrameInfo {
    uint32_t sequence = 0;
    uint64_t capture_time_ns = 0;
    uint32_t width = 0, height = 0, color_width = 0, color_height = 0;
    uint32_t view_count = 2;
    bool mono_atlas = false;
    float near_m = .05f, far_m = 100.f;
    std::array<FrameEye, 2> eyes{};
};
struct DepthFrame {
    DepthFrameInfo info;
    // Top-left, contiguous [eye][row][column], including a duplicated mono eye.
    std::vector<uint16_t> depth_codes;
};
enum class DepthReduction { Minimum, Surfel, Reprojection };

std::vector<uint16_t> quantize_depth(std::span<const float> depth, float near_m, float far_m);
std::vector<float> dequantize_depth(std::span<const uint16_t> codes, float near_m, float far_m);
std::vector<float> reduce_depth(std::span<const float> depth, uint32_t source_width,
                                uint32_t source_height, uint32_t width, uint32_t height,
                                DepthReduction mode, float edge_absolute_m = .08f,
                                float edge_relative = .035f);
// HRRF v1/v2, lossless gzip with deterministic metadata. No implicit clock read.
std::vector<uint8_t> pack_depth_frame(std::span<const float> depth, const DepthFrameInfo &info);
DepthFrame unpack_depth_frame(std::span<const uint8_t> frame);
void embed_frame_marker(std::span<uint8_t> rgb, uint32_t width, uint32_t height, uint32_t sequence);
uint16_t decode_frame_marker(std::span<const uint8_t> rgb, uint32_t width, uint32_t height);
std::vector<uint8_t> pack_ros_debug_frame(std::span<const uint8_t> jpeg,
                                          std::span<const uint8_t> frame);
struct DebugFrameView {
    std::span<const uint8_t> jpeg, frame;
};
DebugFrameView unpack_ros_debug_frame(std::span<const uint8_t> bytes);

enum class LumaDepthFormat { Static8, Dynamic16 };
std::vector<uint8_t> encode_luma_depth(std::span<const float> depth, uint32_t width,
                                       uint32_t height, float near_m, float far_m,
                                       LumaDepthFormat format = LumaDepthFormat::Static8);
std::vector<float> decode_luma_depth(std::span<const uint8_t> rgb, uint32_t width, uint32_t height,
                                     float near_m, float far_m,
                                     LumaDepthFormat format = LumaDepthFormat::Static8);
struct DynamicCameraPose {
    std::array<float, 3> position{};
    std::array<float, 4> rotation{0, 0, 0, 1};
};
uint32_t metadata_rows(uint32_t width);
uint32_t encode_dynamic_pose_metadata(std::span<uint8_t> rgb, uint32_t width, uint32_t height,
                                      const DynamicCameraPose &pose, uint32_t sequence,
                                      float position_range_m);
struct DynamicPoseMetadata {
    uint16_t sequence;
    DynamicCameraPose pose;
};
DynamicPoseMetadata decode_dynamic_pose_metadata(std::span<const uint8_t> rgb, uint32_t width,
                                                 uint32_t height, float position_range_m);
std::vector<uint8_t> pack_dynamic_rgbd(std::span<const uint8_t> color, std::span<const float> depth,
                                       uint32_t width, uint32_t height,
                                       const DynamicCameraPose &pose, uint32_t sequence,
                                       float near_m, float far_m, float position_range_m);
} // namespace horus::remote_rendering
