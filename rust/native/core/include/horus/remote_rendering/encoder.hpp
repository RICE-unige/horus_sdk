#pragma once
#include "protocol.hpp"
#include <memory>

namespace horus::remote_rendering {
struct H264EncoderOptions {
    std::uint32_t width{}, height{}, fps{30}, bitrate_kbps{12000}, gpu_id{};
};
struct EncodedH264Frame {
    std::vector<std::uint8_t> data;
    std::string metadata_json;
    std::uint64_t source_sequence{}, frame_index{};
    bool keyframe{};
};
// Optional in-process NVENC backend. It accepts a complete RGB video atlas;
// this host-input path is not the device-resident CUDA encoder path.
class H264Encoder {
    struct Impl;
    std::unique_ptr<Impl> impl_;

  public:
    explicit H264Encoder(const H264EncoderOptions &);
    ~H264Encoder();
    H264Encoder(const H264Encoder &) = delete;
    H264Encoder &operator=(const H264Encoder &) = delete;
    static bool compiled();
    void request_keyframe();
    std::vector<EncodedH264Frame> encode_rgb(std::span<const std::uint8_t>,
                                             const std::string &metadata_json,
                                             std::uint64_t source_sequence);
    // Flushes delayed access units. Idempotent; encoding after close is rejected.
    std::vector<EncodedH264Frame> close();
};
// One waiting raw frame; obsolete raw frames are replaced before encoding.
// Encoded reference frames are never dropped. A slow consumer backpressures the
// worker, not the producer; close drains accepted work and joins the worker.
class AsyncH264Encoder {
    struct Impl;
    std::unique_ptr<Impl> impl_;

  public:
    explicit AsyncH264Encoder(const H264EncoderOptions &);
    ~AsyncH264Encoder();
    AsyncH264Encoder(const AsyncH264Encoder &) = delete;
    AsyncH264Encoder &operator=(const AsyncH264Encoder &) = delete;
    void submit_rgb(std::vector<std::uint8_t>, std::string metadata_json,
                    std::uint64_t source_sequence);
    void request_keyframe();
    std::vector<EncodedH264Frame> poll_encoded(std::size_t maximum_frames = 1,
                                               std::uint32_t timeout_ms = 0);
    std::vector<EncodedH264Frame> close();
    std::uint64_t replaced_count() const;
};
} // namespace horus::remote_rendering
