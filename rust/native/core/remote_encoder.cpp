#include "horus/remote_rendering/encoder.hpp"
#include <algorithm>
#include <atomic>
#include <map>
#include <utility>

#ifdef HORUS_ENABLE_NVENC
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
}
#endif

namespace horus::remote_rendering {
namespace {
void validate(const H264EncoderOptions &o) {
    if (o.width < 16 || o.height < 16 || o.width > 8192 || o.height > 8192 || o.width % 2 ||
        o.height % 2 || std::uint64_t(o.width) * o.height > 16 * 1024 * 1024 || !o.fps ||
        o.fps > 240 || o.bitrate_kbps < 100 || o.bitrate_kbps > 500000 || o.gpu_id > 63)
        throw std::invalid_argument("Invalid NVENC dimensions, frame rate, bitrate or GPU");
}
#ifdef HORUS_ENABLE_NVENC
void check(int code, const char *operation) {
    if (code < 0) {
        char message[AV_ERROR_MAX_STRING_SIZE]{};
        av_strerror(code, message, sizeof(message));
        throw std::runtime_error(std::string(operation) + ": " + message);
    }
}
struct CodecDeleter {
    void operator()(AVCodecContext *p) const { avcodec_free_context(&p); }
};
struct FrameDeleter {
    void operator()(AVFrame *p) const { av_frame_free(&p); }
};
struct PacketDeleter {
    void operator()(AVPacket *p) const { av_packet_free(&p); }
};
#endif
} // namespace

struct H264Encoder::Impl {
    H264EncoderOptions options;
    std::mutex mutex;
    std::atomic<bool> keyframe{true};
    std::optional<Json> stream_identity;
    bool closed{};
    std::int64_t index{};
    struct Pending {
        std::string metadata;
        std::uint64_t sequence;
    };
    std::map<std::int64_t, Pending> pending;
#ifdef HORUS_ENABLE_NVENC
    std::unique_ptr<AVCodecContext, CodecDeleter> codec;
    std::unique_ptr<AVFrame, FrameDeleter> frame;
    std::unique_ptr<AVPacket, PacketDeleter> packet;
#endif
    explicit Impl(const H264EncoderOptions &o) : options(o) {
        validate(o);
#ifdef HORUS_ENABLE_NVENC
        auto encoder = avcodec_find_encoder_by_name("h264_nvenc");
        if (!encoder)
            throw std::runtime_error("FFmpeg has no h264_nvenc encoder");
        codec.reset(avcodec_alloc_context3(encoder));
        frame.reset(av_frame_alloc());
        packet.reset(av_packet_alloc());
        if (!codec || !frame || !packet)
            throw std::bad_alloc();
        codec->width = o.width;
        codec->height = o.height;
        codec->pix_fmt = AV_PIX_FMT_RGBA;
        codec->time_base = {1, static_cast<int>(o.fps)};
        codec->framerate = {static_cast<int>(o.fps), 1};
        codec->gop_size = o.fps;
        codec->max_b_frames = 0;
        codec->bit_rate = std::int64_t(o.bitrate_kbps) * 1000;
        codec->rc_max_rate = codec->bit_rate;
        codec->rc_buffer_size = std::max<std::int64_t>(100000, codec->bit_rate / o.fps);
        codec->color_primaries = AVCOL_PRI_BT709;
        codec->color_trc = AVCOL_TRC_BT709;
        codec->colorspace = AVCOL_SPC_BT709;
        codec->color_range = AVCOL_RANGE_MPEG;
        for (auto [name, value] : std::initializer_list<std::pair<const char *, const char *>>{
                 {"preset", "p1"},
                 {"tune", "ull"},
                 {"profile", "baseline"},
                 {"rc", "cbr"},
                 {"rgb_mode", "yuv420"},
                 {"multipass", "disabled"}})
            check(av_opt_set(codec->priv_data, name, value, 0), name);
        for (auto [name, value] :
             std::initializer_list<std::pair<const char *, std::int64_t>>{{"rc-lookahead", 0},
                                                                          {"delay", 0},
                                                                          {"zerolatency", 1},
                                                                          {"forced-idr", 1},
                                                                          {"surfaces", 2},
                                                                          {"gpu", o.gpu_id}})
            check(av_opt_set_int(codec->priv_data, name, value, 0), name);
        check(avcodec_open2(codec.get(), encoder, nullptr), "Open NVENC encoder");
        frame->format = codec->pix_fmt;
        frame->width = o.width;
        frame->height = o.height;
        frame->color_primaries = codec->color_primaries;
        frame->color_trc = codec->color_trc;
        frame->colorspace = codec->colorspace;
        frame->color_range = codec->color_range;
        check(av_frame_get_buffer(frame.get(), 32), "Allocate encoder surface");
#else
        throw std::runtime_error("Native NVENC support was not built; enable HORUS_ENABLE_NVENC or "
                                 "Rust's nvenc feature");
#endif
    }
    std::vector<EncodedH264Frame> receive() {
        std::vector<EncodedH264Frame> output;
#ifdef HORUS_ENABLE_NVENC
        while (true) {
            int code = avcodec_receive_packet(codec.get(), packet.get());
            if (code == AVERROR(EAGAIN) || code == AVERROR_EOF)
                break;
            check(code, "Receive H.264 access unit");
            auto found = pending.find(packet->pts);
            if (found == pending.end()) {
                av_packet_unref(packet.get());
                throw std::runtime_error("NVENC returned an uncorrelated frame timestamp");
            }
            EncodedH264Frame out;
            out.data.assign(packet->data, packet->data + packet->size);
            out.metadata_json = std::move(found->second.metadata);
            out.source_sequence = found->second.sequence;
            out.frame_index = static_cast<std::uint64_t>(packet->pts);
            out.keyframe = (packet->flags & AV_PKT_FLAG_KEY) != 0;
            pending.erase(found);
            av_packet_unref(packet.get());
            output.push_back(std::move(out));
        }
#endif
        return output;
    }
};
H264Encoder::H264Encoder(const H264EncoderOptions &o) : impl_(std::make_unique<Impl>(o)) {}
H264Encoder::~H264Encoder() = default;
bool H264Encoder::compiled() {
#ifdef HORUS_ENABLE_NVENC
    return true;
#else
    return false;
#endif
}
void H264Encoder::request_keyframe() { impl_->keyframe.store(true); }
std::vector<EncodedH264Frame> H264Encoder::encode_rgb(std::span<const std::uint8_t> rgb,
                                                      const std::string &metadata,
                                                      std::uint64_t sequence) {
    const auto &o = impl_->options;
    if (rgb.size() != std::size_t(o.width) * o.height * 3 ||
        metadata.size() > maximum_metadata_bytes)
        throw std::invalid_argument("Encoder input size or metadata exceeds its contract");
    auto parsed = validate_frame_metadata(Json::parse(metadata));
    if (parsed.at("codec") != "h264" ||
        parsed.at("width_per_eye").get<std::uint64_t>() * 2 != o.width ||
        parsed.at("video_height") != o.height || parsed.at("frame_id") != sequence)
        throw std::invalid_argument("Encoder atlas does not match H.264 frame metadata");
    std::lock_guard lock(impl_->mutex);
    if (impl_->closed)
        throw std::runtime_error("Encoder is closed");
    if (impl_->pending.size() >= 8 || impl_->index == INT64_MAX)
        throw std::runtime_error("NVENC pending-frame bound exceeded");
#ifdef HORUS_ENABLE_NVENC
    check(av_frame_make_writable(impl_->frame.get()), "Acquire encoder surface");
    for (std::size_t y = 0; y < o.height; ++y) {
        auto src = rgb.data() + y * o.width * 3;
        auto dst = impl_->frame->data[0] + y * impl_->frame->linesize[0];
        for (std::size_t x = 0; x < o.width; ++x) {
            dst[4 * x] = src[3 * x];
            dst[4 * x + 1] = src[3 * x + 1];
            dst[4 * x + 2] = src[3 * x + 2];
            dst[4 * x + 3] = 255;
        }
    }
    const Json identity =
        Json::array({parsed.at("session_id"), parsed.value("view_stream_id", std::string{}),
                     parsed.at("workspace_revision")});
    // A viewer/revision restart cannot reference frames discarded from the old stream.
    const bool requested = impl_->keyframe.exchange(false);
    const bool force = requested || !impl_->stream_identity || *impl_->stream_identity != identity;
    impl_->frame->pict_type = force ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_NONE;
    impl_->frame->pts = impl_->index;
    impl_->pending.emplace(impl_->index, Impl::Pending{metadata, sequence});
    int code = avcodec_send_frame(impl_->codec.get(), impl_->frame.get());
    if (code < 0) {
        impl_->pending.erase(impl_->index);
        if (force)
            impl_->keyframe.store(true);
        check(code, "Encode H.264 frame");
    }
    impl_->stream_identity = identity;
    ++impl_->index;
#endif
    return impl_->receive();
}
std::vector<EncodedH264Frame> H264Encoder::close() {
    std::lock_guard lock(impl_->mutex);
    if (impl_->closed)
        return {};
    impl_->closed = true;
#ifdef HORUS_ENABLE_NVENC
    check(avcodec_send_frame(impl_->codec.get(), nullptr), "Flush NVENC");
#endif
    auto output = impl_->receive();
    if (!impl_->pending.empty())
        throw std::runtime_error("NVENC closed with missing encoded frames");
    return output;
}
} // namespace horus::remote_rendering
