#include "horus/remote_rendering/encoder.hpp"
#include <condition_variable>
#include <deque>
#include <thread>

namespace horus::remote_rendering {
struct AsyncH264Encoder::Impl {
    H264Encoder codec;
    const H264EncoderOptions options;
    struct Input {
        std::vector<uint8_t> rgb;
        std::string metadata;
        uint64_t sequence;
    };
    mutable std::mutex mutex;
    std::mutex close_mutex;
    std::condition_variable wake;
    std::optional<Input> pending;
    std::deque<EncodedH264Frame> output;
    std::exception_ptr error;
    bool closing{}, done{};
    uint64_t replaced{};
    std::thread worker;
    explicit Impl(const H264EncoderOptions &o) : codec(o), options(o), worker([this] { run(); }) {}
    ~Impl() {
        try {
            close();
        } catch (...) {
        }
    }
    void append(std::vector<EncodedH264Frame> frames) {
        std::lock_guard lock(mutex);
        for (auto &frame : frames)
            output.push_back(std::move(frame));
        wake.notify_all();
    }
    void run() noexcept {
        try {
            while (true) {
                Input input;
                {
                    std::unique_lock lock(mutex);
                    wake.wait(lock, [&] { return closing || (pending && output.size() < 8); });
                    if (!pending)
                        break;
                    input = std::move(*pending);
                    pending.reset();
                }
                append(codec.encode_rgb(input.rgb, input.metadata, input.sequence));
            }
            append(codec.close());
        } catch (...) {
            std::lock_guard lock(mutex);
            error = std::current_exception();
        }
        {
            std::lock_guard lock(mutex);
            done = true;
            pending.reset();
        }
        wake.notify_all();
    }
    void check_error() {
        if (error)
            std::rethrow_exception(error);
    }
    std::vector<EncodedH264Frame> drain(std::size_t maximum) {
        std::vector<EncodedH264Frame> frames;
        while (!output.empty() && frames.size() < maximum) {
            frames.push_back(std::move(output.front()));
            output.pop_front();
        }
        wake.notify_all();
        return frames;
    }
    std::vector<EncodedH264Frame> close() {
        std::lock_guard serial(close_mutex);
        std::vector<EncodedH264Frame> frames;
        std::exception_ptr failure;
        {
            std::unique_lock lock(mutex);
            closing = true;
            wake.notify_all();
            wake.wait(lock, [&] { return done; });
            frames = drain(output.size());
            failure = error;
        }
        if (worker.joinable())
            worker.join();
        if (failure)
            std::rethrow_exception(failure);
        return frames;
    }
};
AsyncH264Encoder::AsyncH264Encoder(const H264EncoderOptions &o)
    : impl_(std::make_unique<Impl>(o)) {}
AsyncH264Encoder::~AsyncH264Encoder() = default;
void AsyncH264Encoder::submit_rgb(std::vector<uint8_t> rgb, std::string metadata,
                                  uint64_t sequence) {
    const auto &o = impl_->options;
    if (rgb.size() != std::size_t(o.width) * o.height * 3 ||
        metadata.size() > maximum_metadata_bytes)
        throw std::invalid_argument("Invalid asynchronous encoder input size");
    const auto m = validate_frame_metadata(Json::parse(metadata));
    if (m.at("codec") != "h264" || m.at("width_per_eye").get<uint64_t>() * 2 != o.width ||
        m.at("video_height") != o.height || m.at("frame_id") != sequence)
        throw std::invalid_argument("Asynchronous encoder metadata mismatch");
    std::lock_guard lock(impl_->mutex);
    impl_->check_error();
    if (impl_->closing || impl_->done)
        throw std::runtime_error("Asynchronous encoder is closed");
    if (impl_->pending)
        ++impl_->replaced;
    impl_->pending = Impl::Input{std::move(rgb), std::move(metadata), sequence};
    impl_->wake.notify_all();
}
void AsyncH264Encoder::request_keyframe() { impl_->codec.request_keyframe(); }
std::vector<EncodedH264Frame> AsyncH264Encoder::poll_encoded(std::size_t maximum,
                                                             uint32_t timeout) {
    if (!maximum || maximum > 32 || timeout > 60000)
        throw std::invalid_argument("Invalid encoder polling bounds");
    std::unique_lock lock(impl_->mutex);
    impl_->wake.wait_for(lock, std::chrono::milliseconds(timeout),
                         [&] { return impl_->done || !impl_->output.empty() || impl_->error; });
    impl_->check_error();
    return impl_->drain(maximum);
}
std::vector<EncodedH264Frame> AsyncH264Encoder::close() { return impl_->close(); }
uint64_t AsyncH264Encoder::replaced_count() const {
    std::lock_guard lock(impl_->mutex);
    return impl_->replaced;
}
} // namespace horus::remote_rendering
