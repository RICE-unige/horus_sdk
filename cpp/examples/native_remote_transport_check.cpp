#include "horus/remote_rendering/publisher.hpp"
#include <cassert>
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
    using namespace horus::remote_rendering;
    RemotePublisherOptions config;
    config.view_topic = "/native_remote/view";
    config.stream_topic = "/native_remote/image";
    config.status_topic = "/native_remote/status";
    config.maximum_view_age_seconds = .5;
    if (argc > 1)
        config.mode = argv[1];
    RemotePublisher publisher(config);
    GpuRenderOptions options{192, 48, .1f, 10.f, 0};
    std::unique_ptr<H264Encoder> encoder;
    if (config.mode == "h264")
        encoder = std::make_unique<H264Encoder>(H264EncoderOptions{384, 64});
    const std::array<Point3f, 3> points{{{-.5f, 0, 2}, {0, 0, 2}, {.5f, 0, 2}}};
    const std::array<Rgb8, 3> colors{{{255, 0, 0}, {0, 255, 0}, {0, 0, 255}}};
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    uint64_t sequence = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        auto view = publisher.next_view(100);
        if (!view)
            continue;
        const auto rendered = render_colored_points(
            points, colors, std::array{view->left_eye, view->right_eye}, options);
        auto context = StereoFrameContext::from_view(*view, ++sequence);
        context.codec = config.mode == "diagnostic" ? "png" : "h264";
        context.marker_height = config.mode == "diagnostic" ? 0 : 16;
        auto frame = assemble_stereo_frame(rendered.rgb, rendered.depth, options, context);
        if (config.mode == "diagnostic") {
            if (publisher.publish_diagnostic(frame))
                assert(!publisher.publish_diagnostic(frame));
        } else if (config.mode == "rgb") {
            if (publisher.publish_rgb(frame))
                assert(!publisher.publish_rgb(frame));
        } else {
            if (publisher.take_keyframe_request())
                encoder->request_keyframe();
            for (const auto &packet :
                 encoder->encode_rgb(frame.video_rgb, frame.metadata.dump(), sequence))
                if (publisher.publish_h264(packet))
                    assert(!publisher.publish_h264(packet));
        }
    }
    if (encoder)
        for (const auto &packet : encoder->close())
            publisher.publish_h264(packet);
    // A stopped publisher must unblock a waiter instead of waiting out its timeout.
    std::thread waiter([&] {
        while (publisher.next_view(60000)) {
        };
    });
    publisher.stop();
    waiter.join();
    publisher.stop();
    assert(!publisher.next_view(0));
    std::cout << publisher.status().dump() << '\n';
}
