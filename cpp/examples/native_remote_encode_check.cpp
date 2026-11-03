#include "horus/remote_rendering/encoder.hpp"
#include "horus/remote_rendering/stereo.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
    using namespace horus::remote_rendering;
    if (argc < 2 || argc > 3)
        return 2;
    const std::string mode = argc == 3 ? argv[2] : "sync";
    const bool asynchronous = mode != "sync", burst = mode == "burst";
    std::unique_ptr<H264Encoder> encoder;
    std::unique_ptr<AsyncH264Encoder> background;
    if (asynchronous)
        background = std::make_unique<AsyncH264Encoder>(H264EncoderOptions{384, 64, 30, 12000, 0});
    else
        encoder = std::make_unique<H264Encoder>(H264EncoderOptions{384, 64, 30, 12000, 0});
    GpuRenderOptions options{192, 48, .1f, 10.f, 0};
    std::vector<uint8_t> rgb(192 * 48 * 2 * 3);
    std::vector<float> depth(192 * 48 * 2, 2);
    std::ofstream file(argv[1], std::ios::binary);
    if (!file)
        return 2;
    Json packets = Json::array();
    auto save = [&](const std::vector<EncodedH264Frame> &frames) {
        for (const auto &f : frames) {
            file.write(reinterpret_cast<const char *>(f.data.data()), f.data.size());
            packets.push_back({{"frame_index", f.frame_index},
                               {"source_sequence", f.source_sequence},
                               {"keyframe", f.keyframe},
                               {"metadata", Json::parse(f.metadata_json)},
                               {"bytes", f.data.size()}});
        }
    };
    const uint64_t count = burst ? 200 : 12;
    for (uint64_t i = 0; i < count; ++i) {
        for (size_t p = 0; p < depth.size(); ++p) {
            rgb[3 * p] = p < 192 * 48 ? 200 : 40;
            rgb[3 * p + 1] = 30 + (i % 12) * 5;
            rgb[3 * p + 2] = p < 192 * 48 ? 40 : 200;
        }
        StereoFrameContext context;
        context.codec = "h264";
        context.sequence = 100 + i;
        context.marker_height = 16;
        context.workspace_revision = i < 5 ? 0 : 1;
        const auto frame = assemble_stereo_frame(rgb, depth, options, context);
        if (i == 7) {
            if (background)
                background->request_keyframe();
            else
                encoder->request_keyframe();
        }
        if (background) {
            background->submit_rgb(frame.video_rgb, frame.metadata.dump(), context.sequence);
            if (!burst) {
                const auto frames = background->poll_encoded(1, 5000);
                assert(frames.size() == 1);
                save(frames);
            } else if (i < 10)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
            auto encoded =
                encoder->encode_rgb(frame.video_rgb, frame.metadata.dump(), context.sequence);
            assert(encoded.size() == 1);
            save(encoded);
        }
    }
    if (background) {
        save(background->close());
        assert(background->close().empty());
        assert(background->poll_encoded(1, 0).empty());
        assert(packets.size() + background->replaced_count() == count);
        if (burst)
            assert(background->replaced_count() > 0 && packets.back()["source_sequence"] == 299);
        try {
            background->poll_encoded(0, 0);
            assert(false);
        } catch (const std::invalid_argument &) {
        }
    } else {
        save(encoder->close());
        assert(encoder->close().empty());
    }
    assert(packets[0]["keyframe"] == true);
    if (!burst)
        assert(packets.size() == 12 && packets[5]["keyframe"] == true &&
               packets[7]["keyframe"] == true);
    assert(file.good());
    std::cout << packets.dump() << '\n';
}
