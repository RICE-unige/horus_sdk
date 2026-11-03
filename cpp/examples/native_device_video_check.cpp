#include <fstream>
#include <horus/remote_rendering/scene.hpp>
#include <horus/remote_rendering/video.hpp>
#include <iostream>

int main(int argc, char **argv) {
    using namespace horus::remote_rendering;
    try {
        if (argc != 7)
            throw std::invalid_argument("Usage: native_device_video_check SCENE KIND RENDER_PLUGIN "
                                        "VIDEO_PLUGIN VIEW_JSON OUTPUT_PREFIX");
        std::ifstream input(argv[5]);
        auto view = ViewState::from_json(Json::parse(input).dump());
        std::unique_ptr<CudaRenderer> geometry;
        std::unique_ptr<GaussianRenderer> gaussian;
        if (std::string(argv[2]) == "gaussian") {
            auto scene = load_gaussian_ply(argv[1]);
            gaussian = std::make_unique<GaussianRenderer>(argv[3], scene);
        } else {
            auto scene = load_scene(argv[1], {.bake_lighting = false});
            if (scene.faces.empty()) {
                geometry = std::make_unique<CudaRenderer>(argv[3], scene.vertices.size());
                geometry->upload(scene.vertices, scene.point_colors);
            } else
                geometry = std::make_unique<CudaRenderer>(argv[3], scene.mesh_input());
        }
        GpuRenderOptions o{256, 192, .05f, 100.f, 2};
        GpuVideoEncoder encoder(argv[4], o.width, o.height, 48, 30, 64000);
        std::ofstream output(std::string(argv[6]) + ".h264", std::ios::binary);
        if (!output)
            throw std::runtime_error("Cannot write video");
        Json packets = Json::array();
        auto save = [&](const auto &frames) {
            for (const auto &f : frames) {
                output.write(reinterpret_cast<const char *>(f.data.data()), f.data.size());
                packets.push_back({{"frame_index", f.frame_index},
                                   {"source_sequence", f.source_sequence},
                                   {"keyframe", f.keyframe},
                                   {"metadata", Json::parse(f.metadata_json)}});
            }
        };
        for (uint64_t i = 0; i < 12; ++i) {
            view.workspace.revision = i < 5 ? 0 : 1;
            view.view_sequence = i + 1;
            auto context = StereoFrameContext::from_view(view, 100 + i);
            const std::array eyes{view.left_eye, view.right_eye};
            if (i == 0) {
                auto rendered =
                    gaussian ? gaussian->render_views(eyes, o) : geometry->render_views(eyes, o);
                context.codec = "h264";
                context.marker_height = 16;
                context.depth_strip_height = 48;
                auto reference = assemble_stereo_frame(rendered.rgb, rendered.depth, o, context);
                std::ofstream raw(std::string(argv[6]) + ".rgb", std::ios::binary);
                raw.write(reinterpret_cast<const char *>(reference.video_rgb.data()),
                          reference.video_rgb.size());
                if (!raw)
                    throw std::runtime_error("Cannot write reference");
            }
            if (i == 7)
                encoder.request_keyframe();
            save(gaussian ? encoder.encode(*gaussian, eyes, o, context)
                          : encoder.encode(*geometry, eyes, o, context));
        }
        save(encoder.close());
        if (!encoder.close().empty() || !output || packets.size() != 12)
            throw std::runtime_error("Incomplete device video output");
        std::cout << packets.dump() << '\n';
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
