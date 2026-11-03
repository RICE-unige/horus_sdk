#include <cmath>
#include <csignal>
#include <fstream>
#include <horus/remote_rendering/publisher.hpp>
#include <horus/remote_rendering/scene.hpp>
#include <horus/remote_rendering/gaussian.hpp>
#include <horus/remote_rendering/video.hpp>
#include <iostream>
#include <thread>

namespace {
volatile std::sig_atomic_t interrupted = 0;
}
int main(int argc, char **argv) {
    using namespace horus::remote_rendering;
    try {
        if (argc != 4)
            throw std::invalid_argument(
                "Usage: native_remote_render_agent SCENE CUDA_LIBRARY CONFIG_JSON");
        std::ifstream input(argv[3]);
        const auto cfg = Json::parse(input);
        RemotePublisherOptions p;
        p.mode = cfg.value("mode", p.mode);
        p.session_id = cfg.value("session_id", p.session_id);
        p.view_topic = cfg.value("view_topic", p.view_topic);
        p.stream_topic = cfg.value("stream_topic", p.stream_topic);
        p.status_topic = cfg.value("status_topic", p.status_topic);
        p.maximum_view_age_seconds =
            cfg.value("maximum_view_age_seconds", p.maximum_view_age_seconds);
        GpuRenderOptions options{cfg.value("width", 640u), cfg.value("height", 480u),
                                 cfg.value("near_m", .05f), cfg.value("far_m", 100.f),
                                 cfg.value("point_radius", 1)};
        const auto fps = cfg.value("fps", 30u),
                   depth_height = cfg.value("depth_strip_height", 120u);
        const double duration = cfg.value("duration_seconds", 0.0);
        const uint64_t video_height =
            uint64_t(options.height) + (p.mode == "diagnostic" ? 0 : uint64_t(depth_height) + 16);
        if (p.mode != "diagnostic" && p.mode != "rgb" && p.mode != "h264")
            throw std::invalid_argument("mode must be diagnostic, rgb or h264");
        if (!std::isfinite(options.near_m) || !std::isfinite(options.far_m) ||
            options.near_m <= 0 || options.far_m <= options.near_m || options.point_radius < 0 ||
            options.point_radius > 8 ||
            uint64_t(options.width) * 2 * video_height > maximum_image_bytes / 3)
            throw std::invalid_argument(
                "Invalid depth/point settings or frame exceeds publication budget");
        if (!fps || fps > 120 || !std::isfinite(duration) || duration < 0 || duration > 86400 ||
            options.width < 192 || options.width > 4096 || options.width % 4 ||
            options.height < 16 || options.height > 4096 || options.height % 2 ||
            depth_height > 4096 || depth_height % 2)
            throw std::invalid_argument("Invalid render dimensions, frame rate or duration");
        std::unique_ptr<CudaRenderer> renderer;
        std::unique_ptr<GaussianRenderer> gaussian;
        std::size_t vertices=0, triangles=0, gaussians=0;
        const auto kind=cfg.value("scene_kind", "geometry");
        if (kind == "gaussian") {
            auto scene=load_gaussian_ply(argv[1], {.world_scale=cfg.value("world_scale",1.0),
                .canonicalize_y_down=cfg.value("canonicalize_y_down",false)});
            gaussians=scene.means.size();
            gaussian=std::make_unique<GaussianRenderer>(argv[2],scene);
        } else if (kind == "geometry") {
            auto scene=load_scene(argv[1], {.world_scale=cfg.value("world_scale",1.0),
                .bake_lighting=cfg.value("bake_lighting",true)});
            vertices=scene.vertices.size(); triangles=scene.faces.size();
            if (scene.faces.empty()) {
                renderer=std::make_unique<CudaRenderer>(argv[2],vertices);
                renderer->upload(scene.vertices,scene.point_colors);
            } else renderer=std::make_unique<CudaRenderer>(argv[2],scene.mesh_input());
        } else throw std::invalid_argument("scene_kind must be geometry or gaussian");
        std::unique_ptr<AsyncH264Encoder> encoder;
        std::unique_ptr<GpuVideoEncoder> device_encoder;
        const auto video_library = cfg.value("video_library", std::string{});
        if (!video_library.empty() && p.mode != "h264")
            throw std::invalid_argument("video_library requires h264 mode");
        if (p.mode == "h264" && !video_library.empty())
            device_encoder = std::make_unique<GpuVideoEncoder>(video_library, options.width,
                options.height, depth_height, fps, cfg.value("bitrate_kbps",12000u));
        else if (p.mode == "h264")
            encoder = std::make_unique<AsyncH264Encoder>(
                H264EncoderOptions{options.width * 2, options.height + depth_height + 16, fps,
                                   cfg.value("bitrate_kbps", 12000u)});
        RemotePublisher publisher(p);
        const auto old_int = std::signal(SIGINT, +[](int) { interrupted = 1; });
        const auto old_term = std::signal(SIGTERM, +[](int) { interrupted = 1; });
        const auto start = std::chrono::steady_clock::now();
        auto next_frame = start;
        uint64_t sequence = 0;
        const auto host_pixels = device_encoder ? 0 : std::size_t(options.width) * options.height * 2;
        std::vector<uint8_t> rgb(host_pixels * 3);
        std::vector<float> depth(host_pixels);
        auto forward = [&](const std::vector<EncodedH264Frame> &frames) {
            for (const auto &packet : frames)
                if (!publisher.publish_h264(packet)) {
                    if (encoder) encoder->request_keyframe();
                    if (device_encoder) device_encoder->request_keyframe();
                }
        };
        while (!interrupted &&
               (!duration ||
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <
                    duration)) {
            if (encoder)
                forward(encoder->poll_encoded(8, 0));
            std::this_thread::sleep_until(next_frame);
            auto view = publisher.next_view(50);
            if (!view) {
                if (!publisher.status().at("running").get<bool>())
                    throw std::runtime_error("Remote transport stopped");
                continue;
            }
            auto context = StereoFrameContext::from_view(*view, ++sequence);
            const auto render_start = std::chrono::steady_clock::now();
            if (device_encoder) {
                if (publisher.take_keyframe_request()) device_encoder->request_keyframe();
                const std::array eyes{view->left_eye, view->right_eye};
                forward(gaussian ? device_encoder->encode(*gaussian,eyes,options,context)
                    : device_encoder->encode(*renderer,eyes,options,context));
                next_frame = render_start + std::chrono::nanoseconds(1000000000 / fps);
                continue;
            }
            context.render_start_ns = monotonic_time_ns();
            float elapsed = 0;
            if (gaussian) gaussian->render_into(std::array{view->left_eye,view->right_eye},options,rgb,depth,elapsed);
            else renderer->render_into(std::array{view->left_eye, view->right_eye}, options, rgb, depth, elapsed);
            context.render_complete_ns = monotonic_time_ns();
            context.capture_time_ns = context.render_complete_ns;
            context.codec = p.mode == "diagnostic" ? "png" : "h264";
            context.marker_height = p.mode == "diagnostic" ? 0 : 16;
            context.depth_strip_height = p.mode == "diagnostic" ? 0 : depth_height;
            auto frame = assemble_stereo_frame(rgb, depth, options, context);
            if (encoder) {
                if (publisher.take_keyframe_request())
                    encoder->request_keyframe();
                encoder->submit_rgb(std::move(frame.video_rgb), frame.metadata.dump(), sequence);
            } else if (p.mode == "rgb")
                publisher.publish_rgb(frame);
            else
                publisher.publish_diagnostic(frame);
            next_frame = render_start + std::chrono::nanoseconds(1000000000 / fps);
        }
        if (encoder)
            forward(encoder->close());
        if (device_encoder) forward(device_encoder->close());
        publisher.stop();
        std::signal(SIGINT, old_int);
        std::signal(SIGTERM, old_term);
        auto status = publisher.status();
        status["scene_vertices"] = vertices;
        status["scene_triangles"] = triangles;
        status["scene_gaussians"] = gaussians;
        status["device_video"] = bool(device_encoder);
        status["encoded_inputs_replaced"] = encoder ? encoder->replaced_count() : 0;
        std::cout << status.dump() << '\n';
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
