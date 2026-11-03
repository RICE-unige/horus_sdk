#include "registration_common.hpp"
#include <fstream>
#include <nlohmann/json.hpp>

int main(int argc, char **argv) {
    try {
        if (argc != 2)
            throw std::invalid_argument("Usage: remote_map_registration CONFIG_JSON");
        std::ifstream input(argv[1]);
        const auto config = nlohmann::json::parse(input);
        const auto mode = config.at("mode").get<std::string>();
        if (mode != "diagnostic" && mode != "rgb" && mode != "h264")
            throw std::invalid_argument("mode must be diagnostic, rgb or h264");
        horus::dataviz::RenderOptions options{
            {"protocol_version", std::string("horus.remote_render.v3")},
            {"transport", std::string(mode == "diagnostic" ? "ros_compressed" : "webrtc")},
            {"image_type", std::string(mode == "h264" ? "h264" : "raw")},
            {"frame_data_topic", std::string{}},
            {"session_id", config.at("session_id").get<std::string>()},
            {"viewer_pose_topic", config.at("view_topic").get<std::string>()},
            {"status_topic", config.at("status_topic").get<std::string>()},
            {"ros_compressed_topic", config.at("stream_topic").get<std::string>()},
            {"viewport_width", config.at("width").get<int>()},
            {"viewport_height", config.at("height").get<int>()},
            {"framerate", config.at("fps").get<int>()},
            {"bitrate_kbps", config.at("bitrate_kbps").get<int>()},
            {"maximum_frame_age_ms", config.at("maximum_frame_age_ms").get<int>()}};
        auto resource = horus::robot::make_workspace_visualization("workspace_remote_map");
        auto viz = resource.create_dataviz();
        viz->add_remote_rendered_map(config.at("stream_topic").get<std::string>(), "map", options);
        examples::RegistrationSet set{{std::move(resource)}, {*viz}};
        return examples::register_or_fail(set, config.at("workspace_scale").get<double>(),
            config.value("keep_alive", true), {.auto_launch=config.value("auto_launch_bridge", true)});
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
