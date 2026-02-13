#include "horus/robot/robot.hpp"
#include "horus/robot/sensors.hpp"
#include "horus/utils/branding.hpp"
#include "horus/utils/logging.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {
std::optional<std::string> get_arg(int argc, char** argv, const std::string& name) {
    for (int i = 1; i < argc - 1; ++i) {
        if (argv[i] == name) {
            return std::string(argv[i + 1]);
        }
    }
    return std::nullopt;
}
} // namespace

int main(int argc, char** argv) {
    horus::utils::show_ascii_art();

    const auto robot_count = get_arg(argc, argv, "--robot-count").has_value()
                                 ? std::max(1, std::stoi(*get_arg(argc, argv, "--robot-count")))
                                 : 4;
    const auto with_camera = !get_arg(argc, argv, "--with-camera").has_value() ||
                             (*get_arg(argc, argv, "--with-camera") != "false");
    const auto with_occupancy = !get_arg(argc, argv, "--with-occupancy-grid").has_value() ||
                                (*get_arg(argc, argv, "--with-occupancy-grid") != "false");
    const auto workspace_scale = get_arg(argc, argv, "--workspace-scale")
                                     ? std::optional<double>(std::stod(*get_arg(argc, argv, "--workspace-scale")))
                                     : std::nullopt;

    horus::utils::print_step("Defining Robot Configuration...");

    std::vector<horus::robot::Robot> robots;
    std::vector<horus::dataviz::DataViz> datavizs;
    std::vector<horus::robot::Robot*> robot_ptrs;

    robots.reserve(static_cast<std::size_t>(robot_count));
    datavizs.reserve(static_cast<std::size_t>(robot_count));
    robot_ptrs.reserve(static_cast<std::size_t>(robot_count));

    for (int i = 0; i < robot_count; ++i) {
        const auto name = "test_bot_" + std::to_string(i + 1);
        horus::robot::RobotDimensions dims{
            0.8 + (0.1 * i),
            0.6 + (0.05 * i),
            0.4 + (0.03 * i),
        };
        robots.emplace_back(name, horus::core::RobotType::WHEELED, dims);
        auto& robot = robots.back();
        robot.add_metadata(
            "robot_manager_config",
            std::map<std::string, std::any>{
                {"enabled", true},
                {"prefab_asset_path", std::string("Assets/Prefabs/UI/RobotManager.prefab")},
                {"prefab_resource_path", std::string("")},
                {"sections",
                 std::map<std::string, std::any>{
                     {"status", true},
                     {"data_viz", true},
                     {"teleop", true},
                     {"tasks", true},
                 }},
            });

        if (with_camera) {
            auto camera = std::make_shared<horus::robot::Camera>(
                "front_camera",
                name + "/camera_link",
                "/" + name + "/camera/image_raw/compressed");
            camera->set_streaming_type("ros");
            camera->set_minimap_streaming_type("ros");
            camera->set_teleop_streaming_type("webrtc");
            camera->set_startup_mode("minimap");
            camera->set_resolution({320, 180});
            camera->set_fps(6);
            camera->set_encoding("jpeg");
            camera->add_metadata("image_type", std::string("compressed"));
            camera->add_metadata("streaming_type", std::string("ros"));
            camera->add_metadata("minimap_streaming_type", std::string("ros"));
            camera->add_metadata("teleop_streaming_type", std::string("webrtc"));
            camera->add_metadata("startup_mode", std::string("minimap"));
            robot.add_sensor(camera);
        }

        auto dataviz = robot.create_dataviz();
        if (with_occupancy) {
            horus::dataviz::RenderOptions render_options;
            render_options["show_unknown_space"] = true;
            dataviz->add_occupancy_grid("/map", "map", render_options);
        }
        datavizs.push_back(*dataviz);
        robot_ptrs.push_back(&robot);
    }

    horus::utils::print_step("Registering robots...");
    auto [ok, result] =
        horus::robot::register_robots(robot_ptrs, datavizs, 10.0, true, true, workspace_scale);
    if (!ok) {
        horus::utils::print_error("Registration failed");
        return 1;
    }

    horus::utils::print_success("Registration completed");
    if (const auto it = result.find("results"); it != result.end()) {
        if (it->second.type() == typeid(std::vector<std::map<std::string, std::any>>)) {
            const auto entries =
                std::any_cast<std::vector<std::map<std::string, std::any>>>(it->second);
            horus::utils::print_info("Registered robots: " + std::to_string(entries.size()));
        }
    }
    return 0;
}

