#include "horus/robot/robot.hpp"
#include "horus/robot/sensors.hpp"
#include "horus/utils/branding.hpp"
#include "horus/utils/logging.hpp"

#include <any>
#include <map>
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
        robot.configure_ros_binding("prefixed", "prefixed", "base_link");
        robot.configure_robot_manager(true, true, true, true);
        robot.configure_workspace_compass(true, 8088, "auto");
        robot.configure_workspace_tutorial("multi_robot_intro");
        if (i == 0) {
            robot.configure_local_body_model("rosbot", true);
        }
        robot.set_metadata(
            "teleop_config",
            std::map<std::string, std::any>{
                {"enabled", true},
                {"robot_profile", std::string("wheeled")},
                {"response_mode", std::string("analog")},
                {"publish_rate_hz", 60.0},
                {"axes",
                 std::map<std::string, std::any>{
                     {"deadzone", 0.12},
                     {"linear_xy_max_mps", 1.1 + (0.05 * i)},
                     {"angular_z_max_rps", 1.4},
                 }},
            });
        robot.set_metadata(
            "task_config",
            std::map<std::string, std::any>{
                {"go_to_point",
                 std::map<std::string, std::any>{
                     {"frame_id", std::string("map")},
                     {"position_tolerance_m", 0.18},
                 }},
                {"waypoint",
                 std::map<std::string, std::any>{
                     {"frame_id", std::string("map")},
                     {"position_tolerance_m", 0.25},
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
            camera->set_minimap_topic("/" + name + "/camera/minimap/compressed");
            camera->set_teleop_topic("/" + name + "/camera/image_raw/compressed");
            camera->set_minimap_image_type("compressed");
            camera->set_teleop_image_type("compressed");
            camera->set_minimap_max_fps(12);
            camera->add_metadata("image_type", std::string("compressed"));
            camera->add_metadata("streaming_type", std::string("ros"));
            camera->add_metadata("minimap_streaming_type", std::string("ros"));
            camera->add_metadata("teleop_streaming_type", std::string("webrtc"));
            camera->add_metadata("startup_mode", std::string("minimap"));
            camera->add_metadata("display_mode", std::string("projected"));
            camera->add_metadata("projection_target_frame", name + "/camera_link");
            camera->add_metadata(
                "projected_scale_multiplier",
                std::vector<float>{1.0f, 1.0f, 1.0f});
            robot.add_sensor(camera);
        }

        auto dataviz = robot.create_dataviz();
        robot.add_path_planning_to_dataviz(
            *dataviz,
            robot.resolve_topic("global_path"),
            robot.resolve_topic("local_path"),
            robot.resolve_topic("trajectory"));
        robot.add_navigation_safety_to_dataviz(*dataviz);
        if (with_occupancy) {
            horus::dataviz::RenderOptions render_options;
            render_options["show_unknown_space"] = true;
            dataviz->add_occupancy_grid("/map", "map", render_options);
        }
        dataviz->add_3d_map(
            "/" + name + "/map_points",
            "map",
            {
                {"point_size", 0.01},
                {"max_points_per_frame", 60000},
                {"enable_adaptive_quality", true},
                {"render_mode", std::string("opaque_fast")},
            });
        dataviz->add_3d_mesh(
            "/" + name + "/map_mesh",
            "map",
            {
                {"max_triangles", 200000},
                {"source_coordinate_space", std::string("enu")},
            });
        dataviz->add_3d_octomap(
            "/" + name + "/octomap_mesh",
            "map",
            {
                {"native_topic", "/" + name + "/octomap_binary"},
                {"native_frame", std::string("map")},
                {"render_mode", std::string("surface_mesh")},
            });
        dataviz->add_semantic_box(
            name + "_dock",
            "Dock",
            {static_cast<float>(i), 0.0f, 0.0f},
            {0.6f, 0.4f, 0.3f});
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
