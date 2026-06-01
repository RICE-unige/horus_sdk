#pragma once

#include "horus/robot/robot.hpp"
#include "horus/robot/sensors.hpp"

#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace examples {

using Robot = horus::robot::Robot;
using DataViz = horus::dataviz::DataViz;

struct RegistrationSet {
    std::vector<Robot> robots;
    std::vector<DataViz> datavizs;
};

inline std::string topic(const std::string& robot_name, const std::string& suffix) {
    return "/" + robot_name + "/" + suffix;
}

inline std::shared_ptr<horus::robot::Camera> standard_camera(
    const std::string& robot_name,
    const std::string& name = "front_camera") {
    auto camera = std::make_shared<horus::robot::Camera>(
        name,
        robot_name + "/camera_link",
        topic(robot_name, "camera/image_raw/compressed"),
        false,
        std::pair<int, int>{160, 90},
        6,
        60.0f,
        "jpeg",
        "ros",
        "ros",
        "ros");
    camera->set_minimap_image_type("compressed");
    camera->set_teleop_image_type("compressed");
    camera->add_metadata("image_type", std::string("compressed"));
    camera->configure_projected_view({
        .image_scale = 1.0f,
        .focal_length_scale = 0.55f,
        .show_frustum = true,
        .frustum_color = std::string("#E6E6E0A0"),
    });
    camera->configure_minimap_view({
        .size = 10.0f,
        .position_offset = std::vector<float>{0.0f, 2.0f, 0.0f},
        .face_camera = true,
        .rotation_offset = std::vector<float>{90.0f, 0.0f, 0.0f},
    });
    return camera;
}

inline Robot standard_robot(
    const std::string& name,
    horus::core::RobotType type,
    horus::robot::RobotDimensions dimensions,
    const std::string& profile,
    const std::string& base_frame = "base_link") {
    Robot robot(name, type, dimensions);
    robot.configure_ros_binding("prefixed", "prefixed", base_frame);
    robot.configure_robot_manager();
    robot.configure_teleop({
        .command_topic = topic(name, "cmd_vel"),
        .robot_profile = profile,
    });
    robot.configure_navigation_tasks({
        .goal_topic = topic(name, "goal_pose"),
        .cancel_topic = topic(name, "goal_cancel"),
        .goal_status_topic = topic(name, "goal_status"),
        .waypoint_path_topic = topic(name, "waypoint_path"),
        .waypoint_status_topic = topic(name, "waypoint_status"),
    });
    robot.add_sensor(standard_camera(name));
    return robot;
}

inline DataViz standard_dataviz(Robot& robot, bool include_trajectory = false) {
    auto dataviz = robot.create_dataviz();
    robot.add_path_planning_to_dataviz(
        *dataviz,
        robot.resolve_topic("global_path"),
        robot.resolve_topic("local_path"),
        include_trajectory ? std::optional<std::string>(robot.resolve_topic("trajectory")) : std::nullopt);
    robot.add_navigation_safety_to_dataviz(*dataviz);
    return *dataviz;
}

inline RegistrationSet standard_fleet(
    const std::vector<std::string>& names,
    horus::core::RobotType type,
    horus::robot::RobotDimensions dimensions,
    const std::string& profile) {
    RegistrationSet set;
    set.robots.reserve(names.size());
    set.datavizs.reserve(names.size());
    for (const auto& name : names) {
        set.robots.push_back(standard_robot(name, type, dimensions, profile));
        set.datavizs.push_back(standard_dataviz(set.robots.back()));
    }
    return set;
}

inline void add_world_maps(DataViz& dataviz) {
    dataviz.add_occupancy_grid(
        "/map",
        "map",
        {{"color_free", std::string("#222222")}, {"color_occupied", std::string("#F4F4F4")}, {"alpha", 0.85}});
    dataviz.add_3d_map(
        "/map_3d",
        "map",
        {
            {"point_size", 0.045},
            {"max_points_per_frame", 0},
            {"render_all_points", true},
            {"auto_point_size_by_workspace_scale", true},
            {"min_point_size", 0.002},
            {"max_point_size", 0.018},
            {"point_shape", std::string("circle")},
            {"enable_view_frustum_culling", false},
            {"enable_subpixel_culling", false},
            {"color", std::string("#6ED7FF")},
        });
    dataviz.add_3d_mesh(
        "/map_3d_mesh",
        "map",
        {{"max_triangles", 60000}, {"use_vertex_colors", true}});
    dataviz.add_3d_octomap(
        "/map_3d_octomap_mesh",
        "map",
        {{"render_mode", std::string("surface_mesh")}, {"native_topic", std::string("/map_3d_octomap")}});
}

inline void add_semantic_boxes(DataViz& dataviz) {
    dataviz.add_semantic_box("person_1", "person", {1.6f, 0.8f, 0.9f}, {0.45f, 0.45f, 1.8f});
    dataviz.add_semantic_box("person_2", "person", {-1.2f, 1.4f, 0.9f}, {0.45f, 0.45f, 1.8f});
    dataviz.add_semantic_box("equipment_1", "equipment", {0.3f, -1.6f, 0.4f}, {0.8f, 0.6f, 0.8f});
}

inline std::filesystem::path asset_dir() {
    return std::filesystem::path("..") / "python" / "examples" / ".local_assets" / "robot_descriptions";
}

inline void configure_robot_description(
    Robot& robot,
    const std::string& urdf_file,
    const std::string& base_frame,
    int triangle_budget = 90000,
    const std::string& body_mesh_mode = "preview_mesh") {
    robot.configure_ros_binding("prefixed", "prefixed", base_frame);
    robot.configure_robot_description({
        .urdf_path = (asset_dir() / urdf_file).string(),
        .base_frame = base_frame,
        .visual_mesh_triangle_budget = triangle_budget,
        .body_mesh_mode = body_mesh_mode,
    });
}

inline RegistrationSet robot_description_fleet() {
    struct Model {
        const char* name;
        horus::core::RobotType type;
        horus::robot::RobotDimensions dimensions;
        const char* base_frame;
        const char* urdf_file;
    };
    const std::vector<Model> models{
        {"anymal_c", horus::core::RobotType::LEGGED, {0.95, 0.55, 0.70}, "base", "anymal_c.urdf"},
        {"jackal", horus::core::RobotType::WHEELED, {0.51, 0.43, 0.25}, "base_link", "jackal.urdf"},
        {"go1", horus::core::RobotType::LEGGED, {0.65, 0.32, 0.45}, "base", "go1.urdf"},
        {"h1", horus::core::RobotType::LEGGED, {0.55, 0.38, 1.25}, "pelvis", "h1.urdf"},
    };

    RegistrationSet set;
    set.robots.reserve(models.size());
    set.datavizs.reserve(models.size());
    for (const auto& model : models) {
        auto robot = standard_robot(model.name, model.type, model.dimensions, horus::core::robot_type_to_string(model.type), model.base_frame);
        configure_robot_description(robot, model.urdf_file, model.base_frame);
        set.robots.push_back(std::move(robot));
        set.datavizs.push_back(standard_dataviz(set.robots.back()));
    }
    return set;
}

inline int register_or_fail(RegistrationSet& set, double workspace_scale = 0.1, bool keep_alive = true) {
    std::vector<Robot*> robot_ptrs;
    robot_ptrs.reserve(set.robots.size());
    for (auto& robot : set.robots) {
        robot_ptrs.push_back(&robot);
    }
    auto [ok, result] = horus::robot::register_robots(
        robot_ptrs,
        set.datavizs,
        10.0,
        keep_alive,
        true,
        workspace_scale);
    if (!ok) {
        std::cerr << "HORUS registration failed\n";
        return 1;
    }
    return 0;
}

} // namespace examples
