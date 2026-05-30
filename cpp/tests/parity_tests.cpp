#include "horus/bridge/robot_registry.hpp"
#include "horus/robot/robot.hpp"
#include "horus/robot/sensors.hpp"

#include <any>
#include <cassert>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace {
template <typename T>
T any_value(const std::map<std::string, std::any>& values, const std::string& key) {
    return std::any_cast<T>(values.at(key));
}

const horus::bridge::VisualizationPayload* find_viz(
    const std::vector<horus::bridge::VisualizationPayload>& visualizations,
    const std::string& type) {
    for (const auto& visualization : visualizations) {
        if (visualization.type == type) {
            return &visualization;
        }
    }
    return nullptr;
}
} // namespace

static void test_camera_defaults_and_validation() {
    horus::robot::Camera camera("front", "robot/camera_link", "/robot/camera/image_raw");
    assert(camera.get_streaming_type() == "ros");
    assert(camera.get_minimap_streaming_type() == "ros");
    assert(camera.get_teleop_streaming_type() == "webrtc");
    assert(camera.get_startup_mode() == "minimap");

    bool failed = false;
    try {
        horus::robot::Camera invalid(
            "front",
            "robot/camera_link",
            "/robot/camera/image_raw",
            false,
            {640, 480},
            30,
            60.0f,
            "bgr8",
            "ros",
            "invalid");
    } catch (...) {
        failed = true;
    }
    assert(failed);
}

static void test_payload_parity_rules() {
    horus::robot::Robot robot("test_bot", horus::core::RobotType::WHEELED);
    auto camera = std::make_shared<horus::robot::Camera>(
        "front_camera",
        "test_bot/camera_link",
        "/test_bot/camera/image_raw/compressed");
    robot.add_sensor(camera);

    auto dataviz = robot.create_dataviz();
    dataviz->add_occupancy_grid("/map", "map");

    horus::bridge::RobotRegistryClient client;
    auto payload = client.build_robot_config_dict(robot, *dataviz);

    assert(payload.robot_type == "wheeled");
    assert(payload.control.drive_topic == "/test_bot/cmd_vel");
    assert(payload.control.teleop.command_topic == "/test_bot/cmd_vel");
    assert(payload.control.teleop.raw_input_topic == "/horus/teleop/test_bot/joy");
    assert(any_value<std::string>(payload.control.teleop.deadman, "policy") == "either_grip_trigger");
    assert(any_value<bool>(payload.control.teleop.axes, "invert_linear_x") == false);
    assert(any_value<std::string>(payload.control.tasks.go_to_point, "goal_topic") == "/test_bot/goal_pose");
    assert(any_value<std::string>(payload.control.tasks.waypoint, "path_topic") == "/test_bot/waypoint_path");
    assert(payload.ros_binding.logical_name == "test_bot");
    assert(payload.ros_binding.topic_prefix == "/test_bot");
    assert(payload.ros_binding.tf_prefix == "test_bot");
    assert(!payload.global_visualizations.empty());

    const auto& camera_payload = payload.sensors.front().camera_config.value();
    assert(camera_payload.streaming_type == "ros");
    assert(camera_payload.minimap_streaming_type == "ros");
    assert(camera_payload.teleop_streaming_type == "webrtc");
}

static void test_legacy_streaming_fallback() {
    horus::robot::Robot robot("legacy_bot", horus::core::RobotType::WHEELED);
    auto camera = std::make_shared<horus::robot::Camera>(
        "front_camera",
        "legacy_bot/camera_link",
        "/legacy_bot/camera/image_raw/compressed",
        false,
        std::pair<int, int>{640, 480},
        30,
        60.0f,
        "bgr8",
        "webrtc");

    camera->set_minimap_streaming_type("");
    camera->set_teleop_streaming_type("");
    camera->set_startup_mode("");
    robot.add_sensor(camera);

    auto dataviz = robot.create_dataviz();
    horus::bridge::RobotRegistryClient client;
    auto payload = client.build_robot_config_dict(robot, *dataviz);
    const auto& camera_payload = payload.sensors.front().camera_config.value();

    assert(camera_payload.streaming_type == "webrtc");
    assert(camera_payload.minimap_streaming_type == "webrtc");
    assert(camera_payload.teleop_streaming_type == "webrtc");
    assert(camera_payload.startup_mode == "minimap");
}

static void test_camera_topic_profiles() {
    horus::robot::Robot robot("profile_bot", horus::core::RobotType::WHEELED);
    auto camera = std::make_shared<horus::robot::Camera>(
        "front_camera",
        "profile_bot/camera_link",
        "/profile_bot/camera/image_raw");
    camera->set_minimap_topic("/profile_bot/camera/minimap/compressed");
    camera->set_teleop_topic("/profile_bot/camera/teleop/raw");
    camera->set_minimap_image_type("compressed");
    camera->set_teleop_image_type("raw");
    camera->set_minimap_max_fps(15);
    camera->set_teleop_streaming_type("webrtc");
    camera->set_startup_mode("teleop");
    robot.add_sensor(camera);

    auto dataviz = robot.create_dataviz();
    horus::bridge::RobotRegistryClient client;
    auto payload = client.build_robot_config_dict(robot, *dataviz);
    const auto& camera_payload = payload.sensors.front().camera_config.value();
    assert(camera_payload.minimap_topic == "/profile_bot/camera/minimap/compressed");
    assert(camera_payload.teleop_topic == "/profile_bot/camera/teleop/raw");
    assert(camera_payload.minimap_image_type == "compressed");
    assert(camera_payload.teleop_image_type == "raw");
    assert(camera_payload.minimap_max_fps == 15);

    const auto profiles = client.extract_camera_topic_profiles(payload);
    assert(profiles.at("/profile_bot/camera/minimap/compressed").at("source_mode") == "minimap");
    assert(profiles.at("/profile_bot/camera/teleop/raw").at("source_mode") == "teleop");
    assert(profiles.at("/profile_bot/camera/teleop/raw").at("startup") == "teleop");
}

static void test_control_overrides_and_ros_binding() {
    horus::robot::Robot robot("drone_bot", horus::core::RobotType::DRONE);
    robot.configure_ros_binding("flat", "flat", "base_link");
    robot.set_metadata(
        "teleop_config",
        std::map<std::string, std::any>{
            {"enabled", false},
            {"robot_profile", std::string("drone")},
            {"response_mode", std::string("discrete")},
            {"publish_rate_hz", 400.0},
            {"deadman",
             std::map<std::string, std::any>{
                 {"policy", std::string("bad_policy")},
                 {"timeout_ms", 5},
             }},
            {"axes",
             std::map<std::string, std::any>{
                 {"deadzone", 0.9},
                 {"invert_linear_y", true},
             }},
            {"discrete",
             std::map<std::string, std::any>{
                 {"threshold", 0.01},
             }},
        });
    robot.set_metadata(
        "task_config",
        std::map<std::string, std::any>{
            {"go_to_point",
             std::map<std::string, std::any>{
                 {"position_tolerance_m", 20.0},
                 {"min_altitude_m", 5.0},
                 {"max_altitude_m", 3.0},
             }},
            {"waypoint",
             std::map<std::string, std::any>{
                 {"frame_id", std::string("odom")},
                 {"yaw_tolerance_deg", 500.0},
             }},
        });

    auto dataviz = robot.create_dataviz();
    horus::bridge::RobotRegistryClient client;
    auto payload = client.build_robot_config_dict(robot, *dataviz);

    assert(payload.robot_type == "drone");
    assert(payload.ros_binding.topic_mode == "flat");
    assert(payload.ros_binding.tf_mode == "flat");
    assert(payload.control.drive_topic == "/cmd_vel");
    assert(payload.control.teleop.enabled == false);
    assert(payload.control.teleop.robot_profile == "drone");
    assert(payload.control.teleop.response_mode == "discrete");
    assert(std::abs(payload.control.teleop.publish_rate_hz - 120.0) < 1e-6);
    assert(any_value<std::string>(payload.control.teleop.deadman, "policy") == "either_grip_trigger");
    assert(any_value<int>(payload.control.teleop.deadman, "timeout_ms") == 50);
    assert(std::abs(any_value<float>(payload.control.teleop.axes, "deadzone") - 0.5f) < 1e-6f);
    assert(any_value<bool>(payload.control.teleop.axes, "invert_linear_y"));
    assert(std::abs(any_value<float>(payload.control.teleop.discrete, "threshold") - 0.1f) < 1e-6f);
    assert(std::abs(any_value<float>(payload.control.tasks.go_to_point, "position_tolerance_m") - 10.0f) < 1e-6f);
    assert(std::abs(any_value<float>(payload.control.tasks.go_to_point, "max_altitude_m") - 5.1f) < 1e-6f);
    assert(any_value<std::string>(payload.control.tasks.waypoint, "frame_id") == "odom");
    assert(std::abs(any_value<float>(payload.control.tasks.waypoint, "yaw_tolerance_deg") - 180.0f) < 1e-6f);
}

static void test_workspace_scale() {
    horus::robot::Robot robot("scale_bot", horus::core::RobotType::WHEELED);
    robot.configure_workspace_compass(true, 9000, "batch");
    robot.configure_workspace_tutorial("intro");
    robot.configure_local_body_model("ROSBot", true);
    auto dataviz = robot.create_dataviz();
    horus::bridge::RobotRegistryClient client;

    auto with_scale = client.build_robot_config_dict(robot, *dataviz, std::nullopt, 0.25);
    assert(with_scale.workspace_config.has_value());
    assert(with_scale.workspace_config->position_scale.has_value());
    assert(std::abs(*with_scale.workspace_config->position_scale - 0.25f) < 1e-6f);
    assert(any_value<bool>(with_scale.workspace_config->compass, "enabled"));
    assert(any_value<int>(with_scale.workspace_config->compass, "gateway_port") == 9000);
    assert(any_value<std::string>(with_scale.workspace_config->tutorial, "preset_id") == "intro");
    assert(with_scale.robot_model_id == "rosbot");
    assert(with_scale.has_visual_mesh_model == true);

    horus::robot::Robot unscaled("unscaled_bot", horus::core::RobotType::WHEELED);
    auto unscaled_dataviz = unscaled.create_dataviz();
    auto without_scale = client.build_robot_config_dict(unscaled, *unscaled_dataviz, std::nullopt, 0.0);
    assert(!without_scale.workspace_config.has_value());
}

static void test_visualization_payloads() {
    horus::robot::Robot robot("viz_bot", horus::core::RobotType::WHEELED);
    auto dataviz = robot.create_dataviz();
    robot.add_navigation_safety_to_dataviz(*dataviz);
    dataviz->add_3d_map(
        "/viz_bot/map_points",
        "map",
        {
            {"point_size", 0.01},
            {"render_mode", std::string("transparent_hq")},
            {"point_shape", std::string("disc")},
        });
    dataviz->add_3d_mesh(
        "/viz_bot/map_mesh",
        "map",
        {
            {"max_triangles", 750},
            {"source_coordinate_space", std::string("optical")},
        });
    dataviz->add_3d_octomap(
        "/viz_bot/octomap_mesh",
        "map",
        {
            {"native_topic", std::string("/viz_bot/octomap_binary")},
            {"render_mode", std::string("voxel_cubes")},
        });
    dataviz->add_semantic_box(
        "dock",
        "Dock",
        {1.0f, 2.0f, 3.0f},
        {0.4f, 0.5f, 0.6f});

    horus::bridge::RobotRegistryClient client;
    auto payload = client.build_robot_config_dict(robot, *dataviz);

    const auto* velocity = find_viz(payload.visualizations, "velocity_data");
    const auto* trail = find_viz(payload.visualizations, "odometry_trail");
    const auto* collision = find_viz(payload.visualizations, "collision_risk");
    assert(velocity != nullptr);
    assert(trail != nullptr);
    assert(collision != nullptr);
    assert(any_value<std::string>(velocity->velocity, "units") == "m/s");
    assert(any_value<int>(trail->trail, "max_points") == 48);
    assert(std::abs(any_value<float>(collision->collision, "threshold_m") - 0.45f) < 1e-6f);

    const auto* point_cloud = find_viz(payload.global_visualizations, "point_cloud");
    const auto* mesh = find_viz(payload.global_visualizations, "mesh");
    const auto* octomap = find_viz(payload.global_visualizations, "octomap");
    const auto* semantic_box = find_viz(payload.global_visualizations, "semantic_box");
    assert(point_cloud != nullptr);
    assert(mesh != nullptr);
    assert(octomap != nullptr);
    assert(semantic_box != nullptr);
    assert(any_value<std::string>(point_cloud->point_cloud, "render_mode") == "transparent_hq");
    assert(any_value<std::string>(point_cloud->point_cloud, "point_shape") == "circle");
    assert(any_value<int>(mesh->mesh, "max_triangles") == 1000);
    assert(any_value<std::string>(octomap->octomap, "native_topic") == "/viz_bot/octomap_binary");
    assert(any_value<std::string>(semantic_box->semantic_box, "id") == "dock");
}

int main() {
    test_camera_defaults_and_validation();
    test_payload_parity_rules();
    test_legacy_streaming_fallback();
    test_camera_topic_profiles();
    test_control_overrides_and_ros_binding();
    test_workspace_scale();
    test_visualization_payloads();
    std::cout << "cpp_parity_tests passed" << std::endl;
    return 0;
}
