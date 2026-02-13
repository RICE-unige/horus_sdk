#include "horus/bridge/robot_registry.hpp"
#include "horus/robot/robot.hpp"
#include "horus/robot/sensors.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

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

static void test_workspace_scale() {
    horus::robot::Robot robot("scale_bot", horus::core::RobotType::WHEELED);
    auto dataviz = robot.create_dataviz();
    horus::bridge::RobotRegistryClient client;

    auto with_scale = client.build_robot_config_dict(robot, *dataviz, std::nullopt, 0.25);
    assert(with_scale.workspace_config.has_value());
    assert(std::abs(with_scale.workspace_config->position_scale - 0.25f) < 1e-6f);

    auto without_scale = client.build_robot_config_dict(robot, *dataviz, std::nullopt, 0.0);
    assert(!without_scale.workspace_config.has_value());
}

int main() {
    test_camera_defaults_and_validation();
    test_payload_parity_rules();
    test_legacy_streaming_fallback();
    test_workspace_scale();
    std::cout << "cpp_parity_tests passed" << std::endl;
    return 0;
}
