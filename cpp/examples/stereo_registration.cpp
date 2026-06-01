#include "registration_common.hpp"

int main() {
    examples::RegistrationSet set;
    for (int index = 1; index <= 4; ++index) {
        const auto name = "stereo_bot_" + std::to_string(index);
        horus::robot::Robot robot(name, horus::core::RobotType::WHEELED, horus::robot::RobotDimensions{0.8, 0.55, 0.45});
        robot.configure_robot_manager();
        robot.configure_teleop({.command_topic = examples::topic(name, "cmd_vel"), .robot_profile = std::string("wheeled")});
        robot.configure_navigation_tasks({
            .goal_topic = examples::topic(name, "goal_pose"),
            .cancel_topic = examples::topic(name, "goal_cancel"),
            .goal_status_topic = examples::topic(name, "goal_status"),
            .waypoint_path_topic = examples::topic(name, "waypoint_path"),
            .waypoint_status_topic = examples::topic(name, "waypoint_status"),
        });

        const auto eye = index == 1 ? std::pair<int, int>{1920, 1080} : std::pair<int, int>{960, 540};
        auto camera = std::make_shared<horus::robot::Camera>(
            "front_stereo_camera",
            name + "/camera_link",
            examples::topic(name, "camera/minimap/image_raw/compressed"),
            true,
            std::pair<int, int>{eye.first * 2, eye.second},
            index == 1 ? 90 : 10,
            60.0f,
            "jpeg",
            "ros",
            "ros",
            "webrtc");
        camera->set_minimap_topic(examples::topic(name, "camera/minimap/image_raw/compressed"));
        camera->set_teleop_topic(examples::topic(name, "camera/teleop/image_raw/compressed"));
        camera->set_minimap_image_type("compressed");
        camera->set_teleop_image_type("compressed");
        camera->set_minimap_max_fps(30);
        camera->set_stereo_layout("side_by_side");
        camera->set_teleop_stereo_layout("side_by_side");
        camera->configure_projected_view({.image_scale = 0.06f, .focal_length_scale = 0.09f});
        robot.add_sensor(camera);

        set.robots.push_back(std::move(robot));
        set.datavizs.push_back(examples::standard_dataviz(set.robots.back()));
    }
    return examples::register_or_fail(set);
}
