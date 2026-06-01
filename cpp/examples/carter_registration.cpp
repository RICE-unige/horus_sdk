#include "registration_common.hpp"

int main() {
    examples::RegistrationSet set;
    for (const std::string name : {"carter1", "carter2", "carter3"}) {
        auto robot = examples::standard_robot(name, horus::core::RobotType::WHEELED, {0.82, 0.56, 0.60}, "wheeled");
        robot.configure_local_body_model("nova_carter");
        robot.remove_sensor("front_camera");
        auto camera = examples::standard_camera(name);
        camera->set_resolution({1280, 720});
        camera->set_fps(20);
        camera->set_teleop_streaming_type("webrtc");
        camera->configure_webrtc_transport({.bitrate_kbps = 2000, .framerate = 20});
        robot.add_sensor(camera);
        robot.add_sensor(std::make_shared<horus::robot::LaserScan>(
            "front_2d_lidar",
            name + "/front_2d_lidar",
            examples::topic(name, "front_2d_lidar/scan"),
            -3.14159f,
            3.14159f,
            0.005f,
            0.1f,
            30.0f,
            0.01f,
            "#6ED7FF",
            0.025f));
        set.robots.push_back(std::move(robot));
        auto dataviz = set.robots.back().create_dataviz();
        dataviz->add_robot_global_path(name, "/rviz/" + name + "/plan", "global_odom");
        dataviz->add_robot_local_path(name, "/rviz/" + name + "/local_plan", "global_odom");
        dataviz->add_robot_velocity_data(name, examples::topic(name, "chassis/odom"), "global_odom");
        dataviz->add_robot_odometry_trail(name, examples::topic(name, "chassis/odom"), "global_odom");
        set.datavizs.push_back(*dataviz);
    }
    set.datavizs.front().add_occupancy_grid("/shared_map", "map");
    return examples::register_or_fail(set);
}
