#include "registration_common.hpp"

int main() {
    constexpr const char* robot_name = "unitree_go1";
    examples::RegistrationSet set;
    horus::robot::Robot robot(robot_name, horus::core::RobotType::LEGGED, horus::robot::RobotDimensions{0.54, 0.30, 0.18});
    robot.configure_ros_binding("prefixed", "prefixed", "trunk");
    robot.configure_robot_description({
        .urdf_path = "/home/omotoye/Unitree_ros2_to_real/ros2_ws/src/go1_description/urdf/go1.urdf",
        .base_frame = "trunk",
        .chunk_size_bytes = 64000,
        .visual_mesh_triangle_budget = 220000,
        .body_mesh_mode = "runtime_high_mesh",
    });
    robot.configure_robot_manager();
    robot.configure_teleop({
        .command_topic = examples::topic(robot_name, "cmd_vel"),
        .robot_profile = std::string("legged"),
        .linear_xy_max_mps = 0.25,
        .linear_z_max_mps = 0.0,
        .angular_z_max_rps = 0.9,
        .invert_angular_z = true,
    });

    auto camera = std::make_shared<horus::robot::Camera>(
        "front_camera",
        "unitree_go1/camera_optical_left_face",
        "/unitree_go1/front_camera/left/color/image_rect/compressed",
        false,
        std::pair<int, int>{928, 800},
        30,
        116.0f,
        "jpeg");
    camera->set_minimap_image_type("compressed");
    camera->set_teleop_image_type("compressed");
    camera->configure_projected_view({.image_scale = 0.0875f, .focal_length_scale = 0.0875f, .show_frustum = true});
    camera->configure_immersive_view({.ros_flip_x = false, .ros_flip_y = true});
    camera->configure_minimap_view({.size = 1.0f});
    robot.add_sensor(camera);
    robot.add_sensor(std::make_shared<horus::robot::LaserScan>(
        "front_laser_scan",
        "unitree_go1/laser",
        "/unitree_go1/scan",
        -2.3561945f,
        2.3561945f,
        0.004363323f,
        0.023f,
        60.0f,
        0.01f,
        "#6ED7FF",
        0.04f));

    auto dataviz = robot.create_dataviz();
    dataviz->add_occupancy_grid("/map", "map");
    dataviz->add_robot_velocity_data(robot_name, "/unitree_go1/odom", "map");
    dataviz->add_robot_odometry_trail(robot_name, "/unitree_go1/odom", "map");
    dataviz->add_robot_collision_risk(robot_name, "/unitree_go1/collision_risk", "unitree_go1/trunk");
    set.robots.push_back(std::move(robot));
    set.datavizs.push_back(*dataviz);
    return examples::register_or_fail(set);
}
