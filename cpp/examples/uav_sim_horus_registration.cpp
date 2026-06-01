#include "registration_common.hpp"

int main() {
    examples::RegistrationSet set;
    horus::robot::Robot robot("arancino_uav", horus::core::RobotType::DRONE, horus::robot::RobotDimensions{0.46, 0.46, 0.18});
    robot.configure_ros_binding("prefixed", "prefixed", "base_link");
    robot.configure_robot_manager();
    robot.configure_teleop({
        .command_topic = std::string("/uav_sim/teleop/cmd_vel"),
        .robot_profile = std::string("drone"),
    });
    robot.configure_navigation_tasks({
        .goal_topic = std::string("/arancino_uav/goal"),
        .cancel_topic = std::string("/arancino_uav/goal_cancel"),
        .goal_status_topic = std::string("/uav_sim/status"),
        .waypoint_path_topic = std::string("/arancino_uav/waypoint_path"),
        .waypoint_status_topic = std::string("/uav_sim/status"),
        .min_altitude_m = 0.0,
        .max_altitude_m = 10.0,
    });

    auto dataviz = robot.create_dataviz("arancino_uav_viz");
    robot.add_path_planning_to_dataviz(*dataviz, "/uav_sim/path", std::nullopt, std::nullopt);
    dataviz->add_3d_octomap(
        "/uav_sim/octomap_mesh",
        "map",
        {{"native_topic", std::string("/uav_sim/octomap_binary")}, {"native_frame", std::string("map")}});
    set.robots.push_back(std::move(robot));
    set.datavizs.push_back(*dataviz);
    return examples::register_or_fail(set);
}
