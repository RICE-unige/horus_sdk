#include "registration_common.hpp"

int main() {
    auto set = examples::standard_fleet(
        {"drone_1", "drone_2", "drone_3"},
        horus::core::RobotType::DRONE,
        {0.46, 0.46, 0.18},
        "drone");
    for (auto& robot : set.robots) {
        robot.configure_navigation_tasks({
            .goal_topic = robot.resolve_topic("goal_pose"),
            .cancel_topic = robot.resolve_topic("goal_cancel"),
            .goal_status_topic = robot.resolve_topic("goal_status"),
            .waypoint_path_topic = robot.resolve_topic("waypoint_path"),
            .waypoint_status_topic = robot.resolve_topic("waypoint_status"),
            .min_altitude_m = 0.0,
            .max_altitude_m = 10.0,
        });
    }
    return examples::register_or_fail(set);
}
