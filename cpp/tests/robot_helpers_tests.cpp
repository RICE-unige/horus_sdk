#include "horus/bridge/robot_registry.hpp"
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <source_location>
#include <stdexcept>

using Json = nlohmann::json;
void check(bool condition, std::source_location where = std::source_location::current()) {
    if (!condition)
        throw std::runtime_error("Robot helpers differ from Python at line " +
                                 std::to_string(where.line()));
}
bool matches(const Json &actual, const Json &expected) {
    if (actual.is_number() && expected.is_number())
        return std::abs(actual.get<double>() - expected.get<double>()) < 1e-6;
    if (actual.is_object() && expected.is_object()) {
        if (actual.size() != expected.size())
            return false;
        for (const auto &[key, value] : expected.items())
            if (!actual.contains(key) || !matches(actual[key], value))
                return false;
        return true;
    }
    return actual == expected;
}
int main() {
    using namespace horus::robot;
    std::ifstream file(HORUS_ROBOT_HELPERS_FIXTURE);
    Json expected;
    file >> expected;
    Robot robot("native_helpers", horus::core::RobotType::DRONE);
    robot.set_metadata("task_config",
                       std::map<std::string, std::any>{
                           {"custom", std::map<std::string, std::any>{{"enabled", true}}}});
    WaypointTaskOptions waypoint;
    waypoint.enabled = false;
    waypoint.path_topic = "/route";
    waypoint.frame_id = "map";
    robot.configure_waypoint_task(waypoint);
    GoToPointTaskOptions goal;
    goal.goal_topic = "/goal";
    goal.frame_id = "world";
    goal.min_altitude_m = 1;
    goal.max_altitude_m = 5;
    goal.allow_unbounded_placement = true;
    robot.configure_go_to_point_task(goal);
    auto capabilities = EntityCapabilities::for_robot();
    capabilities.teleoperable = false;
    capabilities.guidable = true;
    robot.configure_capabilities(capabilities);
    robot.configure_workspace_experiment(true, " experiment.v1 ");
    check(!robot.get_capabilities().teleoperable && robot.get_capabilities().guidable);
    check(!robot.get_field_teammate_config());
    horus::bridge::RobotRegistryClient client;
    auto wire = [&]() {
        return Json::parse(horus::bridge::registration_json(
            client.build_robot_config_dict(robot, *robot.create_dataviz())));
    };
    auto data = wire();
    check(matches(data["control"]["tasks"], expected["tasks"]));
    check(data["workspace_config"] == expected["workspace"]);
    check(data["capabilities"] == expected["capabilities"]);
    robot.configure_go_to_point_task();
    data = wire();
    check(data["control"]["tasks"]["go_to_point"]["allow_unbounded_placement"] == false);
    check(matches(data["control"]["tasks"]["waypoint"], expected["tasks"]["waypoint"]));
    NavigationTaskOptions navigation;
    navigation.allow_unbounded_placement = true;
    robot.configure_navigation_tasks(navigation);
    auto tasks = std::any_cast<std::map<std::string, std::any>>(*robot.get_metadata("task_config"));
    check(tasks.count("custom") == 1);
    check(wire()["control"]["tasks"]["go_to_point"]["allow_unbounded_placement"] == true);
    robot.configure_workspace_experiment(false, " ");
    check(wire()["workspace_config"]["experiment"] ==
          Json({{"enabled", false}, {"contract_version", "experiment.v1"}}));
}
