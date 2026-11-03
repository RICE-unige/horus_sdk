#include "horus/bridge/robot_registry.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <limits>
#include <stdexcept>

void check(bool result) { if (!result) throw std::runtime_error("Costmap payload contract failed"); }
int main() {
    horus::dataviz::DataViz viz("maps");
    viz.add_costmap(" /local_costmap/costmap ", " odom ");
    viz.add_costmap("/global_costmap/costmap", "map");
    horus::bridge::RobotRegistryClient client;
    auto values = client.build_global_visualizations_payload({viz});
    check(values.size() == 2);
    horus::robot::Robot robot("costmap_test", horus::core::RobotType::WHEELED);
    const auto payload = client.build_robot_config_dict(robot, viz, values);
    const auto wire = nlohmann::json::parse(horus::bridge::registration_json(payload));
    std::ifstream fixture(HORUS_COSTMAP_FIXTURE);
    nlohmann::json expected;
    fixture >> expected;
    check(wire["global_visualizations"][0] == expected);
    for (const double alpha : {-1., 1.01, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try { viz.add_costmap("/bad", "map", {.alpha = alpha}); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected);
    }
    check(client.build_global_visualizations_payload({viz}).size() == 2);
    horus::dataviz::DataViz mesh("mesh");
    mesh.add_3d_mesh("/mesh", "map", {{"transport_lane",std::string(" BULK_STRICT ")}});
    check(client.build_global_visualizations_payload({mesh})[0].transport_lane == "bulk_strict");
    const std::map<std::string, std::any> display = {
        {"lod_enabled", true}, {"spacewarp_enabled", true}, {"lod_error_pixels", 1.0}
    };
    horus::dataviz::DataViz display_mesh("display_mesh");
    display_mesh.add_live_mesh("/mesh_lod", "map", {{"display", display}});
    const auto mesh_globals = client.build_global_visualizations_payload({display_mesh});
    const auto mesh_payload = client.build_robot_config_dict(robot, display_mesh, mesh_globals);
    const auto mesh_wire = nlohmann::json::parse(horus::bridge::registration_json(mesh_payload));
    check(mesh_wire["global_visualizations"][0]["mesh"]["display"]["lod_enabled"] == true);
    check(mesh_wire["global_visualizations"][0]["mesh"]["display"]["spacewarp_enabled"] == true);
}
