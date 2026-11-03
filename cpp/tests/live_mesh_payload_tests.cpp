#include "horus/bridge/robot_registry.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

void check(bool value) { if (!value) throw std::runtime_error("Live mesh payload contract failed"); }

int main() {
    horus::bridge::RobotRegistryClient client;
    horus::robot::Robot robot("mesh_test", horus::core::RobotType::WHEELED);
    std::ifstream input(HORUS_LIVE_MESH_FIXTURE);
    nlohmann::json fixtures;
    input >> fixtures;
    for (const auto& test : fixtures) {
        std::cout << test["name"] << '\n';
        horus::dataviz::RenderOptions options;
        for (const auto& [key, value] : test["options"].items()) {
            if (value.is_boolean()) options[key] = value.get<bool>();
            else if (value.is_number_integer()) options[key] = value.get<int>();
            else if (value.is_number()) options[key] = value.get<double>();
            else options[key] = value.get<std::string>();
        }
        horus::dataviz::DataViz viz("mesh");
        if (test["helper"] == "live") viz.add_live_mesh("/horus/maps/live", "map", options);
        else if (test["helper"] == "nvblox") viz.add_nvblox_mesh("/horus/maps/live", "map", options);
        else viz.add_3d_mesh("/horus/maps/live", "map", options);
        // Both global entry points must use the same contract.
        for (bool explicit_global : {false, true}) {
            auto payload = explicit_global
                ? client.build_robot_config_dict(robot, viz, client.build_global_visualizations_payload({viz}))
                : client.build_robot_config_dict(robot, viz);
            const auto wire = nlohmann::json::parse(horus::bridge::registration_json(payload));
            check(wire["global_visualizations"].size() == 1);
            check(wire["global_visualizations"][0] == test["expected"]);
        }
    }
    horus::dataviz::DataViz extreme("extreme");
    extreme.add_live_mesh("/mesh", "map", {
        {"max_chunk_renderers", std::numeric_limits<double>::infinity()},
        {"max_chunk_uploads_per_frame", 1e40},
        {"max_triangle_uploads_per_frame", std::string("4567.8")},
        {"max_triangles", std::numeric_limits<unsigned long long>::max()}
    });
    const auto values = client.build_global_visualizations_payload({extreme});
    check(std::any_cast<int>(values[0].mesh.at("max_chunk_renderers")) == 1024);
    check(std::any_cast<int>(values[0].mesh.at("max_chunk_uploads_per_frame")) == 32);
    check(std::any_cast<int>(values[0].mesh.at("max_triangle_uploads_per_frame")) == 4567);
    check(std::any_cast<int>(values[0].mesh.at("max_triangles")) == std::numeric_limits<int>::max());
    const nlohmann::json adaptive_policy = {
        {"enabled", true}, {"initial_triangle_budget", 500000},
        {"regions", nlohmann::json::array({{{"id", "robot"}, {"frame", "rover/base_link"},
            {"shape", "sphere"}, {"size", {2, 2, 2}}, {"priority", 2}}})}
    };
    horus::dataviz::DataViz adaptive("adaptive");
    adaptive.add_live_mesh("/map_adaptive", "map", {{"adaptive", adaptive_policy}});
    const auto adaptive_wire = nlohmann::json::parse(horus::bridge::registration_json(
        client.build_robot_config_dict(robot, adaptive)));
    check(adaptive_wire["global_visualizations"][0]["mesh"]["adaptive"] == adaptive_policy);
    std::ifstream map_input(HORUS_MAP_RENDER_FIXTURE);
    nlohmann::json map_cases;
    map_input >> map_cases;
    for (const auto& test : map_cases) {
        horus::dataviz::RenderOptions options;
        for (const auto& [key, value] : test["options"].items()) {
            if (value.is_boolean()) options[key] = value.get<bool>();
            else if (value.is_number()) options[key] = value.get<double>();
            else options[key] = value.get<std::string>();
        }
        horus::dataviz::DataViz viz("maps");
        if (test["type"] == "point_cloud") viz.add_3d_map("/map", "odom", options);
        else if (test["type"] == "gaussian_splat") viz.add_gaussian_splat_map("/map", "odom", options);
        else viz.add_3d_octomap("/map", "odom", options);
        for (bool explicit_global : {false, true}) {
            const auto payload = explicit_global
                ? client.build_robot_config_dict(robot, viz, client.build_global_visualizations_payload({viz}))
                : client.build_robot_config_dict(robot, viz);
            const auto wire = nlohmann::json::parse(horus::bridge::registration_json(payload));
            const auto& fields = wire["global_visualizations"][0][test["type"].get<std::string>()];
            for (const auto& [key, expected] : test["fields"].items()) check(fields.at(key) == expected);
            for (const auto& key : test.value("absent", std::vector<std::string>{})) check(!fields.contains(key));
        }
    }
}
