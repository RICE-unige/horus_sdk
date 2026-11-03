#include "horus/bridge/robot_registry.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

using Json = nlohmann::json;
void check(bool value) { if (!value) throw std::runtime_error("Workspace/remote contract mismatch"); }
horus::dataviz::RenderOptions options_from(const Json& object) {
    horus::dataviz::RenderOptions options;
    for (const auto& [key, value] : object.items()) {
        if (value.is_boolean()) options[key] = value.get<bool>();
        else if (value.is_number()) options[key] = value.get<double>();
        else options[key] = value.get<std::string>();
    }
    return options;
}
int main() {
    std::ifstream file(HORUS_REMOTE_RENDER_FIXTURE);
    Json fixture;
    file >> fixture;
    horus::bridge::RobotRegistryClient client;
    auto entity = horus::robot::make_workspace_visualization("map_view");
    check(entity.get_entity_kind() == "workspace_visualization");
    check(entity.create_dataviz()->get_visualizations().empty());
    for (const auto& test : fixture["cases"]) {
        horus::dataviz::DataViz viz("remote");
        viz.add_remote_rendered_map("/remote/map", "world", options_from(test["options"]));
        Json expected = fixture["defaults"];
        expected.update(test["expected"]);
        for (const auto& key : test.value("absent", std::vector<std::string>{})) expected.erase(key);
        for (bool explicit_global : {false, true}) {
            const auto payload = explicit_global
                ? client.build_robot_config_dict(entity, viz, client.build_global_visualizations_payload({viz}))
                : client.build_robot_config_dict(entity, viz);
            const auto wire = Json::parse(horus::bridge::registration_json(payload));
            check(wire["entity_kind"] == "workspace_visualization");
            for (const auto& value : wire["capabilities"]) check(value == false);
            check(wire["robot_manager_config"]["enabled"] == false);
            for (const auto& value : wire["robot_manager_config"]["sections"]) check(value == false);
            check(wire["sensors"].empty() && wire["visualizations"].empty());
            check(wire["control"]["drive_topic"] == "");
            check(wire["control"]["teleop"]["enabled"] == false);
            for (const auto& value : wire["control"]["tasks"]) check(value["enabled"] == false);
            check(wire["global_visualizations"].size() == 1);
            const auto& map = wire["global_visualizations"][0];
            check(map["type"] == "remote_render" && map["frame"] == "world");
            check(map["remote_render"] == expected);
        }
        viz.add_remote_rendered_map("/remote/map", "world");
        check(viz.get_visualizations().size() == 1);
    }
    for (const auto& options : fixture["invalid"]) {
        horus::dataviz::DataViz viz("invalid");
        bool rejected = false;
        try { viz.add_remote_rendered_map("/remote/map", "world", options_from(options)); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected && viz.get_visualizations().empty());
    }
    horus::robot::Robot ordinary("robot", horus::core::RobotType::WHEELED);
    check(ordinary.get_entity_kind() == "robot");
    check(!ordinary.create_dataviz()->get_visualizations().empty());
}
