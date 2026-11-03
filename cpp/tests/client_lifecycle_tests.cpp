#include "horus/client.hpp"
#include "horus/bridge/robot_registry.hpp"
#include <cassert>
#include <nlohmann/json.hpp>
#include <stdexcept>

int main() {
    horus::Client first("ros2",false), second("ros2",false);
    first.shutdown();
    assert(nlohmann::json::parse(first.registry().status_json())["stopped"]==true);
    assert(nlohmann::json::parse(second.registry().status_json())["stopped"]==false);
    assert(nlohmann::json::parse(horus::bridge::get_robot_registry_client().status_json())["stopped"]==false);
    bool rejected=false;
    try { horus::Client unsupported("unknown",false); unsupported.registry(); }
    catch(const std::invalid_argument&) { rejected=true; }
    assert(rejected);
    first.shutdown();
}
