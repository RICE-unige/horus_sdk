#include "horus/monitoring/dashboard.hpp"
#include <cassert>
#include <sstream>
using namespace horus::monitoring;
using Json = nlohmann::json;
int main() {
    assert(is_bridge_node("/horus_unity_bridge"));
    assert(is_bridge_node("/team/horus_unity_bridge"));
    assert(is_bridge_node("horus_unity_bridge_bulk_7"));
    assert(!is_bridge_node("my_horus_robot"));
    assert(!is_bridge_node("horus_unity_bridge_impostor_suffix!"));
    Json bridge = Json::array({{{"name", "horus_unity_bridge"}, {"namespace", "/"}}});
    Json source = Json::array({{{"name", "camera"}, {"namespace", "/"}}});
    Json empty = Json::array();
    assert(graph_link("backend_sub", empty, bridge, "sdk", "/") == "NO_SOURCE");
    assert(graph_link("backend_sub", source, empty, "sdk", "/") == "NO_BRIDGE");
    assert(graph_link("backend_sub", source, bridge, "sdk", "/") == "OK");
    assert(graph_link("backend_pub", bridge, empty, "sdk", "/") == "NO_CONSUMER");
    assert(graph_link("backend_pub", bridge, source, "sdk", "/") == "OK");
    assert(graph_link("backend_pub", nullptr, source, "sdk", "/") == "UNKNOWN");
    assert(graph_link("sdk_pub", source, bridge, "camera", "/elsewhere") == "NO_SDK");
    assert(graph_link("sdk_pub", source, bridge, "camera", "/") == "OK");
    auto payload = Json::parse(
        R"([{"robot_name":"demo","sensors":[{"type":"camera","topic":"/cam","camera_config":{"minimap_topic":"/mini","teleop_topic":"/teleop","teleop_streaming_type":"webrtc"}}],"control":{"drive_topic":"/drive","teleop":{"raw_input_topic":"/joy"},"tasks":{"go_to_point":{"goal_topic":"/goal"}}},"visualizations":[{"type":"mesh","topic":"/map","mesh":{"use_indexed_binary":true}}]}])");
    auto topics = registration_topics(payload);
    auto row = [&](const char *topic) {
        for (auto &r : topics)
            if (r["topic"] == topic)
                return r;
        return Json();
    };
    assert(row("/mini")["camera_profile"]["source_mode"] == "minimap");
    assert(row("/teleop")["camera_profile"]["teleop"] == "webrtc");
    assert(row("/joy")["role"] == "backend_pub");
    assert(row("/goal")["role"] == "backend_pub");
    assert(row("/map/indexed_request")["role"] == "backend_pub");
    Json state = {{"app_connected", false},
                  {"heartbeat_seen", true},
                  {"registration_state", "Accepted"},
                  {"topics", topics}};
    for (unsigned width : {48u, 80u, 100u, 180u}) {
        const auto output = render_dashboard(state, width, 24);
        assert(output.find("DISCONNECTED") != std::string::npos);
        assert(output.find("Accepted") != std::string::npos);
        assert(output.find("\033") == std::string::npos);
        std::istringstream lines(output);
        for (std::string line; std::getline(lines, line);)
            assert(line.size() == width);
    }
    assert(render_dashboard(state, 100, 12, 0) != render_dashboard(state, 100, 12, 1));
    state["error"] = "bad\033[2Jmessage\nnew";
    assert(render_dashboard(state).find('\033') == std::string::npos);
}
