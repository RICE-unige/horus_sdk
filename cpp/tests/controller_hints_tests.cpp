#include "horus/controller_hints.hpp"
#include "horus/bridge/robot_registry.hpp"
#include <cassert>
#include <fstream>

using namespace horus;
using Json = teaching::Json;
template<class F> void invalid(F f) { bool failed = false; try { f(); } catch (const std::exception&) { failed = true; } assert(failed); }
int main() {
    std::ifstream stream(HORUS_HINT_FIXTURE); Json fixture; stream >> fixture;
    auto hints = ControllerHints::from_dict(fixture.at("controller_hints"));
    assert(hints.to_dict() == fixture["controller_hints"]);
    assert(hints.profiles.size() == 4 && hints.context_topic() == "/hint_demo/controller_hints/context");
    robot::Robot robot("hint_demo", core::RobotType::WHEELED);
    bridge::RobotRegistryClient registry;
    auto wire = [&]() { return Json::parse(bridge::registration_json(registry.build_robot_config_dict(robot, *robot.create_dataviz()))); };
    auto before = wire(); assert(!before.contains("controller_hints"));
    robot.configure_controller_hints(hints);
    auto after = wire(); assert(after.at("controller_hints") == hints.to_dict()); assert(before["control"] == after["control"]);
    robot.clear_controller_hints(); assert(!wire().contains("controller_hints"));
    auto c = Json{{"schema_version", 1}, {"robot_name", "hint_demo"}, {"config_hash", hints.digest()}, {"profile_id", "inspection"},
        {"mode", "teach"}, {"context_id", "00000000-0000-4000-8000-000000000001"}, {"context_sequence", 3}};
    auto context = ControllerHintContext::from_json(c.dump());
    auto update = hints.reply(context, "hint_demo", {{"tool_mode", "Mode: inspect"}}, 1).to_dict();
    assert(update["context_sequence"] == 3 && update["labels"][0]["label"] == "Mode: inspect");
    assert(!update.contains("control") && !update.contains("authority"));
    invalid([&] { hints.reply(context, "other_robot", {}, 1); });
    invalid([&] { hints.reply(context, "hint_demo", {{"unknown", "Do something"}}, 1); });
    invalid([&] { hints.reply(context, "hint_demo", {}, -1); });
    invalid([&] { hints.reply(context, "hint_demo", {{"tool_mode", "Line\nbreak"}}, 1); });
    invalid([&] { auto bad=hints; bad.state_timeout_seconds=0; bad.to_dict(); });
    invalid([&] { auto bad=hints; bad.profiles.push_back(bad.profiles.front()); bad.to_dict(); });
    invalid([&] { auto bad=hints; bad.profiles[0].bindings.push_back(bad.profiles[0].bindings[0]); bad.to_dict(); });
    invalid([&] { auto bad=hints.to_dict(); bad["config_hash"]=std::string(64,'0'); ControllerHints::from_dict(bad); });
    std::ifstream unicode_stream(HORUS_HINT_UNICODE_FIXTURE); Json unicode; unicode_stream >> unicode;
    assert(ControllerHints::from_dict(unicode).to_dict() == unicode);
    for (size_t i=0; i<hint_detail::controls.size(); ++i)
        assert((ControllerHint{static_cast<ControllerControl>(i), "test", "Label"}).to_dict()["control"] == hint_detail::controls[i]);
}
