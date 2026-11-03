#include "horus/bridge/robot_registry.hpp"
#include "horus/robot/robot.hpp"
#include "horus/teaching/cartesian.hpp"
#include "horus/teaching/session.hpp"
#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <thread>

using namespace horus::teaching;
namespace fs = std::filesystem;
struct FixtureAdapter : TeachAdapter {
  Json last_sample;
  int anchors = 0, commands = 0, holds = 0;
  bool confirm = true;
  bool supports_pause = false, pause_confirm = true, healthy = true;
  int pauses = 0;
  AdapterCapabilities capabilities() const override {
    AdapterCapabilities c;
    c.adapter_id = "fixture";
    c.qualification = "simulation";
    c.joint_names = {"joint"};
    c.command_frame = "base";
    c.calibration_id = "v1";
    c.controller_watchdog = "local";
    c.authority_contract = "lease";
    c.supports_tracking_pause = supports_pause;
    return c;
  }
  std::pair<bool, std::string> ready() override { return {healthy, healthy ? "" : "worker fault"}; }
  void anchor(const Json &s) override {
    assert(s.at("authority").at("owner") == "operator");
    ++anchors;
  }
  Json apply(const Json &sample) override {
    last_sample = sample;
    ++commands;
    return {{"boundary", "accepted"}};
  }
  bool hold(const Json &s) override {
    assert(s.at("owner") == "operator");
    ++holds;
    return confirm;
  }
  bool pause(const Json &s) override {
    assert(s.at("owner") == "operator");
    ++pauses;
    return pause_confirm;
  }
};
Json request(TeachSession &s, const TeachConfig &c, const std::string &action,
             Json extra = Json::object()) {
  auto state = s.state();
  Json r = {{"request_id", new_uuid()},
            {"config_hash", c.digest()},
            {"action", action},
            {"owner", "operator"},
            {"session_id", state.at("session_id")},
            {"alignment_id", state.at("alignment_id")},
            {"expected_revision", state.at("revision")}};
  r.update(extra);
  return s.request(r);
}
void align(TeachSession &s, const TeachConfig &c) {
  assert(request(s, c, "align", {{"alignment", {
      {"schema_version", 1}, {"alignment_id", new_uuid()}, {"tracking_origin_id", new_uuid()},
      {"frame_id", "quest_tracking_flu"}, {"child_frame_id", "teach_calibrated_floor_flu"},
      {"position", {0, 0, 0}}, {"rotation", {0, 0, 0, 1}}}}})["accepted"] == true);
}
void tracking_input(Json &sample, const Json &state) {
  sample["frame_id"] = "quest_tracking_flu";
  sample["alignment_id"] = state.at("alignment_id");
  sample["tracking_origin_id"] = state.at("tracking_origin_id");
}
void wait_capture(TeachSession &s, const std::string &expected) {
  for (int i = 0; i < 500; ++i) {
    s.tick();
    auto state = s.state();
    if (state["capture"] == expected)
      return;
    assert(state["error"] == "");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  assert(false && "capture timed out");
}
int main() {
  TeachStream depth{"depth", "/robot/front/points", "sensor_msgs/msg/PointCloud2"};
  depth.required = false;
  auto spatial = TeachConfig("/tmp/episodes").with_stream(depth).with_spatial_view(TeachPointCloud{"depth"});
  assert(spatial.digest() == "25c4fbfbee7b73fd598a923b7ac5e46b20527f3fa67fc9e8d85ec55c64627e91");
  assert(spatial.registration("robot")["spatial_views"][0]["topic"] == depth.topic);
  bool rejected_duplicate_view = false;
  try { spatial.with_spatial_view(TeachPointCloud{"depth"}); }
  catch (const std::exception &) { rejected_duplicate_view = true; }
  assert(rejected_duplicate_view);
  TeachCamera portal_camera;
  portal_camera.sensor_id = "front";
  portal_camera.portal = TeachPortal{};
  auto portal_config = TeachConfig("/data/teach").with_camera(portal_camera);
  assert(portal_config.digest() == "ab34bb8486283d80f600f8f77e8dd22df07a3e787b6f2aac52965014f7175c5f");
  assert(portal_config.registration("robot")["cameras"][0]["portal"]["mode"] == "stereo");
  portal_camera.portal->crop = 0.2;
  bool rejected_portal = false;
  try { TeachConfig("/data/teach").with_camera(portal_camera); }
  catch (const std::exception &) { rejected_portal = true; }
  assert(rejected_portal);
  TeachConfig continuous_contract("/data/teach", {{"input_schema_version", 2}, {"engagement_mode", "continuous"}});
  assert(continuous_contract.digest() == "b4a500fb99e6ba3460b1212b74b2bb990be0e4371a3328f88bcf1b6d056690fe");
  assert(continuous_contract.with_options({{"provenance", "fixture"}, {"tracking_recovery", "auto_resume"}}).digest() ==
         "4073155edb3d2526d447ef4c370f81606329f467b8aa2171989db15bed9a6127");
  TeachConfig literal("/tmp/horus-teach-literal");
  assert(!literal.digest().empty());
  std::ifstream file(HORUS_TEACH_FIXTURE);
  auto fixture = Json::parse(file);
  for (auto &item : fixture.at("cases")) {
    TeachConfig c(item.at("input"));
    assert(c.to_dict() == item.at("normalized"));
    assert(c.digest() == item.at("digest").get<std::string>());
    assert(c.registration("teach_robot") == item.at("registration"));
  }
  auto root = fs::temp_directory_path() / ("horus-native-teach-" + new_uuid());
  TeachConfig observe(root.string(), {{"minimum_free_bytes", 1024}});
  for (auto bad : {Json{{"publish_rate_hz", 0}},
                   Json{{"streams",
                         {{{"stream_id", "bad"},
                           {"topic", "relative"},
                           {"message_type", "std_msgs/msg/String"}}}}},
                   Json{{"unknown", 1}}}) {
    auto value = observe.to_dict();
    value.update(bad);
    bool rejected = false;
    try {
      TeachConfig invalid(value);
    } catch (const std::exception &) {
      rejected = true;
    }
    assert(rejected);
  }
  horus::robot::Robot humanoid("teach_robot", horus::core::RobotType::HUMANOID);
  humanoid.configure_teaching(observe);
  horus::bridge::RobotRegistryClient registry;
  auto payload = registry.build_robot_config_dict(
      humanoid, horus::dataviz::DataViz("teach_robot"));
  assert(Json::parse(horus::bridge::registration_json(payload))["teaching"] ==
         observe.registration("teach_robot"));
  bool wrong_type = false;
  try {
    horus::robot::Robot rover("rover", horus::core::RobotType::WHEELED);
    rover.configure_teaching(observe);
  } catch (const std::exception &) {
    wrong_type = true;
  }
  assert(wrong_type);
  double now = 10;
  auto adapter = std::make_shared<FixtureAdapter>();
  TeachConfig motion(root.string(), {{"adapter_id", "fixture"},
                                     {"provenance", "fixture"},
                                     {"workflow", "practice"},
                                     {"joint_names", {"joint"}},
                                     {"joint_groups", {{"arm", {"joint"}}}},
                                     {"command_frame", "base"},
                                     {"calibration_id", "v1"},
                                     {"minimum_free_bytes", 1024}});
  TeachSession s(motion, "teach_robot", adapter, [&] { return now; });
  assert(request(s, motion, "open")["accepted"] == true);
  assert(request(s, motion, "engage",
                 {{"authority_generation", "g1"}})["accepted"] == false);
  s.update_lease("operator", "g1", 3);
  assert(request(s, motion, "engage", {{"authority_generation", "g1"}})["accepted"] == false);
  align(s, motion);
  auto state = request(s, motion, "engage", {{"authority_generation", "g1"}});
  assert(state["control"] == "armed");
  Json pose = {{"position", {0, 0, 0}}, {"rotation", {0, 0, 0, 1}}};
  Json sample = {{"session_id", state["session_id"]},
                 {"input_epoch", state["input_epoch"]},
                 {"challenge", state["challenge"]},
                 {"owner", "operator"},
                 {"sequence", 0},
                 {"tracking_valid", true},
                 {"clutch", true},
                 {"ui_focus", false},
                 {"left", pose},
                 {"right", pose}};
  tracking_input(sample, state);
  assert(s.input(sample));
  s.tick();
  assert(adapter->anchors == 1 && adapter->commands == 0);
  assert(!s.input(sample));
  sample["sequence"] = 1;
  assert(s.input(sample));
  s.tick();
  assert(adapter->commands == 1);
  now += 0.3;
  s.tick();
  assert(s.state()["control"] == "holding");
  assert(adapter->holds == 1);
  state = request(s, motion, "engage", {{"authority_generation", "g1"}});
  assert(state["accepted"] == true);
  adapter->confirm = false;
  s.update_lease("other", "g2", 3);
  assert(s.state()["control"] == "hold_requested");
  adapter->confirm = true;
  now += 0.2;
  s.tick();
  assert(s.state()["control"] == "holding");
  s.close();

  {
    auto mapped_adapter = std::make_shared<FixtureAdapter>();
    TeachSession mapped(motion, "teach_robot", mapped_adapter, [&] { return now; });
    assert(request(mapped, motion, "open")["accepted"] == true);
    mapped.update_lease("operator", "generation", 3);
    Json alignment = {{"schema_version", 1}, {"alignment_id", new_uuid()}, {"tracking_origin_id", new_uuid()},
        {"frame_id", "quest_tracking_flu"}, {"child_frame_id", "teach_calibrated_floor_flu"},
        {"position", {2, 3, 0}}, {"rotation", {0, 0, std::sqrt(.5), std::sqrt(.5)}}};
    assert(request(mapped, motion, "align", {{"alignment", alignment}})["accepted"] == true);
    for (const auto &bad : {Json{{"position", {101, 0, 0}}}, Json{{"rotation", {0, 0, 0, 2}}},
                           Json{{"rotation", {.70710678, 0, 0, .70710678}}}, Json{{"schema_version", true}},
                           Json{{"frame_id", "map"}}, Json{{"alignment_id", "bad"}}}) {
      auto malformed = alignment;
      malformed.update(bad);
      assert(request(mapped, motion, "align", {{"alignment", malformed}})["accepted"] == false);
      assert(mapped.state()["alignment_id"] == alignment["alignment_id"]);
    }
    auto armed = request(mapped, motion, "engage", {{"authority_generation", "generation"}});
    assert(armed["accepted"] == true);
    assert(request(mapped, motion, "align", {{"alignment", alignment}})["accepted"] == false);
    auto raw = sample;
    raw.update({{"session_id", armed["session_id"]}, {"input_epoch", armed["input_epoch"]},
                {"challenge", armed["challenge"]}, {"sequence", 0},
                {"left", {{"position", {2, 4, 1}}, {"rotation", alignment["rotation"]}}}});
    tracking_input(raw, armed);
    auto original = raw;
    assert(mapped.input(raw)); mapped.tick();
    raw["sequence"] = 1;
    assert(mapped.input(raw)); mapped.tick();
    const auto &command = mapped_adapter->last_sample;
    assert(command["frame_id"] == "teach_calibrated_floor_flu");
    assert(command["source_frame_id"] == "quest_tracking_flu");
    assert(std::abs(command["left"]["position"][0].get<double>() - 1) < 1e-8);
    assert(std::abs(command["left"]["position"][1].get<double>()) < 1e-8);
    assert(std::abs(command["left"]["position"][2].get<double>() - 1) < 1e-8);
    assert(std::abs(command["left"]["rotation"][2].get<double>()) < 1e-8);
    assert(raw["left"] == original["left"]);
    raw["sequence"] = 2;
    raw["tracking_origin_id"] = new_uuid();
    assert(!mapped.input(raw)); mapped.tick();
    assert(mapped.state()["alignment_id"] == "");
    assert(mapped_adapter->holds == 1 && mapped_adapter->commands == 1);
    assert(request(mapped, motion, "engage", {{"authority_generation", "generation"}})["accepted"] == false);
    mapped.close();
  }

  auto v2 = motion.with_options(
      {{"input_schema_version", 2},
       {"required_input_features", fixture["input_v2"]["features"]},
       {"clutch_binding", "both_primary"},
       {"grasp_bindings",
        {{"left_grasp", "left_grip"},
         {"right_grasp", "right_grip"},
         {"left_pinch", "left_trigger"},
         {"right_pinch", "right_trigger"}}}});
  auto negotiate = Json{{"input_schema_version", 2},
                        {"input_features", fixture["input_v2"]["features"]}};
  auto continuous = v2.with_options({{"engagement_mode", "continuous"}});
  auto recovery = continuous.with_options({{"tracking_recovery", "auto_resume"}});
  assert(recovery.registration("teach_robot")["tracking_recovery"] == "auto_resume");
  bool real_recovery_rejected = false;
  try { recovery.with_options({{"provenance", "real"}}); }
  catch (const std::exception &) { real_recovery_rejected = true; }
  assert(real_recovery_rejected);
  for (auto fault : {Json{{"left_tracking_valid", false}, {"left", nullptr}}, Json{{"ui_focus", true}},
                     Json{{"left_tracking_valid", false}, {"left", Json::object()}},
                     Json{{"left_tracking_valid", false}, {"left", {{"position", nullptr}, {"rotation", nullptr}}}},
                     Json{{"head_tracking_valid", false}, {"head", {{"position", Json::array()}, {"rotation", Json::array()}}}},
                     Json{{"tracking_valid", false}}, Json{{"head_tracking_valid", false}},
                     Json{{"left_buttons_valid", false}}, Json{{"challenge", "old"}}}) {
    auto driver = std::make_shared<FixtureAdapter>();
    TeachSession session(recovery, "teach_robot", driver, [&] { return now; });
    auto fields = negotiate;
    fields.update({{"engagement_mode", "continuous"}, {"tracking_recovery", "auto_resume"}});
    assert(!request(session, recovery, "open", negotiate)["accepted"].get<bool>());
    assert(request(session, recovery, "open", fields)["accepted"] == true);
    session.update_lease("operator", "g1", 3);
    assert(!request(session, recovery, "engage", {{"authority_generation", "g1"}})["accepted"].get<bool>());
    driver->supports_pause = true;
    align(session, recovery);
    auto armed = request(session, recovery, "engage", {{"authority_generation", "g1"}});
    assert(armed["accepted"] == true);
    auto value = fixture["input_v2"]["sample"];
    tracking_input(value, armed);
    value.update({{"owner", "operator"}, {"session_id", armed["session_id"]},
                  {"input_epoch", armed["input_epoch"]}, {"engagement_mode", "continuous"},
                  {"tracking_recovery", "auto_resume"}, {"left_primary", false}, {"right_primary", false},
                  {"sequence", 1}, {"challenge", armed["challenge"]}});
    assert(session.input(value)); session.tick();
    value["sequence"] = 2;
    auto lost = value; lost.update(fault);
    assert(!session.input(lost));
    assert(session.state()["control"] == "paused");
    assert(session.state()["input_epoch"] == armed["input_epoch"]);
    assert(driver->pauses == 1 && driver->holds == 0);
    value["sequence"] = 3;
    assert(!session.input(value)); // Old challenge is invalidated at pause confirmation.
    value["challenge"] = session.state()["challenge"];
    assert(!session.input(value)); // Same rejected sequence is not replayable.
    value["sequence"] = 4;
    assert(session.input(value)); session.tick();
    assert(driver->anchors == 1 && driver->commands == 1);
    driver->pause_confirm = false;
    now += 1;
    session.tick();
    assert(session.state()["control"] == "pause_requested");
    value["sequence"] = 5; value["challenge"] = session.state()["challenge"];
    assert(!session.input(value));
    driver->pause_confirm = true; now += .2; session.tick();
    assert(session.state()["control"] == "paused");
    assert(driver->commands == 1);
    value["sequence"] = 6; value["challenge"] = session.state()["challenge"];
    assert(session.input(value)); session.tick();
    assert(driver->anchors == 1 && driver->commands == 2);
    value["sequence"] = 7; value["right_secondary"] = true; value["challenge"] = "expired";
    assert(!session.input(value));
    assert(session.state()["control"] == "holding");
    assert(session.state()["input_epoch"] == "");
    value["sequence"] = 8; value["right_secondary"] = false; value["challenge"] = session.state()["challenge"];
    assert(!session.input(value));
  }
  assert(continuous.registration("teach_robot")["engage_countdown_seconds"] == 3.0);
  {
    auto driver = std::make_shared<FixtureAdapter>();
    TeachSession session(continuous, "teach_robot", driver, [&] { return now; });
    auto fields = negotiate;
    fields["engagement_mode"] = "continuous";
    assert(request(session, continuous, "open", fields)["accepted"] == true);
    align(session, continuous);
    session.update_lease("operator", "g1", 3);
    auto armed = request(session, continuous, "engage", {{"authority_generation", "g1"}});
    assert(armed["accepted"] == true);
    auto value = fixture["input_v2"]["sample"];
    tracking_input(value, armed);
    value.update({{"owner", "operator"}, {"session_id", armed["session_id"]},
                  {"input_epoch", armed["input_epoch"]}, {"engagement_mode", "continuous"},
                  {"left_primary", false}, {"right_primary", false}});
    for (int sequence = 1; sequence <= 1800; ++sequence) {
      now += 1.0 / 30;
      session.update_lease("operator", "g1", 3);
      value["sequence"] = sequence;
      value["challenge"] = session.state()["challenge"];
      assert(session.input(value));
      session.tick();
      assert(session.state()["control"] == "engaged");
    }
    now += 1;
    session.tick();
    assert(session.state()["last_hold_reason"] == "input watchdog expired");
    const auto stopped = session.state()["last_hold_input"];
    assert(stopped["received_sequence"] == 1800);
    assert(stopped["accepted_sequence"] == 1800);
    assert(stopped["accepted_age_ms"].get<double>() >= 999.9);
    assert(stopped["callback_lock_wait_ms"].get<double>() >= 0);
    assert(request(session, continuous, "heartbeat")["last_hold_reason"] == "input watchdog expired");
    value["sequence"] = 1801;
    assert(!session.input(value));
    assert(session.state()["input_health"]["received_sequence"] == 1801);
    assert(session.state()["input_health"]["accepted_sequence"] == 1800);
    assert(session.state()["last_hold_input"] == stopped);
    session.close();
  }
  for (auto fault : {Json{{"left_primary", true}, {"right_primary", true}},
                     Json{{"right_secondary", true}}, Json{{"left_buttons_valid", false}},
                     Json{{"tracking_valid", false}}, Json{{"ui_focus", true}},
                     Json{{"clutch", false}}, Json{{"engagement_mode", "hold_to_run"}}}) {
    auto driver = std::make_shared<FixtureAdapter>();
    TeachSession session(continuous, "teach_robot", driver, [&] { return now; });
    assert(request(session, continuous, "open", negotiate)["accepted"] == false);
    auto fields = negotiate;
    fields["engagement_mode"] = "continuous";
    assert(request(session, continuous, "open", fields)["accepted"] == true);
    align(session, continuous);
    session.update_lease("operator", "g1", 3);
    auto armed = request(session, continuous, "engage", {{"authority_generation", "g1"}});
    assert(armed["accepted"] == true && armed["engagement_mode"] == "continuous");
    auto value = fixture["input_v2"]["sample"];
    tracking_input(value, armed);
    value.update({{"owner", "operator"}, {"session_id", armed["session_id"]},
                  {"input_epoch", armed["input_epoch"]}, {"challenge", armed["challenge"]},
                  {"sequence", 1}, {"engagement_mode", "continuous"},
                  {"left_primary", false}, {"right_primary", false}});
    assert(session.input(value));
    session.tick();
    assert(driver->anchors == 1 && driver->commands == 0);
    value["sequence"] = 2;
    assert(session.input(value));
    session.tick();
    assert(driver->commands == 1 && driver->last_sample["left_primary"] == false);
    value["sequence"] = 3;
    auto stopped = value;
    stopped.update(fault);
    assert(!session.input(stopped));
    session.tick();
    assert(session.state()["control"] == "holding" && driver->commands == 1);
    assert(!session.state()["last_hold_reason"].get<std::string>().empty());
    assert(!session.input(value));
    session.close();
  }
  for (auto &fault : fixture["input_v2"]["invalid"]) {
    auto driver = std::make_shared<FixtureAdapter>();
    TeachSession session(v2, "teach_robot", driver, [&] { return now; });
    assert(request(session, v2, "open")["accepted"] == false);
    assert(session.state()["owner"] == "");
    assert(request(session, v2, "open",
                   {{"input_schema_version", 2},
                    {"input_features", Json::array()}})["accepted"] == false);
    assert(request(session, v2, "open", negotiate)["accepted"] == true);
    align(session, v2);
    session.update_lease("operator", "g1", 3);
    auto armed =
        request(session, v2, "engage", {{"authority_generation", "g1"}});
    assert(armed["accepted"] == true);
    auto value = fixture["input_v2"]["sample"];
    tracking_input(value, armed);
    value.update({{"owner", "operator"},
                  {"session_id", armed["session_id"]},
                  {"input_epoch", armed["input_epoch"]},
                  {"challenge", armed["challenge"]},
                  {"sequence", 1}});
    assert(session.input(value));
    session.tick();
    assert(driver->anchors == 1 && driver->commands == 0);
    value["sequence"] = 2;
    assert(session.input(value));
    session.tick();
    for (auto it = value.begin(); it != value.end(); ++it)
      if (it.key() != "frame_id") assert(driver->last_sample[it.key()] == it.value());
    assert(driver->last_sample["frame_id"] == "teach_calibrated_floor_flu");
    assert(driver->last_sample["tools"]["left_grasp"] == 0.6);
    value["sequence"] = 3;
    value.update(fault.value("patch", Json::object()));
    if (fault.contains("remove"))
      value.erase(fault["remove"].get<std::string>());
    assert(!session.input(value));
    session.tick();
    assert(session.state()["control"] == "holding" && driver->commands == 1);
    assert(request(session, v2, "close")["accepted"] == true);
    assert(session.state()["input_features"].empty());
  }
  {
    TeachSession session(v2, "teach_robot", adapter, [&] { return now; });
    auto duplicate = negotiate;
    duplicate["input_features"].push_back("head_pose");
    assert(request(session, v2, "open", duplicate)["accepted"] == false);
    assert(request(session, v2, "open", negotiate)["accepted"] == true);
    auto reordered = negotiate;
    std::reverse(reordered["input_features"].begin(),
                 reordered["input_features"].end());
    assert(request(session, v2, "open", reordered)["accepted"] == true);
    now += 6;
    session.tick();
    assert(session.state()["input_features"].empty() &&
           session.state()["owner"] == "");
    session.close();
  }

  auto cap = adapter->capabilities();
  cap.action_representation = "cartesian_pose_absolute";
  cap.command_boundary = "accepted";
  CartesianLimits limits{{-1, -1, -1}, {1, 1, 1}, 1, 1};
  limits.floor_to_command_rotation = {0, 0, std::sqrt(0.5), std::sqrt(0.5)};
  int published = 0, held = 0;
  bool feedback_ready = true;
  double feedback_time = now;
  CartesianAdapter cartesian(
      cap, limits,
      [&] {
        return Json{{"ready", feedback_ready},
                    {"received_monotonic", feedback_time},
                    {"poses", {{"left", pose}}}};
      },
      [&](const Json &command) {
        ++published;
        assert(command["authority"]["owner"] == "operator");
        return Json{{"ok", true}, {"boundary", "accepted"}};
      },
      [&](const Json &) {
        ++held;
        return true;
      },
      {"left"}, Json::object(), [&] { return now; });
  Json hand = {{"left", pose},
               {"sequence", 0},
               {"authority",
                {{"robot_name", "teach_robot"},
                 {"owner", "operator"},
                 {"session_id", "s1"},
                 {"authority_generation", "g1"}}}};
  cartesian.anchor(hand);
  assert(published == 0);
  hand["left"]["position"][0] = 0.05;
  now += 0.1;
  auto target = cartesian.apply(hand)["targets"]["left"]["position"];
  assert(std::abs(target[0].get<double>()) < 1e-12 &&
         std::abs(target[1].get<double>() - 0.05) < 1e-12);
  auto rejects = [&](const Json &input) {
    bool failed = false;
    try {
      cartesian.apply(input);
    } catch (const std::exception &) {
      failed = true;
    }
    assert(failed);
  };
  hand["left"]["position"][0] = 0.9;
  now += 0.01;
  rejects(hand);
  assert(published == 1);
  hand["authority"]["owner"] = "other";
  rejects(hand);
  assert(published == 1);
  now += 0.3;
  assert(!cartesian.ready().first);
  rejects(hand);
  assert(cartesian.hold(Json::object()) && held == 1);
  feedback_time = now;
  hand["authority"]["owner"] = "operator";
  rejects(hand);
  cartesian.anchor(hand);
  now += 0.1;
  cartesian.apply(hand);
  assert(published == 2);
  feedback_ready = false;
  assert(!cartesian.ready().first);

  {
    TeachSession diagnostic_session(observe, "teach_robot");
    assert(request(diagnostic_session, observe, "open")["accepted"] == true);
    Json calibration = {{"schema_version", 1}, {"alignment_id", new_uuid()}, {"tracking_origin_id", new_uuid()},
        {"frame_id", "quest_tracking_flu"}, {"child_frame_id", "teach_calibrated_floor_flu"},
        {"position", {0, 0, 0}}, {"rotation", {0, 0, 0, 1}}};
    std::string unicode;
    for (int i = 0; i < 200; ++i) unicode += "\xc3\xa9";
    Json diagnostic = {{"build_guid", unicode}, {"tracking_origin", "Stage"}, {"app_version", false}, {"unknown", "ignored"}};
    assert(request(diagnostic_session, observe, "align", {{"alignment", calibration}, {"client_diagnostics", diagnostic}})["accepted"] == true);
    assert(request(diagnostic_session, observe, "invalidate_alignment", {{"owner", "spoofed"}, {"client_diagnostics", diagnostic}})["accepted"] == false);
    assert(diagnostic_session.state()["alignment_id"] == calibration["alignment_id"]);
    auto diagnostic_id = new_uuid();
    assert(request(diagnostic_session, observe, "record", {{"episode_id", diagnostic_id}})["accepted"] == true);
    wait_capture(diagnostic_session, "recording");
    diagnostic["invalidation_reason"] = "Headset recentered; align again";
    assert(request(diagnostic_session, observe, "invalidate_alignment", {{"expected_revision", -100}, {"client_diagnostics", diagnostic}})["accepted"] == true);
    assert(diagnostic_session.state()["alignment_id"] == "");
    assert(request(diagnostic_session, observe, "finish")["accepted"] == true);
    wait_capture(diagnostic_session, "saved");
    Episode diagnostic_episode((root / diagnostic_id).string());
    int evidence_count = 0;
    diagnostic_episode.samples([&](const Json &, const std::uint8_t *data, std::size_t size) {
      const auto event = Json::parse(data, data + size);
      const auto kind = event.at("kind").get<std::string>();
      if (kind != "alignment" && kind != "recording_started" && kind != "alignment_invalidated") return true;
      const auto context = event.at("data").at("client_diagnostics");
      assert(context.at("build_guid").get<std::string>() == unicode.substr(0, 256));
      assert(!context.contains("app_version") && !context.contains("unknown"));
      if (kind == "alignment_invalidated") {
        assert(event.at("data").at("reason") == "tracking alignment invalidated");
        assert(context.at("invalidation_reason") == "Headset recentered; align again");
      }
      ++evidence_count;
      return true;
    }, {"__events"});
    assert(evidence_count == 3);
    for (const auto &malformed : {Json(), Json::array(), Json("not-an-object"), Json{{"build_guid", Json::array()}}})
      assert(request(diagnostic_session, observe, "invalidate_alignment", {{"expected_revision", -1}, {"client_diagnostics", malformed}})["accepted"] == true);
    diagnostic_session.close();
  }

  TeachSession recorder_session(observe, "teach_robot");
  assert(request(recorder_session, observe, "open")["accepted"] == true);
  assert(request(recorder_session, observe, "engage")["accepted"] == false);
  auto id = new_uuid();
  assert(request(recorder_session, observe, "record",
                 {{"episode_id", id}})["accepted"] == true);
  wait_capture(recorder_session, "recording");
  assert(request(recorder_session, observe, "finish",
                 {{"outcome", "success"}})["accepted"] == true);
  wait_capture(recorder_session, "saved");
  recorder_session.close();
  Episode episode((root / id).string());
  assert(episode.validate().empty());
  int events = 0;
  episode.samples(
      [&](const Json &, const std::uint8_t *data, std::size_t size) {
        auto e = Json::parse(data, data + size);
        assert(e.contains("kind"));
        ++events;
        return true;
      },
      {"__events"});
  assert(events >= 3);
  bool rejected = false;
  try {
    EpisodeRecorder duplicate(observe, "teach_robot", id, "");
  } catch (...) {
    rejected = true;
  }
  assert(rejected);
  TeachConfig small(root.string(), {{"queue_budget_bytes", 1024},
                                    {"minimum_free_bytes", 1024}});
  EpisodeRecorder bounded(small, "teach_robot", new_uuid(), "overflow");
  assert(!bounded.event("huge", std::string(4096, 'x')));
  auto result = bounded.finish();
  assert(result["quality"] == "unacceptable");
  assert(bounded.finish() == result);
  auto damaged = root / (new_uuid() + ".inprogress");
  fs::copy(root / id, damaged, fs::copy_options::recursive);
  auto data = damaged / "data.mcap";
  fs::resize_file(data, fs::file_size(data) - 20);
  auto before = file_digest(data.string());
  auto recovered = recover_episode(damaged.string());
  assert(file_digest(data.string()) == before);
  Episode restored(recovered);
  assert(restored.validate().empty());
  assert(restored.manifest()["quality"] == "unacceptable" &&
         restored.manifest()["capture"] == "interrupted");
  assert(restored.manifest()["recovery"]["lost_tail"] == "unknown");
  EpisodeRecorder active(observe, "teach_robot", new_uuid(), "active lock");
  bool active_rejected = false;
  try {
    recover_episode(active.path());
  } catch (const std::exception &) {
    active_rejected = true;
  }
  assert(active_rejected);
  active.finish();
  {
    auto collision_id = new_uuid();
    EpisodeRecorder collision(observe, "teach_robot", collision_id,
                              "late destination conflict");
    assert(collision.event("preserved", {{"value", 42}}));
    auto staging = fs::path(collision.path());
    auto destination = root / collision_id;
    fs::create_directory(destination);
    bool refused = false;
    try {
      collision.finish();
    } catch (const std::exception &) {
      refused = true;
    }
    assert(refused);
    assert(fs::is_empty(destination));
    assert(fs::is_regular_file(staging / "data.mcap"));
    assert(Episode(staging.string()).validate().empty());
  }
  fs::remove_all(root);
}
