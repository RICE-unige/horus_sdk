#include "horus/teaching/session.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <future>
#include <mutex>
#include <set>
#include <array>

namespace horus::teaching {
namespace {
using Quat = std::array<double, 4>;
Json client_diagnostics(const Json &request) {
  Json result = Json::object();
  auto value = request.find("client_diagnostics");
  if (value == request.end() || !value->is_object()) return result;
  for (auto key : {"build_guid", "app_version", "tracking_origin", "tracking_origin_id", "invalidation_reason"}) {
    auto item = value->find(key);
    if (item == value->end() || !item->is_string()) continue;
    auto text = item->get<std::string>();
    // Keep UTF-8 code points intact at the diagnostic byte limit.
    std::size_t end = std::min<std::size_t>(text.size(), 256);
    while (end < text.size() && end > 0 && (static_cast<unsigned char>(text[end]) & 0xc0) == 0x80) --end;
    result[key] = text.substr(0, end);
  }
  return result;
}
Quat multiply(const Quat &a, const Quat &b) {
  return {a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
          a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
          a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
          a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]};
}
Json validate_alignment(const Json &a) {
  if (!a.is_object() || !a.value("schema_version", Json()).is_number_integer() ||
      a.at("schema_version") != 1)
    throw std::invalid_argument("Teach alignment schema 1 is required");
  validate_uuid(a.at("alignment_id").get<std::string>());
  validate_uuid(a.at("tracking_origin_id").get<std::string>());
  if (a.value("frame_id", "") != "quest_tracking_flu" ||
      a.value("child_frame_id", "") != "teach_calibrated_floor_flu")
    throw std::invalid_argument("Unexpected alignment frames");
  validate_pose(a);
  auto q = a.at("rotation").get<Quat>();
  double norm = 0;
  for (auto v : q) norm += v*v;
  for (auto &v : a.at("position"))
    if (std::abs(v.get<double>()) > 100)
      throw std::invalid_argument("Alignment exceeds metric tracking bounds");
  if (std::abs(norm-1) > 0.001 || q[0]*q[0]+q[1]*q[1] > std::pow(std::sin(3.141592653589793/360), 2))
    throw std::invalid_argument("Floor alignment must be normalized and gravity aligned");
  Json result = Json::object();
  for (auto key : {"schema_version", "alignment_id", "tracking_origin_id", "frame_id", "child_frame_id", "position", "rotation"})
    result[key] = a.at(key);
  return result;
}
Json map_input(const Json &s, const Json &a) {
  if (a.is_null()) throw std::invalid_argument("Complete alignment before engagement");
  if (s.value("alignment_id", Json()) != a.at("alignment_id") ||
      s.value("tracking_origin_id", Json()) != a.at("tracking_origin_id") ||
      s.value("frame_id", "") != "quest_tracking_flu")
    throw std::invalid_argument("Alignment or tracking origin changed; align again");
  Json result = s;
  auto q = a.at("rotation").get<Quat>();
  double norm = 0;
  for (auto v : q) norm += v*v;
  for (auto &v : q) v /= std::sqrt(norm);
  Quat inverse{-q[0], -q[1], -q[2], q[3]};
  for (auto device : {"head", "left", "right"}) {
    if (s.value("input_schema_version", Json(1)) == 2 &&
        s.value(std::string(device) + "_tracking_valid", Json()) == false) {
      result[device] = nullptr;
      continue;
    }
    if (!s.contains(device) || s[device].is_null()) continue;
    const auto &pose = s[device];
    validate_pose(pose);
    Quat offset{0,0,0,0};
    for (int i=0; i<3; ++i) {
      if (std::abs(pose.at("position")[i].get<double>()) > 100)
        throw std::invalid_argument("XR pose exceeds metric tracking bounds");
      offset[i] = pose.at("position")[i].get<double>() - a.at("position")[i].get<double>();
    }
    const auto p = multiply(multiply(inverse, offset), q);
    result[device] = {{"position", {p[0], p[1], p[2]}},
                      {"rotation", multiply(inverse, pose.at("rotation").get<Quat>())}};
  }
  result["frame_id"] = "teach_calibrated_floor_flu";
  result["source_frame_id"] = "quest_tracking_flu";
  return result;
}
struct TrackingUnavailable : std::invalid_argument {
  using std::invalid_argument::invalid_argument;
};
Json negotiate_input(const Json &config, const Json &request) {
  if (request.value("tracking_recovery", "manual") != config.value("tracking_recovery", "manual"))
    throw std::invalid_argument("Teach tracking recovery mismatch; update the client or profile");
  if (request.value("engagement_mode", "hold_to_run") !=
      config.value("engagement_mode", "hold_to_run"))
    throw std::invalid_argument("Teach engagement mode mismatch; update the client or profile");
  auto version = request.value("input_schema_version", Json(1));
  if (!version.is_number_integer() ||
      version != config.value("input_schema_version", 1))
    throw std::invalid_argument("Teach input schema mismatch; update the "
                                "client or select a compatible profile");
  if (version == 1)
    return Json::array();
  auto features = request.value("input_features", Json());
  const std::set<std::string> supported{"head_pose", "left_stick",
                                        "right_stick", "left_buttons",
                                        "right_buttons"};
  std::set<std::string> unique;
  if (!features.is_array())
    throw std::invalid_argument("Client must declare input_features");
  for (auto &f : features) {
    if (!f.is_string() || !supported.count(f.get<std::string>()) ||
        !unique.insert(f.get<std::string>()).second)
      throw std::invalid_argument("Invalid or duplicate input feature");
  }
  for (auto &f : config.at("required_input_features"))
    if (!unique.count(f.get<std::string>()))
      throw std::invalid_argument("Client lacks required Teach input features");
  std::sort(features.begin(), features.end());
  return features;
}
void validate_input_v2(const Json &config, const Json &s) {
  if (s.value("tracking_recovery", "manual") != config.value("tracking_recovery", "manual"))
    throw std::invalid_argument("Teach input tracking recovery mismatch");
  if (s.value("engagement_mode", "hold_to_run") !=
      config.value("engagement_mode", "hold_to_run"))
    throw std::invalid_argument("Teach input engagement mode mismatch");
  auto version = s.value("input_schema_version", Json());
  if (!version.is_number_integer() || version != 2)
    throw std::invalid_argument("Expected Teach input schema version 2");
  if (s.value("frame_id", "") != "teach_calibrated_floor_flu" ||
      s.value("clock_id", "") != "quest_monotonic" ||
      s.value("pose_time_semantics", "") != "unity_xr_latest_available")
    throw std::invalid_argument(
        "Unexpected Teach pose frame, clock or time semantics");
  auto finite_range = [](const Json &v, double low, double high) {
    return v.is_number() && std::isfinite(v.get<double>()) &&
           v.get<double>() >= low && v.get<double>() <= high;
  };
  auto timestamp = s.value("source_monotonic_seconds", Json());
  if (!timestamp.is_number() || !std::isfinite(timestamp.get<double>()) ||
      timestamp.get<double>() < 0)
    throw std::invalid_argument("Invalid XR sample timestamp");
  auto flag = [&](const std::string &key) {
    auto v = s.value(key, Json());
    if (!v.is_boolean())
      throw std::invalid_argument("Missing boolean input field: " + key);
    return v.get<bool>();
  };
  for (auto key : {"tracking_valid", "clutch", "ui_focus"}) flag(key);
  std::string unavailable;
  std::map<std::string, bool> available;
  for (std::string device : {"head", "left", "right"}) {
    bool valid = flag(device + "_tracking_valid");
    if (valid)
      validate_pose(s.at(device));
    for (auto &hand : config.at("controlled_hands"))
      if (hand == device && !valid)
        unavailable += "Controlled hand tracking unavailable: " + device + "; ";
    if (device == "head")
      available["head_pose"] = valid;
  }
  for (std::string hand : {"left", "right"}) {
    auto stick = hand + "_stick";
    available[stick] = flag(stick + "_valid");
    auto v = s.value(stick, Json());
    if (!v.is_array() || v.size() != 2 || !finite_range(v[0], -1, 1) ||
        !finite_range(v[1], -1, 1))
      throw std::invalid_argument(
          "Stick must contain [x-right, y-forward] within [-1, 1]");
    available[hand + "_buttons"] = flag(hand + "_buttons_valid");
    for (std::string button : {"primary", "secondary", "stick_click"})
      flag(hand + "_" + button);
    for (std::string axis : {"trigger", "grip"}) {
      flag(hand + "_" + axis + "_valid");
      if (!finite_range(s.value(hand + "_" + axis, Json()), 0, 1))
        throw std::invalid_argument("Invalid measured axis");
    }
  }
  for (auto &f : config.at("required_input_features"))
    if (!available.at(f.get<std::string>()))
      unavailable += "Required input feature unavailable: " + f.get<std::string>() + "; ";
  std::string hold = config.at("hold_binding");
  if (flag(hold)) throw std::invalid_argument("Hold button pressed");
  if (!available.at(hold.substr(0, hold.find('_')) + "_buttons"))
    unavailable += "Hold button unavailable; ";
  const bool continuous = config.value("engagement_mode", "hold_to_run") == "continuous";
  std::string clutch = continuous ? config.at("disengage_binding") : config.at("clutch_binding");
  bool grips = clutch.find("grip") != std::string::npos;
  bool all_pressed = true;
  for (std::string hand : {"left", "right"}) {
    if (clutch.rfind("both_", 0) != 0 && clutch.rfind(hand + "_", 0) != 0)
      continue;
    if (!(grips ? flag(hand + "_grip_valid") : available.at(hand + "_buttons"))) {
      if (!continuous) throw std::invalid_argument("Clutch input unavailable");
      unavailable += "Disengage buttons unavailable; ";
    }
    bool held = grips ? s.at(hand + "_grip").get<double>() >= 0.5
                      : flag(hand + "_primary");
    all_pressed = all_pressed && held;
    if (!continuous && s.value("clutch", Json()) == true && !held)
      throw std::invalid_argument("Clutch disagrees with measured controls");
  }
  if (continuous && all_pressed)
    throw std::invalid_argument("Disengage pressed");
  for (auto &binding : config.at("grasp_bindings"))
    if (!flag(binding.get<std::string>() + "_valid"))
      unavailable += "Configured grasp axis unavailable; ";
  if (!unavailable.empty()) throw TrackingUnavailable(unavailable);
}
} // namespace
Json AdapterCapabilities::to_dict() const {
  return {{"adapter_id", adapter_id},
          {"qualification", qualification},
          {"joint_names", joint_names},
          {"command_frame", command_frame},
          {"calibration_id", calibration_id},
          {"action_representation", action_representation},
          {"controller_watchdog", controller_watchdog},
          {"profile_revision", profile_revision},
          {"joint_units", joint_units},
          {"command_boundary", command_boundary},
          {"hold_description", hold_description},
          {"command_period_seconds", command_period_seconds
                                         ? Json(*command_period_seconds)
                                         : Json(nullptr)},
          {"authority_contract", authority_contract},
          {"supports_tracking_pause", supports_tracking_pause}};
}
AdapterCapabilities AdapterCapabilities::from_dict(const Json &v) {
  AdapterCapabilities c;
#define TEXT(key) c.key = v.value(#key, c.key)
  TEXT(adapter_id);
  TEXT(qualification);
  TEXT(command_frame);
  TEXT(calibration_id);
  TEXT(action_representation);
  TEXT(controller_watchdog);
  TEXT(profile_revision);
  TEXT(command_boundary);
  TEXT(hold_description);
  TEXT(authority_contract);
#undef TEXT
  c.joint_names = v.value("joint_names", c.joint_names);
  c.supports_tracking_pause = v.value("supports_tracking_pause", false);
  c.joint_units = v.value("joint_units", c.joint_units);
  if (v.contains("command_period_seconds") &&
      !v["command_period_seconds"].is_null())
    c.command_period_seconds = v["command_period_seconds"].get<double>();
  return c;
}
void ObserveAdapter::anchor(const Json &) {
  throw std::runtime_error("Observation-only adapter cannot engage");
}
Json ObserveAdapter::apply(const Json &) {
  throw std::runtime_error("Observation-only adapter cannot send commands");
}

struct TeachSession::Impl {
  TeachConfig config;
  std::string robot;
  std::shared_ptr<TeachAdapter> adapter;
  std::function<double()> clock;
  mutable std::recursive_mutex mutex;
  std::string control = "viewing", capture = "idle", quality = "pending",
              outcome = "unknown", error, episode;
  std::string owner, session, host = new_uuid(), generation, input_epoch,
                              lease_owner, lease_generation, hold_reason;
  std::uint64_t revision = 0;
  std::int64_t last_seq = -1, last_processed_seq = -1;
  std::string pause_reason;
  double next_pause_check = 0;
  std::int64_t received_seq = -1;
  double input_lock_wait_ms = 0;
  Json hold_input;
  double deadline = 0, last_input = 0, owner_seen = 0, next_hold_check = 0;
  bool anchored = false, closed = false;
  Json latest, authority, alignment;
  Json client_context = Json::object();
  Json input_features = Json::array();
  std::map<std::string, double> health, challenges;
  std::deque<std::string> challenge_order, request_order;
  std::map<std::string, std::tuple<std::string, bool, std::string>> requests;
  std::shared_ptr<EpisodeRecorder> recorder;
  std::map<std::pair<std::string, std::string>, std::shared_ptr<const Sample>> calibration_samples;
  std::future<std::shared_ptr<EpisodeRecorder>> preparing;
  std::future<Json> finishing;
  Impl(TeachConfig c, std::string name, std::shared_ptr<TeachAdapter> a,
       std::function<double()> cl)
      : config(std::move(c)), robot(std::move(name)),
        adapter(a ? std::move(a) : std::make_shared<ObserveAdapter>()),
        clock(std::move(cl)) {
    validate_identifier(robot);
    if (adapter->capabilities().adapter_id !=
        config.to_dict().at("adapter_id").get<std::string>())
      throw std::invalid_argument(
          "Installed adapter does not match declared profile");
  }
  const Json &cfg() const { return config.to_dict(); }
  bool active() const {
    return control == "armed" || control == "engaged" || control == "pause_requested" || control == "paused";
  }
  bool auto_resume() const { return cfg().value("tracking_recovery", "manual") == "auto_resume"; }
  bool owned() const {
    return !owner.empty() && !generation.empty() && owner == lease_owner &&
           generation == lease_generation && clock() < deadline;
  }
  std::vector<std::string> missing(bool critical) const {
    std::set<std::string> cameras;
    for (auto &c : cfg().at("cameras"))
      if (c.at("required_for_control").get<bool>())
        cameras.insert(c.at("stream_id"));
    std::vector<std::string> result;
    for (auto &s : cfg().at("streams")) {
      std::string id = s.at("stream_id");
      auto it = health.find(id);
      if ((critical ? cameras.count(id) > 0 : s.at("required").get<bool>()) &&
          (it == health.end() ||
           (!retained_calibration(s) && clock() - it->second > s.at("max_gap_seconds").get<double>())))
        result.push_back(id);
    }
    return result;
  }
  std::pair<bool, std::string> ready() const {
    try {
      auto c = adapter->capabilities();
      auto result = adapter->ready();
      bool qualified =
          (c.qualification == "hardware" && cfg().at("provenance") == "real") ||
          (c.qualification == "simulation" &&
           (cfg().at("provenance") == "simulation" ||
            cfg().at("provenance") == "fixture"));
      if (!qualified)
        return {false, "Adapter is not qualified for this data provenance"};
      if (Json(c.joint_names) != cfg().at("joint_names") ||
          c.command_frame != cfg().at("command_frame").get<std::string>() ||
          c.calibration_id != cfg().at("calibration_id").get<std::string>())
        return {false, "Adapter joint order, command frame or calibration does "
                       "not match the profile"};
      if (c.controller_watchdog == "unavailable")
        return {false, "Controller-local watchdog is unavailable"};
      if (c.authority_contract == "unavailable")
        return {false, "Controller-scoped authority and hold are unavailable"};
      if (auto_resume() && !c.supports_tracking_pause)
        return {false, "Adapter does not support confirmed tracking pause"};
      return result;
    } catch (const std::exception &e) {
      return {false, std::string("Controller readiness failed: ") + e.what()};
    }
  }
  void event(const std::string &kind, const Json &data) {
    if (recorder && capture == "recording")
      recorder->event(kind, data);
  }
  void confirm_hold() {
    next_hold_check = clock() + 0.1;
    control = "hold_requested";
    try {
      auto context = authority;
      context["reason"] = hold_reason;
      if (adapter->hold(context)) {
        control = "holding";
        authority = nullptr;
      }
    } catch (const std::exception &e) {
      control = "faulted";
      error = e.what();
    }
  }
  void hold(const std::string &reason) {
    latest = nullptr;
    input_epoch.clear();
    anchored = false;
    pause_reason.clear();
    if (authority.is_null())
      return;
    hold_reason = reason;
    hold_input = input_health();
    confirm_hold();
    event("hold", {{"reason", reason}, {"control", control}});
  }
  void invalidate_alignment(const std::string &reason, const Json &diagnostic = Json()) {
    alignment = nullptr;
    hold(reason);
    event("alignment_invalidated", {{"reason", reason},
        {"client_diagnostics", diagnostic.is_null() ? client_context : diagnostic}});
  }
  void confirm_pause() {
    next_pause_check = clock() + 0.1;
    try {
      auto context = authority;
      context["reason"] = pause_reason;
      if (!anchored || adapter->pause(context)) {
        control = "paused";
        challenges.clear();
        challenge_order.clear();
      }
    } catch (const std::exception &e) {
      std::string why = e.what();
      hold("adapter pause fault");
      control = "faulted";
      error = why;
    }
  }
  void pause(const std::string &reason) {
    if (!auto_resume()) { hold(reason); return; }
    latest = nullptr;
    if (authority.is_null() || control == "paused" || control == "pause_requested") return;
    pause_reason = reason;
    challenges.clear();
    challenge_order.clear();
    control = "pause_requested";
    confirm_pause();
    event("tracking_pause", {{"reason", reason}, {"control", control}});
  }
  Json input_health() const {
    return {{"received_sequence", received_seq}, {"accepted_sequence", last_seq},
            {"accepted_age_ms", last_seq >= 0 ? std::max(0.0, (clock() - last_input) * 1000) : -1.0},
            {"callback_lock_wait_ms", input_lock_wait_ms}};
  }
  Json state(const std::string &request = "", bool accepted = true,
             const std::string &reason = "") {
    while (!challenge_order.empty() &&
           (challenges.at(challenge_order.front()) <= clock() ||
            challenge_order.size() >= 32)) {
      challenges.erase(challenge_order.front());
      challenge_order.pop_front();
    }
    auto challenge = new_uuid();
    challenges[challenge] =
        clock() + cfg().at("input_timeout_ms").get<double>() / 1000;
    challenge_order.push_back(challenge);
    auto [ready_now, blocked] = ready();
    auto missing_control = missing(true);
    auto missing_recording = missing(false);
    if (cfg().at("workflow") == "observe_record") {
      ready_now = false;
      blocked = "Observation-only workflow";
    } else if (cfg().at("workflow") == "collection" &&
               (capture != "recording" || quality == "unacceptable")) {
      ready_now = false;
      blocked = "Start a healthy recording before engagement";
    }
    if (blocked.empty() && !missing_control.empty()) {
      blocked = "Stale control streams: ";
      for (auto &id : missing_control)
        blocked += id + " ";
    }
    if (alignment.is_null() && cfg().at("workflow") != "observe_record") {
      ready_now = false;
      blocked = "Complete alignment before engagement";
    }
    Json stale = Json::array();
    for (auto &s : cfg().at("streams")) {
      std::string id = s.at("stream_id");
      if (!health.count(id) ||
          (!retained_calibration(s) && clock() - health.at(id) > s.at("max_gap_seconds").get<double>()))
        stale.push_back(id);
    }
    Json result = {{"schema_version", 1},
                   {"robot_name", robot},
                   {"config_hash", config.digest()},
                   {"request_id", request},
                   {"accepted", accepted},
                   {"reason", reason},
                   {"revision", revision},
                   {"host_id", host},
                   {"session_id", session},
                   {"owner", owner},
                   {"input_epoch", input_epoch},
                   {"challenge", challenge},
                   {"control", control},
                   {"capture", capture},
                   {"quality", quality},
                   {"outcome", outcome},
                   {"episode_id", episode},
                   {"export", "not_requested"},
                   {"control_ready", ready_now && missing_control.empty()},
                   {"control_reason", blocked},
                   {"last_hold_reason", hold_reason},
                   {"input_health", input_health()},
                   {"last_hold_input", hold_input},
                   {"recording_ready", missing_recording.empty()},
                   {"missing_streams", missing_recording},
                   {"stale_streams", stale},
                   {"error", error}};
    result["alignment_schema_version"] = 1;
    result["alignment_required"] = true;
    result["alignment_id"] = alignment.is_null() ? Json("") : alignment.at("alignment_id");
    result["tracking_origin_id"] = alignment.is_null() ? Json("") : alignment.at("tracking_origin_id");
    if (cfg().value("input_schema_version", 1) == 2) {
      result["input_schema_version"] = 2;
      result["input_features"] = input_features;
      result["required_input_features"] = cfg().at("required_input_features");
      result["engagement_mode"] = cfg().value("engagement_mode", "hold_to_run");
    }
    if (auto_resume()) {
      result["tracking_recovery"] = "auto_resume";
      result["pause_reason"] = pause_reason;
    }
    return result;
  }
  void transition(const std::string &action, const Json &r) {
    if (action == "heartbeat")
      return;
    if (action == "hold") {
      hold("operator");
      if (authority.is_null() && control == "faulted") {
        control = "holding";
        error.clear();
      }
      return;
    }
    if (action == "align") {
      if (!authority.is_null() || (control != "viewing" && control != "holding"))
        throw std::invalid_argument("Confirm hold before alignment");
      alignment = validate_alignment(r.at("alignment"));
      client_context = client_diagnostics(r);
      auto evidence = alignment;
      evidence.update(Json{{"owner", owner}, {"session_id", session}, {"config_hash", config.digest()},
          {"client_diagnostics", client_context}});
      event("alignment", evidence);
      return;
    }
    if (action == "invalidate_alignment") {
      invalidate_alignment("tracking alignment invalidated", client_diagnostics(r));
      return;
    }
    if (action == "engage") {
      if (!authority.is_null() ||
          (control != "viewing" && control != "holding"))
        throw std::invalid_argument("Confirm hold before engaging again");
      generation = r.value("authority_generation", "");
      if (!owned())
        throw std::invalid_argument(
            "A current positive robot lease is required");
      if (alignment.is_null() || r.value("alignment_id", Json()) != alignment.at("alignment_id"))
        throw std::invalid_argument("Complete alignment before engagement");
      auto [ok, reason] = ready();
      if (!ok || !missing(true).empty())
        throw std::invalid_argument(
            reason.empty() ? "Required control stream is stale" : reason);
      if (cfg().at("workflow") == "observe_record")
        throw std::invalid_argument("This workflow is observation only");
      if (cfg().at("workflow") == "collection" &&
          (capture != "recording" || quality == "unacceptable"))
        throw std::invalid_argument(
            "Healthy recording must be confirmed before engagement");
      control = "armed";
      authority = {{"robot_name", robot},
                   {"owner", owner},
                   {"session_id", session},
                   {"authority_generation", generation}};
      input_epoch = new_uuid();
      last_seq = -1;
      last_processed_seq = -1;
      pause_reason.clear();
      latest = nullptr;
      anchored = false;
      last_input = clock();
      event("armed", {{"authority_generation", generation}});
      return;
    }
    if (action == "record") {
      if ((capture != "idle" && capture != "saved" &&
           capture != "interrupted") ||
          preparing.valid() || finishing.valid())
        throw std::invalid_argument("Recorder is busy");
      if (!missing(false).empty())
        throw std::invalid_argument("Required recording streams are stale");
      auto id = r.value("episode_id", "");
      validate_uuid(id);
      auto task = r.value("task", cfg().at("task").get<std::string>());
      if (task.empty())
        task = cfg().at("task");
      if (task.size() > 4096)
        throw std::invalid_argument("Task exceeds 4096 characters");
      hold("new episode");
      episode = id;
      capture = "preparing";
      quality = "pending";
      outcome = "unknown";
      error.clear();
      preparing =
          std::async(std::launch::async, [c = config, name = robot, id, task] {
            return std::make_shared<EpisodeRecorder>(c, name, id, task);
          });
      return;
    }
    if (action == "finish") {
      if (capture != "recording" || !recorder)
        throw std::invalid_argument("No active recording to finish");
      auto v = r.value("outcome", "unknown");
      if (v != "unknown" && v != "success" && v != "failure" && v != "abort")
        throw std::invalid_argument("Invalid outcome");
      hold("finish");
      outcome = v;
      event("finish_requested", {{"outcome", v}, {"control", control}});
      capture = "finalizing";
      finishing = std::async(std::launch::async,
                             [rec = recorder, v] { return rec->finish(v); });
      return;
    }
    if (action == "close") {
      hold("exit");
      if (capture == "preparing" || capture == "recording" ||
          capture == "finalizing")
        throw std::invalid_argument(
            "Finish the recording before closing the session");
      owner.clear();
      session.clear();
      alignment = nullptr;
      input_features = Json::array();
      control = "viewing";
      return;
    }
    throw std::invalid_argument("Unknown teaching action");
  }
  void tick() {
    if (recorder && (!recorder->error().empty() || recorder->dropped())) {
      error = recorder->error().empty()
                  ? "Recorder queue dropped " +
                        std::to_string(recorder->dropped()) + " samples"
                  : recorder->error();
      quality = "unacceptable";
      if (cfg().at("workflow") == "collection" &&
          active())
        hold("recorder failure");
    }
    if (control == "hold_requested" && clock() >= next_hold_check)
      confirm_hold();
    if (!owner.empty() && clock() - owner_seen > 5) {
      invalidate_alignment("operator disconnected");
      owner.clear();
      session.clear();
      input_features = Json::array();
      if (capture == "recording" && recorder) {
        capture = "finalizing";
        finishing = std::async(std::launch::async, [rec = recorder] {
          return rec->finish("abort", true);
        });
      }
      ++revision;
    }
    if (active()) {
      const auto readiness = ready();
      const auto missing_control = missing(true);
      std::string stop_reason;
      if (!owned()) stop_reason = "authority expired";
      else if (!readiness.first) stop_reason = "controller unavailable: " + readiness.second;
      else if (!missing_control.empty()) {
        stop_reason = "stale control streams: ";
        for (size_t i = 0; i < missing_control.size(); ++i) {
          if (i) stop_reason += ", ";
          stop_reason += missing_control[i];
        }
      }
      if (!stop_reason.empty()) {
        if (!owned()) invalidate_alignment(stop_reason);
        else hold(stop_reason);
      }
      else if (control == "pause_requested" && clock() >= next_pause_check)
        confirm_pause();
      else if (control == "engaged" && clock() - last_input > cfg().at("input_timeout_ms").get<double>() / 1000)
        pause("input watchdog expired");
      else if (!latest.is_null()) {
        auto sample = std::move(latest);
        latest = nullptr;
        try {
          if (!anchored) {
            adapter->anchor(sample);
            anchored = true;
            event("anchored", {{"sequence", sample.at("sequence")}});
          } else
            event("command_result", adapter->apply(sample));
        } catch (const std::exception &e) {
          auto reason = std::string(e.what());
          hold("adapter fault");
          error = reason;
          control = "faulted";
        }
      }
    }
    try {
      if (preparing.valid() && preparing.wait_for(std::chrono::seconds(0)) ==
                                   std::future_status::ready) {
        recorder = preparing.get();
        capture = "recording";
        event("adapter_contract", adapter->capabilities().to_dict());
        event("recording_started",
              {{"session_id", session},
               {"lease_owner", lease_owner},
               {"authority_generation", lease_generation}, {"client_diagnostics", client_context}});
        if (!alignment.is_null()) {
          auto evidence = alignment;
          evidence.update(Json{{"owner", owner}, {"session_id", session}, {"config_hash", config.digest()},
              {"client_diagnostics", client_context}});
          event("alignment", evidence);
        }
        std::set<std::shared_ptr<const Sample>> unique;
        for (auto &[key, sample] : calibration_samples) unique.insert(sample);
        std::vector<std::shared_ptr<const Sample>> snapshots(unique.begin(), unique.end());
        std::sort(snapshots.begin(), snapshots.end(), [](auto &a, auto &b) {
          return a->received_monotonic_ns < b->received_monotonic_ns;
        });
        for (auto &sample : snapshots) {
          recorder->append(*sample);
          event("calibration_snapshot", {{"stream_id", sample->stream_id},
              {"original_receipt_monotonic_ns", sample->received_monotonic_ns},
              {"original_receipt_unix_ns", sample->received_unix_ns}});
        }
        if (owner.empty() && !closed) {
          capture = "finalizing";
          finishing = std::async(std::launch::async, [rec = recorder] {
            return rec->finish("abort", true);
          });
        }
        ++revision;
      }
      if (finishing.valid() && finishing.wait_for(std::chrono::seconds(0)) ==
                                   std::future_status::ready) {
        auto result = finishing.get();
        capture = result.at("capture");
        quality = result.at("quality");
        recorder.reset();
        ++revision;
      }
    } catch (const std::exception &e) {
      capture = "interrupted";
      quality = "unacceptable";
      error = e.what();
      ++revision;
    }
  }
};

TeachSession::TeachSession(TeachConfig c, std::string name,
                           std::shared_ptr<TeachAdapter> a,
                           std::function<double()> clock)
    : impl_(std::make_unique<Impl>(std::move(c), std::move(name), std::move(a),
                                   std::move(clock))) {}
TeachSession::~TeachSession() {
  try {
    close();
  } catch (...) {
  }
}
void TeachSession::update_lease(const std::string &owner,
                                const std::string &generation, double ttl) {
  std::lock_guard guard(impl_->mutex);
  if (!std::isfinite(ttl))
    ttl = 0;
  ttl = std::clamp(ttl, 0.0, 3.0);
  bool changed =
      owner != impl_->lease_owner || generation != impl_->lease_generation;
  impl_->lease_owner = owner;
  impl_->lease_generation = generation;
  impl_->deadline = impl_->clock() + ttl;
  if (changed)
    impl_->event("authority_changed", {{"owner", owner},
                                       {"authority_generation", generation},
                                       {"remaining_seconds", ttl}});
  if (impl_->active() &&
      !impl_->owned())
    impl_->invalidate_alignment("authority changed");
}
void TeachSession::note_stream(const std::string &id) {
  std::lock_guard guard(impl_->mutex);
  for (auto &s : impl_->cfg().at("streams"))
    if (s.at("stream_id") == id) {
      impl_->health[id] = impl_->clock();
      return;
    }
  throw std::invalid_argument("Undeclared stream");
}
std::vector<std::string> TeachSession::missing_streams(bool control) const {
  std::lock_guard guard(impl_->mutex);
  return impl_->missing(control);
}
std::pair<bool, std::string> TeachSession::readiness() const {
  std::lock_guard guard(impl_->mutex);
  return impl_->ready();
}
Json TeachSession::state(const std::string &id, bool accepted,
                         const std::string &reason) {
  std::lock_guard guard(impl_->mutex);
  return impl_->state(id, accepted, reason);
}
Json TeachSession::request(const Json &r) {
  std::lock_guard guard(impl_->mutex);
  std::string id, fingerprint, reason;
  bool accepted = false;
  try {
    if (!r.is_object())
      throw std::invalid_argument("Request must be an object");
    id = r.value("request_id", "");
    validate_uuid(id);
    fingerprint = canonical(r);
    auto previous = impl_->requests.find(id);
    if (previous != impl_->requests.end()) {
      auto [f, ok, why] = previous->second;
      return impl_->state(
          id, f == fingerprint && ok,
          f == fingerprint ? why : "Request ID reused with different content");
    }
    if (impl_->closed)
      throw std::invalid_argument("Session is closed");
    if (r.value("config_hash", "") != impl_->config.digest())
      throw std::invalid_argument("Registration/profile changed; reopen Teach");
    auto action = r.value("action", "");
    if (action == "open") {
      auto features = negotiate_input(impl_->cfg(), r);
      auto owner = r.value("owner", "");
      if (owner.empty() || owner.size() > 128)
        throw std::invalid_argument("Operator identity is required");
      if (!impl_->owner.empty() && impl_->owner != owner)
        throw std::invalid_argument(
            "Another operator owns this teaching session");
      if (impl_->owner.empty()) {
        if (impl_->capture == "preparing" || impl_->capture == "recording" ||
            impl_->capture == "finalizing")
          throw std::invalid_argument("Previous session is still finalizing");
        impl_->owner = owner;
        impl_->session = new_uuid();
        impl_->control = "viewing";
        impl_->input_features = features;
        impl_->alignment = nullptr;
        impl_->client_context = client_diagnostics(r);
      } else if (features != impl_->input_features) {
        throw std::invalid_argument(
            "Input capabilities changed; close and reopen Teach");
      }
    } else {
      if (r.value("session_id", "") != impl_->session ||
          r.value("owner", "") != impl_->owner || impl_->owner.empty())
        throw std::invalid_argument("Wrong teaching session or operator");
      if (action != "hold" && action != "heartbeat" && action != "invalidate_alignment" &&
          (!r.contains("expected_revision") ||
           r["expected_revision"] != impl_->revision))
        throw std::invalid_argument("State changed; refresh before retrying");
      impl_->transition(action, r);
    }
    impl_->owner_seen = impl_->clock();
    if (action != "heartbeat")
      ++impl_->revision;
    accepted = true;
  } catch (const std::exception &e) {
    reason = e.what();
  }
  if (!id.empty() && !fingerprint.empty()) {
    impl_->requests[id] = {fingerprint, accepted, reason};
    impl_->request_order.push_back(id);
    while (impl_->request_order.size() > 256) {
      impl_->requests.erase(impl_->request_order.front());
      impl_->request_order.pop_front();
    }
  }
  return impl_->state(id, accepted, reason);
}
bool TeachSession::input(const Json &raw) {
  Json s = raw;
  const auto received = impl_->clock();
  std::lock_guard guard(impl_->mutex);
  if (s.is_object() && !impl_->owner.empty() &&
      s.value("session_id", Json()) == impl_->session &&
      s.value("owner", Json()) == impl_->owner && s.contains("sequence") &&
      s["sequence"].is_number_integer() &&
      (!s["sequence"].is_number_unsigned() || s["sequence"].get<std::uint64_t>() <= std::uint64_t(INT64_MAX))) {
    impl_->received_seq = s["sequence"].get<std::int64_t>();
    impl_->input_lock_wait_ms = std::max(0.0, (impl_->clock() - received) * 1000);
  }
  if (!s.is_object() || !impl_->active())
    return false;
  try {
    if (s.value("session_id", "") != impl_->session ||
        s.value("input_epoch", "") != impl_->input_epoch ||
        s.value("owner", "") != impl_->owner || !impl_->owned())
      return false;
    if (!s.contains("sequence") || !s["sequence"].is_number_integer())
      return false;
    if (s["sequence"].is_number_unsigned() &&
        s["sequence"].get<std::uint64_t>() > std::uint64_t(INT64_MAX))
      return false;
    if (s["sequence"].get<std::int64_t>() <= impl_->last_processed_seq)
      return false;
    impl_->last_processed_seq = s["sequence"].get<std::int64_t>();
    try { s = map_input(s, impl_->alignment); }
    catch (...) { impl_->invalidate_alignment("invalid alignment or tracking pose"); throw; }
    if (impl_->cfg().value("input_schema_version", 1) == 2)
      validate_input_v2(impl_->cfg(), s);
    else if (s.value("input_schema_version", Json(1)) != 1)
      throw std::invalid_argument("Expected legacy Teach input");
    auto challenge = s.value("challenge", "");
    auto found = impl_->challenges.find(challenge);
    if (found == impl_->challenges.end() || found->second <= impl_->clock()) {
      impl_->pause("stale transport sample");
      return false;
    }
    if (impl_->cfg().value("engagement_mode", "hold_to_run") == "hold_to_run" &&
        impl_->control == "armed" && s.value("clutch", Json()) == false)
      return false;
    if (s.value("clutch", Json()) != true) {
      impl_->hold("clutch released");
      return false;
    }
    if (s.value("tracking_valid", Json()) != true || s.value("ui_focus", Json()) != false) {
      impl_->owner_seen = impl_->clock();
      impl_->pause("tracking or UI focus unavailable");
      return false;
    }
    for (auto &hand : impl_->cfg().at("controlled_hands"))
      validate_pose(s.at(hand.get<std::string>()));
    Json tools = Json::object();
    for (auto it = impl_->cfg().at("grasp_bindings").begin();
         it != impl_->cfg().at("grasp_bindings").end(); ++it) {
      auto v = s.at(it.value().get<std::string>());
      if (!v.is_number() || !std::isfinite(v.get<double>()) ||
          v.get<double>() < 0 || v.get<double>() > 1)
        throw std::invalid_argument("Invalid grasp axis");
      tools[it.key()] = v;
    }
    if (impl_->control == "pause_requested") return false;
    if (impl_->control == "paused") {
      auto [ready, reason] = impl_->ready();
      if (!ready || !impl_->missing(true).empty()) {
        impl_->hold(!ready ? "controller unavailable: " + reason : "stale control streams");
        return false;
      }
      impl_->event("tracking_resumed", {{"sequence", s.at("sequence")}, {"reason", impl_->pause_reason}});
      impl_->pause_reason.clear();
    }
    impl_->last_seq = s.at("sequence");
    impl_->last_input = impl_->clock();
    impl_->owner_seen = impl_->last_input;
    impl_->control = "engaged";
    impl_->latest = s;
    impl_->latest["tools"] = tools;
    impl_->latest["authority"] = impl_->authority;
    impl_->event("operator_intent", s);
    return true;
  } catch (const TrackingUnavailable &e) {
    impl_->owner_seen = impl_->clock();
    if (s.value("clutch", Json()) == true) impl_->pause(e.what());
    else impl_->hold("clutch released");
    return false;
  } catch (const std::exception &e) {
    impl_->hold("invalid input");
    impl_->error = e.what();
    return false;
  }
}
bool TeachSession::append(Sample sample, const std::vector<std::string> &cache_keys) {
  std::shared_ptr<EpisodeRecorder> rec;
  {
    std::lock_guard guard(impl_->mutex);
    auto &streams = impl_->cfg().at("streams");
    auto stream = std::find_if(streams.begin(), streams.end(), [&](const Json &s) {
      return s.at("stream_id") == sample.stream_id;
    });
    if (stream == streams.end()) throw std::invalid_argument("Undeclared recording stream");
    note_stream(sample.stream_id);
    if (retained_calibration(*stream)) {
      auto next = impl_->calibration_samples;
      auto retained = std::make_shared<const Sample>(sample);
      for (auto &key : cache_keys) {
        if (key.empty() || key.size() > 512) throw std::invalid_argument("Invalid calibration cache key");
        next[{sample.stream_id, key}] = retained;
      }
      std::set<const Sample *> unique;
      std::size_t bytes = 0;
      for (auto &[key, value] : next)
        if (unique.insert(value.get()).second) bytes += value->payload.size() + value->schema.size() + 512;
      if (next.size() > 2048 || bytes > 16 * 1024 * 1024) {
        impl_->error = "Calibration snapshot cache limit exceeded";
        impl_->quality = "unacceptable";
        impl_->hold(impl_->error);
        return false;
      }
      impl_->calibration_samples = std::move(next);
    }
    if (impl_->capture == "recording")
      rec = impl_->recorder;
  }
  return rec && rec->append(std::move(sample));
}
bool TeachSession::recording() const {
  std::lock_guard guard(impl_->mutex);
  return impl_->capture == "recording";
}
void TeachSession::tick() {
  std::lock_guard guard(impl_->mutex);
  impl_->tick();
}
void TeachSession::close() {
  std::lock_guard guard(impl_->mutex);
  if (impl_->closed)
    return;
  impl_->closed = true;
  impl_->invalidate_alignment("runtime shutdown");
  impl_->owner.clear();
  impl_->session.clear();
  impl_->input_features = Json::array();
  if (impl_->preparing.valid())
    impl_->preparing.wait();
  if (impl_->finishing.valid())
    impl_->finishing.wait();
  impl_->tick();
  if (impl_->recorder && impl_->capture == "recording") {
    auto result = impl_->recorder->finish("abort", true);
    impl_->capture = result.at("capture");
    impl_->quality = result.at("quality");
    impl_->recorder.reset();
  }
}
} // namespace horus::teaching
