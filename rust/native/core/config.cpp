#include "horus/teaching/config.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace horus::teaching {
namespace {
void require(bool ok, const char *reason) {
  if (!ok)
    throw std::invalid_argument(reason);
}
Json normalized(const Json &value, const Json &defaults) {
  require(value.is_object(), "Expected a configuration object");
  Json out = defaults;
  for (auto it = value.begin(); it != value.end(); ++it) {
    require(defaults.contains(it.key()),
            ("Unknown configuration key: " + it.key()).c_str());
    out[it.key()] = it.value();
  }
  return out;
}
void text(const Json &value, bool nonempty = true) {
  require(value.is_string() &&
              (!nonempty || !value.get_ref<const std::string &>().empty()),
          "Expected a string");
}
void choice(const Json &value, std::initializer_list<const char *> values) {
  require(value.is_string(), "Expected an option string");
  for (auto v : values)
    if (value == v)
      return;
  throw std::invalid_argument("Unsupported configuration option: " +
                              value.dump());
}
void number(const Json &value, double low, double high) {
  require(value.is_number() && std::isfinite(value.get<double>()) &&
              value.get<double>() >= low && value.get<double>() <= high,
          "Numeric configuration is outside its finite range");
}
void integer(const Json &value, std::int64_t low, std::int64_t high) {
  number(value, static_cast<double>(low), static_cast<double>(high));
  require(value.is_number_integer(), "Expected an integer configuration value");
}
void vector(const Json &v, std::size_t size) {
  require(v.is_array() && v.size() == size, "Wrong vector length");
  for (const auto &x : v)
    number(x, -1e100, 1e100);
}
std::set<std::string> unique_strings(const Json &v) {
  require(v.is_array(), "Expected an array");
  std::set<std::string> out;
  for (const auto &item : v) {
    text(item);
    require(out.insert(item.get<std::string>()).second, "Duplicate entry");
  }
  return out;
}
std::string float_repr(double value) {
  require(std::isfinite(value), "Nonfinite JSON number");
  char buffer[64];
  auto result = std::to_chars(buffer, buffer + sizeof(buffer), value,
                              std::chars_format::scientific);
  if (result.ec != std::errc())
    throw std::runtime_error("Number formatting failed");
  std::string repr(buffer, result.ptr);
  auto e = repr.find('e');
  int exponent = std::stoi(repr.substr(e + 1));
  std::string sign = value < 0 || std::signbit(value) ? "-" : "";
  std::string digits = repr.substr(sign.size(), e - sign.size());
  digits.erase(std::remove(digits.begin(), digits.end(), '.'), digits.end());
  if (exponent < -4 || exponent >= 16) {
    std::string mantissa = digits.substr(0, 1);
    if (digits.size() > 1)
      mantissa += "." + digits.substr(1);
    std::string exp = std::to_string(std::abs(exponent));
    if (exp.size() < 2)
      exp = "0" + exp;
    return sign + mantissa + "e" + (exponent < 0 ? "-" : "+") + exp;
  }
  int point = exponent + 1;
  if (point <= 0)
    return sign + "0." + std::string(-point, '0') + digits;
  if (point >= static_cast<int>(digits.size()))
    return sign + digits + std::string(point - digits.size(), '0') + ".0";
  return sign + digits.substr(0, point) + "." + digits.substr(point);
}
} // namespace

std::string canonical(const Json &value) {
  if (value.is_number_float())
    return float_repr(value.get<double>());
  if (value.is_array() || value.is_object()) {
    std::string out = value.is_array() ? "[" : "{";
    bool first = true;
    for (auto it = value.begin(); it != value.end(); ++it) {
      if (!first)
        out += ',';
      first = false;
      if (value.is_object())
        out += Json(it.key()).dump(-1, ' ', true) + ':';
      out += canonical(it.value());
    }
    return out + (value.is_array() ? "]" : "}");
  }
  return value.dump(-1, ' ', true);
}

std::string sha256(const std::string &value) {
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int length = 0;
  if (EVP_Digest(value.data(), value.size(), digest, &length, EVP_sha256(),
                 nullptr) != 1)
    throw std::runtime_error("SHA-256 failed");
  std::ostringstream out;
  for (unsigned int i = 0; i < length; ++i)
    out << std::hex << std::setfill('0') << std::setw(2) << unsigned(digest[i]);
  return out.str();
}
void validate_identifier(const std::string &value) {
  static const std::regex pattern("[A-Za-z][A-Za-z0-9_-]{0,95}");
  require(std::regex_match(value, pattern), "Invalid identifier");
}
void validate_uuid(const std::string &value) {
  static const std::regex pattern("[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}"
                                  "-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}");
  require(std::regex_match(value, pattern), "Expected a canonical UUID");
}
std::string new_uuid() {
  unsigned char bytes[16];
  if (RAND_bytes(bytes, sizeof(bytes)) != 1)
    throw std::runtime_error("UUID entropy unavailable");
  bytes[6] = (bytes[6] & 15) | 64;
  bytes[8] = (bytes[8] & 63) | 128;
  std::ostringstream out;
  for (int i = 0; i < 16; ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10)
      out << '-';
    out << std::hex << std::setfill('0') << std::setw(2) << unsigned(bytes[i]);
  }
  return out.str();
}
std::uint64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
std::uint64_t unix_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
double monotonic_seconds() { return monotonic_ns() / 1e9; }
void validate_pose(const Json &value) {
  require(value.is_object() && value.contains("position") &&
              value.contains("rotation"),
          "Missing controller pose");
  vector(value.at("position"), 3);
  vector(value.at("rotation"), 4);
  double norm = 0;
  for (auto &x : value.at("rotation"))
    norm += x.get<double>() * x.get<double>();
  require(std::abs(norm - 1) <= 0.02,
          "Controller quaternion is not normalized");
}

TeachConfig::TeachConfig(const std::string &root, Json options)
    : TeachConfig([&] {
        options["recording_root"] = root;
        return options;
      }()) {}
Json TeachStream::to_dict() const {
  return {{"stream_id", stream_id},
          {"topic", topic},
          {"message_type", message_type},
          {"role", role},
          {"required", required},
          {"max_gap_seconds", max_gap_seconds},
          {"clock_id", clock_id},
          {"frame_id", frame_id},
          {"source_identity", source_identity},
          {"stamp_semantics", stamp_semantics},
          {"calibration", calibration},
          {"reliability", reliability},
          {"durability", durability},
          {"queue_depth", queue_depth}};
}
bool retained_calibration(const Json &stream) {
  return stream.at("message_type") == "sensor_msgs/msg/CameraInfo" ||
      (stream.at("message_type") == "tf2_msgs/msg/TFMessage" && stream.at("durability") == "transient_local");
}
Json TeachPortal::to_dict() const {
  return {{"mode", mode}, {"edge_fade", edge_fade}, {"corner_radius", corner_radius},
          {"crop", crop}, {"swap_eyes", swap_eyes}, {"max_skew_ms", max_skew_ms}};
}
Json TeachCamera::to_dict() const {
  Json value = {{"sensor_id", sensor_id},
          {"role", role},
          {"position", position},
          {"width_m", width_m},
          {"stream_id", stream_id},
          {"required_for_control", required_for_control},
          {"rotation_deg", rotation_deg}};
  if (portal) value["portal"] = portal->to_dict();
  return value;
}
TeachConfig TeachConfig::with_options(const Json &options) const {
  require(options.is_object(), "Teach options must be an object");
  auto value = config_;
  value.update(options);
  return TeachConfig(value);
}
TeachConfig TeachConfig::with_stream(const TeachStream &stream) const {
  auto value = config_;
  value["streams"].push_back(stream.to_dict());
  return TeachConfig(value);
}
TeachConfig TeachConfig::with_camera(const TeachCamera &camera) const {
  auto value = config_;
  value["cameras"].push_back(camera.to_dict());
  return TeachConfig(value);
}
Json TeachPointCloud::to_dict() const {
  return {{"stream_id", stream_id}, {"point_size_m", point_size_m},
          {"max_points", max_points}, {"max_age_seconds", max_age_seconds}};
}
TeachConfig TeachConfig::with_spatial_view(const TeachPointCloud &view) const {
  auto value = config_;
  if (!value.contains("spatial_views")) value["spatial_views"] = Json::array();
  value["spatial_views"].push_back(view.to_dict());
  return TeachConfig(value);
}
TeachConfig::TeachConfig(Json value) {
  const auto array = Json::array();
  const auto object = Json::object();
  config_ = normalized(value, {{"recording_root", ""},
                               {"streams", array},
                               {"cameras", array},
                               {"spatial_views", array},
                               {"task", ""},
                               {"adapter_id", "observe"},
                               {"workflow", "observe_record"},
                               {"joint_names", array},
                               {"joint_groups", object},
                               {"command_frame", ""},
                               {"calibration_id", ""},
                               {"provenance", "real"},
                               {"publish_rate_hz", 30.0},
                               {"input_timeout_ms", 200},
                               {"controlled_hands", {"left", "right"}},
                               {"clutch_binding", "both_grips"},
                               {"hold_binding", "right_secondary"},
                               {"grasp_bindings", object},
                               {"task_presets", array},
                               {"queue_budget_bytes", 64 * 1024 * 1024},
                               {"minimum_free_bytes", 512 * 1024 * 1024},
                               {"input_schema_version", 1},
                               {"required_input_features", array},
                               {"engagement_mode", "hold_to_run"},
                               {"engage_countdown_seconds", 3.0},
                               {"disengage_binding", "both_primary"},
                               {"tracking_recovery", "manual"},
                               {"schema_version", 1}});
  auto &c = config_;
  text(c["recording_root"]);
  std::string root = c["recording_root"];
  require(std::filesystem::path(root).is_absolute() || root.rfind("~/", 0) == 0,
          "recording_root must be an absolute host path");
  integer(c["schema_version"], 1, 1);
  integer(c["input_schema_version"], 1, 2);
  choice(c["engagement_mode"], {"hold_to_run", "continuous"});
  choice(c["tracking_recovery"], {"manual", "auto_resume"});
  require(c["tracking_recovery"] != "auto_resume" ||
              (c["engagement_mode"] == "continuous" && c["input_schema_version"] == 2 &&
               (c["provenance"] == "simulation" || c["provenance"] == "fixture")),
          "Automatic tracking recovery requires continuous v2 simulation or fixture input");
  require(c["engagement_mode"] != "continuous" || c["input_schema_version"] == 2,
          "Continuous engagement requires input schema version 2");
  number(c["engage_countdown_seconds"], 3, 10);
  c["engage_countdown_seconds"] = c["engage_countdown_seconds"].get<double>();
  choice(c["disengage_binding"], {"both_primary", "left_primary", "right_primary"});
  auto features = unique_strings(c["required_input_features"]);
  for (auto &feature : features)
    choice(feature, {"head_pose", "left_stick", "right_stick", "left_buttons",
                     "right_buttons"});
  if (c["input_schema_version"] == 1) {
    require(features.empty(),
            "Required input features need input schema version 2");
    // Preserve the existing v1 normalized JSON and digest.
    c.erase("input_schema_version");
    c.erase("required_input_features");
  }
  choice(c["workflow"], {"practice", "collection", "observe_record"});
  choice(c["provenance"], {"real", "simulation", "fixture"});
  text(c["adapter_id"]);
  validate_identifier(c["adapter_id"]);
  for (auto key : {"task", "command_frame", "calibration_id"})
    text(c[key], false);
  number(c["publish_rate_hz"], 5, 90);
  integer(c["input_timeout_ms"], 50, 500);
  auto hands = unique_strings(c["controlled_hands"]);
  require(!hands.empty() &&
              std::all_of(hands.begin(), hands.end(),
                          [](auto &h) { return h == "left" || h == "right"; }),
          "Invalid controlled hands");
  choice(c["clutch_binding"],
         {"both_grips", "left_grip", "right_grip", "both_primary",
          "left_primary", "right_primary"});
  choice(c["hold_binding"], {"left_secondary", "right_secondary"});
  require(c["grasp_bindings"].is_object(), "Expected grasp bindings object");
  for (auto it = c["grasp_bindings"].begin(); it != c["grasp_bindings"].end();
       ++it) {
    validate_identifier(it.key());
    choice(it.value(),
           {"left_trigger", "right_trigger", "left_grip", "right_grip"});
    require(c["engagement_mode"] == "continuous" || (it.value() != c["clutch_binding"] &&
                !(c["clutch_binding"] == "both_grips" &&
                  (it.value() == "left_grip" || it.value() == "right_grip"))),
            "Clutch and grasp share an input");
  }
  require(c["task_presets"].is_array(), "Expected task presets array");
  for (auto &task : c["task_presets"]) {
    text(task);
    require(task.get<std::string>().find_first_not_of(" \t\r\n") !=
                std::string::npos,
            "Empty task preset");
  }
  integer(c["queue_budget_bytes"], 1024, 1024LL * 1024 * 1024);
  integer(c["minimum_free_bytes"], 1024, INT64_MAX);
  require(c["streams"].is_array() && c["cameras"].is_array(),
          "Expected streams/cameras arrays");
  std::set<std::string> ids, topics, cameras, camera_sources;
  for (auto &s : c["streams"]) {
    s = normalized(s, {{"stream_id", ""},
                       {"topic", ""},
                       {"message_type", ""},
                       {"role", "observation"},
                       {"required", true},
                       {"max_gap_seconds", 1.0},
                       {"clock_id", "ros"},
                       {"frame_id", ""},
                       {"source_identity", "unknown"},
                       {"stamp_semantics", "unknown"},
                       {"calibration", object},
                       {"reliability", "best_effort"},
                       {"durability", "volatile"},
                       {"queue_depth", 5}});
    text(s["stream_id"]);
    validate_identifier(s["stream_id"]);
    text(s["topic"]);
    text(s["message_type"]);
    require(std::regex_match(s["topic"].get<std::string>(),
                             std::regex("/[A-Za-z0-9_/]+")),
            "Expected absolute ROS topic");
    require(std::regex_match(s["message_type"].get<std::string>(),
                             std::regex("[A-Za-z0-9_]+/msg/[A-Za-z0-9_]+")),
            "Expected package/msg/Type");
    require(ids.insert(s["stream_id"]).second &&
                topics.insert(s["topic"]).second,
            "Duplicate stream ID/topic");
    choice(s["role"], {"observation", "camera", "measured_state",
                       "accepted_command", "feedback", "tf"});
    require(s["required"].is_boolean(), "Expected required boolean");
    number(s["max_gap_seconds"], std::numeric_limits<double>::min(), 1e100);
    text(s["clock_id"]);
    text(s["source_identity"], false);
    text(s["frame_id"], false);
    choice(s["stamp_semantics"],
           {"unknown", "acquisition", "receipt", "simulation"});
    choice(s["reliability"], {"best_effort", "reliable"});
    choice(s["durability"], {"volatile", "transient_local"});
    integer(s["queue_depth"], 1, 100);
    if (s["role"] == "camera" && s["source_identity"] != "unknown")
      require(camera_sources.insert(s["source_identity"]).second,
              "Duplicate physical camera source");
  }
  for (auto &camera : c["cameras"]) {
    camera = normalized(camera, {{"sensor_id", ""},
                                 {"role", "front"},
                                 {"position", {0.0, 1.35, 1.2}},
                                 {"width_m", 0.7},
                                 {"stream_id", ""},
                                 {"required_for_control", false},
                                 {"rotation_deg", {0.0, 0.0, 0.0}},
                                 {"portal", nullptr}});
    if (camera["portal"].is_null()) camera.erase("portal");
    else {
      auto &p = camera["portal"];
      p = normalized(p, {{"mode", "stereo"}, {"edge_fade", 0.06}, {"corner_radius", 0.08},
                         {"crop", 0.02}, {"swap_eyes", false}, {"max_skew_ms", 50.0}});
      choice(p["mode"], {"mono", "stereo", "estimated"});
      number(p["edge_fade"], 0, 0.25); number(p["corner_radius"], 0, 0.25);
      number(p["crop"], 0, 0.1); number(p["max_skew_ms"], 0, 100);
      require(p["swap_eyes"].is_boolean(), "Expected portal eye-swap boolean");
    }
    text(camera["sensor_id"]);
    require(camera["sensor_id"].get<std::string>().size() <= 128 &&
                cameras.insert(camera["sensor_id"]).second,
            "Duplicate/invalid camera sensor");
    text(camera["role"]);
    text(camera["stream_id"], false);
    vector(camera["position"], 3);
    vector(camera["rotation_deg"], 3);
    number(camera["width_m"], 0.1, 3);
    require(camera["required_for_control"].is_boolean(),
            "Expected camera control boolean");
    require(camera["stream_id"] == "" || ids.count(camera["stream_id"]),
            "Undeclared camera recording stream");
    require(!camera["required_for_control"].get<bool>() ||
                camera["stream_id"] != "",
            "Control camera requires freshness stream");
  }
  require(c["spatial_views"].is_array() && c["spatial_views"].size() <= 2,
          "Choose at most two spatial views");
  std::set<std::string> spatial_ids;
  for (auto &view : c["spatial_views"]) {
    view = normalized(view, {{"stream_id", ""}, {"point_size_m", 0.004},
                             {"max_points", 1000000}, {"max_age_seconds", 1.0}});
    text(view["stream_id"]);
    require(spatial_ids.insert(view["stream_id"]).second, "Duplicate spatial stream");
    number(view["point_size_m"], .001, .03);
    number(view["max_age_seconds"], .1, 5.0);
    integer(view["max_points"], 1000, 2000000);
    require(std::any_of(c["streams"].begin(), c["streams"].end(), [&](const Json &s) {
      return s["stream_id"] == view["stream_id"] && s["message_type"] == "sensor_msgs/msg/PointCloud2";
    }), "Spatial view requires a declared PointCloud2 stream");
  }
  if (c["spatial_views"].empty()) c.erase("spatial_views");
  auto joints = unique_strings(c["joint_names"]);
  std::set<std::string> grouped;
  require(c["joint_groups"].is_object(), "Expected joint group object");
  for (const auto &group : c["joint_groups"])
    for (const auto &joint : unique_strings(group))
      require(grouped.insert(joint).second, "Overlapping joint groups");
  require(grouped == joints,
          "Each joint must belong to exactly one owned group");
  require(c["adapter_id"] == "observe" ||
              (!joints.empty() && c["command_frame"] != "" &&
               c["calibration_id"] != ""),
          "Motion profile requires joints, command frame and calibration");
  if (c["engagement_mode"] == "hold_to_run") {
    // Preserve legacy v1/v2 manifests and their hashes.
    c.erase("engagement_mode");
    c.erase("engage_countdown_seconds");
    c.erase("disengage_binding");
  }
  if (c["tracking_recovery"] == "manual") c.erase("tracking_recovery");
  digest_ = sha256(canonical(c));
}
Json TeachConfig::registration(const std::string &name) const {
  validate_identifier(name);
  const std::string prefix = "/horus/teach/" + name;
  Json out = {{"schema_version", 1},
              {"enabled", true},
              {"config_hash", digest_},
              {"request_topic", prefix + "/request"},
              {"input_topic", prefix + "/input"},
              {"state_topic", prefix + "/state"}};
  for (auto key : {"workflow", "task", "publish_rate_hz", "input_timeout_ms",
                   "controlled_hands", "clutch_binding", "hold_binding",
                   "task_presets", "cameras"})
    out[key] = config_.at(key);
  for (auto &c : out["cameras"])
    for (auto key : {"position", "rotation_deg"}) {
      auto a = c[key];
      c[key] = {{"x", a[0]}, {"y", a[1]}, {"z", a[2]}};
    }
  if (config_.value("input_schema_version", 1) == 2) {
    out["input_schema_version"] = 2;
    out["required_input_features"] = config_.at("required_input_features");
  }
  if (config_.value("engagement_mode", "hold_to_run") == "continuous")
    for (auto key : {"engagement_mode", "engage_countdown_seconds", "disengage_binding"})
      out[key] = config_.at(key);
  if (config_.contains("tracking_recovery")) out["tracking_recovery"] = config_.at("tracking_recovery");
  if (config_.contains("grasp_bindings") && !config_["grasp_bindings"].empty()) {
    out["grasp_hints"] = Json::array();
    for (auto it = config_["grasp_bindings"].begin(); it != config_["grasp_bindings"].end(); ++it) {
      auto label = it.key();
      if (label.rfind("left_", 0) == 0) label.erase(0, 5);
      if (label.rfind("right_", 0) == 0) label.erase(0, 6);
      std::replace(label.begin(), label.end(), '_', ' ');
      if (!label.empty()) label[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(label[0])));
      label = label.substr(0, 24);
      out["grasp_hints"].push_back({{"control", it.value()}, {"action_id", it.key()}, {"label", label}});
    }
  }
  if (config_.contains("spatial_views")) {
    out["spatial_views"] = config_.at("spatial_views");
    for (auto &view : out["spatial_views"])
      for (auto &stream : config_.at("streams"))
        if (view["stream_id"] == stream["stream_id"]) view["topic"] = stream["topic"];
  }
  return out;
}
} // namespace horus::teaching
