#pragma once
#include "horus/teaching/config.hpp"
#include <algorithm>
#include <cmath>
#include <codecvt>
#include <locale>
#include <regex>
#include <set>
#include <vector>

namespace horus {
enum class ControllerControl { LeftPrimary, LeftSecondary, LeftTrigger, LeftGrip, LeftStickX, LeftStickY,
    LeftStickClick, RightPrimary, RightSecondary, RightTrigger, RightGrip, RightStickX, RightStickY, RightStickClick };
enum class TeleopHintMode { Minimap, Immersive, FollowLeader, Teach };
namespace hint_detail {
using Json = teaching::Json;
inline const std::vector<std::string> controls = {"left_primary", "left_secondary", "left_trigger", "left_grip",
    "left_stick_x", "left_stick_y", "left_stick_click", "right_primary", "right_secondary", "right_trigger",
    "right_grip", "right_stick_x", "right_stick_y", "right_stick_click"};
inline const std::vector<std::string> modes = {"minimap", "immersive", "follow_leader", "teach"};
inline void require(bool valid, const char* reason) { if (!valid) throw std::invalid_argument(reason); }
inline std::string id(const std::string& value) {
    require(std::regex_match(value, std::regex("[a-z][a-z0-9_]{0,63}")), "Invalid hint action/profile identifier"); return value;
}
inline std::string label(const std::string& value) {
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> codec;
    auto units = codec.from_bytes(value);
    require(units.size() <= 24, "Controller labels must fit 24 UTF-16 units");
    for (auto c : units) require(c >= 32 && !(c >= 127 && c <= 159) && c != 0x2028 && c != 0x2029, "Controller labels must be single line");
    // JSON validates complete UTF-8, including rejection of encoded surrogate code points.
    (void)Json(value).dump();
    auto space = [](char16_t c) { return c == 0x20 || c == 0xa0 || c == 0x1680 ||
        (c >= 0x2000 && c <= 0x200a) || c == 0x202f || c == 0x205f || c == 0x3000; };
    size_t first = 0, last = units.size();
    while (first < last && space(units[first])) ++first;
    while (last > first && space(units[last - 1])) --last;
    return codec.to_bytes(units.substr(first, last - first));
}
inline size_t index(const std::vector<std::string>& values, const std::string& value) {
    auto it = std::find(values.begin(), values.end(), value); require(it != values.end(), "Unknown controller input or mode"); return size_t(it - values.begin());
}
inline std::string name(ControllerControl value) { return controls.at(static_cast<size_t>(value)); }
inline std::string name(TeleopHintMode value) { return modes.at(static_cast<size_t>(value)); }
inline void integer(const Json& value) { require(value.is_number_integer() && value.get<int64_t>() >= 0 && value.get<int64_t>() <= 2147483647, "Expected nonnegative 32-bit hint sequence"); }
}

struct ControllerHint {
    ControllerControl control;
    std::string action_id, label;
    teaching::Json to_dict() const { return {{"control", hint_detail::name(control)}, {"action_id", hint_detail::id(action_id)}, {"label", hint_detail::label(label)}}; }
};
struct ControllerHintProfile {
    TeleopHintMode mode;
    std::vector<ControllerHint> bindings;
    std::string profile_id = "default";
    teaching::Json to_dict() const {
        hint_detail::require(bindings.size() <= 14, "Too many controller hint bindings");
        std::set<ControllerControl> unique;
        auto values = teaching::Json::array();
        for (const auto& b : bindings) { hint_detail::require(unique.insert(b.control).second, "Duplicate controller binding"); values.push_back(b.to_dict()); }
        return {{"mode", hint_detail::name(mode)}, {"profile_id", hint_detail::id(profile_id)}, {"bindings", values}};
    }
};
struct ControllerHintContext {
    std::string robot_name, config_hash, profile_id;
    TeleopHintMode mode;
    std::string context_id;
    int context_sequence = 0;
    static ControllerHintContext from_json(const std::string& payload) {
        hint_detail::require(payload.size() <= 16384, "Hint context too large");
        auto j = teaching::Json::parse(payload);
        hint_detail::require(j.is_object() && j.size() == 7 && j.at("schema_version") == 1, "Invalid hint context schema");
        hint_detail::integer(j.at("schema_version")); hint_detail::integer(j.at("context_sequence"));
        auto robot = j.at("robot_name").get<std::string>();
        auto hash = j.at("config_hash").get<std::string>(); auto nonce = j.at("context_id").get<std::string>();
        hint_detail::require(!robot.empty() && robot.size() <= 128 && std::none_of(robot.begin(), robot.end(), [](unsigned char c){ return c < 32; }), "Invalid hint robot name");
        hint_detail::require(std::regex_match(hash, std::regex("[0-9a-f]{64}")) &&
            std::regex_match(nonce, std::regex("[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}")), "Invalid hint identity");
        return {robot, hash, hint_detail::id(j.at("profile_id")), static_cast<TeleopHintMode>(hint_detail::index(hint_detail::modes, j.at("mode"))), nonce, j.at("context_sequence").get<int>()};
    }
    teaching::Json to_dict() const { return {{"schema_version", 1}, {"robot_name", robot_name}, {"config_hash", config_hash},
        {"profile_id", profile_id}, {"mode", hint_detail::name(mode)}, {"context_id", context_id}, {"context_sequence", context_sequence}}; }
};
struct ControllerHintLabel { std::string action_id, label; };
struct ControllerHintUpdate {
    ControllerHintContext context;
    int revision;
    std::vector<ControllerHintLabel> labels;
    teaching::Json to_dict() const {
        hint_detail::require(revision >= 0 && labels.size() <= 14, "Invalid hint revision or label count");
        // Validate directly constructed contexts too.
        auto value = context.to_dict(); (void)ControllerHintContext::from_json(value.dump()); value["revision"] = revision;
        value["labels"] = teaching::Json::array(); std::set<std::string> unique;
        for (const auto& label : labels) {
            hint_detail::require(unique.insert(label.action_id).second, "Duplicate action label");
            value["labels"].push_back({{"action_id", hint_detail::id(label.action_id)}, {"label", hint_detail::label(label.label)}});
        }
        return value;
    }
    std::string to_json() const { return to_dict().dump(); }
};
struct ControllerHints {
    std::vector<ControllerHintProfile> profiles;
    std::string state_topic;
    double state_timeout_seconds = 2.0;
    std::string context_topic() const { return state_topic.empty() ? "" : state_topic + "/context"; }
    teaching::Json content() const {
        hint_detail::require(profiles.size() <= 4 && std::isfinite(state_timeout_seconds) && state_timeout_seconds >= .5 && state_timeout_seconds <= 10, "Invalid hint profile count/timeout");
        hint_detail::require(state_topic.size() <= 240 && (state_topic.empty() || std::regex_match(state_topic,
            std::regex("/(?:[a-zA-Z_][a-zA-Z0-9_]*)(?:/[a-zA-Z_][a-zA-Z0-9_]*)*"))), "Invalid absolute hint topic");
        teaching::Json value = {{"schema_version", 1}, {"profiles", teaching::Json::array()}, {"state_timeout_seconds", state_timeout_seconds}};
        std::set<TeleopHintMode> modes;
        for (const auto& p : profiles) { hint_detail::require(modes.insert(p.mode).second, "Duplicate hint mode"); value["profiles"].push_back(p.to_dict()); }
        if (!state_topic.empty()) value["state_topic"] = state_topic;
        return value;
    }
    std::string digest() const { return teaching::sha256(teaching::canonical(content())); }
    teaching::Json to_dict() const { auto result = content(); result["config_hash"] = digest(); return result; }
    static ControllerHints from_dict(const teaching::Json& value) {
        hint_detail::require(value.is_object() && value.at("schema_version").is_number_integer() && value.at("schema_version") == 1, "Invalid hint schema");
        const std::set<std::string> keys = {"schema_version", "profiles", "state_topic", "state_timeout_seconds", "config_hash"};
        for (auto it = value.begin(); it != value.end(); ++it) hint_detail::require(keys.count(it.key()), "Unknown hint configuration field");
        ControllerHints result;
        result.state_topic = value.value("state_topic", "");
        if (value.contains("state_timeout_seconds")) hint_detail::require(value["state_timeout_seconds"].is_number(), "Hint timeout must be numeric");
        result.state_timeout_seconds = value.value("state_timeout_seconds", 2.0);
        hint_detail::require(value.at("profiles").is_array(), "Hint profiles must be an array");
        for (const auto& p : value["profiles"]) {
            ControllerHintProfile profile{static_cast<TeleopHintMode>(hint_detail::index(hint_detail::modes, p.at("mode"))), {}, p.value("profile_id", "default")};
            hint_detail::require(p.at("bindings").is_array(), "Hint bindings must be an array");
            for (const auto& b : p["bindings"]) profile.bindings.push_back({static_cast<ControllerControl>(hint_detail::index(hint_detail::controls, b.at("control"))), b.at("action_id"), b.at("label")});
            result.profiles.push_back(std::move(profile));
        }
        auto hash = result.digest();
        hint_detail::require(!value.contains("config_hash") || value["config_hash"] == hash, "Hint configuration hash mismatch");
        return result;
    }
    ControllerHintUpdate reply(const ControllerHintContext& context, const std::string& robot,
                              const std::vector<ControllerHintLabel>& labels, int revision) const {
        hint_detail::require(context.robot_name == robot && context.config_hash == digest(), "Hint robot/configuration mismatch");
        const ControllerHintProfile* profile = nullptr;
        for (const auto& p : profiles) if (p.mode == context.mode && p.profile_id == context.profile_id) profile = &p;
        hint_detail::require(profile != nullptr, "Hint profile mismatch");
        for (const auto& label : labels) hint_detail::require(std::any_of(profile->bindings.begin(), profile->bindings.end(),
            [&](const auto& b){ return b.action_id == label.action_id; }), "Hint action is undeclared");
        ControllerHintUpdate result{context, revision, labels}; (void)result.to_dict(); return result;
    }
};
}
