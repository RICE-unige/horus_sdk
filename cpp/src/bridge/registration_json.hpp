#pragma once
#include <any>
#include <map>
#include <nlohmann/json.hpp>

namespace horus::bridge {
struct RobotRegistrationPayload;
nlohmann::json payload_json(const RobotRegistrationPayload& payload);
nlohmann::json any_json(const std::any& value);
nlohmann::json metadata_json(const std::map<std::string, std::any>& value);
std::any json_any(const nlohmann::json& value);
}
