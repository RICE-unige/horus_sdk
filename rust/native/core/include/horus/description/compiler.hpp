#pragma once
#include <nlohmann/json.hpp>
#include <string>

namespace horus::description {
struct Artifact {
    nlohmann::json manifest;
    nlohmann::json payload;
    std::string payload_json;
    std::string encoded_payload;
    nlohmann::json to_dict() const;
};

// Mesh coordinates stay in ROS axes. Fixed-link meshes are combined only within
// rigid groups; movable joints retain independent TF bindings.
Artifact compile_urdf(const std::string &xml, const nlohmann::json &options);
Artifact compile_file(const nlohmann::json &options);
std::string encode_description(const std::string &payload);
} // namespace horus::description
