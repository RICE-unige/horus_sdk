#pragma once
#include <nlohmann/json.hpp>
#include <string>

namespace horus::monitoring {
// Endpoint discovery is evidence of routing, not message delivery or rendering.
bool is_bridge_node(const std::string &name);
nlohmann::json registration_topics(const nlohmann::json &payloads);
std::string graph_link(const std::string &role, const nlohmann::json &publishers,
                       const nlohmann::json &subscribers, const std::string &local_name,
                       const std::string &local_namespace);
// Plain-text rendering is deterministic and usable in redirected logs.
std::string render_dashboard(const nlohmann::json &snapshot, unsigned width = 100,
                             unsigned rows = 24, unsigned page = 0, bool color = false);
} // namespace horus::monitoring
