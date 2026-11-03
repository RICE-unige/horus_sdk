#include "horus/mapping/live_mesh.hpp"
#include <chrono>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace horus::mapping {
namespace {
using Json = nlohmann::json;
constexpr auto max_revision = std::numeric_limits<std::uint64_t>::max();
void add_counter(std::uint64_t& value, std::uint64_t increment) {
    value = increment > max_revision - value ? max_revision : value + increment;
}
}
LiveMeshState::LiveMeshState(LiveMeshOptions options) : options_(std::move(options)) {
    if (options_.frame_id.empty() || options_.max_pages < 1 || options_.max_pages > 1024 ||
        options_.max_payload_bytes < HMSH_V2_HEADER_SIZE || options_.max_cache_bytes < options_.max_payload_bytes)
        throw std::invalid_argument("Invalid live mesh frame or capacity limits");
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    if (!options_.initial_epoch && now < 0) throw std::invalid_argument("Invalid epoch clock");
    epoch_ = options_.initial_epoch.value_or(static_cast<std::uint64_t>(now));
    if (epoch_ == max_revision) throw std::invalid_argument("Map epoch exhausted");
}
std::vector<MeshPage> LiveMeshState::replace(const std::string& key, const BlockGeometry* geometry) {
    if (geometry && (geometry->vertices.empty() || geometry->triangles.empty()))
        throw std::invalid_argument("Incomplete mesh geometry");
    const auto existing = keys_.find(key);
    if (!geometry && existing == keys_.end()) return {};
    const bool is_new = existing == keys_.end();
    const auto id = is_new ? (free_.empty() ? next_id_ : *free_.begin()) : existing->second;
    if (id > options_.max_pages) throw std::length_error("Mesh page capacity exceeded; previous map retained");
    const auto found = current_.find(id);
    const MeshPage old = found == current_.end() ? nullptr : found->second;
    if (old && old->revision == max_revision) throw std::overflow_error("Revision exhausted; clear the map");
    const auto revision = old ? old->revision + 1 : 1;
    const auto operation = geometry ? MeshOperation::Upsert : MeshOperation::Delete;
    auto payload = encode_live_payload(id, {0,0,0}, epoch_, old ? old->revision : revision,
        operation, geometry, options_.max_payload_bytes);
    if (old && payload == old->payload) {
        add_counter(unchanged_pages_, 1); add_counter(suppressed_bytes_, payload.size()); return {};
    }
    for (int i=0; i<8; ++i) payload[32+i] = static_cast<std::uint8_t>(revision >> (8*i));
    const auto remaining = cached_bytes_ - (old ? old->payload.size() : 0);
    if (payload.size() > options_.max_cache_bytes - remaining)
        throw std::length_error("Mesh cache capacity exceeded; previous map retained");
    const auto size = remaining + payload.size();
    auto update = std::make_shared<const MeshPageUpdate>(MeshPageUpdate{id, epoch_, revision, operation,
        std::move(payload), geometry ? geometry->vertices.size() : 0, geometry ? geometry->triangles.size()/3 : 0});
    if (is_new) { keys_.emplace(key, id); if (free_.erase(id) == 0) ++next_id_; }
    if (!geometry) { keys_.erase(key); free_.insert(id); }
    current_[id] = update;
    cached_bytes_ = size;
    return {update};
}
std::vector<MeshPage> LiveMeshState::replace_many(std::span<const std::pair<std::string, const BlockGeometry*>> changes) {
    // Clone bounded metadata only; unchanged mesh bytes remain shared.
    auto staged = *this;
    std::vector<MeshPage> updates;
    for (const auto& [key, geometry] : changes) {
        auto changed = staged.replace(key, geometry);
        updates.insert(updates.end(), changed.begin(), changed.end());
    }
    *this = std::move(staged);
    return updates;
}
MeshPage LiveMeshState::snapshot_clear_update() const {
    return std::make_shared<const MeshPageUpdate>(MeshPageUpdate{0, epoch_, 0, MeshOperation::Clear,
        encode_live_payload(0, {0,0,0}, epoch_, 0, MeshOperation::Clear, nullptr, options_.max_payload_bytes),0,0});
}
MeshPage LiveMeshState::clear() {
    if (epoch_ == max_revision) throw std::overflow_error("Map epoch exhausted");
    ++epoch_;
    keys_.clear(); current_.clear(); free_.clear(); next_id_ = 1; cached_bytes_ = 0;
    return snapshot_clear_update();
}
std::vector<MeshPage> LiveMeshState::snapshot_updates() const {
    std::vector<MeshPage> updates;
    updates.reserve(current_.size());
    for (const auto& [id, page] : current_) updates.push_back(page);
    return updates;
}
std::string LiveMeshState::manifest_json() const {
    Json pages = Json::array();
    std::size_t triangles = 0;
    for (const auto& [id, u] : current_) {
        pages.push_back({{"chunk_id",id},{"revision",std::to_string(u->revision)},
            {"operation",static_cast<int>(u->operation)}});
        triangles += u->triangle_count;
    }
    return Json{{"version",1},{"epoch",std::to_string(epoch_)},{"frame_id",options_.frame_id},
        {"pages",pages},{"triangles",triangles}}.dump();
}
std::vector<MeshPage> LiveMeshState::repair_updates(const std::string& request_json) const {
    const auto request = Json::parse(request_json, nullptr, false);
    if (!request.is_object() || !request.contains("request") || !request["request"].is_string()) return {};
    const auto kind = request["request"].get<std::string>();
    if (kind != "snapshot" && kind != "pages") return {};
    if (current_.empty()) return {snapshot_clear_update()};
    const auto epoch = request.find("epoch");
    const auto requested = epoch == request.end() ? "" : (epoch->is_string() ? epoch->get<std::string>() : epoch->dump());
    if (kind == "snapshot" || requested != std::to_string(epoch_)) return snapshot_updates();
    const auto ids = request.find("ids");
    if (ids == request.end() || !ids->is_array() || ids->size() > options_.max_pages) return {};
    std::set<std::uint32_t> selected;
    for (const auto& id : *ids) {
        if (!id.is_number_integer() || id < 1 || id > options_.max_pages) return {};
        selected.insert(id.get<std::uint32_t>());
    }
    std::vector<MeshPage> updates;
    for (const auto id : selected) {
        const auto found = current_.find(id);
        if (found != current_.end()) updates.push_back(found->second);
    }
    return updates;
}
}
