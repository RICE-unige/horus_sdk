#pragma once
#include "horus/mapping/mesh_protocol.hpp"
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>

namespace horus::mapping {
struct MeshPageUpdate {
    std::uint32_t chunk_id;
    std::uint64_t epoch, revision;
    MeshOperation operation;
    std::vector<std::uint8_t> payload;
    std::size_t vertex_count, triangle_count;
};
using MeshPage = std::shared_ptr<const MeshPageUpdate>;
struct LiveMeshOptions {
    std::string frame_id{"map"};
    std::uint32_t max_pages{1024};
    std::size_t max_payload_bytes{1024 * 1024}, max_cache_bytes{256 * 1024 * 1024};
    std::optional<std::uint64_t> initial_epoch;
};
// Mutations are caller-serialized. Immutable snapshots can outlive the state.
class LiveMeshState {
public:
    explicit LiveMeshState(LiveMeshOptions options = {});
    std::vector<MeshPage> replace(const std::string& key, const BlockGeometry* geometry);
    std::vector<MeshPage> replace_many(std::span<const std::pair<std::string, const BlockGeometry*>> changes);
    MeshPage clear();
    MeshPage snapshot_clear_update() const;
    std::vector<MeshPage> snapshot_updates() const;
    std::vector<MeshPage> repair_updates(const std::string& request_json) const;
    std::string manifest_json() const;
    std::uint64_t epoch() const { return epoch_; }
    std::size_t page_count() const { return keys_.size(); }
    std::size_t cached_bytes() const { return cached_bytes_; }
    std::uint64_t unchanged_pages() const { return unchanged_pages_; }
    std::uint64_t suppressed_bytes() const { return suppressed_bytes_; }
private:
    LiveMeshOptions options_;
    std::uint64_t epoch_;
    std::map<std::string, std::uint32_t> keys_;
    std::map<std::uint32_t, MeshPage> current_;
    std::set<std::uint32_t> free_;
    std::uint32_t next_id_{1};
    std::size_t cached_bytes_{0};
    std::uint64_t unchanged_pages_{0}, suppressed_bytes_{0};
};
}
