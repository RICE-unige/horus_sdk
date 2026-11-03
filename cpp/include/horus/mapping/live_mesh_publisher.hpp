#pragma once
#include "horus/mapping/live_mesh.hpp"
#include <memory>

namespace horus::mapping {
struct LiveMeshPublisherOptions {
    std::string topic{"/horus/maps/live"};
    LiveMeshOptions map;
    double pages_per_second{60.0}, bytes_per_second{4.0 * 1024 * 1024};
};
// Caller-serialized. Call pump regularly; no global ROS context/signal ownership.
// Inputs must already be paged and expressed in map.frame_id.
class LiveMeshPublisher {
public:
    explicit LiveMeshPublisher(LiveMeshPublisherOptions options = {});
    ~LiveMeshPublisher();
    LiveMeshPublisher(const LiveMeshPublisher&) = delete;
    LiveMeshPublisher& operator=(const LiveMeshPublisher&) = delete;
    std::size_t replace(const std::string& key, const BlockGeometry* geometry);
    std::size_t replace_many(std::span<const std::pair<std::string, const BlockGeometry*>> changes);
    void clear();
    std::size_t pump();
    const LiveMeshState& state() const;
    std::size_t pending_count() const;
    std::uint64_t dropped_repair_requests() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
