#pragma once
#include "horus/mapping/live_mesh.hpp"
#include <deque>
#include <unordered_map>

namespace horus::mapping {
struct MeshSchedulerOptions {
    double pages_per_second{30.0}, bytes_per_second{4.0 * 1024 * 1024};
    std::size_t max_pages{1024}, max_payload_bytes{1024 * 1024};
};
// Caller-serialized token bucket. Times are seconds from a monotonic clock.
class MeshPageScheduler {
public:
    explicit MeshPageScheduler(MeshSchedulerOptions options = {}, double now = 0.0);
    bool enqueue(MeshPage update);
    MeshPage pop_ready(double now);
    std::size_t pending_count() const { return pages_.size() + (clear_ ? 1 : 0); }
    std::size_t pending_bytes() const { return pending_bytes_; }
private:
    MeshSchedulerOptions options_;
    std::optional<std::uint64_t> epoch_;
    MeshPage clear_;
    std::unordered_map<std::uint32_t, MeshPage> pages_;
    std::deque<std::uint32_t> order_;
    std::size_t pending_bytes_{0};
    double page_tokens_{0}, byte_tokens_{0}, last_time_;
};
}
