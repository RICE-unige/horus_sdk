#include "horus/mapping/mesh_page_scheduler.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace horus::mapping {
MeshPageScheduler::MeshPageScheduler(MeshSchedulerOptions options, double now)
    : options_(options), last_time_(now) {
    if (!std::isfinite(now) || !std::isfinite(options.pages_per_second) ||
        !std::isfinite(options.bytes_per_second) || options.pages_per_second <= 0 ||
        options.bytes_per_second <= 0 || options.max_pages == 0 || options.max_payload_bytes == 0 ||
        options.max_pages > std::numeric_limits<std::size_t>::max() / options.max_payload_bytes - 1)
        throw std::invalid_argument("Invalid mesh scheduler capacity, rate or clock");
}

bool MeshPageScheduler::enqueue(MeshPage update) {
    if (!update || update->payload.size() > options_.max_payload_bytes)
        throw std::invalid_argument("Mesh page exceeds scheduler payload capacity or is null");
    if (epoch_ && update->epoch < *epoch_) return false;
    const bool new_epoch = !epoch_ || update->epoch > *epoch_;
    if (new_epoch) {
        pages_.clear();
        order_.clear();
        clear_.reset();
        pending_bytes_ = 0;
        epoch_ = update->epoch;
    }
    if (update->operation == MeshOperation::Clear) {
        if (!new_epoch && !pages_.empty()) return false;
        if (clear_) pending_bytes_ -= clear_->payload.size();
        pending_bytes_ += update->payload.size();
        clear_ = std::move(update);
        return true;
    }
    auto found = pages_.find(update->chunk_id);
    if (found != pages_.end()) {
        if (update->revision <= found->second->revision) return false;
        pending_bytes_ -= found->second->payload.size();
        pending_bytes_ += update->payload.size();
        found->second = std::move(update);
    } else {
        if (pages_.size() >= options_.max_pages)
            throw std::length_error("Mesh scheduler page capacity exceeded");
        const auto id = update->chunk_id;
        // Preserve insertion order when replacing a frequently updated page.
        order_.push_back(id);
        try { pages_.emplace(id, update); }
        catch (...) { order_.pop_back(); throw; }
        pending_bytes_ += update->payload.size();
    }
    return true;
}

MeshPage MeshPageScheduler::pop_ready(double now) {
    if (!std::isfinite(now)) throw std::invalid_argument("Mesh scheduler clock must be finite");
    const double elapsed = std::max(0.0, now - last_time_);
    last_time_ = std::max(last_time_, now);
    MeshPage update = clear_ ? clear_ : (order_.empty() ? nullptr : pages_.at(order_.front()));
    page_tokens_ = std::min(4.0, page_tokens_ + elapsed * options_.pages_per_second);
    const auto bytes = update ? update->payload.size() : 0;
    const double capacity = std::max(options_.bytes_per_second * 0.1, static_cast<double>(bytes));
    byte_tokens_ = std::min(capacity, byte_tokens_ + elapsed * options_.bytes_per_second);
    if (!update || page_tokens_ < 1.0 || byte_tokens_ < bytes) return nullptr;
    --page_tokens_;
    byte_tokens_ -= bytes;
    pending_bytes_ -= bytes;
    if (clear_) clear_.reset();
    else { pages_.erase(order_.front()); order_.pop_front(); }
    return update;
}
}
