#include "horus/mapping/live_mesh_publisher.hpp"
#include "horus/mapping/mesh_page_scheduler.hpp"
#include <chrono>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <tuple>
#if defined(HORUS_ROS2_ENABLED)
#include "mesh_transport.h"
#include "registry.h"
#endif

namespace horus::mapping {
struct LiveMeshPublisher::Impl {
    LiveMeshState map;
    MeshPageScheduler queue;
    std::chrono::steady_clock::time_point start{std::chrono::steady_clock::now()};
    double last_manifest{-std::numeric_limits<double>::infinity()};
    std::map<std::uint32_t, std::tuple<std::uint64_t,std::uint64_t,double>> sent;
    std::uint64_t dropped_requests{0};
#if defined(HORUS_ROS2_ENABLED)
    std::unique_ptr<void, decltype(&horus_native_mesh_destroy)> transport{nullptr, horus_native_mesh_destroy};
    static std::string take(char* text) {
        std::unique_ptr<char, decltype(&horus_native_free)> owned(text, horus_native_free);
        return text ? std::string(text) : "Native mesh transport did not return a result";
    }
    static void check(int result, char* error) {
        if (result) { if (error) horus_native_free(error); return; }
        throw std::runtime_error(take(error));
    }
#endif
    explicit Impl(const LiveMeshPublisherOptions& options) : map(options.map),
        queue({options.pages_per_second, options.bytes_per_second, options.map.max_pages, options.map.max_payload_bytes}) {
#if defined(HORUS_ROS2_ENABLED)
        char* error = nullptr;
        transport.reset(horus_native_mesh_create(options.topic.c_str(), options.map.max_payload_bytes, &error));
        if (!transport) throw std::runtime_error(take(error));
#else
        throw std::runtime_error("Build HORUS with ROS 2 enabled to publish a live mesh");
#endif
    }
    std::size_t enqueue(const std::vector<MeshPage>& updates) {
        for (const auto& page : updates) queue.enqueue(page);
        return updates.size();
    }
    std::size_t pump() {
#if defined(HORUS_ROS2_ENABLED)
        const double now = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        const auto status = nlohmann::json::parse(take(horus_native_mesh_poll(transport.get())));
        const auto error = status.value("error", std::string{});
        if (!error.empty()) throw std::runtime_error(error);
        dropped_requests = status.at("dropped_requests");
        if (status.at("request").is_string()) {
            for (const auto& update : map.repair_updates(status["request"].get<std::string>())) {
                const auto previous = sent.find(update->chunk_id);
                if (previous != sent.end() && std::get<0>(previous->second) == update->epoch &&
                    std::get<1>(previous->second) == update->revision && now - std::get<2>(previous->second) < 2.0) continue;
                queue.enqueue(update);
            }
        }
        if (now - last_manifest >= 2.0) {
            char* failure = nullptr;
            const int result = horus_native_mesh_manifest(transport.get(), map.manifest_json().c_str(), &failure);
            check(result, failure);
            last_manifest = now;
        }
        if (status.at("subscribers").get<std::size_t>() == 0) return 0;
        std::size_t count = 0;
        while (count < 4) {
            const auto update = queue.pop_ready(now);
            if (!update) break;
            char* failure = nullptr;
            const int result = horus_native_mesh_publish(transport.get(), update->payload.data(), update->payload.size(), &failure);
            if (!result) queue.enqueue(update);
            check(result, failure);
            sent[update->chunk_id] = {update->epoch,update->revision,now};
            ++count;
        }
        return count;
#else
        return 0;
#endif
    }
};
LiveMeshPublisher::LiveMeshPublisher(LiveMeshPublisherOptions options) : impl_(std::make_unique<Impl>(options)) {}
LiveMeshPublisher::~LiveMeshPublisher() = default;
std::size_t LiveMeshPublisher::replace(const std::string& key, const BlockGeometry* geometry) {
    return impl_->enqueue(impl_->map.replace(key, geometry));
}
std::size_t LiveMeshPublisher::replace_many(std::span<const std::pair<std::string, const BlockGeometry*>> changes) {
    return impl_->enqueue(impl_->map.replace_many(changes));
}
void LiveMeshPublisher::clear() {
    impl_->queue.enqueue(impl_->map.clear());
    impl_->sent.clear();
    impl_->last_manifest = -std::numeric_limits<double>::infinity();
}
std::size_t LiveMeshPublisher::pump() { return impl_->pump(); }
const LiveMeshState& LiveMeshPublisher::state() const { return impl_->map; }
std::size_t LiveMeshPublisher::pending_count() const { return impl_->queue.pending_count(); }
std::uint64_t LiveMeshPublisher::dropped_repair_requests() const { return impl_->dropped_requests; }
}
