#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>

namespace horus::tf {
struct Transform {
    std::string parent, child;
    std::array<double, 3> translation{};
    std::array<double, 4> rotation{0, 0, 0, 1}; // ROS xyzw.
    std::optional<int64_t> stamp_ns;            // Empty uses ROS time, not process uptime.
};
struct PublisherOptions {
    std::string topic = "/tf", static_topic = "/tf_static";
    bool use_sim_time = false;
};
// Owns a private ROS context. Serialize calls; static frames are retained for late joiners.
class Publisher {
  public:
    explicit Publisher(const PublisherOptions &options = {});
    ~Publisher();
    Publisher(const Publisher &) = delete;
    Publisher &operator=(const Publisher &) = delete;
    void publish(std::span<const Transform> transforms, bool is_static = false);
    // Matched DDS subscriptions, not proof of message delivery.
    size_t subscriber_count(bool is_static = false) const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace horus::tf
