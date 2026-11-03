#pragma once
#include <memory>
#include <string>
#include <nlohmann/json.hpp>

namespace horus::mapping {
struct CostmapAdapterOptions {
    std::string input_topic, input_format = "occupancy", updates_topic, snapshot_service;
    std::string output_topic = "/horus/costmap";
    double publish_hz = 5, resync_seconds = 2;
    bool volatile_full = false;
};
// Owns a private ROS context and worker. Destruction stops only this adapter.
class CostmapAdapter {
 public:
    explicit CostmapAdapter(const CostmapAdapterOptions&);
    ~CostmapAdapter();
    CostmapAdapter(const CostmapAdapter&) = delete;
    CostmapAdapter& operator=(const CostmapAdapter&) = delete;
    nlohmann::json status() const;
 private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
