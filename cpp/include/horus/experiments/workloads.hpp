#pragma once
#include "manifest.hpp"
namespace horus::experiments {
struct CameraWorkload {
    int streams = 0;
    std::string resolution;
    double fps = 0;
    std::string encoding = "none";
    double bitrate_mbps = 0;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(CameraWorkload, streams, resolution, fps, encoding,
                                                bitrate_mbps)
struct PointCloudWorkload {
    int64_t points = 0;
    double hz = 0;
    int point_step = 16;
    bool xyzrgb = true;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(PointCloudWorkload, points, hz, point_step, xyzrgb)
struct MapWorkload {
    std::string representation = "none";
    int64_t chunks = 0, vertices = 0, triangles = 0;
    double hz = 0;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(MapWorkload, representation, chunks, vertices,
                                                triangles, hz)
struct WorkloadConfig {
    std::string experiment = "E0_baseline", condition = "default", transport = "ros2",
                topology = "lan", robot_profile = "minimal", map_profile = "none";
    double duration_s = 120, warmup_s = 30;
    int repetition = 1, robot_count = 1, operator_count = 1;
    CameraWorkload camera;
    PointCloudWorkload pointcloud;
    MapWorkload map;
    Json extra = Json::object();
    int64_t stream_count() const {
        return int64_t(camera.streams) +
               (pointcloud.points > 0 && pointcloud.hz > 0 ? int64_t(robot_count) : 0) +
               (map.representation != "none" && map.hz > 0 ? int64_t(robot_count) : 0);
    }
    std::string resolution() const { return camera.resolution; }
    double target_fps() const { return camera.fps; }
    Json to_dict() const;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WorkloadConfig, experiment, condition, transport,
                                                topology, robot_profile, map_profile, duration_s,
                                                warmup_s, repetition, robot_count, operator_count,
                                                camera, pointcloud, map, extra)
inline Json WorkloadConfig::to_dict() const { return *this; }
inline WorkloadConfig workload_from_dict(const Json &payload) {
    if (!payload.is_object())
        throw std::invalid_argument("Workload must be an object");
    auto result = payload.get<WorkloadConfig>();
    if (!result.extra.is_object())
        throw std::invalid_argument("Workload extra must be an object");
    const Json known = WorkloadConfig{};
    for (const auto &[key, value] : payload.items())
        if (!known.contains(key))
            result.extra[key] = value;
    return result;
}
inline WorkloadConfig load_workload_config(const std::filesystem::path &path) {
    if (std::filesystem::file_size(path) > 1024 * 1024)
        throw std::invalid_argument("Workload exceeds 1 MiB");
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("Cannot read workload");
    return workload_from_dict(Json::parse(input));
}
} // namespace horus::experiments
