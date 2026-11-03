#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>

namespace horus::dataviz {
struct CollisionObservation {
    std::optional<double> min_distance_m;
    std::array<double, 3> direction{};
};
inline double risk_from_distance(std::optional<double> distance, double threshold) {
    if (!distance || !std::isfinite(*distance))
        return 0;
    if (!std::isfinite(threshold))
        throw std::invalid_argument("Threshold must be finite");
    threshold = std::max(threshold, 1e-6);
    return std::clamp((threshold - *distance) / threshold, 0., 1.);
}
inline std::array<double, 3> normalize_planar(double x, double y) {
    const double length = std::hypot(x, y);
    return length > 1e-6 ? std::array<double, 3>{x / length, y / length, 0.}
                         : std::array<double, 3>{};
}
template <class T>
CollisionObservation analyze_laser_scan(std::span<const T> ranges, double angle_min,
                                        double increment, double minimum, double maximum) {
    CollisionObservation out;
    if (!std::isfinite(angle_min) || !std::isfinite(increment) || !std::isfinite(minimum) ||
        !std::isfinite(maximum))
        throw std::invalid_argument("Laser parameters must be finite");
    double angle = angle_min;
    for (T value : ranges) {
        if (std::isfinite(value) && value >= minimum && value <= maximum &&
            (!out.min_distance_m || value < *out.min_distance_m)) {
            out.min_distance_m = value;
            out.direction = normalize_planar(std::cos(angle), std::sin(angle));
        }
        angle += increment;
    }
    return out;
}
inline void include_point(CollisionObservation &out, double x, double y, double z,
                          double z_window) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || std::abs(z) > z_window)
        return;
    const double length = std::hypot(x, y);
    if (!out.min_distance_m || length < *out.min_distance_m) {
        out.min_distance_m = length;
        out.direction = normalize_planar(x, y);
    }
}
template <class T>
CollisionObservation analyze_point_cloud(std::span<const std::array<T, 3>> points,
                                         double z_window) {
    if (!std::isfinite(z_window))
        throw std::invalid_argument("Z window must be finite");
    CollisionObservation out;
    for (const auto &p : points)
        include_point(out, p[0], p[1], p[2], std::max(z_window, 0.));
    return out;
}
struct CollisionRiskOptions {
    std::string robot_name, source_type = "laser_scan", source_topic, output_topic;
    double threshold_m = 1.2, publish_hz = 10, smoothing_alpha = .35, z_window_m = .35;
};
// Advisory visualization only, not a collision-avoidance controller. Owns ROS context.
class CollisionRiskAnalyzer {
    struct Impl;
    std::unique_ptr<Impl> impl_;

  public:
    explicit CollisionRiskAnalyzer(const CollisionRiskOptions &);
    ~CollisionRiskAnalyzer();
    CollisionRiskAnalyzer(const CollisionRiskAnalyzer &) = delete;
    void stop();
};
} // namespace horus::dataviz
